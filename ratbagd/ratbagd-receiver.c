/*
 * Logitech receiver object for ratbagd.
 *
 * A receiver is not a configurable device: it has no profiles and no buttons.
 * What it can do is tell you which devices are paired to it and open a pairing
 * window so a new device can join. This object exposes exactly that, over
 * liblur, so a GUI does not have to open hidraw nodes itself.
 *
 * The pairing window lives in the receiver's own firmware: it is armed with a
 * timeout and expires on its own. We keep the fd open while it is armed so the
 * window cannot be affected by us dropping the handle, and close it from an
 * event timer.
 */

#include "ratbagd.h"
#include "shared-macro.h"
#include "libratbag-util.h"

#include <fcntl.h>
#include <liblur.h>
#include <limits.h>
#include <stdio.h>
#include <unistd.h>

#define USEC_PER_SEC 1000000ULL

struct ratbagd_receiver {
	unsigned int refcount;

	struct ratbagd *ctx;
	RBNode node;
	char *sysname;
	char *path;

	/* only valid while a pairing window is armed */
	struct lur_receiver *lur;
	int fd;
	bool pairing_open;
	sd_event_source *pairing_timer;
	sd_bus_slot *vtable_slot;

	unsigned int fw_major;
	unsigned int fw_minor;
	unsigned int fw_build;
};

#define ratbagd_receiver_from_node(_ptr) \
		rbnode_of((_ptr), struct ratbagd_receiver, node)

static struct lur_receiver *
receiver_open(struct ratbagd_receiver *receiver, int *fd_out);

int
ratbagd_receiver_new(struct ratbagd_receiver **out,
		     struct ratbagd *ctx,
		     const char *sysname)
{
	_cleanup_(ratbagd_receiver_unrefp) struct ratbagd_receiver *receiver = NULL;
	int r;

	assert(out);
	assert(ctx);
	assert(sysname);

	receiver = zalloc(sizeof(*receiver));
	receiver->refcount = 1;
	receiver->ctx = ctx;
	receiver->fd = -1;
	rbnode_init(&receiver->node);
	receiver->sysname = strdup_safe(sysname);

	r = sd_bus_path_encode(RATBAGD_OBJ_ROOT "/receiver",
			       receiver->sysname,
			       &receiver->path);
	if (r < 0)
		return r;

	/* Only one hidraw node per receiver USB device answers HID++, and the
	 * node of a paired device shares the same vendor/product. Reading the
	 * receiver's firmware is the cheapest honest probe: a dead interface
	 * returns -EPIPE. */
	{
		struct lur_receiver *lur;
		uint8_t major = 0, minor = 0, build = 0;
		int fd = -1;

		lur = receiver_open(receiver, &fd);
		if (!lur)
			return -ENODEV;

		r = lur_receiver_get_firmware_version(lur, &major, &minor, &build);
		lur_receiver_unref(lur);
		close(fd);
		if (r < 0)
			return r;

		receiver->fw_major = major;
		receiver->fw_minor = minor;
		receiver->fw_build = build;
	}

	*out = receiver;
	receiver = NULL;
	return 0;
}

static void
ratbagd_receiver_free(struct ratbagd_receiver *receiver)
{
	assert(receiver);
	assert(!ratbagd_receiver_linked(receiver));

	free(receiver->path);
	free(receiver->sysname);
	free(receiver);
}

struct ratbagd_receiver *
ratbagd_receiver_ref(struct ratbagd_receiver *receiver)
{
	assert(receiver);
	assert(receiver->refcount > 0);

	++receiver->refcount;
	return receiver;
}

struct ratbagd_receiver *
ratbagd_receiver_unref(struct ratbagd_receiver *receiver)
{
	assert(receiver);

	if (--receiver->refcount > 0)
		return NULL;

	ratbagd_receiver_free(receiver);
	return NULL;
}

const char *
ratbagd_receiver_get_sysname(struct ratbagd_receiver *receiver)
{
	assert(receiver);
	return receiver->sysname;
}

const char *
ratbagd_receiver_get_path(struct ratbagd_receiver *receiver)
{
	assert(receiver);
	return receiver->path;
}

static const char *
device_type_name(enum lur_device_type type)
{
	switch (type) {
	case LUR_DEVICE_TYPE_KEYBOARD:
		return "keyboard";
	case LUR_DEVICE_TYPE_MOUSE:
		return "mouse";
	case LUR_DEVICE_TYPE_NUMPAD:
		return "numpad";
	case LUR_DEVICE_TYPE_PRESENTER:
		return "presenter";
	case LUR_DEVICE_TYPE_TRACKBALL:
		return "trackball";
	case LUR_DEVICE_TYPE_TOUCHPAD:
		return "touchpad";
	case LUR_DEVICE_TYPE_UNKNOWN:
		break;
	}

	return "unknown";
}

/*
 * Open the receiver's hidraw node and wrap it in a lur_receiver. Returns NULL
 * if this node is not a receiver or does not answer HID++ (a receiver USB
 * device usually exposes several interfaces and only one of them speaks it).
 */
static struct lur_receiver *
receiver_open(struct ratbagd_receiver *receiver, int *fd_out)
{
	struct lur_receiver *lur = NULL;
	struct lur_device **devices = NULL;
	char path[PATH_MAX];
	int fd, r;

	snprintf(path, sizeof(path), "/dev/%s", receiver->sysname);

	fd = open(path, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		log_error("%s: cannot open receiver: %m\n", receiver->sysname);
		return NULL;
	}

	r = lur_receiver_new_from_hidraw(fd, NULL, &lur);
	if (r < 0) {
		close(fd);
		return NULL;
	}

	/* Creating the handle does not prove the interface speaks HID++, so
	 * probe it. A receiver with no paired devices enumerates fine and
	 * returns zero devices; a dead interface fails. */
	r = lur_receiver_enumerate(lur, &devices);
	free(devices);
	if (r < 0) {
		lur_receiver_unref(lur);
		close(fd);
		return NULL;
	}

	*fd_out = fd;
	return lur;
}

static void
receiver_close(struct ratbagd_receiver *receiver)
{
	receiver->lur = lur_receiver_unref(receiver->lur);
	if (receiver->fd >= 0) {
		close(receiver->fd);
		receiver->fd = -1;
	}
}

int
ratbagd_receiver_get_devices(struct ratbagd_receiver *receiver,
			     struct ratbagd_receiver_device **devices,
			     size_t *ndevices)
{
	struct lur_device **lurdevices = NULL;
	struct lur_receiver *lur;
	struct ratbagd_receiver_device *out;
	unsigned int i, n = 0;
	int fd = -1;
	int r;

	assert(receiver);

	*devices = NULL;
	*ndevices = 0;

	lur = receiver_open(receiver, &fd);
	if (!lur)
		return -ENODEV;

	r = lur_receiver_enumerate(lur, &lurdevices);
	if (r > 0) {
		out = zalloc(r * sizeof(*out));
		for (i = 0; i < (unsigned int)r; i++) {
			struct lur_device *dev = lurdevices[i];

			out[n].index = i;
			out[n].name = strdup_safe(lur_device_get_name(dev));
			out[n].type = device_type_name(lur_device_get_type(dev));
			out[n].serial = lur_device_get_serial(dev);
			n++;
		}
		*devices = out;
		*ndevices = n;
	}
	free(lurdevices);

	lur_receiver_unref(lur);
	close(fd);

	return 0;
}

void
ratbagd_receiver_devices_free(struct ratbagd_receiver_device *devices,
			      size_t ndevices)
{
	size_t i;

	if (!devices)
		return;

	for (i = 0; i < ndevices; i++)
		free(devices[i].name);
	free(devices);
}

static int
pairing_timeout_cb(sd_event_source *source, uint64_t usec, void *userdata)
{
	struct ratbagd_receiver *receiver = userdata;

	receiver->pairing_timer = sd_event_source_unref(receiver->pairing_timer);
	/* The firmware window has expired by now; release the handle. */
	receiver_close(receiver);
	receiver->pairing_open = false;

	(void)sd_bus_emit_properties_changed(receiver->ctx->bus,
					     receiver->path,
					     RATBAGD_NAME_ROOT ".Receiver",
					     "PairingOpen", NULL);
	return 0;
}

int
ratbagd_receiver_open_pairing(struct ratbagd_receiver *receiver,
			      unsigned int timeout)
{
	struct lur_receiver *lur;
	int fd = -1;
	int r;

	assert(receiver);

	if (receiver->pairing_open)
		return -EBUSY;
	if (timeout == 0 || timeout > 255)
		return -EINVAL;

	lur = receiver_open(receiver, &fd);
	if (!lur)
		return -ENODEV;

	r = lur_receiver_open(lur, timeout);
	if (r < 0) {
		lur_receiver_unref(lur);
		close(fd);
		log_error("%s: cannot open pairing window: %s\n",
			  receiver->sysname, strerror(-r));
		return r;
	}

	receiver->lur = lur;
	receiver->fd = fd;
	receiver->pairing_open = true;

	/* Keep the handle for the whole window, then drop it. One extra second
	 * so the firmware timer always expires before we let go. */
	{
		uint64_t now;

		sd_event_now(receiver->ctx->event, CLOCK_MONOTONIC, &now);
		sd_event_add_time(receiver->ctx->event, &receiver->pairing_timer,
				  CLOCK_MONOTONIC,
				  now + (timeout + 1) * USEC_PER_SEC,
				  0, pairing_timeout_cb, receiver);
	}

	(void)sd_bus_emit_properties_changed(receiver->ctx->bus,
					     receiver->path,
					     RATBAGD_NAME_ROOT ".Receiver",
					     "PairingOpen", NULL);
	log_info("%s: pairing window open for %u seconds\n",
		 receiver->sysname, timeout);
	return 0;
}

int
ratbagd_receiver_close_pairing(struct ratbagd_receiver *receiver)
{
	assert(receiver);

	if (!receiver->pairing_open)
		return -EALREADY;

	(void)lur_receiver_close(receiver->lur);
	receiver->pairing_timer = sd_event_source_unref(receiver->pairing_timer);
	receiver_close(receiver);
	receiver->pairing_open = false;

	(void)sd_bus_emit_properties_changed(receiver->ctx->bus,
					     receiver->path,
					     RATBAGD_NAME_ROOT ".Receiver",
					     "PairingOpen", NULL);
	return 0;
}

bool
ratbagd_receiver_is_pairing(struct ratbagd_receiver *receiver)
{
	assert(receiver);
	return receiver->pairing_open;
}

/*
 * D-Bus surface
 */

static int
ratbagd_receiver_get_pairing_open(sd_bus *bus,
				  const char *path,
				  const char *interface,
				  const char *property,
				  sd_bus_message *reply,
				  void *userdata,
				  sd_bus_error *error)
{
	struct ratbagd_receiver *receiver = userdata;

	return sd_bus_message_append(reply, "b", receiver->pairing_open);
}

static int
ratbagd_receiver_get_sysname_prop(sd_bus *bus,
				  const char *path,
				  const char *interface,
				  const char *property,
				  sd_bus_message *reply,
				  void *userdata,
				  sd_bus_error *error)
{
	struct ratbagd_receiver *receiver = userdata;

	return sd_bus_message_append(reply, "s", receiver->sysname);
}

static int
ratbagd_receiver_get_firmware(sd_bus *bus,
			      const char *path,
			      const char *interface,
			      const char *property,
			      sd_bus_message *reply,
			      void *userdata,
			      sd_bus_error *error)
{
	struct ratbagd_receiver *receiver = userdata;
	_cleanup_(freep) char *version = NULL;

	if (asprintf(&version, "%u.%u.B%04u", receiver->fw_major,
		     receiver->fw_minor, receiver->fw_build) < 0)
		return -ENOMEM;

	return sd_bus_message_append(reply, "s", version);
}

static int
ratbagd_receiver_method_get_devices(sd_bus_message *m,
				    void *userdata,
				    sd_bus_error *error)
{
	struct ratbagd_receiver *receiver = userdata;
	struct ratbagd_receiver_device *devices = NULL;
	size_t ndevices = 0, i;
	sd_bus_message *reply = NULL;
	int r;

	r = ratbagd_receiver_get_devices(receiver, &devices, &ndevices);
	if (r < 0)
		return sd_bus_error_setf(error, SD_BUS_ERROR_FAILED,
					 "Cannot read devices from this receiver");

	r = sd_bus_message_new_method_return(m, &reply);
	if (r < 0)
		goto out;

	r = sd_bus_message_open_container(reply, 'a', "(ussu)");
	if (r < 0)
		goto out;

	for (i = 0; i < ndevices; i++) {
		r = sd_bus_message_append(reply, "(ussu)",
					  devices[i].index,
					  devices[i].name,
					  devices[i].type,
					  devices[i].serial);
		if (r < 0)
			goto out;
	}

	r = sd_bus_message_close_container(reply);
	if (r < 0)
		goto out;

	r = sd_bus_send(NULL, reply, NULL);

out:
	ratbagd_receiver_devices_free(devices, ndevices);
	sd_bus_message_unref(reply);
	return r;
}

static int
ratbagd_receiver_method_open_pairing(sd_bus_message *m,
				     void *userdata,
				     sd_bus_error *error)
{
	struct ratbagd_receiver *receiver = userdata;
	unsigned int timeout = 30;
	int r;

	r = sd_bus_message_read(m, "u", &timeout);
	if (r < 0)
		return r;

	r = ratbagd_receiver_open_pairing(receiver, timeout);
	if (r == -EBUSY)
		return sd_bus_error_setf(error, SD_BUS_ERROR_FAILED,
					 "A pairing window is already open");
	if (r == -EINVAL)
		return sd_bus_error_setf(error, SD_BUS_ERROR_INVALID_ARGS,
					 "Timeout must be between 1 and 255 seconds");
	if (r < 0)
		return sd_bus_error_setf(error, SD_BUS_ERROR_FAILED,
					 "Cannot open a pairing window");

	return sd_bus_reply_method_return(m, NULL);
}

static int
ratbagd_receiver_method_close_pairing(sd_bus_message *m,
				      void *userdata,
				      sd_bus_error *error)
{
	struct ratbagd_receiver *receiver = userdata;
	int r;

	r = ratbagd_receiver_close_pairing(receiver);
	if (r == -EALREADY)
		return sd_bus_error_setf(error, SD_BUS_ERROR_FAILED,
					 "No pairing window is open");

	return sd_bus_reply_method_return(m, NULL);
}

const sd_bus_vtable ratbagd_receiver_vtable[] = {
	SD_BUS_VTABLE_START(0),
	SD_BUS_PROPERTY("Sysname", "s", ratbagd_receiver_get_sysname_prop, 0,
			SD_BUS_VTABLE_PROPERTY_CONST),
	SD_BUS_PROPERTY("FirmwareVersion", "s", ratbagd_receiver_get_firmware, 0,
			SD_BUS_VTABLE_PROPERTY_CONST),
	SD_BUS_PROPERTY("PairingOpen", "b", ratbagd_receiver_get_pairing_open, 0,
			SD_BUS_VTABLE_PROPERTY_EMITS_CHANGE),
	SD_BUS_METHOD("GetDevices", "", "a(ussu)",
		      ratbagd_receiver_method_get_devices,
		      SD_BUS_VTABLE_UNPRIVILEGED),
	SD_BUS_METHOD("OpenPairingWindow", "u", "",
		      ratbagd_receiver_method_open_pairing,
		      SD_BUS_VTABLE_UNPRIVILEGED),
	SD_BUS_METHOD("ClosePairingWindow", "", "",
		      ratbagd_receiver_method_close_pairing,
		      SD_BUS_VTABLE_UNPRIVILEGED),
	SD_BUS_VTABLE_END,
};

/*
 * Link management, mirroring ratbagd_device.
 */

bool
ratbagd_receiver_linked(struct ratbagd_receiver *receiver)
{
	return receiver && rbnode_linked(&receiver->node);
}

void
ratbagd_receiver_link(struct ratbagd_receiver *receiver)
{
	struct ratbagd_receiver *iter;
	RBNode **node, *parent;
	int r, v;

	assert(receiver);
	assert(!ratbagd_receiver_linked(receiver));

	parent = NULL;
	node = &receiver->ctx->receiver_map.root;
	while (*node) {
		parent = *node;
		iter = ratbagd_receiver_from_node(parent);
		v = strcmp(receiver->sysname, iter->sysname);

		assert(v != 0);

		if (v < 0)
			node = &parent->left;
		else
			node = &parent->right;
	}

	rbtree_add(&receiver->ctx->receiver_map, parent, node, &receiver->node);
	++receiver->ctx->n_receivers;

	r = sd_bus_add_object_vtable(receiver->ctx->bus,
				     &receiver->vtable_slot,
				     receiver->path,
				     RATBAGD_NAME_ROOT ".Receiver",
				     ratbagd_receiver_vtable,
				     receiver);
	if (r < 0) {
		errno = -r;
		log_error("%s: failed to register receiver interface: %m\n",
			  receiver->sysname);
	}
}

void
ratbagd_receiver_unlink(struct ratbagd_receiver *receiver)
{
	if (!ratbagd_receiver_linked(receiver))
		return;

	ratbagd_receiver_close_pairing(receiver);
	receiver->vtable_slot = sd_bus_slot_unref(receiver->vtable_slot);

	--receiver->ctx->n_receivers;
	rbtree_remove(&receiver->ctx->receiver_map, &receiver->node);
	rbnode_init(&receiver->node);
}

struct ratbagd_receiver *
ratbagd_receiver_lookup(struct ratbagd *ctx, const char *name)
{
	struct ratbagd_receiver *receiver;
	RBNode *node;
	int v;

	assert(ctx);
	assert(name);

	node = ctx->receiver_map.root;
	while (node) {
		receiver = ratbagd_receiver_from_node(node);
		v = strcmp(name, receiver->sysname);
		if (!v)
			return receiver;
		else if (v < 0)
			node = node->left;
		else
			node = node->right;
	}

	return NULL;
}

struct ratbagd_receiver *
ratbagd_receiver_first(struct ratbagd *ctx)
{
	RBNode *node;

	assert(ctx);

	node = rbtree_first(&ctx->receiver_map);
	return node ? ratbagd_receiver_from_node(node) : NULL;
}

struct ratbagd_receiver *
ratbagd_receiver_next(struct ratbagd_receiver *receiver)
{
	RBNode *node;

	assert(receiver);

	node = rbnode_next(&receiver->node);
	return node ? ratbagd_receiver_from_node(node) : NULL;
}
