// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2015-2017 Google, Inc
 *
 * USB Power Delivery protocol stack.
 */

#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/proc_fs.h>
#include <linux/property.h>
#include <linux/sched/clock.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/usb.h>
#include <linux/usb/pd.h>
#include <linux/usb/pd_ado.h>
#include <linux/usb/pd_bdo.h>
#include <linux/usb/pd_ext_sdb.h>
#include <linux/usb/pd_vdo.h>
#include <linux/usb/role.h>
#include <linux/usb/tcpm.h>
#include <linux/usb/typec_altmode.h>
#include <linux/workqueue.h>

#include <../../../power/supply/google/logbuffer.h>

#define FOREACH_CHUNK_STATE(S)			\
	S(INVALID_CHUNK_STATE),			\
	S(RCH_WAIT_FOR_MESSAGE),		\
	S(RCH_PASS_UP_MESSAGE),			\
	S(RCH_PROCESSING_EXTENDED_MESSAGE),	\
	S(RCH_REQUESTING_CHUNK),		\
	S(RCH_WAITING_CHUNK),			\
	S(RCH_REPORT_ERROR),			\
	S(TCH_WAIT_FOR_MESSAGE_REQUEST),	\
	S(TCH_PASS_DOWN_MESSAGE),		\
	S(TCH_WAIT_FOR_TRANSMISSION_COMPLETE),	\
	S(TCH_MESSAGE_SENT),			\
	S(TCH_PREPARE_TO_SEND_CHUNKED_MESSAGE),	\
	S(TCH_CONSTRUCT_CHUNKED_MESSAGE),	\
	S(TCH_SENDING_CHUNKED_MESSAGE),		\
	S(TCH_WAIT_CHUNK_REQUEST),		\
	S(TCH_MESSAGE_RECEIVED),		\
	S(TCH_REPORT_ERROR)

#define FOREACH_STATE(S)			\
	S(INVALID_STATE),			\
	S(TOGGLING),			\
	S(SRC_UNATTACHED),			\
	S(SRC_ATTACH_WAIT),			\
	S(SRC_ATTACHED),			\
	S(SRC_STARTUP),				\
	S(SRC_SEND_CAPABILITIES),		\
	S(SRC_SEND_CAPABILITIES_TIMEOUT),	\
	S(SRC_NEGOTIATE_CAPABILITIES),		\
	S(SRC_TRANSITION_SUPPLY),		\
	S(SRC_READY),				\
	S(SRC_VPD_READY),			\
	S(SRC_WAIT_NEW_CAPABILITIES),		\
						\
	S(SNK_UNATTACHED),			\
	S(SNK_ATTACH_WAIT),			\
	S(SNK_DEBOUNCED),			\
	S(SNK_ATTACHED),			\
	S(SNK_STARTUP),				\
	S(SNK_DISCOVERY),			\
	S(SNK_DISCOVERY_DEBOUNCE),		\
	S(SNK_DISCOVERY_DEBOUNCE_DONE),		\
	S(SNK_WAIT_CAPABILITIES),		\
	S(SNK_NEGOTIATE_CAPABILITIES),		\
	S(SNK_NEGOTIATE_PPS_CAPABILITIES),	\
	S(SNK_TRANSITION_SINK),			\
	S(SNK_TRANSITION_SINK_VBUS),		\
	S(SNK_READY),				\
						\
	S(ACC_UNATTACHED),			\
	S(DEBUG_ACC_ATTACHED),			\
	S(AUDIO_ACC_ATTACHED),			\
	S(AUDIO_ACC_DEBOUNCE),			\
						\
	S(HARD_RESET_SEND),			\
	S(HARD_RESET_START),			\
	S(SRC_HARD_RESET_VBUS_OFF),		\
	S(SRC_HARD_RESET_VBUS_ON),		\
	S(SNK_HARD_RESET_SINK_OFF),		\
	S(SNK_HARD_RESET_WAIT_VBUS),		\
	S(SNK_HARD_RESET_SINK_ON),		\
						\
	S(SOFT_RESET),				\
	S(SRC_SOFT_RESET_WAIT_SNK_TX),		\
	S(SNK_SOFT_RESET),			\
	S(SOFT_RESET_SEND),			\
						\
	S(DR_SWAP_ACCEPT),			\
	S(DR_SWAP_SEND),			\
	S(DR_SWAP_SEND_TIMEOUT),		\
	S(DR_SWAP_CANCEL),			\
	S(DR_SWAP_CHANGE_DR),			\
						\
	S(PR_SWAP_ACCEPT),			\
	S(PR_SWAP_SEND),			\
	S(PR_SWAP_SEND_TIMEOUT),		\
	S(PR_SWAP_CANCEL),			\
	S(PR_SWAP_START),			\
	S(PR_SWAP_SRC_SNK_TRANSITION_OFF),	\
	S(PR_SWAP_SRC_SNK_SOURCE_OFF),		\
	S(PR_SWAP_SRC_SNK_SOURCE_OFF_CC_DEBOUNCED), \
	S(PR_SWAP_SRC_SNK_SINK_ON),		\
	S(PR_SWAP_SNK_SRC_SINK_OFF),		\
	S(PR_SWAP_SNK_SRC_SOURCE_ON),		\
	S(PR_SWAP_SNK_SRC_SOURCE_ON_VBUS_RAMPED_UP),    \
						\
	S(VCONN_SWAP_ACCEPT),			\
	S(VCONN_SWAP_SEND),			\
	S(VCONN_SWAP_SEND_TIMEOUT),		\
	S(VCONN_SWAP_CANCEL),			\
	S(VCONN_SWAP_START),			\
	S(VCONN_SWAP_WAIT_FOR_VCONN),		\
	S(VCONN_SWAP_TURN_ON_VCONN),		\
	S(VCONN_SWAP_TURN_OFF_VCONN),		\
						\
	S(SNK_TRY),				\
	S(SNK_TRY_WAIT),			\
	S(SNK_TRY_WAIT_DEBOUNCE),               \
	S(SNK_TRY_WAIT_DEBOUNCE_CHECK_VBUS),    \
	S(SRC_TRYWAIT),				\
	S(SRC_TRYWAIT_DEBOUNCE),		\
	S(SRC_TRYWAIT_UNATTACHED),		\
						\
	S(SRC_TRY),				\
	S(SRC_TRY_WAIT),                        \
	S(SRC_TRY_DEBOUNCE),			\
	S(SNK_TRYWAIT),				\
	S(SNK_TRYWAIT_DEBOUNCE),		\
	S(SNK_TRYWAIT_VBUS),			\
	S(BIST_RX),				\
						\
	S(GET_STATUS_SEND),			\
	S(GET_STATUS_SEND_TIMEOUT),		\
	S(GET_PPS_STATUS_SEND),			\
	S(GET_PPS_STATUS_SEND_TIMEOUT),		\
						\
	S(ERROR_RECOVERY),			\
	S(PORT_RESET),				\
	S(PORT_RESET_WAIT_OFF),			\
						\
	S(AMS_START),				\
	S(CHUNK_NOT_SUPP),			\
	S(CHUNK_RX),				\
	S(CHUNK_RX_FINISH),			\
	S(CHUNK_TX),				\
	S(CHUNK_TX_FINISH)

#define FOREACH_AMS(S)				\
	S(NONE_AMS),				\
	S(POWER_NEGOTIATION),			\
	S(GOTOMIN),				\
	S(SOFT_RESET_AMS),			\
	S(HARD_RESET),				\
	S(CABLE_RESET),				\
	S(GET_SOURCE_CAPABILITIES),		\
	S(GET_SINK_CAPABILITIES),		\
	S(POWER_ROLE_SWAP),			\
	S(FAST_ROLE_SWAP),			\
	S(DATA_ROLE_SWAP),			\
	S(VCONN_SWAP),				\
	S(SOURCE_ALERT),			\
	S(GETTING_SOURCE_EXTENDED_CAPABILITIES),\
	S(GETTING_SOURCE_SINK_STATUS),		\
	S(GETTING_BATTERY_CAPABILITIES),	\
	S(GETTING_BATTERY_STATUS),		\
	S(GETTING_MANUFACTURER_INFORMATION),	\
	S(SECURITY),				\
	S(FIRMWARE_UPDATE),			\
	S(DISCOVER_IDENTITY),			\
	S(SOURCE_STARTUP_CABLE_PLUG_DISCOVER_IDENTITY),	\
	S(DISCOVER_SVIDS),			\
	S(DISCOVER_MODES),			\
	S(DFP_TO_UFP_ENTER_MODE),		\
	S(DFP_TO_UFP_EXIT_MODE),		\
	S(DFP_TO_CABLE_PLUG_ENTER_MODE),	\
	S(DFP_TO_CABLE_PLUG_EXIT_MODE),		\
	S(ATTENTION),				\
	S(BIST),				\
	S(UNSTRUCTURED_VDMS),			\
	S(STRUCTURED_VDMS),			\
	S(COUNTRY_INFO),			\
	S(COUNTRY_CODES)

#define GENERATE_ENUM(e)	e
#define GENERATE_STRING(s)	#s

enum tcpm_state {
	FOREACH_STATE(GENERATE_ENUM)
};

static const char * const tcpm_states[] = {
	FOREACH_STATE(GENERATE_STRING)
};

enum tcpm_ams {
	FOREACH_AMS(GENERATE_ENUM)
};

static const char * const tcpm_ams_str[] = {
	FOREACH_AMS(GENERATE_STRING)
};

enum tcpm_chunk_state {
	FOREACH_CHUNK_STATE(GENERATE_ENUM)
};

static const char * const tcpm_chunk_state[] = {
	FOREACH_CHUNK_STATE(GENERATE_STRING)
};

enum vdm_states {
	VDM_STATE_ERR_BUSY = -3,
	VDM_STATE_ERR_SEND = -2,
	VDM_STATE_ERR_TMOUT = -1,
	VDM_STATE_DONE = 0,
	/* Anything >0 represents an active state */
	VDM_STATE_READY = 1,
	VDM_STATE_BUSY = 2,
	VDM_STATE_WAIT_RSP_BUSY = 3,
	VDM_STATE_SEND_MESSAGE = 4,
};

enum pd_msg_request {
	PD_MSG_NONE = 0,
	PD_MSG_CTRL_REJECT,
	PD_MSG_CTRL_WAIT,
	PD_MSG_CTRL_NOT_SUPP,
	PD_MSG_DATA_SINK_CAP,
	PD_MSG_DATA_SOURCE_CAP,
};

/* Events from low level driver */

#define TCPM_CC_EVENT		BIT(0)
#define TCPM_VBUS_EVENT		BIT(1)
#define TCPM_RESET_EVENT	BIT(2)
#define TCPM_PORT_RESET_EVENT   BIT(3)

#define LOG_BUFFER_ENTRIES	1024
#define LOG_BUFFER_ENTRY_SIZE	128

/* Alternate mode support */

#define SVID_DISCOVERY_MAX	16
#define ALTMODE_DISCOVERY_MAX	(SVID_DISCOVERY_MAX * MODE_DISCOVERY_MAX)

#define DISC_ID_RETRY_MS	3000

struct pd_mode_data {
	int svid_index;		/* current SVID index		*/
	int nsvids;
	u16 svids[SVID_DISCOVERY_MAX];
	int altmodes;		/* number of alternate modes	*/
	struct typec_altmode_desc altmode_desc[ALTMODE_DISCOVERY_MAX];
};

/*
 * @min_volt: Actual min voltage at the local port
 * @req_min_volt: Requested min voltage to the port partner
 * @max_volt: Actual max voltage at the local port
 * @req_max_volt: Requested max voltage to the port partner
 * @max_curr: Actual max current at the local port
 * @req_max_curr: Requested max current of the port partner
 * @req_out_volt: Requested output voltage to the port partner
 * @req_op_curr: Requested operating current to the port partner
 * @supported: Parter has atleast one APDO hence supports PPS
 * @active: PPS mode is active
 */
struct pd_pps_data {
	u32 min_volt;
	u32 req_min_volt;
	u32 max_volt;
	u32 req_max_volt;
	u32 max_curr;
	u32 req_max_curr;
	u32 req_out_volt;
	u32 req_op_curr;
	bool supported;
	bool active;
};

struct pd_chunk_event {
	struct pd_message msg;
	enum pd_ext_msg_type expected_type;
	u8 chunk_num_expected;
	u16 num_bytes;
};

struct tcpm_port {
	struct device *dev;

	struct mutex lock;		/* tcpm state machine lock */
	struct workqueue_struct *wq;

	struct typec_capability typec_caps;
	struct typec_port *typec_port;

	struct tcpc_dev	*tcpc;
	struct usb_role_switch *role_sw;

	enum typec_role vconn_role;
	enum typec_role pwr_role;
	enum typec_data_role data_role;
	enum typec_pwr_opmode pwr_opmode;

	struct usb_pd_identity partner_ident;
	struct typec_partner_desc partner_desc;
	struct typec_partner *partner;

	enum tcpm_transmit_type vdo_receiver;

	enum typec_cc_status cc_req;

	enum typec_cc_status cc1;
	enum typec_cc_status cc2;
	enum typec_cc_polarity polarity;

	bool attached;
	bool connected;
	enum typec_port_type port_type;

	/*
	 * Set to true when vbus is greater than VSAFE5V min.
	 * Set to false when vbus falls below vSinkDisconnect max threshold.
	 */
	bool vbus_present;

	/*
	 * Set to true when vbus is less than VSAFE0V max.
	 * Set to false when vbus is greater than VSAFE0V max.
	 */
	bool vbus_vsafe0v;

	bool vbus_never_low;
	bool vbus_source;
	bool vbus_charge;

	bool send_discover;
	bool op_vsafe5v;

	int try_role;
	int try_snk_count;
	int try_src_count;

	enum pd_msg_request queued_message;

	enum tcpm_state enter_state;
	enum tcpm_state prev_state;
	enum tcpm_state state;
	enum tcpm_state delayed_state;
	unsigned long delayed_runtime;
	unsigned long delay_ms;

	spinlock_t pd_event_lock;
	u32 pd_events;

	struct work_struct event_work;
	struct delayed_work state_machine;
	struct delayed_work vdm_state_machine;
	bool state_machine_running;
	bool vdm_sm_running;
	struct delayed_work disc_id_work;

	struct completion tx_complete;
	enum tcpm_transmit_status tx_status;

	struct mutex swap_lock;		/* swap command lock */
	bool swap_pending;
	bool non_pd_role_swap;
	struct completion swap_complete;
	int swap_status;

	unsigned int negotiated_rev;
	unsigned int message_id;
	unsigned int caps_count;
	unsigned int hard_reset_count;
	bool pd_capable;
	bool explicit_contract;
	bool usb_comm_capable;
	unsigned int rx_msgid;

	/* Partner capabilities/requests */
	u32 sink_request;
	u32 source_caps[PDO_MAX_OBJECTS];
	unsigned int nr_source_caps;
	u32 sink_caps[PDO_MAX_OBJECTS];
	unsigned int nr_sink_caps;

	/* Local capabilities */
	u32 src_pdo[PDO_MAX_OBJECTS];
	unsigned int nr_src_pdo;
	u32 snk_pdo[PDO_MAX_OBJECTS];
	unsigned int nr_snk_pdo;
	u32 snk_vdo[VDO_MAX_OBJECTS];
	unsigned int nr_snk_vdo;

	unsigned int operating_snk_mw;
	bool update_sink_caps;

	/* Set current / voltage */
	u32 current_limit;
	u32 supply_voltage;
	/* current / voltage requested to partner */
	u32 req_current_limit;
	u32 req_supply_voltage;

	/* Used to export TA voltage and current */
	struct power_supply *psy;
	struct power_supply_desc psy_desc;
	enum power_supply_usb_type usb_type;

	u32 bist_request;

	/* PD state for Vendor Defined Messages */
	enum vdm_states vdm_state;
	u32 vdm_retries;
	/* next Vendor Defined Message to send */
	u32 vdo_data[VDO_MAX_SIZE];
	u8 vdo_count;
	/* VDO to retry if UFP responder replied busy */
	u32 vdo_retry;
	unsigned int svdm_version;

	/* PPS */
	struct pd_pps_data pps_data;
	struct completion pps_complete;
	bool pps_pending;
	int pps_status;

	/* Alternate mode data */
	struct pd_mode_data mode_data;
	struct typec_altmode *partner_altmode[ALTMODE_DISCOVERY_MAX];
	struct typec_altmode *port_altmode[ALTMODE_DISCOVERY_MAX];
	/* Deadline in jiffies to exit src_try_wait state */
	unsigned long max_wait;

	/* port belongs to a self powered device */
	bool self_powered;

	/* Collision Avoidance and Atomic Message Sequence */
	enum tcpm_state upcoming_state;
	enum tcpm_ams ams;
	enum tcpm_ams next_ams;
	bool in_ams;

	bool vpd_connected;

	/*
	 * Honour psnkstdby after accept is received.
	 * However, in this case it has to be made sure that the tSnkStdby (15
	 * msec) is met.
	 *
	 * When not set, port requests PD_P_SNK_STDBY_5V upon entering SNK_DISCOVERY and
	 * the actual currrent limit after RX of PD_CTRL_PSRDY for PD link,
	 * SNK_READY for non-pd link.
	 *
	 * When set, port requests CC advertisement based current limit during
	 * SNK_DISCOVERY, current gets limited to PD_P_SNK_STDBY during SNK_TRANSITION_SINK,
	 * PD based current limit gets set after RX of PD_CTRL_PSRDY.
	 */
	bool psnkstdby_after_accept;

#ifdef CONFIG_DEBUG_FS
	struct dentry *dentry;
	struct mutex logbuffer_lock;	/* log buffer access lock */
	int logbuffer_head;
	int logbuffer_tail;
	u8 *logbuffer[LOG_BUFFER_ENTRIES];
#endif
	struct logbuffer *log;

	/* Chunk */
	struct mutex chunk_lock;	/* chunk lock */
	struct pd_chunk_event *chunk_event;
	struct delayed_work chunk_state_machine;
	enum tcpm_chunk_state chunk_tx_state;
	enum tcpm_chunk_state chunk_rx_state;
	enum tcpm_chunk_state delayed_chunk_state;
	enum tcpm_transmit_type tx_type;
	enum pd_ext_msg_type ext_type;
	unsigned long chunk_delay_ms;
	bool unchunk_supp;
	bool complete_msg;
	bool is_chunk_tx;
	bool chunk_abort;
	bool abort_supported;
	bool chunking;
	u8 *ext_rx_buf;		/* store chunk rx data block */
	u16 ext_rx_buf_size;
	u8 *ext_tx_buf;
	u16 ext_tx_buf_size;
	u8 chunk_num_to_send;
};

struct pd_rx_event {
	struct work_struct work;
	struct tcpm_port *port;
	struct pd_message msg;
	enum usb_pd_sop_type packet;
};

static const char * const pd_rev[] = {
	[PD_REV10]		= "rev1",
	[PD_REV20]		= "rev2",
	[PD_REV30]		= "rev3",
};

#define tcpm_cc_is_sink(cc) \
	((cc) == TYPEC_CC_RP_DEF || (cc) == TYPEC_CC_RP_1_5 || \
	 (cc) == TYPEC_CC_RP_3_0)

#define tcpm_port_is_sink(port) \
	((tcpm_cc_is_sink((port)->cc1) && !tcpm_cc_is_sink((port)->cc2)) || \
	 (tcpm_cc_is_sink((port)->cc2) && !tcpm_cc_is_sink((port)->cc1)))

#define tcpm_cc_is_source(cc) ((cc) == TYPEC_CC_RD)
#define tcpm_cc_is_audio(cc) ((cc) == TYPEC_CC_RA)
#define tcpm_cc_is_open(cc) ((cc) == TYPEC_CC_OPEN)

#define tcpm_port_is_source(port) \
	((tcpm_cc_is_source((port)->cc1) && \
	 !tcpm_cc_is_source((port)->cc2)) || \
	 (tcpm_cc_is_source((port)->cc2) && \
	  !tcpm_cc_is_source((port)->cc1)))

#define tcpm_port_is_debug(port) \
	(tcpm_cc_is_source((port)->cc1) && tcpm_cc_is_source((port)->cc2))

#define tcpm_port_is_audio(port) \
	(tcpm_cc_is_audio((port)->cc1) && tcpm_cc_is_audio((port)->cc2))

#define tcpm_port_is_audio_detached(port) \
	((tcpm_cc_is_audio((port)->cc1) && tcpm_cc_is_open((port)->cc2)) || \
	 (tcpm_cc_is_audio((port)->cc2) && tcpm_cc_is_open((port)->cc1)))

#define tcpm_try_snk(port) \
	((port)->try_snk_count == 0 && (port)->try_role == TYPEC_SINK && \
	(port)->port_type == TYPEC_PORT_DRP)

#define tcpm_try_src(port) \
	((port)->try_src_count == 0 && (port)->try_role == TYPEC_SOURCE && \
	(port)->port_type == TYPEC_PORT_DRP)

#define tcpm_sink_tx_ok(port) \
	(tcpm_port_is_sink(port) && \
	((port)->cc1 == TYPEC_CC_RP_3_0 || (port)->cc2 == TYPEC_CC_RP_3_0))

static enum tcpm_state tcpm_default_state(struct tcpm_port *port)
{
	if (port->port_type == TYPEC_PORT_DRP) {
		if (port->try_role == TYPEC_SINK)
			return SNK_UNATTACHED;
		else if (port->try_role == TYPEC_SOURCE)
			return SRC_UNATTACHED;
		else if (port->tcpc->config &&
			 port->tcpc->config->default_role == TYPEC_SINK)
			return SNK_UNATTACHED;
		/* Fall through to return SRC_UNATTACHED */
	} else if (port->port_type == TYPEC_PORT_SNK) {
		return SNK_UNATTACHED;
	}
	return SRC_UNATTACHED;
}

static inline
struct tcpm_port *typec_cap_to_tcpm(const struct typec_capability *cap)
{
	return container_of(cap, struct tcpm_port, typec_caps);
}

static bool tcpm_port_is_disconnected(struct tcpm_port *port)
{
	return (!port->attached && port->cc1 == TYPEC_CC_OPEN &&
		port->cc2 == TYPEC_CC_OPEN) ||
	       (port->attached && ((port->polarity == TYPEC_POLARITY_CC1 &&
				    port->cc1 == TYPEC_CC_OPEN) ||
				   (port->polarity == TYPEC_POLARITY_CC2 &&
				    port->cc2 == TYPEC_CC_OPEN)));
}

/*
 * Logging
 */

#ifdef CONFIG_DEBUG_FS

__printf(2, 0)
static void _tcpm_log(struct tcpm_port *port, const char *fmt, va_list args)
{
	char tmpbuffer[LOG_BUFFER_ENTRY_SIZE];
	u64 ts_nsec = local_clock();
	unsigned long rem_nsec;

	mutex_lock(&port->logbuffer_lock);
	if (!port->logbuffer[port->logbuffer_head]) {
		port->logbuffer[port->logbuffer_head] =
				kzalloc(LOG_BUFFER_ENTRY_SIZE, GFP_KERNEL);
		if (!port->logbuffer[port->logbuffer_head]) {
			mutex_unlock(&port->logbuffer_lock);
			return;
		}
	}

	vsnprintf(tmpbuffer, sizeof(tmpbuffer), fmt, args);

	if (port->logbuffer_head < 0 ||
	    port->logbuffer_head >= LOG_BUFFER_ENTRIES) {
		dev_warn(port->dev,
			 "Bad log buffer index %d\n", port->logbuffer_head);
		goto abort;
	}

	if (!port->logbuffer[port->logbuffer_head]) {
		dev_warn(port->dev,
			 "Log buffer index %d is NULL\n", port->logbuffer_head);
		goto abort;
	}

	rem_nsec = do_div(ts_nsec, 1000000000);
	scnprintf(port->logbuffer[port->logbuffer_head],
		  LOG_BUFFER_ENTRY_SIZE, "[%5lu.%06lu] %s",
		  (unsigned long)ts_nsec, rem_nsec / 1000,
		  tmpbuffer);
	port->logbuffer_head = (port->logbuffer_head + 1) % LOG_BUFFER_ENTRIES;
	if (port->logbuffer_head == port->logbuffer_tail)
		port->logbuffer_tail =
			(port->logbuffer_tail + 1) % LOG_BUFFER_ENTRIES;

abort:
	mutex_unlock(&port->logbuffer_lock);
}

#endif

__printf(2, 3)
static void tcpm_log(struct tcpm_port *port, const char *fmt, ...)
{
	va_list args;

	/* Do not log while disconnected and unattached */
	if (tcpm_port_is_disconnected(port) &&
	    (port->state == SRC_UNATTACHED || port->state == SNK_UNATTACHED ||
	     port->state == TOGGLING))
		return;

	if (port->tcpc->log_rtc)
		port->tcpc->log_rtc(port->tcpc);

	va_start(args, fmt);
#ifdef CONFIG_DEBUG_FS
	_tcpm_log(port, fmt, args);
#else
	logbuffer_vlog(port->log, fmt, args);
#endif
	va_end(args);
}

__printf(2, 3)
static void tcpm_log_force(struct tcpm_port *port, const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
#ifdef CONFIG_DEBUG_FS
	_tcpm_log(port, fmt, args);
#else
	logbuffer_vlog(port->log, fmt, args);
#endif
	va_end(args);
}

static void tcpm_log_source_caps(struct tcpm_port *port)
{
	int i;

	for (i = 0; i < port->nr_source_caps; i++) {
		u32 pdo = port->source_caps[i];
		enum pd_pdo_type type = pdo_type(pdo);
		char msg[64];

		switch (type) {
		case PDO_TYPE_FIXED:
			scnprintf(msg, sizeof(msg),
				  "%u mV, %u mA [%s%s%s%s%s%s]",
				  pdo_fixed_voltage(pdo),
				  pdo_max_current(pdo),
				  (pdo & PDO_FIXED_DUAL_ROLE) ?
							"R" : "",
				  (pdo & PDO_FIXED_SUSPEND) ?
							"S" : "",
				  (pdo & PDO_FIXED_HIGHER_CAP) ?
							"H" : "",
				  (pdo & PDO_FIXED_USB_COMM) ?
							"U" : "",
				  (pdo & PDO_FIXED_DATA_SWAP) ?
							"D" : "",
				  (pdo & PDO_FIXED_EXTPOWER) ?
							"E" : "",
				  (pdo & PDO_FIXED_UNCHUNK_EXT) ?
							"N" : "");
			break;
		case PDO_TYPE_VAR:
			scnprintf(msg, sizeof(msg),
				  "%u-%u mV, %u mA",
				  pdo_min_voltage(pdo),
				  pdo_max_voltage(pdo),
				  pdo_max_current(pdo));
			break;
		case PDO_TYPE_BATT:
			scnprintf(msg, sizeof(msg),
				  "%u-%u mV, %u mW",
				  pdo_min_voltage(pdo),
				  pdo_max_voltage(pdo),
				  pdo_max_power(pdo));
			break;
		case PDO_TYPE_APDO:
			if (pdo_apdo_type(pdo) == APDO_TYPE_PPS)
				scnprintf(msg, sizeof(msg),
					  "%u-%u mV, %u mA",
					  pdo_pps_apdo_min_voltage(pdo),
					  pdo_pps_apdo_max_voltage(pdo),
					  pdo_pps_apdo_max_current(pdo));
			else
				strcpy(msg, "undefined APDO");
			break;
		default:
			strcpy(msg, "undefined");
			break;
		}
		tcpm_log(port, " PDO %d: type %d, %s",
			 i, type, msg);
	}
}

#ifdef CONFIG_DEBUG_FS

static int tcpm_debug_show(struct seq_file *s, void *v)
{
	struct tcpm_port *port = (struct tcpm_port *)s->private;
	int tail;

	mutex_lock(&port->logbuffer_lock);
	tail = port->logbuffer_tail;
	while (tail != port->logbuffer_head) {
		seq_printf(s, "%s\n", port->logbuffer[tail]);
		tail = (tail + 1) % LOG_BUFFER_ENTRIES;
	}
	mutex_unlock(&port->logbuffer_lock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(tcpm_debug);

static void tcpm_debugfs_init(struct tcpm_port *port)
{
	char name[NAME_MAX];

	mutex_init(&port->logbuffer_lock);
	snprintf(name, NAME_MAX, "tcpm-%s", dev_name(port->dev));
	port->dentry = debugfs_create_file(name, S_IFREG | 0444, usb_debug_root,
					   port, &tcpm_debug_fops);
}

static void tcpm_debugfs_exit(struct tcpm_port *port)
{
	int i;

	mutex_lock(&port->logbuffer_lock);
	for (i = 0; i < LOG_BUFFER_ENTRIES; i++) {
		kfree(port->logbuffer[i]);
		port->logbuffer[i] = NULL;
	}
	mutex_unlock(&port->logbuffer_lock);

	debugfs_remove(port->dentry);
}

#else

static void tcpm_debugfs_init(const struct tcpm_port *port) { }
static void tcpm_debugfs_exit(const struct tcpm_port *port) { }

#endif

static void tcpm_set_cc(struct tcpm_port *port, enum typec_cc_status cc)
{
	tcpm_log(port, "cc:=%d", cc);
	port->cc_req = cc;
	port->tcpc->set_cc(port->tcpc, cc);
}

static enum typec_cc_status tcpm_rp_cc(struct tcpm_port *port);
static int tcpm_ams_finish(struct tcpm_port *port)
{
	int ret = 0;

	tcpm_log(port, "AMS %s finished", tcpm_ams_str[port->ams]);

	if (port->pd_capable && port->pwr_role == TYPEC_SOURCE) {
		if (port->negotiated_rev >= PD_REV30)
			tcpm_set_cc(port, SINK_TX_OK);
		else
			tcpm_set_cc(port, SINK_TX_NG);
	} else if (port->pwr_role == TYPEC_SOURCE) {
		tcpm_set_cc(port, tcpm_rp_cc(port));
	}

	port->in_ams = false;
	port->ams = NONE_AMS;

	return ret;
}

static int tcpm_pd_transmit(struct tcpm_port *port,
			    enum tcpm_transmit_type type,
			    const struct pd_message *msg)
{
	unsigned long timeout;
	int ret;

	if (msg)
		tcpm_log(port, "PD TX, header: %#x", le16_to_cpu(msg->header));
	else
		tcpm_log(port, "PD TX, type: %#x", type);

	reinit_completion(&port->tx_complete);
	ret = port->tcpc->pd_transmit(port->tcpc, type, msg, port->negotiated_rev);
	if (ret < 0)
		return ret;

	mutex_unlock(&port->lock);
	timeout = wait_for_completion_timeout(&port->tx_complete,
				msecs_to_jiffies(PD_T_TCPC_TX_TIMEOUT));
	mutex_lock(&port->lock);
	if (!timeout)
		return -ETIMEDOUT;

	switch (port->tx_status) {
	case TCPC_TX_SUCCESS:
		port->message_id = (port->message_id + 1) & PD_HEADER_ID_MASK;
		/*
		 * USB PD rev 2.0, 8.3.2.2.1:
		 * USB PD rev 3.0, 8.3.2.1.3:
		 * "... Note that every AMS is Interruptible until the first
		 * Message in the sequence has been successfully sent (GoodCRC
		 * Message received)."
		 */
		if (port->ams != NONE_AMS)
			port->in_ams = true;
		break;
	case TCPC_TX_DISCARDED:
		ret = -EAGAIN;
		break;
	case TCPC_TX_FAILED:
	default:
		ret = -EIO;
		break;
	}

	/* Some AMS don't expect responses. Finish them here. */
	if (port->ams == ATTENTION || port->ams == SOURCE_ALERT)
		tcpm_ams_finish(port);

	return ret;
}

void tcpm_pd_transmit_complete(struct tcpm_port *port,
			       enum tcpm_transmit_status status)
{
	tcpm_log(port, "PD TX complete, status: %u", status);
	port->tx_status = status;
	complete(&port->tx_complete);
}
EXPORT_SYMBOL_GPL(tcpm_pd_transmit_complete);

static int tcpm_tx_chunk_handler(struct tcpm_port *port, struct pd_message *msg,
				 enum chunk_tx_source source);
static int tcpm_chunk_transmit(struct tcpm_port *port,
			       enum tcpm_transmit_type type,
			       struct pd_message *msg)
{
	int ret;

	mutex_lock(&port->chunk_lock);
	port->tx_type = type;
	ret = tcpm_tx_chunk_handler(port, msg, POLICY_ENGINE);
	mutex_unlock(&port->chunk_lock);
	return ret;
}

static int tcpm_mux_set(struct tcpm_port *port, int state,
			enum usb_role usb_role,
			enum typec_orientation orientation)
{
	int ret;

	tcpm_log(port, "Requesting mux state %d, usb-role %d, orientation %d",
		 state, usb_role, orientation);

	ret = typec_set_orientation(port->typec_port, orientation);
	if (ret)
		return ret;

	if (port->role_sw) {
		ret = usb_role_switch_set_role(port->role_sw, usb_role);
		if (ret)
			return ret;
	}

	return typec_set_mode(port->typec_port, state);
}

static int tcpm_set_polarity(struct tcpm_port *port,
			     enum typec_cc_polarity polarity)
{
	int ret;

	tcpm_log(port, "polarity %d", polarity);

	ret = port->tcpc->set_polarity(port->tcpc, polarity);
	if (ret < 0)
		return ret;

	port->polarity = polarity;

	return 0;
}

static int tcpm_set_vconn(struct tcpm_port *port, bool enable)
{
	int ret;

	tcpm_log(port, "vconn:=%d", enable);

	ret = port->tcpc->set_vconn(port->tcpc, enable);
	if (!ret) {
		port->vconn_role = enable ? TYPEC_SOURCE : TYPEC_SINK;
		typec_set_vconn_role(port->typec_port, port->vconn_role);
	}

	return ret;
}

static u32 tcpm_get_current_limit(struct tcpm_port *port)
{
	enum typec_cc_status cc;
	u32 limit;

	cc = port->polarity ? port->cc2 : port->cc1;
	switch (cc) {
	case TYPEC_CC_RP_1_5:
		limit = 1500;
		break;
	case TYPEC_CC_RP_3_0:
		limit = 3000;
		break;
	case TYPEC_CC_RP_DEF:
	default:
		if (port->tcpc->get_current_limit)
			limit = port->tcpc->get_current_limit(port->tcpc);
		else
			limit = 0;
		break;
	}

	return limit;
}

static int tcpm_set_current_limit(struct tcpm_port *port, u32 max_ma, u32 mv)
{
	int ret = -EOPNOTSUPP;

	tcpm_log(port, "Setting voltage/current limit %u mV %u mA", mv, max_ma);

	port->supply_voltage = mv;
	port->current_limit = max_ma;

	if (port->tcpc->set_current_limit)
		ret = port->tcpc->set_current_limit(port->tcpc, max_ma, mv);

	return ret;
}

static void tcpm_set_pd_capable(struct tcpm_port *port, bool capable)
{
	tcpm_log(port, "Setting pd capable %s", capable ? "true" : "false");

	if (port->tcpc->set_pd_capable)
		port->tcpc->set_pd_capable(port->tcpc, capable);

	port->pd_capable = capable;
}

static void tcpm_port_in_hard_reset(struct tcpm_port *port, bool status)
{
	tcpm_log(port, "Setting hard reset %s", status ? "true" : "false");

	if (port->tcpc->set_in_hard_reset)
		port->tcpc->set_in_hard_reset(port->tcpc, status);
}

static int tcpm_set_stdby_current(struct tcpm_port *port)
{
	int ret = -EOPNOTSUPP;
	u32 stdby_ma = 0;

	if (port->supply_voltage)
		stdby_ma = PD_P_SNK_STDBY * 1000 / port->supply_voltage;

	if (port->tcpc->set_current_limit) {
		tcpm_log(port, "Setting standby current %u mV @ %u mA",
			 port->supply_voltage, stdby_ma);
		ret = port->tcpc->set_current_limit(port->tcpc, stdby_ma,
						    port->supply_voltage);
	}

	return ret;
}

/*
 * Determine RP value to set based on maximum current supported
 * by a port if configured as source.
 * Returns CC value to report to link partner.
 */
static enum typec_cc_status tcpm_rp_cc(struct tcpm_port *port)
{
	const u32 *src_pdo = port->src_pdo;
	int nr_pdo = port->nr_src_pdo;
	int i;

	/*
	 * Search for first entry with matching voltage.
	 * It should report the maximum supported current.
	 */
	for (i = 0; i < nr_pdo; i++) {
		const u32 pdo = src_pdo[i];

		if (pdo_type(pdo) == PDO_TYPE_FIXED &&
		    pdo_fixed_voltage(pdo) == 5000) {
			unsigned int curr = pdo_max_current(pdo);

			if (curr >= 3000)
				return TYPEC_CC_RP_3_0;
			else if (curr >= 1500)
				return TYPEC_CC_RP_1_5;
			return TYPEC_CC_RP_DEF;
		}
	}

	return TYPEC_CC_RP_DEF;
}

static int tcpm_set_attached_state(struct tcpm_port *port, bool attached)
{
	return port->tcpc->set_roles(port->tcpc, attached, port->pwr_role,
				     port->data_role, port->usb_comm_capable);
}

static int tcpm_set_roles(struct tcpm_port *port, bool attached,
			  enum typec_role role, enum typec_data_role data)
{
	enum typec_orientation orientation;
	enum usb_role usb_role;
	int ret;

	if (port->polarity == TYPEC_POLARITY_CC1)
		orientation = TYPEC_ORIENTATION_NORMAL;
	else
		orientation = TYPEC_ORIENTATION_REVERSE;

	if (data == TYPEC_HOST)
		usb_role = USB_ROLE_HOST;
	else
		usb_role = USB_ROLE_DEVICE;

	ret = tcpm_mux_set(port, TYPEC_STATE_USB, usb_role, orientation);
	if (ret < 0)
		return ret;

	ret = port->tcpc->set_roles(port->tcpc, attached, role, data,
				    port->usb_comm_capable);
	if (ret < 0)
		return ret;

	port->pwr_role = role;
	port->data_role = data;
	typec_set_data_role(port->typec_port, data);
	typec_set_pwr_role(port->typec_port, role);

	return 0;
}

static int tcpm_set_pwr_role(struct tcpm_port *port, enum typec_role role)
{
	int ret;

	ret = port->tcpc->set_roles(port->tcpc, true, role,
				    port->data_role, port->usb_comm_capable);
	if (ret < 0)
		return ret;

	port->pwr_role = role;
	typec_set_pwr_role(port->typec_port, role);

	return 0;
}

/*
 * Transform the PDO to be compliant to PD rev2.0.
 * Return 0 if the PDO type is not defined in PD rev2.0.
 * Otherwise, return the converted PDO.
 */
static u32 tcpm_forge_legacy_pdo(struct tcpm_port *port, u32 pdo,
				 enum typec_role role)
{
	switch (pdo_type(pdo)) {
	case PDO_TYPE_FIXED:
		if (role == TYPEC_SINK)
			return pdo & ~PDO_FIXED_FRS_CURR_MASK;
		else
			return pdo & ~PDO_FIXED_UNCHUNK_EXT;
	case PDO_TYPE_VAR:
	case PDO_TYPE_BATT:
		return pdo;
	case PDO_TYPE_APDO:
	default:
		return 0;
	}
}

static int tcpm_pd_send_source_caps(struct tcpm_port *port)
{
	struct pd_message msg;
	u32 pdo;
	unsigned int i, nr_pdo = 0;

	memset(&msg, 0, sizeof(msg));

	for (i = 0; i < port->nr_src_pdo; i++) {
		if (port->negotiated_rev >= PD_REV30) {
			msg.payload[nr_pdo++] =	cpu_to_le32(port->src_pdo[i]);
		} else {
			pdo = tcpm_forge_legacy_pdo(port, port->src_pdo[i],
						    TYPEC_SOURCE);
			if (pdo)
				msg.payload[nr_pdo++] = cpu_to_le32(pdo);
		}
	}

	if (!nr_pdo) {
		/* No source capabilities defined, sink only */
		msg.header = PD_HEADER_LE(PD_CTRL_REJECT,
					  port->pwr_role,
					  port->data_role,
					  port->negotiated_rev,
					  port->message_id, 0, 0);
	} else {
		msg.header = PD_HEADER_LE(PD_DATA_SOURCE_CAP,
					  port->pwr_role,
					  port->data_role,
					  port->negotiated_rev,
					  port->message_id,
					  nr_pdo, 0);
	}

	return tcpm_chunk_transmit(port, TCPC_TX_SOP, &msg);
}

static int tcpm_pd_send_sink_caps(struct tcpm_port *port)
{
	struct pd_message msg;
	u32 pdo;
	unsigned int i, nr_pdo = 0;

	memset(&msg, 0, sizeof(msg));

	for (i = 0; i < port->nr_snk_pdo; i++) {
		if (port->negotiated_rev >= PD_REV30) {
			msg.payload[nr_pdo++] =	cpu_to_le32(port->snk_pdo[i]);
		} else {
			pdo = tcpm_forge_legacy_pdo(port, port->snk_pdo[i],
						    TYPEC_SINK);
			if (pdo)
				msg.payload[nr_pdo++] = cpu_to_le32(pdo);
		}
	}

	if (!nr_pdo) {
		/* No sink capabilities defined, source only */
		msg.header = PD_HEADER_LE(PD_CTRL_REJECT,
					  port->pwr_role,
					  port->data_role,
					  port->negotiated_rev,
					  port->message_id, 0, 0);
	} else {
		msg.header = PD_HEADER_LE(PD_DATA_SINK_CAP,
					  port->pwr_role,
					  port->data_role,
					  port->negotiated_rev,
					  port->message_id,
					  nr_pdo, 0);
	}

	return tcpm_chunk_transmit(port, TCPC_TX_SOP, &msg);
}

static void tcpm_set_state(struct tcpm_port *port, enum tcpm_state state,
			   unsigned int delay_ms)
{
	if (delay_ms) {
		tcpm_log(port, "pending state change %s -> %s @ %u ms [%s %s]",
			 tcpm_states[port->state], tcpm_states[state], delay_ms,
			 pd_rev[port->negotiated_rev], tcpm_ams_str[port->ams]);
		port->delayed_state = state;
		mod_delayed_work(port->wq, &port->state_machine,
				 msecs_to_jiffies(delay_ms));
		port->delayed_runtime = jiffies + msecs_to_jiffies(delay_ms);
		port->delay_ms = delay_ms;
	} else {
		tcpm_log(port, "state change %s -> %s [%s %s]",
			 tcpm_states[port->state], tcpm_states[state],
			 pd_rev[port->negotiated_rev], tcpm_ams_str[port->ams]);
		port->delayed_state = INVALID_STATE;
		port->prev_state = port->state;
		port->state = state;
		/*
		 * Don't re-queue the state machine work item if we're currently
		 * in the state machine and we're immediately changing states.
		 * tcpm_state_machine_work() will continue running the state
		 * machine.
		 */
		if (!port->state_machine_running)
			mod_delayed_work(port->wq, &port->state_machine, 0);
	}
}

static void tcpm_set_state_cond(struct tcpm_port *port, enum tcpm_state state,
				unsigned int delay_ms)
{
	if (port->enter_state == port->state)
		tcpm_set_state(port, state, delay_ms);
	else
		tcpm_log(port,
			 "skipped %sstate change %s -> %s [%u ms], context state %s [%s %s]",
			 delay_ms ? "delayed " : "",
			 tcpm_states[port->state], tcpm_states[state],
			 delay_ms, tcpm_states[port->enter_state],
			 pd_rev[port->negotiated_rev], tcpm_ams_str[port->ams]);
}

static void tcpm_queue_message(struct tcpm_port *port,
			       enum pd_msg_request message)
{
	port->queued_message = message;
	mod_delayed_work(port->wq, &port->state_machine, 0);
}

static bool tcpm_vdm_ams(struct tcpm_port *port)
{
	switch (port->ams) {
	case DISCOVER_IDENTITY:
	case SOURCE_STARTUP_CABLE_PLUG_DISCOVER_IDENTITY:
	case DISCOVER_SVIDS:
	case DISCOVER_MODES:
	case DFP_TO_UFP_ENTER_MODE:
	case DFP_TO_UFP_EXIT_MODE:
	case DFP_TO_CABLE_PLUG_ENTER_MODE:
	case DFP_TO_CABLE_PLUG_EXIT_MODE:
	case ATTENTION:
	case UNSTRUCTURED_VDMS:
	case STRUCTURED_VDMS:
		break;
	default:
		return false;
	}

	return true;
}

static bool tcpm_ams_interruptible(struct tcpm_port *port)
{
	switch (port->ams) {
	/* Interruptible AMS */
	case NONE_AMS:
	case SECURITY:
	case FIRMWARE_UPDATE:
	case DISCOVER_IDENTITY:
	case SOURCE_STARTUP_CABLE_PLUG_DISCOVER_IDENTITY:
	case DISCOVER_SVIDS:
	case DISCOVER_MODES:
	case DFP_TO_UFP_ENTER_MODE:
	case DFP_TO_UFP_EXIT_MODE:
	case DFP_TO_CABLE_PLUG_ENTER_MODE:
	case DFP_TO_CABLE_PLUG_EXIT_MODE:
	case UNSTRUCTURED_VDMS:
	case STRUCTURED_VDMS:
	case COUNTRY_INFO:
	case COUNTRY_CODES:
		break;
	/* Non-Interruptible AMS */
	default:
		if (port->in_ams)
			return false;
		break;
	}

	return true;
}

static int tcpm_ams_start(struct tcpm_port *port, enum tcpm_ams ams)
{
	int ret = 0;

	tcpm_log(port, "AMS %s start", tcpm_ams_str[ams]);

	if (!tcpm_ams_interruptible(port) &&
	    !(ams == HARD_RESET || ams == SOFT_RESET_AMS)) {
		port->upcoming_state = INVALID_STATE;
		tcpm_log(port, "AMS %s not interruptible, aborting",
			 tcpm_ams_str[port->ams]);
		return -EAGAIN;
	}

	if (port->pwr_role == TYPEC_SOURCE) {
		enum typec_cc_status cc_req;

		port->ams = ams;

		if (ams == HARD_RESET) {
			tcpm_set_cc(port, tcpm_rp_cc(port));
			tcpm_pd_transmit(port, TCPC_TX_HARD_RESET, NULL);
			tcpm_set_state(port, HARD_RESET_START, 0);
			return ret;
		} else if (ams == SOFT_RESET_AMS) {
			if (!port->explicit_contract)
				tcpm_set_cc(port, tcpm_rp_cc(port));
			tcpm_set_state(port, SOFT_RESET_SEND, 0);
			return ret;
		} else if (tcpm_vdm_ams(port)) {
			/* tSinkTx is enforced in vdm_run_state_machine */
			if (port->negotiated_rev >= PD_REV30)
				tcpm_set_cc(port, SINK_TX_NG);
			return ret;
		}

		if (port->negotiated_rev >= PD_REV30) {
			cc_req = port->cc_req;
			tcpm_set_cc(port, SINK_TX_NG);
		}

		switch (port->state) {
		case SRC_READY:
		case SRC_STARTUP:
		case SRC_SOFT_RESET_WAIT_SNK_TX:
		case SOFT_RESET:
		case SOFT_RESET_SEND:
			if (port->negotiated_rev >= PD_REV30)
				tcpm_set_state(port, AMS_START,
					       cc_req == SINK_TX_OK ?
					       PD_T_SINK_TX : 0);
			else
				tcpm_set_state(port, AMS_START, 0);
			break;
		default:
			if (port->negotiated_rev >= PD_REV30)
				tcpm_set_state(port, SRC_READY,
					       cc_req == SINK_TX_OK ?
					       PD_T_SINK_TX : 0);
			else
				tcpm_set_state(port, SRC_READY, 0);
			break;
		}
	} else {
		if (port->negotiated_rev >= PD_REV30 &&
		    !tcpm_sink_tx_ok(port) &&
		    ams != SOFT_RESET_AMS &&
		    ams != HARD_RESET) {
			port->upcoming_state = INVALID_STATE;
			tcpm_log(port, "Sink TX No Go");
			return -EAGAIN;
		}

		port->ams = ams;

		if (ams == HARD_RESET) {
			tcpm_pd_transmit(port, TCPC_TX_HARD_RESET, NULL);
			tcpm_set_state(port, HARD_RESET_START, 0);
			return ret;
		} else if (tcpm_vdm_ams(port)) {
			return ret;
		}

		if (port->state == SNK_READY ||
		    port->state == SNK_SOFT_RESET)
			tcpm_set_state(port, AMS_START, 0);
		else
			tcpm_set_state(port, SNK_READY, 0);
	}

	return ret;
}

/*
 * VDM/VDO handling functions
 */
static void tcpm_queue_vdm(struct tcpm_port *port, const u32 header,
			   const u32 *data, int cnt,
			   enum tcpm_transmit_type receiver)
{
	port->vdo_count = cnt + 1;
	port->vdo_data[0] = header;
	port->vdo_receiver = receiver;
	memcpy(&port->vdo_data[1], data, sizeof(u32) * cnt);
	/* Set ready, vdm state machine will actually send */
	port->vdm_retries = 0;
	port->vdm_state = VDM_STATE_READY;
}

static void svdm_consume_identity(struct tcpm_port *port, const __le32 *payload,
				  int cnt)
{
	u32 vdo = le32_to_cpu(payload[VDO_INDEX_IDH]);
	u32 product = le32_to_cpu(payload[VDO_INDEX_PRODUCT]);

	memset(&port->mode_data, 0, sizeof(port->mode_data));

	port->partner_ident.id_header = vdo;
	port->partner_ident.cert_stat = le32_to_cpu(payload[VDO_INDEX_CSTAT]);
	port->partner_ident.product = product;

	if (PD_IDH_PTYPE(vdo) == IDH_PTYPE_VPD)
		port->partner_ident.product_type =
			le32_to_cpu(payload[VDO_INDEX_VPD]);

	typec_partner_set_identity(port->partner);

	tcpm_log(port, "Identity: %04x:%04x.%04x",
		 PD_IDH_VID(vdo),
		 PD_PRODUCT_PID(product), product & 0xffff);
}

static bool svdm_consume_svids(struct tcpm_port *port, const __le32 *payload,
			       int cnt)
{
	struct pd_mode_data *pmdata = &port->mode_data;
	int i;

	for (i = 1; i < cnt; i++) {
		u32 p = le32_to_cpu(payload[i]);
		u16 svid;

		svid = (p >> 16) & 0xffff;
		if (!svid)
			return false;

		if (pmdata->nsvids >= SVID_DISCOVERY_MAX)
			goto abort;

		pmdata->svids[pmdata->nsvids++] = svid;
		tcpm_log(port, "SVID %d: 0x%x", pmdata->nsvids, svid);

		svid = p & 0xffff;
		if (!svid)
			return false;

		if (pmdata->nsvids >= SVID_DISCOVERY_MAX)
			goto abort;

		pmdata->svids[pmdata->nsvids++] = svid;
		tcpm_log(port, "SVID %d: 0x%x", pmdata->nsvids, svid);
	}
	return true;
abort:
	tcpm_log(port, "SVID_DISCOVERY_MAX(%d) too low!", SVID_DISCOVERY_MAX);
	return false;
}

static void svdm_consume_modes(struct tcpm_port *port, const __le32 *payload,
			       int cnt)
{
	struct pd_mode_data *pmdata = &port->mode_data;
	struct typec_altmode_desc *paltmode;
	int i;

	if (pmdata->altmodes >= ARRAY_SIZE(port->partner_altmode)) {
		/* Already logged in svdm_consume_svids() */
		return;
	}

	for (i = 1; i < cnt; i++) {
		paltmode = &pmdata->altmode_desc[pmdata->altmodes];
		memset(paltmode, 0, sizeof(*paltmode));

		paltmode->svid = pmdata->svids[pmdata->svid_index];
		paltmode->mode = i;
		paltmode->vdo = le32_to_cpu(payload[i]);

		tcpm_log(port, " Alternate mode %d: SVID 0x%04x, VDO %d: 0x%08x",
			 pmdata->altmodes, paltmode->svid,
			 paltmode->mode, paltmode->vdo);

		pmdata->altmodes++;
	}
}

static void tcpm_register_partner_altmodes(struct tcpm_port *port)
{
	struct pd_mode_data *modep = &port->mode_data;
	struct typec_altmode *altmode;
	int i;

	for (i = 0; i < modep->altmodes; i++) {
		altmode = typec_partner_register_altmode(port->partner,
						&modep->altmode_desc[i]);
		if (IS_ERR_OR_NULL(altmode)) {
			tcpm_log(port, "Failed to register partner SVID 0x%04x",
				 modep->altmode_desc[i].svid);
			altmode = NULL;
		}
		port->partner_altmode[i] = altmode;
	}
}

#define supports_modal(port)	PD_IDH_MODAL_SUPP((port)->partner_ident.id_header)

static int tcpm_pd_svdm(struct tcpm_port *port, const __le32 *payload, int cnt,
			u32 *response)
{
	struct typec_altmode *adev;
	struct typec_altmode *pdev;
	struct pd_mode_data *modep;
	u32 p[PD_MAX_PAYLOAD];
	int rlen = 0;
	int cmd_type;
	int cmd;
	int i;

	for (i = 0; i < cnt; i++)
		p[i] = le32_to_cpu(payload[i]);

	cmd_type = PD_VDO_CMDT(p[0]);
	cmd = PD_VDO_CMD(p[0]);

	tcpm_log(port, "Rx VDM cmd 0x%x type %d cmd %d len %d",
		 p[0], cmd_type, cmd, cnt);

	modep = &port->mode_data;

	adev = typec_match_altmode(port->port_altmode, ALTMODE_DISCOVERY_MAX,
				   PD_VDO_VID(p[0]), PD_VDO_OPOS(p[0]));

	pdev = typec_match_altmode(port->partner_altmode, ALTMODE_DISCOVERY_MAX,
				   PD_VDO_VID(p[0]), PD_VDO_OPOS(p[0]));

	switch (cmd_type) {
	case CMDT_INIT:
		switch (cmd) {
		case CMD_DISCOVER_IDENT:
			if (PD_VDO_VID(p[0]) != USB_SID_PD)
				break;

			if (PD_VDO_SVDM_VER(p[0]) <= SVDM_MAX_VER)
				port->svdm_version = PD_VDO_SVDM_VER(p[0]);
			/* 6.4.4.3.1: Only respond as UFP (device) */
			if (port->data_role == TYPEC_DEVICE &&
			    port->nr_snk_vdo) {
				/*
				 * Product Type DFP is not defined in SVDM
				 * version 1.0 and shall be set to zero.
				 */
				if (port->svdm_version < SVDM_V20)
					response[1] = port->snk_vdo[0] &
							~IDH_PT_DFP_MASK;
				else
					response[1] = port->snk_vdo[0];
				for (i = 1; i <  port->nr_snk_vdo; i++)
					response[i + 1] = port->snk_vdo[i];
				rlen = port->nr_snk_vdo + 1;
			}
			break;
		case CMD_DISCOVER_SVID:
			if (PD_VDO_VID(p[0]) != USB_SID_PD)
				break;
			break;
		case CMD_DISCOVER_MODES:
			break;
		case CMD_ENTER_MODE:
			break;
		case CMD_EXIT_MODE:
			break;
		case CMD_ATTENTION:
			/* Attention command does not have response */
			if (adev)
				typec_altmode_attention(adev, p[1]);
			return 0;
		default:
			break;
		}
		if (rlen >= 1) {
			response[0] = p[0] | VDO_CMDT(CMDT_RSP_ACK);
		} else if (rlen == 0) {
			response[0] = p[0] | VDO_CMDT(CMDT_RSP_NAK);
			rlen = 1;
		} else {
			response[0] = p[0] | VDO_CMDT(CMDT_RSP_BUSY);
			rlen = 1;
		}
		response[0] = (response[0] & ~VDO_SVDM_VERS_MASK) |
			      VDO_SVDM_VERS(port->svdm_version);
		break;
	case CMDT_RSP_ACK:
		/* silently drop message if we are not connected */
		if (IS_ERR_OR_NULL(port->partner))
			break;

		switch (cmd) {
		case CMD_DISCOVER_IDENT:
			if (PD_VDO_SVDM_VER(p[0]) <= SVDM_MAX_VER)
				port->svdm_version = PD_VDO_SVDM_VER(p[0]);
			/* 6.4.4.3.1 */
			svdm_consume_identity(port, payload, cnt);
			if (PD_IDH_PTYPE(port->partner_ident.id_header)
				   == IDH_PTYPE_VPD) {
				tcpm_set_state(port, SRC_VPD_READY, 0);
				break;
			}

			if (port->vdo_receiver == TCPC_TX_SOP) {
				response[0] = VDO(USB_SID_PD, 1,
						  port->svdm_version,
						  CMD_DISCOVER_SVID);
				rlen = 1;
			}
			break;
		case CMD_DISCOVER_SVID:
			/* 6.4.4.3.2 */
			if (svdm_consume_svids(port, payload, cnt)) {
				response[0] = VDO(USB_SID_PD, 1,
						  port->svdm_version,
						  CMD_DISCOVER_SVID);
				rlen = 1;
			} else if (modep->nsvids && supports_modal(port)) {
				response[0] = VDO(modep->svids[0], 1,
						  port->svdm_version,
						  CMD_DISCOVER_MODES);
				rlen = 1;
			}
			break;
		case CMD_DISCOVER_MODES:
			/* 6.4.4.3.3 */
			svdm_consume_modes(port, payload, cnt);
			modep->svid_index++;
			if (modep->svid_index < modep->nsvids) {
				u16 svid = modep->svids[modep->svid_index];
				response[0] = VDO(svid, 1,
						  port->svdm_version,
						  CMD_DISCOVER_MODES);
				rlen = 1;
			} else {
				tcpm_register_partner_altmodes(port);
				port->vdm_sm_running = false;
			}
			break;
		case CMD_ENTER_MODE:
			if (adev && pdev) {
				typec_altmode_update_active(pdev, true);

				if (typec_altmode_vdm(adev, p[0], &p[1], cnt)) {
					response[0] = VDO(adev->svid, 1,
							  port->svdm_version,
							  CMD_EXIT_MODE);
					response[0] |= VDO_OPOS(adev->mode);
					return 1;
				}
			}
			return 0;
		case CMD_EXIT_MODE:
			if (adev && pdev) {
				typec_altmode_update_active(pdev, false);

				/* Back to USB Operation */
				WARN_ON(typec_altmode_notify(adev,
							     TYPEC_STATE_USB,
							     NULL));
			}
			break;
		default:
			/* Unrecognized SVDM */
			response[0] = p[0] | VDO_CMDT(CMDT_RSP_NAK);
			rlen = 1;
			response[0] = (response[0] & ~VDO_SVDM_VERS_MASK) |
				      VDO_SVDM_VERS(port->svdm_version);
			break;
		}
		tcpm_ams_finish(port);
		break;
	case CMDT_RSP_NAK:
		switch (cmd) {
		case CMD_DISCOVER_IDENT:
		case CMD_DISCOVER_SVID:
		case CMD_DISCOVER_MODES:
			break;
		case CMD_ENTER_MODE:
			/* Back to USB Operation */
			if (adev)
				WARN_ON(typec_altmode_notify(adev,
							     TYPEC_STATE_USB,
							     NULL));
			break;
		case CMD_EXIT_MODE:
			typec_altmode_update_active(pdev, false);

			/* Back to USB Operation */
			if (adev)
				WARN_ON(typec_altmode_notify(adev,
							     TYPEC_STATE_USB,
							     NULL));
			break;
		default:
			/* Unrecognized SVDM */
			response[0] = p[0] | VDO_CMDT(CMDT_RSP_NAK);
			rlen = 1;
			response[0] = (response[0] & ~VDO_SVDM_VERS_MASK) |
				      VDO_SVDM_VERS(port->svdm_version);
			break;
		}
		port->vdm_sm_running = false;
		tcpm_ams_finish(port);
		break;
	default:
		response[0] = p[0] | VDO_CMDT(CMDT_RSP_NAK);
		rlen = 1;
		response[0] = (response[0] & ~VDO_SVDM_VERS_MASK) |
			      VDO_SVDM_VERS(port->svdm_version);
		port->vdm_sm_running = false;
		break;
	}

	/* Informing the alternate mode drivers about everything */
	if (adev)
		typec_altmode_vdm(adev, p[0], &p[1], cnt);

	return rlen;
}

static void tcpm_handle_vdm_request(struct tcpm_port *port,
				    const __le32 *payload, int cnt)
{
	int rlen = 0;
	u32 response[8] = { };
	u32 p0 = le32_to_cpu(payload[0]);

	if (port->vdm_state == VDM_STATE_BUSY) {
		/* If UFP responded busy retry after timeout */
		if (PD_VDO_CMDT(p0) == CMDT_RSP_BUSY) {
			port->vdm_state = VDM_STATE_WAIT_RSP_BUSY;
			port->vdo_retry = (p0 & ~VDO_CMDT_MASK) |
				CMDT_INIT;
			mod_delayed_work(port->wq, &port->vdm_state_machine,
					 msecs_to_jiffies(PD_T_VDM_BUSY));
			return;
		}
		port->vdm_state = VDM_STATE_DONE;
	}

	if (PD_VDO_SVDM(p0)) {
		rlen = tcpm_pd_svdm(port, payload, cnt, response);
	} else {
		if (port->negotiated_rev >= PD_REV30)
			tcpm_queue_message(port, PD_MSG_CTRL_NOT_SUPP);
	}

	if (rlen > 0) {
		tcpm_queue_vdm(port, response[0], &response[1], rlen - 1,
			       TCPC_TX_SOP);
		mod_delayed_work(port->wq, &port->vdm_state_machine, 0);
	}
}

static void tcpm_send_vdm(struct tcpm_port *port, u32 vid, int cmd,
			  const u32 *data, int count,
			  enum tcpm_transmit_type dest)
{
	u32 header;

	if (WARN_ON(count > VDO_MAX_SIZE - 1))
		count = VDO_MAX_SIZE - 1;

	/* set VDM header with VID & CMD */
	header = VDO(vid, ((vid & USB_SID_PD) == USB_SID_PD) ?
			1 : (PD_VDO_CMD(cmd) <= CMD_ATTENTION),
			port->svdm_version, cmd);
	tcpm_queue_vdm(port, header, data, count, dest);

	mod_delayed_work(port->wq, &port->vdm_state_machine, 0);
}

static unsigned int vdm_ready_timeout(u32 vdm_hdr)
{
	unsigned int timeout;
	int cmd = PD_VDO_CMD(vdm_hdr);

	/* its not a structured VDM command */
	if (!PD_VDO_SVDM(vdm_hdr))
		return PD_T_VDM_UNSTRUCTURED;

	switch (PD_VDO_CMDT(vdm_hdr)) {
	case CMDT_INIT:
		if (cmd == CMD_ENTER_MODE || cmd == CMD_EXIT_MODE)
			timeout = PD_T_VDM_WAIT_MODE_E;
		else
			timeout = PD_T_VDM_SNDR_RSP;
		break;
	default:
		if (cmd == CMD_ENTER_MODE || cmd == CMD_EXIT_MODE)
			timeout = PD_T_VDM_E_MODE;
		else
			timeout = PD_T_VDM_RCVR_RSP;
		break;
	}
	return timeout;
}

static void vdm_run_state_machine(struct tcpm_port *port)
{
	struct pd_message msg;
	int i, res = 0;

	switch (port->vdm_state) {
	case VDM_STATE_READY:
		/* Only transmit VDM if attached */
		if (!port->attached) {
			port->vdm_state = VDM_STATE_ERR_BUSY;
			break;
		}

		/*
		 * if there's traffic or we're not in PDO ready state don't send
		 * a VDM. Only exception being discover_identity.
		 */
		if (port->vdo_receiver == TCPC_TX_SOP &&
		    !(port->state == SRC_READY || port->state == SNK_READY))
			break;

		if (PD_VDO_CMDT(port->vdo_data[0]) == CMDT_INIT) {
			switch (PD_VDO_CMD(port->vdo_data[0])) {
			case CMD_DISCOVER_IDENT:
				res = tcpm_ams_start(port, DISCOVER_IDENTITY);
				if (res == 0)
					port->send_discover = false;
				break;
			case CMD_DISCOVER_SVID:
				res = tcpm_ams_start(port, DISCOVER_SVIDS);
				break;
			case CMD_DISCOVER_MODES:
				res = tcpm_ams_start(port, DISCOVER_MODES);
				break;
			case CMD_ENTER_MODE:
				res = tcpm_ams_start(port,
						     DFP_TO_UFP_ENTER_MODE);
				break;
			case CMD_EXIT_MODE:
				res = tcpm_ams_start(port,
						     DFP_TO_UFP_EXIT_MODE);
				break;
			case CMD_ATTENTION:
				res = tcpm_ams_start(port, ATTENTION);
				break;
			default:
				res = -EOPNOTSUPP;
				break;
			}

			if (res < 0)
				return;
		}

		port->vdm_state = VDM_STATE_SEND_MESSAGE;
		mod_delayed_work(port->wq, &port->vdm_state_machine,
				 (port->negotiated_rev >= PD_REV30) &&
				 (port->pwr_role == TYPEC_SOURCE) &&
				 (PD_VDO_CMDT(port->vdo_data[0]) == CMDT_INIT) ?
				 PD_T_SINK_TX : 0);
		break;
	case VDM_STATE_WAIT_RSP_BUSY:
		port->vdo_data[0] = port->vdo_retry;
		port->vdo_count = 1;
		port->vdm_state = VDM_STATE_READY;
		tcpm_ams_finish(port);
		break;
	case VDM_STATE_BUSY:
		port->vdm_state = VDM_STATE_ERR_TMOUT;
		if (port->ams != NONE_AMS)
			tcpm_ams_finish(port);
		break;
	case VDM_STATE_ERR_SEND:
		/*
		 * A partner which does not support USB PD will not reply,
		 * so this is not a fatal error. At the same time, some
		 * devices may not return GoodCRC under some circumstances,
		 * so we need to retry.
		 */
		if (port->vdm_retries < 3) {
			tcpm_log(port, "VDM Tx error, retry");
			port->vdm_retries++;
			port->vdm_state = VDM_STATE_READY;
			tcpm_ams_finish(port);
		}
		break;
	case VDM_STATE_SEND_MESSAGE:
		/* Prepare and send VDM */
		memset(&msg, 0, sizeof(msg));
		if (port->vdo_receiver == TCPC_TX_SOP) {
			msg.header = PD_HEADER_LE(PD_DATA_VENDOR_DEF,
					port->pwr_role,
					port->data_role,
					port->negotiated_rev,
					port->message_id,
					port->vdo_count, 0);
		} else if (port->vdo_receiver == TCPC_TX_SOP_PRIME) {
			msg.header = PD_HEADER_SOP_PRIME_LE(
					PD_DATA_VENDOR_DEF,
					port->message_id,
					port->vdo_count,
					PD_MAX_REV);
		}

		for (i = 0; i < port->vdo_count; i++)
			msg.payload[i] = cpu_to_le32(port->vdo_data[i]);
		res = tcpm_chunk_transmit(port, port->vdo_receiver, &msg);
		if (res < 0) {
			port->vdm_state = VDM_STATE_ERR_SEND;
		} else {
			unsigned long timeout;

			port->vdm_retries = 0;
			port->vdm_state = VDM_STATE_BUSY;
			timeout = vdm_ready_timeout(port->vdo_data[0]);
			mod_delayed_work(port->wq, &port->vdm_state_machine,
					 timeout);
		}
		break;
	default:
		break;
	}
}

static void vdm_state_machine_work(struct work_struct *work)
{
	struct tcpm_port *port = container_of(work, struct tcpm_port,
					      vdm_state_machine.work);
	enum vdm_states prev_state;

	mutex_lock(&port->lock);

	/*
	 * Continue running as long as the port is not busy and there was
	 * a state change.
	 */
	do {
		prev_state = port->vdm_state;
		vdm_run_state_machine(port);
	} while (port->vdm_state != prev_state &&
		 port->vdm_state != VDM_STATE_BUSY &&
		 port->vdm_state != VDM_STATE_SEND_MESSAGE);

	if (port->vdm_state < VDM_STATE_READY)
		port->vdm_sm_running = false;

	mutex_unlock(&port->lock);
}

enum pdo_err {
	PDO_NO_ERR,
	PDO_ERR_NO_VSAFE5V,
	PDO_ERR_VSAFE5V_NOT_FIRST,
	PDO_ERR_PDO_TYPE_NOT_IN_ORDER,
	PDO_ERR_FIXED_NOT_SORTED,
	PDO_ERR_VARIABLE_BATT_NOT_SORTED,
	PDO_ERR_DUPE_PDO,
	PDO_ERR_PPS_APDO_NOT_SORTED,
	PDO_ERR_DUPE_PPS_APDO,
};

static const char * const pdo_err_msg[] = {
	[PDO_ERR_NO_VSAFE5V] =
	" err: source/sink caps should atleast have vSafe5V",
	[PDO_ERR_VSAFE5V_NOT_FIRST] =
	" err: vSafe5V Fixed Supply Object Shall always be the first object",
	[PDO_ERR_PDO_TYPE_NOT_IN_ORDER] =
	" err: PDOs should be in the following order: Fixed; Battery; Variable",
	[PDO_ERR_FIXED_NOT_SORTED] =
	" err: Fixed supply pdos should be in increasing order of their fixed voltage",
	[PDO_ERR_VARIABLE_BATT_NOT_SORTED] =
	" err: Variable/Battery supply pdos should be in increasing order of their minimum voltage",
	[PDO_ERR_DUPE_PDO] =
	" err: Variable/Batt supply pdos cannot have same min/max voltage",
	[PDO_ERR_PPS_APDO_NOT_SORTED] =
	" err: Programmable power supply apdos should be in increasing order of their maximum voltage",
	[PDO_ERR_DUPE_PPS_APDO] =
	" err: Programmable power supply apdos cannot have same min/max voltage and max current",
};

static enum pdo_err tcpm_caps_err(struct tcpm_port *port, const u32 *pdo,
				  unsigned int nr_pdo)
{
	unsigned int i;

	/* Should at least contain vSafe5v */
	if (nr_pdo < 1)
		return PDO_ERR_NO_VSAFE5V;

	/* The vSafe5V Fixed Supply Object Shall always be the first object */
	if (pdo_type(pdo[0]) != PDO_TYPE_FIXED ||
	    pdo_fixed_voltage(pdo[0]) != VSAFE5V)
		return PDO_ERR_VSAFE5V_NOT_FIRST;

	for (i = 1; i < nr_pdo; i++) {
		if (pdo_type(pdo[i]) < pdo_type(pdo[i - 1])) {
			return PDO_ERR_PDO_TYPE_NOT_IN_ORDER;
		} else if (pdo_type(pdo[i]) == pdo_type(pdo[i - 1])) {
			enum pd_pdo_type type = pdo_type(pdo[i]);

			switch (type) {
			/*
			 * The remaining Fixed Supply Objects, if
			 * present, shall be sent in voltage order;
			 * lowest to highest.
			 */
			case PDO_TYPE_FIXED:
				if (pdo_fixed_voltage(pdo[i]) <=
				    pdo_fixed_voltage(pdo[i - 1]))
					return PDO_ERR_FIXED_NOT_SORTED;
				break;
			/*
			 * The Battery Supply Objects and Variable
			 * supply, if present shall be sent in Minimum
			 * Voltage order; lowest to highest.
			 */
			case PDO_TYPE_VAR:
			case PDO_TYPE_BATT:
				if (pdo_min_voltage(pdo[i]) <
				    pdo_min_voltage(pdo[i - 1]))
					return PDO_ERR_VARIABLE_BATT_NOT_SORTED;
				else if ((pdo_min_voltage(pdo[i]) ==
					  pdo_min_voltage(pdo[i - 1])) &&
					 (pdo_max_voltage(pdo[i]) ==
					  pdo_max_voltage(pdo[i - 1])))
					return PDO_ERR_DUPE_PDO;
				break;
			/*
			 * The Programmable Power Supply APDOs, if present,
			 * shall be sent in Maximum Voltage order;
			 * lowest to highest.
			 */
			case PDO_TYPE_APDO:
				if (pdo_apdo_type(pdo[i]) != APDO_TYPE_PPS)
					break;

				if (pdo_pps_apdo_max_voltage(pdo[i]) <
				    pdo_pps_apdo_max_voltage(pdo[i - 1]))
					return PDO_ERR_PPS_APDO_NOT_SORTED;
				else if (pdo_pps_apdo_min_voltage(pdo[i]) ==
					  pdo_pps_apdo_min_voltage(pdo[i - 1]) &&
					 pdo_pps_apdo_max_voltage(pdo[i]) ==
					  pdo_pps_apdo_max_voltage(pdo[i - 1]) &&
					 pdo_pps_apdo_max_current(pdo[i]) ==
					  pdo_pps_apdo_max_current(pdo[i - 1]))
					return PDO_ERR_DUPE_PPS_APDO;
				break;
			default:
				tcpm_log_force(port, " Unknown pdo type");
			}
		}
	}

	return PDO_NO_ERR;
}

static int tcpm_validate_caps(struct tcpm_port *port, const u32 *pdo,
			      unsigned int nr_pdo)
{
	enum pdo_err err_index = tcpm_caps_err(port, pdo, nr_pdo);

	if (err_index != PDO_NO_ERR) {
		tcpm_log_force(port, " %s", pdo_err_msg[err_index]);
		return -EINVAL;
	}

	return 0;
}

static int tcpm_altmode_enter(struct typec_altmode *altmode)
{
	struct tcpm_port *port = typec_altmode_get_drvdata(altmode);
	u32 header;

	mutex_lock(&port->lock);
	header = VDO(altmode->svid, 1, port->svdm_version,  CMD_ENTER_MODE);
	header |= VDO_OPOS(altmode->mode);

	tcpm_queue_vdm(port, header, NULL, 0, TCPC_TX_SOP);
	mod_delayed_work(port->wq, &port->vdm_state_machine, 0);
	mutex_unlock(&port->lock);

	return 0;
}

static int tcpm_altmode_exit(struct typec_altmode *altmode)
{
	struct tcpm_port *port = typec_altmode_get_drvdata(altmode);
	u32 header;

	mutex_lock(&port->lock);
	header = VDO(altmode->svid, 1, port->svdm_version, CMD_EXIT_MODE);
	header |= VDO_OPOS(altmode->mode);

	tcpm_queue_vdm(port, header, NULL, 0, TCPC_TX_SOP);
	mod_delayed_work(port->wq, &port->vdm_state_machine, 0);
	mutex_unlock(&port->lock);

	return 0;
}

static int tcpm_altmode_vdm(struct typec_altmode *altmode,
			    u32 header, const u32 *data, int count)
{
	struct tcpm_port *port = typec_altmode_get_drvdata(altmode);

	mutex_lock(&port->lock);
	tcpm_queue_vdm(port, header, data, count - 1, TCPC_TX_SOP);
	mod_delayed_work(port->wq, &port->vdm_state_machine, 0);
	mutex_unlock(&port->lock);

	return 0;
}

static const struct typec_altmode_ops tcpm_altmode_ops = {
	.enter = tcpm_altmode_enter,
	.exit = tcpm_altmode_exit,
	.vdm = tcpm_altmode_vdm,
};

/*
 * PD (data, control) command handling functions
 */
static inline enum tcpm_state ready_state(struct tcpm_port *port)
{
	if (port->pwr_role == TYPEC_SOURCE)
		return SRC_READY;
	else
		return SNK_READY;
}

static int tcpm_pd_send_control(struct tcpm_port *port,
				enum pd_ctrl_msg_type type);

static void tcpm_handle_alert(struct tcpm_port *port, const __le32 *payload,
			      int cnt)
{
	u32 p0 = le32_to_cpu(payload[0]);
	unsigned int type = usb_pd_ado_type(p0);

	if (!type) {
		tcpm_log(port, "Alert message received with no type");
		return;
	}

	/* Just handling non-battery alerts for now */
	if (!(type & USB_PD_ADO_TYPE_BATT_STATUS_CHANGE)) {
		switch (port->state) {
		case SRC_READY:
		case SNK_READY:
			tcpm_set_state(port, GET_STATUS_SEND, 0);
			break;
		default:
			tcpm_queue_message(port, PD_MSG_CTRL_WAIT);
			break;
		}
	}
}

static void tcpm_pd_handle_state(struct tcpm_port *port,
				 enum tcpm_state state,
				 enum tcpm_ams ams,
				 unsigned int delay_ms)
{
	switch (port->state) {
	case SRC_READY:
	case SNK_READY:
		port->ams = ams;
		tcpm_set_state(port, state, delay_ms);
		break;
	/* 8.3.3.4.1.1 and 6.8.1 power transitioning */
	case SNK_TRANSITION_SINK:
	case SNK_TRANSITION_SINK_VBUS:
	case SRC_TRANSITION_SUPPLY:
		tcpm_set_state(port, HARD_RESET_SEND, 0);
		break;
	default:
		if (!tcpm_ams_interruptible(port)) {
			tcpm_set_state(port, port->pwr_role == TYPEC_SOURCE ?
				       SRC_SOFT_RESET_WAIT_SNK_TX :
				       SNK_SOFT_RESET,
				       0);
		} else {
			/* process the Message 6.8.1 */
			port->upcoming_state = state;
			port->next_ams = ams;
			tcpm_set_state(port, ready_state(port), delay_ms);
		}
		break;
	}
}

static void tcpm_pd_handle_msg(struct tcpm_port *port,
			       enum pd_msg_request message,
			       enum tcpm_ams ams)
{
	switch (port->state) {
	case SRC_READY:
	case SNK_READY:
		port->ams = ams;
		tcpm_queue_message(port, message);
		break;
	/* PD 3.0 Spec 8.3.3.4.1.1 and 6.8.1 */
	case SNK_TRANSITION_SINK:
	case SNK_TRANSITION_SINK_VBUS:
	case SRC_TRANSITION_SUPPLY:
		tcpm_set_state(port, HARD_RESET_SEND, 0);
		break;
	default:
		if (!tcpm_ams_interruptible(port)) {
			tcpm_set_state(port, port->pwr_role == TYPEC_SOURCE ?
				       SRC_SOFT_RESET_WAIT_SNK_TX :
				       SNK_SOFT_RESET,
				       0);
		} else {
			port->next_ams = ams;
			tcpm_set_state(port, ready_state(port), 0);
			/* 6.8.1 process the Message */
			tcpm_queue_message(port, message);
		}
		break;
	}
}

static void tcpm_pd_data_request(struct tcpm_port *port,
				 const struct pd_message *msg)
{
	enum pd_data_msg_type type = pd_header_type_le(msg->header);
	unsigned int cnt = pd_header_cnt_le(msg->header);
	unsigned int rev = pd_header_rev_le(msg->header);
	unsigned int i;

	switch (type) {
	case PD_DATA_SOURCE_CAP:
		for (i = 0; i < cnt; i++)
			port->source_caps[i] = le32_to_cpu(msg->payload[i]);

		port->nr_source_caps = cnt;

		tcpm_log_source_caps(port);

		tcpm_validate_caps(port, port->source_caps,
				   port->nr_source_caps);

		/*
		 * Adjust revision in subsequent message headers, as required,
		 * to comply with 6.2.1.1.5 of the USB PD 3.0 spec. We don't
		 * support Rev 1.0 so just do nothing in that scenario.
		 */
		if (rev == PD_REV10) {
			if (port->ams == GET_SOURCE_CAPABILITIES)
				tcpm_ams_finish(port);
			break;
		}

		if (rev < PD_MAX_REV)
			port->negotiated_rev = rev;

		if (port->negotiated_rev >= PD_REV30)
			port->chunking = !(port->unchunk_supp &&
					   (port->source_caps[0] &
					    PDO_FIXED_UNCHUNK_EXT));

		if (port->pwr_role == TYPEC_SOURCE) {
			if (port->ams == GET_SOURCE_CAPABILITIES)
				tcpm_pd_handle_state(port, SRC_READY, NONE_AMS,
						     0);
			/* Unexpected Source Capabilities */
			else
				tcpm_pd_handle_msg(port,
					   port->negotiated_rev < PD_REV30 ?
					   PD_MSG_CTRL_REJECT :
					   PD_MSG_CTRL_NOT_SUPP,
					   NONE_AMS);
		} else if (port->state == SNK_WAIT_CAPABILITIES) {
		/*
		 * This message may be received even if VBUS is not
		 * present. This is quite unexpected; see USB PD
		 * specification, sections 8.3.3.6.3.1 and 8.3.3.6.3.2.
		 * However, at the same time, we must be ready to
		 * receive this message and respond to it 15ms after
		 * receiving PS_RDY during power swap operations, no matter
		 * if VBUS is available or not (USB PD specification,
		 * section 6.5.9.2).
		 * So we need to accept the message either way,
		 * but be prepared to keep waiting for VBUS after it was
		 * handled.
		 */
			port->ams = POWER_NEGOTIATION;
			port->in_ams = true;
			tcpm_set_state(port, SNK_NEGOTIATE_CAPABILITIES, 0);
		} else {
			if (port->ams == GET_SOURCE_CAPABILITIES)
				tcpm_ams_finish(port);
			tcpm_pd_handle_state(port, SNK_NEGOTIATE_CAPABILITIES,
					     POWER_NEGOTIATION, 0);
		}
		break;
	case PD_DATA_REQUEST:
		/*
		 * Adjust revision in subsequent message headers, as required,
		 * to comply with 6.2.1.1.5 of the USB PD 3.0 spec. We don't
		 * support Rev 1.0 so just reject in that scenario.
		 */
		if (rev == PD_REV10) {
			tcpm_pd_handle_msg(port,
					   port->negotiated_rev < PD_REV30 ?
					   PD_MSG_CTRL_REJECT :
					   PD_MSG_CTRL_NOT_SUPP,
					   NONE_AMS);
			break;
		}

		if (rev < PD_MAX_REV)
			port->negotiated_rev = rev;

		if (port->pwr_role != TYPEC_SOURCE || cnt != 1) {
			tcpm_pd_handle_msg(port,
					   port->negotiated_rev < PD_REV30 ?
					   PD_MSG_CTRL_REJECT :
					   PD_MSG_CTRL_NOT_SUPP,
					   NONE_AMS);
			break;
		}

		port->sink_request = le32_to_cpu(msg->payload[0]);

		if (port->negotiated_rev >= PD_REV30)
			port->chunking = !(port->unchunk_supp &&
					   (port->sink_request &
					    PDO_FIXED_UNCHUNK_EXT));

		if (port->vdm_sm_running && port->explicit_contract) {
			tcpm_pd_handle_msg(port, PD_MSG_CTRL_WAIT, port->ams);
			break;
		}

		if (port->state == SRC_SEND_CAPABILITIES)
			tcpm_set_state(port, SRC_NEGOTIATE_CAPABILITIES, 0);
		else
			tcpm_pd_handle_state(port, SRC_NEGOTIATE_CAPABILITIES,
					     POWER_NEGOTIATION, 0);
		break;
	case PD_DATA_SINK_CAP:
		/* We don't do anything with this at the moment... */
		for (i = 0; i < cnt; i++)
			port->sink_caps[i] = le32_to_cpu(msg->payload[i]);
		port->nr_sink_caps = cnt;
		if (port->ams == GET_SINK_CAPABILITIES)
			tcpm_set_state(port, ready_state(port), 0);
		break;
	case PD_DATA_VENDOR_DEF:
		tcpm_handle_vdm_request(port, msg->payload, cnt);
		break;
	case PD_DATA_BIST:
		port->bist_request = le32_to_cpu(msg->payload[0]);
		tcpm_pd_handle_state(port, BIST_RX, BIST, 0);
		break;
	case PD_DATA_ALERT:
		tcpm_handle_alert(port, msg->payload, cnt);
		break;
	case PD_DATA_BATT_STATUS:
	case PD_DATA_GET_COUNTRY_INFO:
		/* Currently unsupported */
		tcpm_pd_handle_msg(port, port->negotiated_rev < PD_REV30 ?
				   PD_MSG_CTRL_REJECT :
				   PD_MSG_CTRL_NOT_SUPP,
				   NONE_AMS);
		break;
	default:
		tcpm_pd_handle_msg(port, port->negotiated_rev < PD_REV30 ?
				   PD_MSG_CTRL_REJECT :
				   PD_MSG_CTRL_NOT_SUPP,
				   NONE_AMS);
		tcpm_log(port, "Unrecognized data message type %#x", type);
		break;
	}
}

static void tcpm_pps_complete(struct tcpm_port *port, int result)
{
	if (port->pps_pending) {
		port->pps_status = result;
		port->pps_pending = false;
		complete(&port->pps_complete);
	}
}

static void tcpm_pd_ctrl_request(struct tcpm_port *port,
				 const struct pd_message *msg)
{
	enum pd_ctrl_msg_type type = pd_header_type_le(msg->header);
	enum tcpm_state next_state;

	switch (type) {
	case PD_CTRL_GOOD_CRC:
	case PD_CTRL_PING:
		break;
	case PD_CTRL_GET_SOURCE_CAP:
		tcpm_pd_handle_msg(port, PD_MSG_DATA_SOURCE_CAP,
				   GET_SOURCE_CAPABILITIES);
		break;
	case PD_CTRL_GET_SINK_CAP:
		tcpm_pd_handle_msg(port, PD_MSG_DATA_SINK_CAP,
					GET_SINK_CAPABILITIES);
		break;
	case PD_CTRL_GOTO_MIN:
		break;
	case PD_CTRL_PS_RDY:
		switch (port->state) {
		case SNK_TRANSITION_SINK:
			tcpm_log(port, "in PR_SWAP := false");
			port->tcpc->set_in_pr_swap(port->tcpc, false);
			if (port->vbus_present) {
				tcpm_set_current_limit(port, port->req_current_limit,
						       port->req_supply_voltage);
				port->explicit_contract = true;
				/* Set VDM running flag ASAP */
				if (port->data_role == TYPEC_HOST &&
				    port->send_discover)
					port->vdm_sm_running = true;
				tcpm_set_state(port, SNK_READY, 0);
			} else {
				/*
				 * Seen after power swap. Keep waiting for VBUS
				 * in a transitional state.
				 */
				tcpm_set_state(port,
					       SNK_TRANSITION_SINK_VBUS, 0);
			}
			break;
		case PR_SWAP_SRC_SNK_SOURCE_OFF_CC_DEBOUNCED:
			tcpm_set_state(port, PR_SWAP_SRC_SNK_SINK_ON, 0);
			break;
		case PR_SWAP_SNK_SRC_SINK_OFF:
			tcpm_set_state(port, PR_SWAP_SNK_SRC_SOURCE_ON, 0);
			break;
		case VCONN_SWAP_WAIT_FOR_VCONN:
			tcpm_set_state(port, VCONN_SWAP_TURN_OFF_VCONN, 0);
			break;
		default:
			tcpm_pd_handle_state(port,
					     port->pwr_role == TYPEC_SOURCE ?
					     SRC_SOFT_RESET_WAIT_SNK_TX :
					     SNK_SOFT_RESET,
					     NONE_AMS, 0);
			break;
		}
		break;
	case PD_CTRL_REJECT:
	case PD_CTRL_WAIT:
	case PD_CTRL_NOT_SUPP:
		switch (port->state) {
		case SNK_NEGOTIATE_CAPABILITIES:
			/* USB PD specification, Figure 8-43 */
			if (port->explicit_contract) {
				next_state = SNK_READY;
				if (port->data_role == TYPEC_HOST &&
				    port->send_discover)
					port->vdm_sm_running = true;
			} else {
				next_state = SNK_WAIT_CAPABILITIES;
			}
			tcpm_set_state(port, next_state, 0);
			break;
		case SNK_NEGOTIATE_PPS_CAPABILITIES:
			/* Revert data back from any requested PPS updates */
			port->pps_data.req_out_volt = port->supply_voltage;
			port->pps_data.req_op_curr = port->current_limit;
			port->pps_status = (type == PD_CTRL_WAIT ?
					    -EAGAIN : -EOPNOTSUPP);

			if (port->data_role == TYPEC_HOST &&
			    port->send_discover)
				port->vdm_sm_running = true;

			tcpm_set_state(port, SNK_READY, 0);
			break;
		case DR_SWAP_SEND:
			port->swap_status = (type == PD_CTRL_WAIT ?
					     -EAGAIN : -EOPNOTSUPP);
			tcpm_set_state(port, DR_SWAP_CANCEL, 0);
			break;
		case PR_SWAP_SEND:
			port->swap_status = (type == PD_CTRL_WAIT ?
					     -EAGAIN : -EOPNOTSUPP);
			tcpm_set_state(port, PR_SWAP_CANCEL, 0);
			break;
		case VCONN_SWAP_SEND:
			port->swap_status = (type == PD_CTRL_WAIT ?
					     -EAGAIN : -EOPNOTSUPP);
			tcpm_set_state(port, VCONN_SWAP_CANCEL, 0);
			break;
		case SRC_READY:
		case SNK_READY:
			if (port->vdm_state > VDM_STATE_READY) {
				port->vdm_state = VDM_STATE_DONE;
				if (tcpm_vdm_ams(port))
					tcpm_ams_finish(port);
				mod_delayed_work(port->wq, &port->vdm_state_machine,
					 msecs_to_jiffies(0));
				break;
			}
		default:
			tcpm_pd_handle_state(port,
					     port->pwr_role == TYPEC_SOURCE ?
					     SRC_SOFT_RESET_WAIT_SNK_TX :
					     SNK_SOFT_RESET,
					     NONE_AMS, 0);
			break;
		}
		break;
	case PD_CTRL_ACCEPT:
		switch (port->state) {
		case SNK_NEGOTIATE_CAPABILITIES:
			port->pps_data.active = false;
			tcpm_set_state(port, SNK_TRANSITION_SINK, 0);
			break;
		case SNK_NEGOTIATE_PPS_CAPABILITIES:
			port->pps_data.active = true;
			port->pps_data.min_volt = port->pps_data.req_min_volt;
			port->pps_data.max_volt = port->pps_data.req_max_volt;
			port->pps_data.max_curr = port->pps_data.req_max_curr;
			port->req_supply_voltage = port->pps_data.req_out_volt;
			port->req_current_limit = port->pps_data.req_op_curr;
			power_supply_changed(port->psy);
			tcpm_set_state(port, SNK_TRANSITION_SINK, 0);
			break;
		case SOFT_RESET_SEND:
			if (port->ams == SOFT_RESET_AMS)
				tcpm_ams_finish(port);
			if (port->pwr_role == TYPEC_SOURCE) {
				port->upcoming_state = SRC_SEND_CAPABILITIES;
				tcpm_ams_start(port, POWER_NEGOTIATION);
			} else {
				tcpm_set_state(port, SNK_WAIT_CAPABILITIES, 0);
			}
			break;
		case DR_SWAP_SEND:
			if (port->data_role == TYPEC_DEVICE &&
			    port->send_discover)
				port->vdm_sm_running = true;

			tcpm_set_state(port, DR_SWAP_CHANGE_DR, 0);
			break;
		case PR_SWAP_SEND:
			tcpm_set_state(port, PR_SWAP_START, 0);
			break;
		case VCONN_SWAP_SEND:
			tcpm_set_state(port, VCONN_SWAP_START, 0);
			break;
		default:
			tcpm_pd_handle_state(port,
					     port->pwr_role == TYPEC_SOURCE ?
					     SRC_SOFT_RESET_WAIT_SNK_TX :
					     SNK_SOFT_RESET,
					     NONE_AMS, 0);
			break;
		}
		break;
	case PD_CTRL_SOFT_RESET:
		port->ams = SOFT_RESET_AMS;
		tcpm_set_state(port, SOFT_RESET, 0);
		break;
	case PD_CTRL_DR_SWAP:
		/*
		 * XXX
		 * 6.3.9: If an alternate mode is active, a request to swap
		 * alternate modes shall trigger a port reset.
		 */
		if (port->port_type != TYPEC_PORT_DRP) {
			tcpm_pd_handle_msg(port,
					   port->negotiated_rev < PD_REV30 ?
					   PD_MSG_CTRL_REJECT :
					   PD_MSG_CTRL_NOT_SUPP,
					   NONE_AMS);
		} else {
			if (port->vdm_sm_running) {
				tcpm_queue_message(port, PD_MSG_CTRL_WAIT);
				break;
			}

			tcpm_pd_handle_state(port, DR_SWAP_ACCEPT,
					     DATA_ROLE_SWAP, 0);
		}
		break;
	case PD_CTRL_PR_SWAP:
		if (port->port_type != TYPEC_PORT_DRP) {
			tcpm_pd_handle_msg(port,
					   port->negotiated_rev < PD_REV30 ?
					   PD_MSG_CTRL_REJECT :
					   PD_MSG_CTRL_NOT_SUPP,
					   NONE_AMS);
		} else {
			if (port->vdm_sm_running) {
				tcpm_queue_message(port, PD_MSG_CTRL_WAIT);
				break;
			}

			tcpm_pd_handle_state(port, PR_SWAP_ACCEPT,
					     POWER_ROLE_SWAP, 0);
		}
		break;
	case PD_CTRL_VCONN_SWAP:
		if (port->vdm_sm_running) {
			tcpm_queue_message(port, PD_MSG_CTRL_WAIT);
			break;
		}

		tcpm_pd_handle_state(port, VCONN_SWAP_ACCEPT, VCONN_SWAP, 0);
		break;
	case PD_CTRL_GET_SOURCE_CAP_EXT:
	case PD_CTRL_GET_STATUS:
	case PD_CTRL_FR_SWAP:
	case PD_CTRL_GET_PPS_STATUS:
	case PD_CTRL_GET_COUNTRY_CODES:
		/* Currently not supported */
		tcpm_pd_handle_msg(port,
				   port->negotiated_rev < PD_REV30 ?
				   PD_MSG_CTRL_REJECT :
				   PD_MSG_CTRL_NOT_SUPP,
				   NONE_AMS);
		break;
	default:
		tcpm_pd_handle_msg(port,
				   port->negotiated_rev < PD_REV30 ?
				   PD_MSG_CTRL_REJECT :
				   PD_MSG_CTRL_NOT_SUPP,
				   NONE_AMS);
		tcpm_log(port, "Unrecognized ctrl message type %#x", type);
		break;
	}
}

static void tcpm_pd_ext_msg_request(struct tcpm_port *port,
				    const struct pd_message *msg)
{
	enum pd_ext_msg_type type = pd_header_type_le(msg->header);

	if (!(msg->ext_msg.header & PD_EXT_HDR_CHUNKED)) {
		tcpm_pd_handle_msg(port, PD_MSG_CTRL_NOT_SUPP, NONE_AMS);
		tcpm_log(port, "Unchunked extended messages unsupported");
		return;
	}

	switch (type) {
	case PD_EXT_STATUS:
		/*
		 * If PPS related events raised then get PPS status to clear
		 * (see USB PD 3.0 Spec, 6.5.2.4)
		 */
		if (msg->ext_msg.data[USB_PD_EXT_SDB_EVENT_FLAGS] &
		    USB_PD_EXT_SDB_PPS_EVENTS)
			tcpm_pd_handle_state(port, GET_PPS_STATUS_SEND,
					     GETTING_SOURCE_SINK_STATUS, 0);

		else
			tcpm_pd_handle_state(port, ready_state(port), NONE_AMS,
					     0);
		break;
	case PD_EXT_PPS_STATUS:
		/*
		 * For now the PPS status message is used to clear events
		 * and nothing more.
		 */
		tcpm_pd_handle_state(port, ready_state(port), NONE_AMS, 0);
		break;
	case PD_EXT_SECURITY_RESPONSE:
	case PD_EXT_SECURITY_REQUEST:
	case PD_EXT_FW_UPDATE_RESPONSE:
	case PD_EXT_FW_UPDATE_REQUEST:
		/*
		 * It must have been in CHUNK_RX state, thus it is safe to send
		 * NOT_SUPPORTED directly here.
		 */
		tcpm_queue_message(port, PD_MSG_CTRL_NOT_SUPP);
		tcpm_set_state(port, CHUNK_RX_FINISH, 0);
		break;
	case PD_EXT_SOURCE_CAP_EXT:
	case PD_EXT_GET_BATT_CAP:
	case PD_EXT_GET_BATT_STATUS:
	case PD_EXT_BATT_CAP:
	case PD_EXT_GET_MANUFACTURER_INFO:
	case PD_EXT_MANUFACTURER_INFO:
	case PD_EXT_COUNTRY_INFO:
	case PD_EXT_COUNTRY_CODES:
		tcpm_pd_handle_msg(port, PD_MSG_CTRL_NOT_SUPP, NONE_AMS);
		break;
	default:
		tcpm_pd_handle_msg(port, PD_MSG_CTRL_NOT_SUPP, NONE_AMS);
		tcpm_log(port, "Unrecognized extended message type %#x", type);
		break;
	}
}

static void tcpm_set_chunk_state(struct tcpm_port *port,
				 enum tcpm_chunk_state state,
				 unsigned int delay_ms)
{
	bool is_tx = (state > RCH_REPORT_ERROR) ? true : false;

	if (delay_ms) {
		tcpm_log(port, " pending chunk state change %s -> %s @ %u ms",
			 is_tx ? tcpm_chunk_state[port->chunk_tx_state] :
			 tcpm_chunk_state[port->chunk_rx_state],
			 tcpm_chunk_state[state],
			 delay_ms);
		port->delayed_chunk_state = state;
		port->chunk_delay_ms = delay_ms;
	} else {
		tcpm_log(port, " Chunk state change %s -> %s",
			 is_tx ? tcpm_chunk_state[port->chunk_tx_state] :
			 tcpm_chunk_state[port->chunk_rx_state],
			 tcpm_chunk_state[state]);
		port->delayed_chunk_state = INVALID_CHUNK_STATE;
		if (is_tx)
			port->chunk_tx_state = state;
		else
			port->chunk_rx_state = state;
	}

	mod_delayed_work(port->wq,
			 &port->chunk_state_machine,
			 delay_ms > 0 ? msecs_to_jiffies(delay_ms) : 0);
}

static int tcpm_tx_chunk_handler(struct tcpm_port *port, struct pd_message *msg,
				 enum chunk_tx_source source)
{
	u8 chunk_num = 0;
	bool is_ext_msg = false;
	bool req_chunk = false;
	bool pass_to_rx = false;
	int ret = 0;

	if (!msg)
		return -EINVAL;

	is_ext_msg = msg->header & PD_HEADER_EXT_HDR;

	/*
	 * 6.11.2.1.3.9 TCH_Message_Received State:
	 * The Chunked Tx State Machine Shall enter the TCH_Message_Received
	 * state when any Message is received and the Chunked Tx State Machine
	 * is not in the TCH_Wait_Chunk_Request state.
	 */
	if (source == PROTOCOL_LAYER &&
	    port->chunk_tx_state != TCH_WAIT_CHUNK_REQUEST) {
		tcpm_set_chunk_state(port, TCH_MESSAGE_RECEIVED, 0);
		return 0;
	}

	if (source == POLICY_ENGINE &&
	    port->chunk_tx_state != TCH_WAIT_FOR_MESSAGE_REQUEST)
		return -EINVAL;

	switch (port->chunk_tx_state) {
	case TCH_WAIT_FOR_MESSAGE_REQUEST:
		if (port->chunk_rx_state != RCH_WAIT_FOR_MESSAGE) {
			if (port->abort_supported)
				tcpm_set_chunk_state(port, TCH_REPORT_ERROR, 0);
			else
				return -EINVAL;
			break;
		}

		if (!is_ext_msg || !port->chunking) {
			/*
			 * XXX:
			 * No matter what the result of the transmission is,
			 * Policy Engine will be informed from the return value.
			 * Omit the states transition and stay in
			 * TCH_WAIT_FOR_MESSAGE_REQUEST
			 */
			ret = tcpm_pd_transmit(port, port->tx_type, msg);
			port->chunk_abort = false;
		} else {
			port->is_chunk_tx = true;
			tcpm_set_chunk_state(port,
					TCH_PREPARE_TO_SEND_CHUNKED_MESSAGE, 0);
		}
		break;
	case TCH_PASS_DOWN_MESSAGE:
	case TCH_WAIT_FOR_TRANSMISSION_COMPLETE:
	case TCH_MESSAGE_SENT:
	case TCH_PREPARE_TO_SEND_CHUNKED_MESSAGE:
	case TCH_CONSTRUCT_CHUNKED_MESSAGE:
	case TCH_SENDING_CHUNKED_MESSAGE:
		break;
	case TCH_WAIT_CHUNK_REQUEST:
		/*
		 * If the extended message type is not the same as what we are
		 * waiting for, pass the Message to Rx.
		 */
		if (is_ext_msg &&
		    (pd_header_type_le(msg->header) == port->ext_type))
			req_chunk = msg->ext_msg.header & PD_EXT_HDR_REQ_CHUNK;
		else
			pass_to_rx = true;

		/* If the Message is not a Chunk Request, pass it to Rx. */
		if (req_chunk && !pass_to_rx)
			chunk_num = pd_ext_header_chunk_num(
							msg->ext_msg.header);
		else
			pass_to_rx = true;

		if (pass_to_rx) {
			tcpm_set_chunk_state(port, TCH_MESSAGE_RECEIVED, 0);
			break;
		}

		if (chunk_num == port->chunk_num_to_send)
			tcpm_set_chunk_state(port,
					     TCH_CONSTRUCT_CHUNKED_MESSAGE, 0);
		else
			tcpm_set_chunk_state(port, TCH_REPORT_ERROR, 0);
		break;
	case TCH_MESSAGE_RECEIVED:
	case TCH_REPORT_ERROR:
		break;
	default:
		tcpm_log(port, "%s: unknown chunk state", __func__);
		break;
	}

	return ret;
}

static void tcpm_rx_chunk_handler(struct tcpm_port *port)
{
	const struct pd_message *msg = &port->chunk_event->msg;
	unsigned int cnt = pd_header_cnt_le(msg->header);
	bool is_ext_msg = msg->header & PD_HEADER_EXT_HDR;
	enum pd_ext_msg_type ext_msg_type = 0;
	unsigned int data_size = 0;
	bool request_chunk = false;
	bool chunked = false;

	if (is_ext_msg) {
		chunked = msg->ext_msg.header & PD_EXT_HDR_CHUNKED;
		ext_msg_type = pd_header_type_le(msg->header);
		request_chunk = msg->ext_msg.header & PD_EXT_HDR_REQ_CHUNK;
		data_size = pd_ext_header_data_size_le(msg->ext_msg.header);

		if (data_size > PD_MAX_EXT_MSG_LEN) {
			tcpm_log(port, "extended data size: %d overflow",
				 data_size);
			return;
		}
	}

	switch (port->chunk_rx_state) {
	case RCH_WAIT_FOR_MESSAGE:
		if (!is_ext_msg || (!port->chunking && !chunked)) {
			if (is_ext_msg)
				tcpm_pd_ext_msg_request(port, msg);
			else if (cnt)
				tcpm_pd_data_request(port, msg);
			else
				tcpm_pd_ctrl_request(port, msg);

			memset(&port->chunk_event->msg, 0,
			       sizeof(port->chunk_event->msg));

			kfree(port->ext_rx_buf);
			port->ext_rx_buf = NULL;
			port->ext_rx_buf_size = 0;
			port->chunk_abort = false;
		} else if (is_ext_msg && port->chunking && chunked) {
			switch (ext_msg_type) {
			case PD_EXT_SECURITY_RESPONSE:
			case PD_EXT_SECURITY_REQUEST:
				tcpm_pd_handle_state(port, CHUNK_RX, SECURITY,
						     0);
				break;
			case PD_EXT_FW_UPDATE_RESPONSE:
			case PD_EXT_FW_UPDATE_REQUEST:
				tcpm_pd_handle_state(port, CHUNK_RX,
						     FIRMWARE_UPDATE, 0);
				break;
			default:
				break;
			}

			tcpm_set_chunk_state(port,
					     RCH_PROCESSING_EXTENDED_MESSAGE,
					     0);
		/* This should only happen when port->chunking != chunked */
		} else {
			tcpm_set_chunk_state(port, RCH_REPORT_ERROR, 0);
		}
		break;
	case RCH_WAITING_CHUNK:
		if (is_ext_msg && chunked && !request_chunk &&
		    (ext_msg_type == port->chunk_event->expected_type))
			tcpm_set_chunk_state(port,
					     RCH_PROCESSING_EXTENDED_MESSAGE,
					     0);
		else
			tcpm_set_chunk_state(port, RCH_REPORT_ERROR, 0);
		break;
	case RCH_PASS_UP_MESSAGE:
	case RCH_PROCESSING_EXTENDED_MESSAGE:
	case RCH_REQUESTING_CHUNK:
	case RCH_REPORT_ERROR:
		/*
		 * Any Message Received anda we are not in state
		 * RCH_WAITING_CHUNK or
		 * RCH_WAIT_FOR_MESSAGE, send to RCH_REPORT_ERROR
		 */
		tcpm_set_chunk_state(port, RCH_REPORT_ERROR, 0);
		break;
	default:
		tcpm_log(port, "rx_hdlr: unknown state: %d",
			 port->chunk_rx_state);
		break;
	}
}

static void run_chunked_tx_state_machine(struct tcpm_port *port)
{
	struct pd_message chunk_msg = {0};
	bool last_chunk = false;
	u8 cnt = 0;
	u8 copy_size = 0;
	u16 remaining_size = 0;
	int ret = 0;

	switch (port->chunk_tx_state) {
	case TCH_WAIT_FOR_MESSAGE_REQUEST:
		port->chunk_abort = false;
		break;
	case TCH_PASS_DOWN_MESSAGE:
	case TCH_WAIT_FOR_TRANSMISSION_COMPLETE:
		break;
	case TCH_MESSAGE_SENT:
		tcpm_set_chunk_state(port, TCH_WAIT_FOR_MESSAGE_REQUEST, 0);
		if (port->state == CHUNK_TX)
			tcpm_set_state(port, CHUNK_TX_FINISH, 0);
		break;
	case TCH_PREPARE_TO_SEND_CHUNKED_MESSAGE:
		port->chunk_num_to_send = 0;
		tcpm_set_chunk_state(port, TCH_CONSTRUCT_CHUNKED_MESSAGE, 0);
		break;
	case TCH_CONSTRUCT_CHUNKED_MESSAGE:
		if (port->chunk_abort) {
			tcpm_set_chunk_state(port, TCH_WAIT_FOR_MESSAGE_REQUEST,
					     0);
			break;
		}

		tcpm_set_chunk_state(port, TCH_SENDING_CHUNKED_MESSAGE, 0);
		/* fall through */
	case TCH_SENDING_CHUNKED_MESSAGE:
		remaining_size = port->ext_tx_buf_size -
				 (PD_EXT_MAX_CHUNK_DATA *
				  port->chunk_num_to_send);

		if (remaining_size > PD_EXT_MAX_CHUNK_DATA) {
			cnt = 7;
			copy_size = PD_EXT_MAX_CHUNK_DATA;
		} else {
			last_chunk = true;
			cnt = (remaining_size + 2) >> 2;
			if ((remaining_size + 2) % 4)
				cnt++;
			copy_size = remaining_size;
		}

		chunk_msg.header = PD_HEADER_LE(port->ext_type,
						port->pwr_role,
						port->data_role,
						port->negotiated_rev,
						port->message_id, cnt, 1);
		chunk_msg.ext_msg.header = PD_EXT_HDR_LE(
					port->ext_tx_buf_size, /* data size */
					0, /* req_chunk */
					port->chunk_num_to_send, /* chunk_num */
					1 /* chunked */);
		if (port->ext_tx_buf)
			memcpy(chunk_msg.ext_msg.data,
			       port->ext_tx_buf +
			       PD_EXT_MAX_CHUNK_DATA * port->chunk_num_to_send,
			       copy_size);

		mutex_lock(&port->lock);
		ret = tcpm_pd_transmit(port, TCPC_TX_SOP, &chunk_msg);
		mutex_unlock(&port->lock);
		if (ret == 0)
			tcpm_set_chunk_state(port,
					     last_chunk ?
					     TCH_MESSAGE_SENT :
					     TCH_WAIT_CHUNK_REQUEST,
					     0);
		else
			tcpm_set_chunk_state(port, TCH_REPORT_ERROR, 0);
		break;
	case TCH_WAIT_CHUNK_REQUEST:
		if (port->chunk_num_to_send > 0)
			tcpm_set_chunk_state(port, TCH_REPORT_ERROR,
					     PD_T_CHUNK_SENDER_REQUEST);
		else
			tcpm_set_chunk_state(port, TCH_MESSAGE_SENT,
					     PD_T_CHUNK_SENDER_REQUEST);
		port->chunk_num_to_send++;
		break;
	case TCH_MESSAGE_RECEIVED:
		kfree(port->ext_tx_buf);
		port->ext_tx_buf = NULL;
		port->ext_tx_buf_size = 0;

		// send msg to chunked Rx
		port->chunk_abort = false;
		port->is_chunk_tx = false;
		port->chunk_tx_state = TCH_WAIT_FOR_MESSAGE_REQUEST;
		tcpm_rx_chunk_handler(port);
		break;
	case TCH_REPORT_ERROR:
		tcpm_set_chunk_state(port, TCH_WAIT_FOR_MESSAGE_REQUEST, 0);
		if (port->state == CHUNK_TX)
			tcpm_set_state(port, CHUNK_TX_FINISH, 0);
		break;
	default:
		WARN(1, "Unexpected chunk tx state %d\n", port->chunk_tx_state);
		break;
	}
}

static void run_chunked_rx_state_machine(struct tcpm_port *port)
{
	const struct pd_message *msg = &port->chunk_event->msg;
	unsigned int cnt = pd_header_cnt_le(msg->header);
	enum pd_ext_msg_type ext_msg_type = 0;
	unsigned int data_size = 0;
	int chunk_num_expected = 0;
	int chunk_num = 0;
	int ret = 0;
	bool is_ext_msg = msg->header & PD_HEADER_EXT_HDR;
	bool request_chunk = false;
	bool chunked = false;

	if (is_ext_msg) {
		chunked = msg->ext_msg.header & PD_EXT_HDR_CHUNKED;
		ext_msg_type = pd_header_type_le(msg->header);
		request_chunk = msg->ext_msg.header & PD_EXT_HDR_REQ_CHUNK;

		if (chunked) {//6.2.1.2.2
			chunk_num =
				pd_ext_header_chunk_num(msg->ext_msg.header);
		}
		data_size = pd_ext_header_data_size_le(msg->ext_msg.header);
	}

	switch (port->chunk_rx_state) {
	case RCH_WAIT_FOR_MESSAGE:
		kfree(port->ext_rx_buf);
		port->ext_rx_buf = NULL;
		port->ext_rx_buf_size = 0;
		port->chunk_abort = false;
		break;
	case RCH_PASS_UP_MESSAGE:
		tcpm_set_chunk_state(port, RCH_WAIT_FOR_MESSAGE, 0);

		if (is_ext_msg)
			tcpm_pd_ext_msg_request(port, msg);
		else if (cnt)
			tcpm_pd_data_request(port, msg);
		else
			tcpm_pd_ctrl_request(port, msg);

		memset(&port->chunk_event->msg, 0,
		       sizeof(port->chunk_event->msg));
		break;
	case RCH_PROCESSING_EXTENDED_MESSAGE:
		if (port->chunk_abort) {
			tcpm_set_chunk_state(port, RCH_WAIT_FOR_MESSAGE, 0);
			memset(&port->chunk_event->msg, 0,
			       sizeof(port->chunk_event->msg));
			break;
		}

		if (chunk_num == 0) {
			port->chunk_event->chunk_num_expected = 0;
			port->chunk_event->num_bytes = 0;
			port->chunk_event->expected_type = 0;
			if (!port->ext_rx_buf) {
				port->ext_rx_buf =
						kzalloc(data_size, GFP_KERNEL);
				if (!port->ext_rx_buf) {
					tcpm_log(port,
						 "failed to alloc ext_rx_buf");
					tcpm_set_chunk_state(port,
							     RCH_REPORT_ERROR,
							     0);
					break;
				}
			}
		}

		if (chunk_num == port->chunk_event->chunk_num_expected) {
			u16 last_chunk_num = data_size / PD_EXT_MAX_CHUNK_DATA;
			u16 size_ext_msg = 0;
			u16 num_bytes = port->chunk_event->num_bytes;

			if (data_size <= PD_EXT_MAX_CHUNK_DATA) {
				size_ext_msg = data_size;
			} else if (chunk_num < last_chunk_num) {
				size_ext_msg = PD_EXT_MAX_CHUNK_DATA;
			} else if (chunk_num == last_chunk_num) {
				size_ext_msg = data_size -
					(PD_EXT_MAX_CHUNK_DATA * chunk_num);
			} else {
				tcpm_log(port, "rx_sm: data length is invalid");
				tcpm_set_chunk_state(port, RCH_REPORT_ERROR, 0);
			}

			memcpy(port->ext_rx_buf + num_bytes, msg->ext_msg.data,
			       size_ext_msg);
			port->chunk_event->num_bytes = num_bytes + size_ext_msg;
			port->ext_rx_buf_size = port->chunk_event->num_bytes;
			if (chunk_num < last_chunk_num)
				port->chunk_event->chunk_num_expected++;

		} else {
			tcpm_log(port, "rx_sm: chunk_num:%d isn't expected:%d",
				 chunk_num,
				 port->chunk_event->chunk_num_expected);
			tcpm_set_chunk_state(port, RCH_REPORT_ERROR, 0);
			break;
		}

		if (port->ext_rx_buf_size >= data_size) {
			tcpm_set_chunk_state(port, RCH_PASS_UP_MESSAGE, 0);
		} else {
			port->chunk_event->expected_type = ext_msg_type;
			memset(&port->chunk_event->msg, 0,
			       sizeof(port->chunk_event->msg));
			tcpm_set_chunk_state(port, RCH_REQUESTING_CHUNK, 0);
		}
		break;
	case RCH_REQUESTING_CHUNK:
	{
		/* Prepare chunk request */
		struct pd_message req_msg;
		enum pd_ext_msg_type ext_msg_type =
					port->chunk_event->expected_type;
		chunk_num_expected = port->chunk_event->chunk_num_expected;

		memset(&req_msg, 0, sizeof(req_msg));

		req_msg.header = PD_HEADER_LE(ext_msg_type,
					      port->pwr_role,
					      port->data_role,
					      port->negotiated_rev,
					      port->message_id, 1, 1);
		req_msg.ext_msg.header = PD_EXT_HDR_LE(0,/* data size */
						       1,/* req_chunk */
						       chunk_num_expected,
						       /* chunk_num */
						       1/* chunked */);
		mutex_lock(&port->lock);
		ret = tcpm_pd_transmit(port, TCPC_TX_SOP, &req_msg);
		mutex_unlock(&port->lock);
		if (ret == 0)
			tcpm_set_chunk_state(port, RCH_WAITING_CHUNK, 0);
		else
			tcpm_set_chunk_state(port, RCH_REPORT_ERROR, 0);
		break;
	}
	case RCH_WAITING_CHUNK:
		tcpm_set_chunk_state(port, RCH_REPORT_ERROR,
				     PD_T_CHUNK_SENDER_RESPONSE);
		break;
	case RCH_REPORT_ERROR:
		tcpm_set_chunk_state(port, RCH_WAIT_FOR_MESSAGE, 0);

		if (msg && port->complete_msg) {
			if (is_ext_msg)
				tcpm_pd_ext_msg_request(port, msg);
			else if (cnt)
				tcpm_pd_data_request(port, msg);
			else
				tcpm_pd_ctrl_request(port, msg);
		}
		memset(&port->chunk_event->msg, 0,
		       sizeof(port->chunk_event->msg));
		break;
	default:
		tcpm_log(port, "rx_sm: unknown chunk state");
		break;
	}

	return;
}

static void chunk_state_machine_work(struct work_struct *work)
{
	struct tcpm_port *port = container_of(work, struct tcpm_port,
					      chunk_state_machine.work);

	mutex_lock(&port->chunk_lock);

	if (port->is_chunk_tx) {
		if (port->delayed_chunk_state) {
			tcpm_log(port,
				 " Chunk state change %s -> %s [delayed %ld ms]",
				 tcpm_chunk_state[port->chunk_tx_state],
				 tcpm_chunk_state[port->delayed_chunk_state],
				 port->chunk_delay_ms);
			port->chunk_tx_state = port->delayed_chunk_state;
			port->delayed_chunk_state = INVALID_CHUNK_STATE;
		}
		run_chunked_tx_state_machine(port);
	} else {
		if (port->delayed_chunk_state) {
			tcpm_log(port,
				 " Chunk state change %s -> %s [delayed %ld ms]",
				 tcpm_chunk_state[port->chunk_rx_state],
				 tcpm_chunk_state[port->delayed_chunk_state],
				 port->chunk_delay_ms);
			port->chunk_rx_state = port->delayed_chunk_state;
			port->delayed_chunk_state = INVALID_CHUNK_STATE;
		}
		run_chunked_rx_state_machine(port);
	}

	mutex_unlock(&port->chunk_lock);
}

static void chunked_msg_router(struct tcpm_port *port,
					      const struct pd_message *msg)
{
	enum pd_ctrl_msg_type ctrl_type;
	u8 cnt = pd_header_cnt_le(msg->header);
	u16 data_size = 0;
	bool is_ext_msg = msg->header & PD_HEADER_EXT_HDR;
	bool chunked = false;

	mutex_lock(&port->chunk_lock);

	if (!is_ext_msg) {
		port->complete_msg = true;
	} else {
		data_size = pd_ext_header_data_size_le(msg->ext_msg.header);
		chunked = msg->ext_msg.header & PD_EXT_HDR_CHUNKED;
		if (data_size <= PD_EXT_MAX_CHUNK_DATA || chunked == 0)
			port->complete_msg = true;
		else
			port->complete_msg = false;
	}

	if (!cnt)
		ctrl_type = pd_header_type_le(msg->header);

	if (ctrl_type == PD_CTRL_PING) {
		/* Handle the PING message directly */
		tcpm_pd_ctrl_request(port, msg);
		goto done;
	} else {
		port->is_chunk_tx =
			(port->chunk_tx_state != TCH_WAIT_FOR_MESSAGE_REQUEST) ?
			true : false;
	}

	memset(&port->chunk_event->msg, 0, sizeof(port->chunk_event->msg));
	memcpy(&port->chunk_event->msg, msg, sizeof(*msg));

	if (port->is_chunk_tx)
		tcpm_tx_chunk_handler(port, &port->chunk_event->msg,
				      PROTOCOL_LAYER);
	else
		tcpm_rx_chunk_handler(port);

done:
	mutex_unlock(&port->chunk_lock);
	return;
}

static void tcpm_pd_rx_handler(struct work_struct *work)
{
	struct pd_rx_event *event = container_of(work,
						 struct pd_rx_event, work);
	const struct pd_message *msg = &event->msg;
	struct tcpm_port *port = event->port;

	mutex_lock(&port->lock);

	tcpm_log(port, "PD RX, header: %#x [%d]", le16_to_cpu(msg->header),
		 port->attached);

	if (port->attached) {
		enum pd_ctrl_msg_type type = pd_header_type_le(msg->header);
		unsigned int msgid = pd_header_msgid_le(msg->header);

		/*
		 * USB PD standard, 6.6.1.2:
		 * "... if MessageID value in a received Message is the
		 * same as the stored value, the receiver shall return a
		 * GoodCRC Message with that MessageID value and drop
		 * the Message (this is a retry of an already received
		 * Message). Note: this shall not apply to the Soft_Reset
		 * Message which always has a MessageID value of zero."
		 */
		if (msgid == port->rx_msgid && type != PD_CTRL_SOFT_RESET)
			goto done;
		port->rx_msgid = msgid;

		/*
		 * If both ends believe to be DFP/host, we have a data role
		 * mismatch.
		 */
		if (event->packet == SOP && !!(le16_to_cpu(msg->header) &
		    PD_HEADER_DATA_ROLE) == (port->data_role == TYPEC_HOST)) {
			tcpm_log(port,
				 "Data role mismatch, initiating error recovery");
			tcpm_set_state(port, ERROR_RECOVERY, 0);
		} else {
			//FIXME: make property to switch mechanism
			chunked_msg_router(port, msg);
		}
	}

done:
	mutex_unlock(&port->lock);
	kfree(event);
}

void tcpm_pd_receive(struct tcpm_port *port, const struct pd_message *msg,
		     enum usb_pd_sop_type packet)
{
	struct pd_rx_event *event;

	event = kzalloc(sizeof(*event), GFP_ATOMIC);
	if (!event)
		return;

	INIT_WORK(&event->work, tcpm_pd_rx_handler);
	event->port = port;
	event->packet = packet;
	memcpy(&event->msg, msg, sizeof(*msg));
	queue_work(port->wq, &event->work);
}
EXPORT_SYMBOL_GPL(tcpm_pd_receive);

static int tcpm_pd_send_control(struct tcpm_port *port,
				enum pd_ctrl_msg_type type)
{
	struct pd_message msg;

	memset(&msg, 0, sizeof(msg));
	msg.header = PD_HEADER_LE(type, port->pwr_role,
				  port->data_role,
				  port->negotiated_rev,
				  port->message_id, 0, 0);

	return tcpm_chunk_transmit(port, TCPC_TX_SOP, &msg);
}

static int tcpm_pd_send_extend(struct tcpm_port *port)
{
	struct pd_message msg;

	memset(&msg, 0, sizeof(msg));
	msg.header = PD_HEADER_LE(port->ext_type,
				  port->pwr_role,
				  port->data_role,
				  port->negotiated_rev,
				  port->message_id, 0, 1);
	msg.ext_msg.header = PD_EXT_HDR_LE(
			port->ext_tx_buf_size,/* data size */
			0,/* req_chunk: don't care here */
			0,/* chunk_num: don't care here */
			1/* chunked */);

	return tcpm_chunk_transmit(port, TCPC_TX_SOP, &msg);
}

/*
 * Send queued message without affecting state.
 * Return true if state machine should go back to sleep,
 * false otherwise.
 */
static bool tcpm_send_queued_message(struct tcpm_port *port)
{
	enum pd_msg_request queued_message;
	int ret;

	do {
		queued_message = port->queued_message;
		port->queued_message = PD_MSG_NONE;

		switch (queued_message) {
		case PD_MSG_CTRL_WAIT:
			tcpm_pd_send_control(port, PD_CTRL_WAIT);
			break;
		case PD_MSG_CTRL_REJECT:
			tcpm_pd_send_control(port, PD_CTRL_REJECT);
			break;
		case PD_MSG_CTRL_NOT_SUPP:
			tcpm_pd_send_control(port, PD_CTRL_NOT_SUPP);
			break;
		case PD_MSG_DATA_SINK_CAP:
			ret = tcpm_pd_send_sink_caps(port);
			if (ret < 0) {
				tcpm_log(port,
					 "Unable to send snk caps, ret=%d",
					 ret);
				tcpm_set_state(port, SNK_SOFT_RESET, 0);
			}
			tcpm_ams_finish(port);
			break;
		case PD_MSG_DATA_SOURCE_CAP:
			ret = tcpm_pd_send_source_caps(port);
			if (ret < 0) {
				tcpm_log(port,
					 "Unable to send src caps, ret=%d",
					 ret);
				tcpm_set_state(port, SOFT_RESET_SEND, 0);
			} else if (port->pwr_role == TYPEC_SOURCE) {
				tcpm_ams_finish(port);
				tcpm_set_state(port, HARD_RESET_SEND,
					       PD_T_SENDER_RESPONSE);
			} else {
				tcpm_ams_finish(port);
			}
			break;
		default:
			break;
		}
	} while (port->queued_message != PD_MSG_NONE);

	if (port->delayed_state != INVALID_STATE) {
		if (time_is_after_jiffies(port->delayed_runtime)) {
			mod_delayed_work(port->wq, &port->state_machine,
					 port->delayed_runtime - jiffies);
			return true;
		}
		port->delayed_state = INVALID_STATE;
	}
	return false;
}

static int tcpm_pd_check_request(struct tcpm_port *port)
{
	u32 pdo, rdo = port->sink_request;
	unsigned int max, op, pdo_max, index;
	enum pd_pdo_type type;

	index = rdo_index(rdo);
	if (!index || index > port->nr_src_pdo)
		return -EINVAL;

	pdo = port->src_pdo[index - 1];
	type = pdo_type(pdo);
	switch (type) {
	case PDO_TYPE_FIXED:
	case PDO_TYPE_VAR:
		max = rdo_max_current(rdo);
		op = rdo_op_current(rdo);
		pdo_max = pdo_max_current(pdo);

		if (op > pdo_max)
			return -EINVAL;
		if (max > pdo_max && !(rdo & RDO_CAP_MISMATCH))
			return -EINVAL;

		if (type == PDO_TYPE_FIXED)
			tcpm_log(port,
				 "Requested %u mV, %u mA for %u / %u mA",
				 pdo_fixed_voltage(pdo), pdo_max, op, max);
		else
			tcpm_log(port,
				 "Requested %u -> %u mV, %u mA for %u / %u mA",
				 pdo_min_voltage(pdo), pdo_max_voltage(pdo),
				 pdo_max, op, max);
		break;
	case PDO_TYPE_BATT:
		max = rdo_max_power(rdo);
		op = rdo_op_power(rdo);
		pdo_max = pdo_max_power(pdo);

		if (op > pdo_max)
			return -EINVAL;
		if (max > pdo_max && !(rdo & RDO_CAP_MISMATCH))
			return -EINVAL;
		tcpm_log(port,
			 "Requested %u -> %u mV, %u mW for %u / %u mW",
			 pdo_min_voltage(pdo), pdo_max_voltage(pdo),
			 pdo_max, op, max);
		break;
	default:
		return -EINVAL;
	}

	port->op_vsafe5v = index == 1;

	return 0;
}

#define min_power(x, y) min(pdo_max_power(x), pdo_max_power(y))
#define min_current(x, y) min(pdo_max_current(x), pdo_max_current(y))

static int tcpm_pd_select_pdo(struct tcpm_port *port, int *sink_pdo,
			      int *src_pdo)
{
	struct tcpc_dev *tcpc = port->tcpc;
	unsigned int i, j, max_src_mv = 0, min_src_mv = 0, max_mw = 0,
		     max_mv = 0, src_mw = 0, src_ma = 0, max_snk_mv = 0,
		     min_snk_mv = 0;
	int ret = -EINVAL;
	int fixed_5V3A = 1;

	port->pps_data.supported = false;
	port->usb_type = POWER_SUPPLY_USB_TYPE_PD;

	if (tcpc->fixed_5V3A)
		port->nr_snk_pdo = fixed_5V3A;

	/*
	 * Select the source PDO providing the most power which has a
	 * matchig sink cap.
	 */
	for (i = 0; i < port->nr_source_caps; i++) {
		u32 pdo = port->source_caps[i];
		enum pd_pdo_type type = pdo_type(pdo);

		switch (type) {
		case PDO_TYPE_FIXED:
			max_src_mv = pdo_fixed_voltage(pdo);
			min_src_mv = max_src_mv;
			break;
		case PDO_TYPE_BATT:
		case PDO_TYPE_VAR:
			max_src_mv = pdo_max_voltage(pdo);
			min_src_mv = pdo_min_voltage(pdo);
			break;
		case PDO_TYPE_APDO:
			if (pdo_apdo_type(pdo) == APDO_TYPE_PPS) {
				port->pps_data.supported = true;
				port->usb_type =
					POWER_SUPPLY_USB_TYPE_PD_PPS;
			}
			continue;
		default:
			tcpm_log(port, "Invalid source PDO type, ignoring");
			continue;
		}

		switch (type) {
		case PDO_TYPE_FIXED:
		case PDO_TYPE_VAR:
			src_ma = pdo_max_current(pdo);
			src_mw = src_ma * min_src_mv / 1000;
			break;
		case PDO_TYPE_BATT:
			src_mw = pdo_max_power(pdo);
			break;
		case PDO_TYPE_APDO:
			continue;
		default:
			tcpm_log(port, "Invalid source PDO type, ignoring");
			continue;
		}

		for (j = 0; j < port->nr_snk_pdo; j++) {
			pdo = port->snk_pdo[j];

			switch (pdo_type(pdo)) {
			case PDO_TYPE_FIXED:
				max_snk_mv = pdo_fixed_voltage(pdo);
				min_snk_mv = max_snk_mv;
				break;
			case PDO_TYPE_BATT:
			case PDO_TYPE_VAR:
				max_snk_mv = pdo_max_voltage(pdo);
				min_snk_mv = pdo_min_voltage(pdo);
				break;
			case PDO_TYPE_APDO:
				continue;
			default:
				tcpm_log(port, "Invalid sink PDO type, ignoring");
				continue;
			}

			if (max_src_mv <= max_snk_mv &&
				min_src_mv >= min_snk_mv) {
				/* Prefer higher voltages if available */
				if ((src_mw == max_mw && min_src_mv > max_mv) ||
							src_mw > max_mw) {
					*src_pdo = i;
					*sink_pdo = j;
					max_mw = src_mw;
					max_mv = min_src_mv;
					ret = 0;
				}
			}
		}
	}

	return ret;
}

#define min_pps_apdo_current(x, y)	\
	min(pdo_pps_apdo_max_current(x), pdo_pps_apdo_max_current(y))

static unsigned int tcpm_pd_select_pps_apdo(struct tcpm_port *port)
{
	unsigned int i, j, max_mw = 0, max_mv = 0;
	unsigned int min_src_mv, max_src_mv, src_ma, src_mw;
	unsigned int min_snk_mv, max_snk_mv;
	unsigned int max_op_mv;
	u32 pdo, src, snk;
	unsigned int src_pdo = 0, snk_pdo = 0;

	/*
	 * Select the source PPS APDO providing the most power while staying
	 * within the board's limits. We skip the first PDO as this is always
	 * 5V 3A.
	 */
	for (i = 1; i < port->nr_source_caps; ++i) {
		pdo = port->source_caps[i];

		switch (pdo_type(pdo)) {
		case PDO_TYPE_APDO:
			if (pdo_apdo_type(pdo) != APDO_TYPE_PPS) {
				tcpm_log(port, "Not PPS APDO (source), ignoring");
				continue;
			}

			min_src_mv = pdo_pps_apdo_min_voltage(pdo);
			max_src_mv = pdo_pps_apdo_max_voltage(pdo);
			src_ma = pdo_pps_apdo_max_current(pdo);
			src_mw = (src_ma * max_src_mv) / 1000;

			/*
			 * Now search through the sink PDOs to find a matching
			 * PPS APDO. Again skip the first sink PDO as this will
			 * always be 5V 3A.
			 */
			for (j = 1; j < port->nr_snk_pdo; j++) {
				pdo = port->snk_pdo[j];

				switch (pdo_type(pdo)) {
				case PDO_TYPE_APDO:
					if (pdo_apdo_type(pdo) != APDO_TYPE_PPS) {
						tcpm_log(port,
							 "Not PPS APDO (sink), ignoring");
						continue;
					}

					min_snk_mv =
						pdo_pps_apdo_min_voltage(pdo);
					max_snk_mv =
						pdo_pps_apdo_max_voltage(pdo);
					break;
				default:
					tcpm_log(port,
						 "Not APDO type (sink), ignoring");
					continue;
				}

				if (min_src_mv <= max_snk_mv &&
				    max_src_mv >= min_snk_mv) {
					max_op_mv = min(max_src_mv, max_snk_mv);
					src_mw = (max_op_mv * src_ma) / 1000;
					/* Prefer higher voltages if available */
					if ((src_mw == max_mw &&
					     max_op_mv > max_mv) ||
					    src_mw > max_mw) {
						src_pdo = i;
						snk_pdo = j;
						max_mw = src_mw;
						max_mv = max_op_mv;
					}
				}
			}

			break;
		default:
			tcpm_log(port, "Not APDO type (source), ignoring");
			continue;
		}
	}

	if (src_pdo) {
		src = port->source_caps[src_pdo];
		snk = port->snk_pdo[snk_pdo];

		port->pps_data.req_min_volt = max(pdo_pps_apdo_min_voltage(src),
						  pdo_pps_apdo_min_voltage(snk));
		port->pps_data.req_max_volt = min(pdo_pps_apdo_max_voltage(src),
						  pdo_pps_apdo_max_voltage(snk));
		port->pps_data.req_max_curr = min_pps_apdo_current(src, snk);
		port->pps_data.req_out_volt = min(port->pps_data.req_max_volt,
						  max(port->pps_data.req_min_volt,
						      port->pps_data.req_out_volt));
		port->pps_data.req_op_curr = min(port->pps_data.req_max_curr,
						 port->pps_data.req_op_curr);
	}

	return src_pdo;
}

static int tcpm_pd_build_request(struct tcpm_port *port, u32 *rdo)
{
	unsigned int mv, ma, mw, flags;
	unsigned int max_ma, max_mw;
	enum pd_pdo_type type;
	u32 pdo, matching_snk_pdo;
	int src_pdo_index = 0;
	int snk_pdo_index = 0;
	int ret;

	ret = tcpm_pd_select_pdo(port, &snk_pdo_index, &src_pdo_index);
	if (ret < 0)
		return ret;

	pdo = port->source_caps[src_pdo_index];
	matching_snk_pdo = port->snk_pdo[snk_pdo_index];
	type = pdo_type(pdo);

	switch (type) {
	case PDO_TYPE_FIXED:
		mv = pdo_fixed_voltage(pdo);
		break;
	case PDO_TYPE_BATT:
	case PDO_TYPE_VAR:
		mv = pdo_min_voltage(pdo);
		break;
	default:
		tcpm_log(port, "Invalid PDO selected!");
		return -EINVAL;
	}

	/* Select maximum available current within the sink pdo's limit */
	if (type == PDO_TYPE_BATT) {
		mw = min_power(pdo, matching_snk_pdo);
		ma = 1000 * mw / mv;
	} else {
		ma = min_current(pdo, matching_snk_pdo);
		mw = ma * mv / 1000;
	}

	flags = RDO_USB_COMM | RDO_NO_SUSPEND;

	/* Set mismatch bit if offered power is less than operating power */
	max_ma = ma;
	max_mw = mw;
	if (mw < port->operating_snk_mw) {
		flags |= RDO_CAP_MISMATCH;
		if (type == PDO_TYPE_BATT &&
		    (pdo_max_power(matching_snk_pdo) > pdo_max_power(pdo)))
			max_mw = pdo_max_power(matching_snk_pdo);
		else if (pdo_max_current(matching_snk_pdo) >
			 pdo_max_current(pdo))
			max_ma = pdo_max_current(matching_snk_pdo);
	}

	tcpm_log(port, "cc=%d cc1=%d cc2=%d vbus=%d vconn=%s polarity=%d",
		 port->cc_req, port->cc1, port->cc2, port->vbus_source,
		 port->vconn_role == TYPEC_SOURCE ? "source" : "sink",
		 port->polarity);

	if (type == PDO_TYPE_BATT) {
		*rdo = RDO_BATT(src_pdo_index + 1, mw, max_mw, flags);

		tcpm_log(port, "Requesting PDO %d: %u mV, %u mW%s",
			 src_pdo_index, mv, mw,
			 flags & RDO_CAP_MISMATCH ? " [mismatch]" : "");
	} else {
		*rdo = RDO_FIXED(src_pdo_index + 1, ma, max_ma, flags);

		tcpm_log(port, "Requesting PDO %d: %u mV, %u mA%s",
			 src_pdo_index, mv, ma,
			 flags & RDO_CAP_MISMATCH ? " [mismatch]" : "");
	}

	port->req_current_limit = ma;
	port->req_supply_voltage = mv;

	return 0;
}

static int tcpm_pd_send_request(struct tcpm_port *port)
{
	struct pd_message msg;
	int ret;
	u32 rdo;

	ret = tcpm_pd_build_request(port, &rdo);
	if (ret < 0)
		return ret;

	memset(&msg, 0, sizeof(msg));
	msg.header = PD_HEADER_LE(PD_DATA_REQUEST,
				  port->pwr_role,
				  port->data_role,
				  port->negotiated_rev,
				  port->message_id, 1, 0);
	msg.payload[0] = cpu_to_le32(rdo);

	return tcpm_chunk_transmit(port, TCPC_TX_SOP, &msg);
}

static int tcpm_pd_build_pps_request(struct tcpm_port *port, u32 *rdo)
{
	unsigned int out_mv, op_ma, op_mw, max_mv, max_ma, flags;
	enum pd_pdo_type type;
	unsigned int src_pdo_index;
	u32 pdo;

	src_pdo_index = tcpm_pd_select_pps_apdo(port);
	if (!src_pdo_index)
		return -EOPNOTSUPP;

	pdo = port->source_caps[src_pdo_index];
	type = pdo_type(pdo);

	switch (type) {
	case PDO_TYPE_APDO:
		if (pdo_apdo_type(pdo) != APDO_TYPE_PPS) {
			tcpm_log(port, "Invalid APDO selected!");
			return -EINVAL;
		}
		max_mv = port->pps_data.req_max_volt;
		max_ma = port->pps_data.req_max_curr;
		out_mv = port->pps_data.req_out_volt;
		op_ma = port->pps_data.req_op_curr;
		break;
	default:
		tcpm_log(port, "Invalid PDO selected!");
		return -EINVAL;
	}

	flags = RDO_USB_COMM | RDO_NO_SUSPEND;

	op_mw = (op_ma * out_mv) / 1000;
	if (op_mw < port->operating_snk_mw) {
		/*
		 * Try raising current to meet power needs. If that's not enough
		 * then try upping the voltage. If that's still not enough
		 * then we've obviously chosen a PPS APDO which really isn't
		 * suitable so abandon ship.
		 */
		op_ma = (port->operating_snk_mw * 1000) / out_mv;
		if ((port->operating_snk_mw * 1000) % out_mv)
			++op_ma;
		op_ma += RDO_PROG_CURR_MA_STEP - (op_ma % RDO_PROG_CURR_MA_STEP);

		if (op_ma > max_ma) {
			op_ma = max_ma;
			out_mv = (port->operating_snk_mw * 1000) / op_ma;
			if ((port->operating_snk_mw * 1000) % op_ma)
				++out_mv;
			out_mv += RDO_PROG_VOLT_MV_STEP -
				  (out_mv % RDO_PROG_VOLT_MV_STEP);

			if (out_mv > max_mv) {
				tcpm_log(port, "Invalid PPS APDO selected!");
				return -EINVAL;
			}
		}
	}

	tcpm_log(port, "cc=%d cc1=%d cc2=%d vbus=%d vconn=%s polarity=%d",
		 port->cc_req, port->cc1, port->cc2, port->vbus_source,
		 port->vconn_role == TYPEC_SOURCE ? "source" : "sink",
		 port->polarity);

	*rdo = RDO_PROG(src_pdo_index + 1, out_mv, op_ma, flags);

	tcpm_log(port, "Requesting APDO %d: %u mV, %u mA",
		 src_pdo_index, out_mv, op_ma);

	port->pps_data.req_op_curr = op_ma;
	port->pps_data.req_out_volt = out_mv;

	return 0;
}

static int tcpm_pd_send_pps_request(struct tcpm_port *port)
{
	struct pd_message msg;
	int ret;
	u32 rdo;

	ret = tcpm_pd_build_pps_request(port, &rdo);
	if (ret < 0)
		return ret;

	memset(&msg, 0, sizeof(msg));
	msg.header = PD_HEADER_LE(PD_DATA_REQUEST,
				  port->pwr_role,
				  port->data_role,
				  port->negotiated_rev,
				  port->message_id, 1, 0);
	msg.payload[0] = cpu_to_le32(rdo);

	return tcpm_chunk_transmit(port, TCPC_TX_SOP, &msg);
}

static int tcpm_set_vbus(struct tcpm_port *port, bool enable)
{
	int ret;

	if (enable && port->vbus_charge)
		return -EINVAL;

	tcpm_log(port, "vbus:=%d charge=%d", enable, port->vbus_charge);

	ret = port->tcpc->set_vbus(port->tcpc, enable, port->vbus_charge);
	if (ret < 0)
		return ret;

	port->vbus_source = enable;
	return 0;
}

static int tcpm_set_charge(struct tcpm_port *port, bool charge)
{
	int ret;

	if (charge && port->vbus_source)
		return -EINVAL;

	if (charge != port->vbus_charge) {
		tcpm_log(port, "vbus=%d charge:=%d", port->vbus_source, charge);
		ret = port->tcpc->set_vbus(port->tcpc, port->vbus_source,
					   charge);
		if (ret < 0)
			return ret;
	}
	port->vbus_charge = charge;
	return 0;
}

static bool tcpm_start_toggling(struct tcpm_port *port, enum typec_cc_status cc)
{
	int ret;

	if (!port->tcpc->start_toggling)
		return false;

	tcpm_log_force(port, "Start toggling");
	ret = port->tcpc->start_toggling(port->tcpc, port->port_type, cc);
	return ret == 0;
}

static int tcpm_init_vbus(struct tcpm_port *port)
{
	int ret;

	ret = port->tcpc->set_vbus(port->tcpc, false, false);
	port->vbus_source = false;
	port->vbus_charge = false;
	return ret;
}

static int tcpm_init_vconn(struct tcpm_port *port)
{
	int ret;

	ret = port->tcpc->set_vconn(port->tcpc, false);
	port->vconn_role = TYPEC_SINK;
	return ret;
}

static void tcpm_typec_connect(struct tcpm_port *port)
{
	if (!port->connected) {
		/* Make sure we don't report stale identity information */
		memset(&port->partner_ident, 0, sizeof(port->partner_ident));
		port->partner_desc.usb_pd = port->pd_capable;
		if (tcpm_port_is_debug(port))
			port->partner_desc.accessory = TYPEC_ACCESSORY_DEBUG;
		else if (tcpm_port_is_audio(port))
			port->partner_desc.accessory = TYPEC_ACCESSORY_AUDIO;
		else
			port->partner_desc.accessory = TYPEC_ACCESSORY_NONE;
		port->partner = typec_register_partner(port->typec_port,
						       &port->partner_desc);
		port->connected = true;
	}
}

static int tcpm_src_attach(struct tcpm_port *port)
{
	enum typec_cc_polarity polarity =
				port->cc2 == TYPEC_CC_RD ? TYPEC_POLARITY_CC2
							 : TYPEC_POLARITY_CC1;
	int ret;

	if (port->attached)
		return 0;

	ret = tcpm_set_polarity(port, polarity);
	if (ret < 0)
		return ret;

	ret = tcpm_set_roles(port, true, TYPEC_SOURCE, TYPEC_HOST);
	if (ret < 0)
		return ret;

	ret = port->tcpc->set_pd_rx(port->tcpc, true,
				    FRAME_FILTER_EN_SOP |
				    FRAME_FILTER_EN_HARD_RESET);
	if (ret < 0)
		goto out_disable_mux;

	/*
	 * USB Type-C specification, version 1.2,
	 * chapter 4.5.2.2.8.1 (Attached.SRC Requirements)
	 * Enable VCONN only if the non-RD port is set to RA.
	 */
	if ((polarity == TYPEC_POLARITY_CC1 && port->cc2 == TYPEC_CC_RA) ||
	    (polarity == TYPEC_POLARITY_CC2 && port->cc1 == TYPEC_CC_RA)) {
		ret = tcpm_set_vconn(port, true);
		if (ret < 0)
			goto out_disable_pd;
	}

	ret = tcpm_set_vbus(port, true);
	if (ret < 0)
		goto out_disable_vconn;

	tcpm_set_pd_capable(port, false);

	port->partner = NULL;

	port->attached = true;
	port->send_discover = true;

	return 0;

out_disable_vconn:
	tcpm_set_vconn(port, false);
out_disable_pd:
	port->tcpc->set_pd_rx(port->tcpc, false, FRAME_FILTER_EN_SOP |
			      FRAME_FILTER_EN_HARD_RESET);
out_disable_mux:
	tcpm_mux_set(port, TYPEC_STATE_SAFE, USB_ROLE_NONE,
		     TYPEC_ORIENTATION_NONE);
	return ret;
}

static void tcpm_typec_disconnect(struct tcpm_port *port)
{
	if (port->connected) {
		typec_unregister_partner(port->partner);
		port->partner = NULL;
		port->connected = false;
	}
}

static void tcpm_unregister_altmodes(struct tcpm_port *port)
{
	struct pd_mode_data *modep = &port->mode_data;
	int i;

	for (i = 0; i < modep->altmodes; i++) {
		typec_unregister_altmode(port->partner_altmode[i]);
		port->partner_altmode[i] = NULL;
	}

	memset(modep, 0, sizeof(*modep));
}

static void tcpm_reset_port(struct tcpm_port *port)
{
	port->in_ams = false;
	port->ams = NONE_AMS;
	port->vdm_sm_running = false;
	tcpm_unregister_altmodes(port);
	tcpm_typec_disconnect(port);
	port->attached = false;
	tcpm_set_pd_capable(port, false);
	port->pps_data.supported = false;
	port->usb_comm_capable = false;

	port->chunk_rx_state = RCH_WAIT_FOR_MESSAGE;
	port->chunk_tx_state = TCH_WAIT_FOR_MESSAGE_REQUEST;
	kfree(port->ext_rx_buf);
	port->ext_rx_buf = NULL;
	port->ext_rx_buf_size = 0;
	port->ext_type = 0;
	port->chunk_abort = false;
	port->chunk_num_to_send = 0;
	kfree(port->ext_tx_buf);
	port->ext_tx_buf = NULL;
	port->ext_tx_buf_size = 0;

	/*
	 * First Rx ID should be 0; set this to a sentinel of -1 so that
	 * we can check tcpm_pd_rx_handler() if we had seen it before.
	 */
	port->rx_msgid = -1;

	port->tcpc->set_pd_rx(port->tcpc, false, FRAME_FILTER_EN_HARD_RESET |
			      FRAME_FILTER_EN_SOP);
	tcpm_init_vbus(port);	/* also disables charging */
	tcpm_init_vconn(port);
	tcpm_set_current_limit(port, 0, 0);
	tcpm_set_polarity(port, TYPEC_POLARITY_CC1);
	tcpm_mux_set(port, TYPEC_STATE_SAFE, USB_ROLE_NONE,
		     TYPEC_ORIENTATION_NONE);
	tcpm_set_attached_state(port, false);
	port->try_src_count = 0;
	port->try_snk_count = 0;
	port->usb_type = POWER_SUPPLY_USB_TYPE_C;
	port->vpd_connected = false;
}

static void tcpm_detach(struct tcpm_port *port)
{
	if (tcpm_port_is_disconnected(port))
		port->hard_reset_count = 0;

	if (!port->attached)
		return;

	tcpm_reset_port(port);
}

static void tcpm_src_detach(struct tcpm_port *port)
{
	tcpm_detach(port);
}

static int tcpm_snk_attach(struct tcpm_port *port)
{
	int ret;

	if (port->attached)
		return 0;

	ret = tcpm_set_polarity(port, port->cc2 != TYPEC_CC_OPEN ?
				TYPEC_POLARITY_CC2 : TYPEC_POLARITY_CC1);
	if (ret < 0)
		return ret;

	ret = tcpm_set_roles(port, true, TYPEC_SINK, TYPEC_DEVICE);
	if (ret < 0)
		return ret;

	tcpm_set_pd_capable(port, false);

	port->partner = NULL;

	port->attached = true;
	port->send_discover = true;

	return 0;
}

static void tcpm_snk_detach(struct tcpm_port *port)
{
	/* Conservatively reset back to suspend should be honored */
	if (port->tcpc->set_suspend_supported)
		port->tcpc->set_suspend_supported(port->tcpc, true);
	tcpm_detach(port);
}

static int tcpm_acc_attach(struct tcpm_port *port)
{
	int ret;

	if (port->attached)
		return 0;

	ret = tcpm_set_roles(port, true, TYPEC_SOURCE, TYPEC_HOST);
	if (ret < 0)
		return ret;

	port->partner = NULL;

	tcpm_typec_connect(port);

	port->attached = true;

	return 0;
}

static void tcpm_acc_detach(struct tcpm_port *port)
{
	tcpm_detach(port);
}

static inline enum tcpm_state hard_reset_state(struct tcpm_port *port)
{
	if (port->hard_reset_count < PD_N_HARD_RESET_COUNT)
		return HARD_RESET_SEND;
	if (port->pd_capable)
		return ERROR_RECOVERY;
	if (port->pwr_role == TYPEC_SOURCE)
		return SRC_UNATTACHED;
	if (port->state == SNK_WAIT_CAPABILITIES)
		return SNK_READY;
	return SNK_UNATTACHED;
}

static inline enum tcpm_state unattached_state(struct tcpm_port *port)
{
	if (port->port_type == TYPEC_PORT_DRP) {
		if (port->pwr_role == TYPEC_SOURCE)
			return SRC_UNATTACHED;
		else
			return SNK_UNATTACHED;
	} else if (port->port_type == TYPEC_PORT_SRC) {
		return SRC_UNATTACHED;
	}

	return SNK_UNATTACHED;
}

static void tcpm_check_send_discover(struct tcpm_port *port)
{
	if ((port->data_role == TYPEC_HOST || port->negotiated_rev > PD_REV20)
	     && port->send_discover && port->pd_capable) {
		tcpm_send_vdm(port, USB_SID_PD, CMD_DISCOVER_IDENT, NULL, 0,
			      TCPC_TX_SOP);
	/*
	 * From: 4.10.2 VCONN-Powered USB Devices (VPDs)
	 * A VCONN-powered USB Device shall only respond to USB PD messaging
	 * on SOP’.
	 */
	} else if (!port->pd_capable && port->vconn_role == TYPEC_SOURCE &&
		   port->data_role == TYPEC_HOST &&
		   port->pwr_role == TYPEC_SOURCE) {
		port->vdm_sm_running = true;
		port->tcpc->set_pd_rx(port->tcpc, true,
				      FRAME_FILTER_EN_HARD_RESET |
				      FRAME_FILTER_EN_SOP |
				      FRAME_FILTER_EN_SOPI);
		/* PD 3.0 discover identity */
		tcpm_send_vdm(port, USB_SID_PD, CMD_DISCOVER_IDENT,
			      NULL, 0, TCPC_TX_SOP_PRIME);
	}
}

static void disc_id_work(struct work_struct *work)
{
	struct tcpm_port *port = container_of(work, struct tcpm_port,
					      disc_id_work.work);

	if ((port->data_role == TYPEC_HOST || port->negotiated_rev > PD_REV20)
	    && port->send_discover && port->pd_capable) {
		tcpm_check_send_discover(port);
		mod_delayed_work(port->wq, &port->disc_id_work,
				 msecs_to_jiffies(DISC_ID_RETRY_MS));
	}
}

static void tcpm_swap_complete(struct tcpm_port *port, int result)
{
	if (port->swap_pending) {
		port->swap_status = result;
		port->swap_pending = false;
		port->non_pd_role_swap = false;
		complete(&port->swap_complete);
	}
}

static enum typec_pwr_opmode tcpm_get_pwr_opmode(enum typec_cc_status cc)
{
	switch (cc) {
	case TYPEC_CC_RP_1_5:
		return TYPEC_PWR_MODE_1_5A;
	case TYPEC_CC_RP_3_0:
		return TYPEC_PWR_MODE_3_0A;
	case TYPEC_CC_RP_DEF:
	default:
		return TYPEC_PWR_MODE_USB;
	}
}

static void run_state_machine(struct tcpm_port *port)
{
	int ret;
	enum typec_pwr_opmode opmode;
	unsigned int msecs;
	enum tcpm_state upcoming_state;

	port->enter_state = port->state;
	switch (port->state) {
	case TOGGLING:
		tcpm_log(port, "in PR_SWAP := false");
		port->tcpc->set_in_pr_swap(port->tcpc, false);
		break;
	/* SRC states */
	case SRC_UNATTACHED:
		tcpm_port_in_hard_reset(port, false);
		if (!port->non_pd_role_swap)
			tcpm_swap_complete(port, -ENOTCONN);
		tcpm_src_detach(port);
		if (tcpm_start_toggling(port, tcpm_rp_cc(port))) {
			tcpm_set_state(port, TOGGLING, 0);
			break;
		}
		tcpm_set_cc(port, tcpm_rp_cc(port));
		if (port->port_type == TYPEC_PORT_DRP)
			tcpm_set_state(port, SNK_UNATTACHED, PD_T_DRP_SNK);
		break;
	case SRC_ATTACH_WAIT:
		if (tcpm_port_is_debug(port))
			tcpm_set_state(port, DEBUG_ACC_ATTACHED,
				       PD_T_CC_DEBOUNCE);
		else if (tcpm_port_is_audio(port))
			tcpm_set_state(port, AUDIO_ACC_ATTACHED,
				       PD_T_CC_DEBOUNCE);
		else if (tcpm_port_is_source(port) && port->vbus_vsafe0v)
			tcpm_set_state(port,
				       tcpm_try_snk(port) ? SNK_TRY
							  : SRC_ATTACHED,
				       PD_T_CC_DEBOUNCE);
		break;

	case SNK_TRY:
		port->try_snk_count++;
		/*
		 * Requirements:
		 * - Do not drive vconn or vbus
		 * - Terminate CC pins (both) to Rd
		 * Action:
		 * - Wait for tDRPTry (PD_T_DRP_TRY).
		 *   Until then, ignore any state changes.
		 */
		tcpm_set_cc(port, TYPEC_CC_RD);
		tcpm_set_state(port, SNK_TRY_WAIT, PD_T_DRP_TRY);
		break;
	case SNK_TRY_WAIT:
		if (tcpm_port_is_sink(port)) {
			tcpm_set_state(port, SNK_TRY_WAIT_DEBOUNCE, 0);
		} else {
			tcpm_set_state(port, SRC_TRYWAIT, 0);
			port->max_wait = 0;
		}
		break;
	case SNK_TRY_WAIT_DEBOUNCE:
		tcpm_set_state(port, SNK_TRY_WAIT_DEBOUNCE_CHECK_VBUS,
			       PD_T_PD_DEBOUNCE);
		break;
	case SNK_TRY_WAIT_DEBOUNCE_CHECK_VBUS:
		if (port->vbus_present && tcpm_port_is_sink(port)) {
			tcpm_set_state(port, SNK_ATTACHED, 0);
		} else {
			tcpm_set_state(port, SRC_TRYWAIT, 0);
			port->max_wait = 0;
		}
		break;
	case SRC_TRYWAIT:
		tcpm_set_cc(port, tcpm_rp_cc(port));
		if (port->max_wait == 0) {
			port->max_wait = jiffies +
					 msecs_to_jiffies(PD_T_DRP_TRY);
			tcpm_set_state(port, SRC_TRYWAIT_UNATTACHED,
				       PD_T_DRP_TRY);
		} else {
			if (time_is_after_jiffies(port->max_wait))
				tcpm_set_state(port, SRC_TRYWAIT_UNATTACHED,
					       jiffies_to_msecs(port->max_wait -
								jiffies));
			else
				tcpm_set_state(port, SNK_UNATTACHED, 0);
		}
		break;
	case SRC_TRYWAIT_DEBOUNCE:
		tcpm_set_state(port, SRC_ATTACHED, PD_T_CC_DEBOUNCE);
		break;
	case SRC_TRYWAIT_UNATTACHED:
		tcpm_set_state(port, SNK_UNATTACHED, 0);
		break;

	case SRC_ATTACHED:
		ret = tcpm_src_attach(port);
		tcpm_set_state(port, SRC_UNATTACHED,
			       ret < 0 ? 0 : PD_T_PS_SOURCE_ON);
		break;
	case SRC_STARTUP:
		opmode =  tcpm_get_pwr_opmode(tcpm_rp_cc(port));
		typec_set_pwr_opmode(port->typec_port, opmode);
		port->pwr_opmode = TYPEC_PWR_MODE_USB;
		port->caps_count = 0;
		port->negotiated_rev = PD_MAX_REV;
		port->svdm_version = SVDM_MAX_VER;
		port->message_id = 0;
		port->rx_msgid = -1;
		port->explicit_contract = false;
		/* SNK -> SRC POWER/FAST_ROLE_SWAP finished */
		if (port->ams == POWER_ROLE_SWAP ||
		    port->ams == FAST_ROLE_SWAP)
			tcpm_ams_finish(port);
		port->upcoming_state = SRC_SEND_CAPABILITIES;
		tcpm_ams_start(port, POWER_NEGOTIATION);
		break;
	case SRC_SEND_CAPABILITIES:
		port->caps_count++;
		if (port->caps_count > PD_N_CAPS_COUNT) {
			tcpm_set_state(port, SRC_READY, 0);
			break;
		}
		ret = tcpm_pd_send_source_caps(port);
		if (ret < 0) {
			tcpm_set_state(port, SRC_SEND_CAPABILITIES,
				       PD_T_SEND_SOURCE_CAP);
		} else {
			/*
			 * Per standard, we should clear the reset counter here.
			 * However, that can result in state machine hang-ups.
			 * Reset it only in READY state to improve stability.
			 */
			/* port->hard_reset_count = 0; */
			port->caps_count = 0;
			tcpm_set_pd_capable(port, true);
			/*
			 * This should ideally timeout with PD_T_SENDER_RESPONSE
			 * to pass compliance. However trading compliance to
			 * user behavior while connecting to slow starting sink.
			 */
			tcpm_set_state_cond(port, SRC_SEND_CAPABILITIES_TIMEOUT,
					    PD_T_SEND_SOURCE_CAP);
		}
		break;
	case SRC_SEND_CAPABILITIES_TIMEOUT:
		/*
		 * Error recovery for a PD_DATA_SOURCE_CAP reply timeout.
		 *
		 * PD 2.0 sinks are supposed to accept src-capabilities with a
		 * 3.0 header and simply ignore any src PDOs which the sink does
		 * not understand such as PPS but some 2.0 sinks instead ignore
		 * the entire PD_DATA_SOURCE_CAP message, causing contract
		 * negotiation to fail.
		 *
		 * After PD_N_HARD_RESET_COUNT hard-reset attempts, we try
		 * sending src-capabilities with a lower PD revision to
		 * make these broken sinks work.
		 */
		if (port->hard_reset_count < PD_N_HARD_RESET_COUNT) {
			tcpm_set_state(port, HARD_RESET_SEND, 0);
		} else if (port->negotiated_rev > PD_REV20) {
			port->negotiated_rev--;
			port->hard_reset_count = 0;
			tcpm_set_state(port, SRC_SEND_CAPABILITIES, 0);
		} else {
			tcpm_set_state(port, hard_reset_state(port), 0);
		}
		break;
	case SRC_NEGOTIATE_CAPABILITIES:
		ret = tcpm_pd_check_request(port);
		if (ret < 0) {
			tcpm_pd_send_control(port, PD_CTRL_REJECT);
			if (!port->explicit_contract) {
				tcpm_set_state(port,
					       SRC_WAIT_NEW_CAPABILITIES, 0);
			} else {
				tcpm_set_state(port, SRC_READY, 0);
			}
		} else {
			tcpm_pd_send_control(port, PD_CTRL_ACCEPT);
			port->usb_comm_capable = port->sink_request &
						 RDO_USB_COMM;
			/* Notify TCPC of usb_comm_capable. */
			tcpm_set_attached_state(port, true);
			tcpm_set_state(port, SRC_TRANSITION_SUPPLY,
				       PD_T_SRC_TRANSITION);
		}
		break;
	case SRC_TRANSITION_SUPPLY:
		/* XXX: regulator_set_voltage(vbus, ...) */
		tcpm_pd_send_control(port, PD_CTRL_PS_RDY);
		port->explicit_contract = true;
		typec_set_pwr_opmode(port->typec_port, TYPEC_PWR_MODE_PD);
		port->pwr_opmode = TYPEC_PWR_MODE_PD;
		tcpm_set_state_cond(port, SRC_READY, 0);
		break;
	case SRC_READY:
		tcpm_log(port, "in PR_SWAP := false");
		port->tcpc->set_in_pr_swap(port->tcpc, false);
#if 1
		port->hard_reset_count = 0;
#endif
		port->try_src_count = 0;

		tcpm_swap_complete(port, 0);
		tcpm_typec_connect(port);

		if (port->ams != NONE_AMS)
			tcpm_ams_finish(port);
		if (port->next_ams != NONE_AMS) {
			port->ams = port->next_ams;
			port->next_ams = NONE_AMS;
		}

		/*
		 * If previous AMS is interrupted, switch to the upcoming
		 * state.
		 */
		upcoming_state = port->upcoming_state;
		if (port->upcoming_state != INVALID_STATE) {
			port->upcoming_state = INVALID_STATE;
			tcpm_set_state(port, upcoming_state, 0);
			break;
		}

		tcpm_check_send_discover(port);
		/*
		 * 6.3.5
		 * Sending ping messages is not necessary if
		 * - the source operates at vSafe5V
		 * or
		 * - The system is not operating in PD mode
		 * or
		 * - Both partners are connected using a Type-C connector
		 *
		 * There is no actual need to send PD messages since the local
		 * port type-c and the spec does not clearly say whether PD is
		 * possible when type-c is connected to Type-A/B
		 */
		break;
	case SRC_VPD_READY:
		tcpm_set_vbus(port, false);
		port->vpd_connected = true;
		break;
	case SRC_WAIT_NEW_CAPABILITIES:
		/* Nothing to do... */
		break;

	/* SNK states */
	case SNK_UNATTACHED:
		tcpm_port_in_hard_reset(port, false);
		if (!port->non_pd_role_swap)
			tcpm_swap_complete(port, -ENOTCONN);
		tcpm_pps_complete(port, -ENOTCONN);
		tcpm_snk_detach(port);
		if (tcpm_start_toggling(port, TYPEC_CC_RD)) {
			tcpm_set_state(port, TOGGLING, 0);
			break;
		}
		tcpm_set_cc(port, TYPEC_CC_RD);
		if (port->port_type == TYPEC_PORT_DRP)
			tcpm_set_state(port, SRC_UNATTACHED, PD_T_DRP_SRC);
		break;
	case SNK_ATTACH_WAIT:
		if ((port->cc1 == TYPEC_CC_OPEN &&
		     port->cc2 != TYPEC_CC_OPEN) ||
		    (port->cc1 != TYPEC_CC_OPEN &&
		     port->cc2 == TYPEC_CC_OPEN))
			tcpm_set_state(port, SNK_DEBOUNCED,
				       PD_T_CC_DEBOUNCE);
		else if (tcpm_port_is_disconnected(port))
			tcpm_set_state(port, SNK_UNATTACHED,
				       PD_T_PD_DEBOUNCE);
		break;
	case SNK_DEBOUNCED:
		if (tcpm_port_is_disconnected(port))
			tcpm_set_state(port, SNK_UNATTACHED,
				       PD_T_PD_DEBOUNCE);
		else if (port->vbus_present)
			tcpm_set_state(port,
				       tcpm_try_src(port) ? SRC_TRY
							  : SNK_ATTACHED,
				       0);
		break;
	case SRC_TRY:
		port->try_src_count++;
		tcpm_set_cc(port, tcpm_rp_cc(port));
		port->max_wait = 0;
		tcpm_set_state(port, SRC_TRY_WAIT, 0);
		break;
	case SRC_TRY_WAIT:
		if (port->max_wait == 0) {
			port->max_wait = jiffies +
					 msecs_to_jiffies(PD_T_DRP_TRY);
			msecs = PD_T_DRP_TRY;
		} else {
			if (time_is_after_jiffies(port->max_wait))
				msecs = jiffies_to_msecs(port->max_wait -
							 jiffies);
			else
				msecs = 0;
		}
		tcpm_set_state(port, SNK_TRYWAIT, msecs);
		break;
	case SRC_TRY_DEBOUNCE:
		tcpm_set_state(port, SRC_ATTACHED, PD_T_PD_DEBOUNCE);
		break;
	case SNK_TRYWAIT:
		tcpm_set_cc(port, TYPEC_CC_RD);
		tcpm_set_state(port, SNK_TRYWAIT_VBUS, PD_T_CC_DEBOUNCE);
		break;
	case SNK_TRYWAIT_VBUS:
		/*
		 * TCPM stays in this state indefinitely until VBUS
		 * is detected as long as Rp is not detected for
		 * more than a time period of tPDDebounce.
		 */
		if (port->vbus_present && tcpm_port_is_sink(port)) {
			tcpm_set_state(port, SNK_ATTACHED, 0);
			break;
		}
		if (!tcpm_port_is_sink(port))
			tcpm_set_state(port, SNK_TRYWAIT_DEBOUNCE, 0);
		break;
	case SNK_TRYWAIT_DEBOUNCE:
		tcpm_set_state(port, SNK_UNATTACHED, PD_T_PD_DEBOUNCE);
		break;
	case SNK_ATTACHED:
		ret = tcpm_snk_attach(port);
		if (ret < 0)
			tcpm_set_state(port, SNK_UNATTACHED, 0);
		else
			tcpm_set_state(port, SNK_STARTUP, 0);
		break;
	case SNK_STARTUP:
		tcpm_port_in_hard_reset(port, false);
		opmode =  tcpm_get_pwr_opmode(port->polarity ?
					      port->cc2 : port->cc1);
		typec_set_pwr_opmode(port->typec_port, opmode);
		port->pwr_opmode = TYPEC_PWR_MODE_USB;
		port->negotiated_rev = PD_MAX_REV;
		port->svdm_version = SVDM_MAX_VER;
		port->message_id = 0;
		port->rx_msgid = -1;
		port->explicit_contract = false;

		if (port->ams == POWER_ROLE_SWAP ||
		    port->ams == FAST_ROLE_SWAP)
			/* SRC -> SNK POWER/FAST_ROLE_SWAP finished */
			tcpm_ams_finish(port);

		tcpm_set_state(port, SNK_DISCOVERY, 500);
		break;
	case SNK_DISCOVERY:
		if (port->vbus_present) {
			if (port->psnkstdby_after_accept && tcpm_get_current_limit(port) <=
			    PD_P_SNK_STDBY_5V)
				tcpm_set_current_limit(port, tcpm_get_current_limit(port), 5000);
			else
				tcpm_set_current_limit(port, PD_P_SNK_STDBY_5V, 5000);
			tcpm_set_charge(port, true);
			tcpm_set_state(port, SNK_WAIT_CAPABILITIES, 0);
			break;
		}
		/*
		 * For DRP, timeouts differ. Also, handling is supposed to be
		 * different and much more complex (dead battery detection;
		 * see USB power delivery specification, section 8.3.3.6.1.5.1).
		 */
		tcpm_set_state(port, hard_reset_state(port),
			       port->port_type == TYPEC_PORT_DRP ?
					PD_T_DB_DETECT : PD_T_NO_RESPONSE);
		break;
	case SNK_DISCOVERY_DEBOUNCE:
		tcpm_set_state(port, SNK_DISCOVERY_DEBOUNCE_DONE,
			       PD_T_CC_DEBOUNCE);
		break;
	case SNK_DISCOVERY_DEBOUNCE_DONE:
		if (!tcpm_port_is_disconnected(port) &&
		    tcpm_port_is_sink(port) &&
		    time_is_after_jiffies(port->delayed_runtime)) {
			tcpm_set_state(port, SNK_DISCOVERY,
				       jiffies_to_msecs(port->delayed_runtime -
							jiffies));
			break;
		}
		tcpm_set_state(port, unattached_state(port), 0);
		break;
	case SNK_WAIT_CAPABILITIES:
		ret = port->tcpc->set_pd_rx(port->tcpc, true,
					    FRAME_FILTER_EN_HARD_RESET |
					    FRAME_FILTER_EN_SOP);
		if (ret < 0) {
			tcpm_set_state(port, SNK_READY, 0);
			break;
		}
		/*
		 * If VBUS has never been low, and we time out waiting
		 * for source cap, try a soft reset first, in case we
		 * were already in a stable contract before this boot.
		 * Do this only once.
		 */
		if (port->vbus_never_low) {
			port->vbus_never_low = false;
			tcpm_set_state(port, SNK_SOFT_RESET,
				       PD_T_SINK_WAIT_CAP);
		} else {
			tcpm_set_state(port, hard_reset_state(port),
				       PD_T_SINK_WAIT_CAP);
		}
		break;
	case SNK_NEGOTIATE_CAPABILITIES:
		tcpm_set_pd_capable(port, true);
		port->usb_comm_capable = port->source_caps[0] &
					 PDO_FIXED_USB_COMM;
		/* Notify TCPC of usb_comm_capable. */
		tcpm_set_attached_state(port, true);
		port->hard_reset_count = 0;
		ret = tcpm_pd_send_request(port);
		if (ret < 0) {
			/* Let the Source send capabilities again. */
			tcpm_set_state(port, SNK_WAIT_CAPABILITIES, 0);
		} else {
			tcpm_set_state_cond(port, hard_reset_state(port),
					    PD_T_SENDER_RESPONSE);
		}
		if (port->tcpc->set_suspend_supported) {
			port->tcpc->set_suspend_supported(port->tcpc,
							  port->source_caps[0] &
							  PDO_FIXED_SUSPEND);
		}
		break;
	case SNK_NEGOTIATE_PPS_CAPABILITIES:
		ret = tcpm_pd_send_pps_request(port);
		if (ret < 0) {
			port->pps_status = ret;
			/*
			 * If this was called due to updates to sink
			 * capabilities, and pps is no longer valid, we should
			 * safely fall back to a standard PDO.
			 */
			if (port->update_sink_caps)
				tcpm_set_state(port, SNK_NEGOTIATE_CAPABILITIES, 0);
			else
				tcpm_set_state(port, SNK_READY, 0);
		} else {
			tcpm_set_state_cond(port, hard_reset_state(port),
					    PD_T_SENDER_RESPONSE);
		}
		break;
	case SNK_TRANSITION_SINK:
		if (port->psnkstdby_after_accept &&
		    port->supply_voltage != port->req_supply_voltage &&
		    !port->pps_data.active) {
			if (PD_P_SNK_STDBY <
			    port->current_limit * port->supply_voltage / 1000)
				tcpm_set_stdby_current(port);
		}
	case SNK_TRANSITION_SINK_VBUS:
		tcpm_set_state(port, hard_reset_state(port),
			       PD_T_PS_TRANSITION);
		break;
	case SNK_READY:
		port->try_snk_count = 0;
		port->update_sink_caps = false;
		if (port->explicit_contract) {
			typec_set_pwr_opmode(port->typec_port,
					     TYPEC_PWR_MODE_PD);
			port->pwr_opmode = TYPEC_PWR_MODE_PD;
		}

		/* Set current limit for NON-PD link when psnkstdby_after_accept is not set*/
		if (!port->pd_capable && !port->psnkstdby_after_accept)
			tcpm_set_current_limit(port, tcpm_get_current_limit(port), 5000);

		tcpm_swap_complete(port, 0);
		tcpm_typec_connect(port);
		tcpm_pps_complete(port, port->pps_status);

		if (port->ams != NONE_AMS)
			tcpm_ams_finish(port);
		if (port->next_ams != NONE_AMS) {
			port->ams = port->next_ams;
			port->next_ams = NONE_AMS;
		}

		/*
		 * If previous AMS is interrupted, switch to the upcoming
		 * state.
		 */
		upcoming_state = port->upcoming_state;
		if (port->upcoming_state != INVALID_STATE) {
			port->upcoming_state = INVALID_STATE;
			tcpm_set_state(port, upcoming_state, 0);
			break;
		}

		tcpm_check_send_discover(port);
		mod_delayed_work(port->wq, &port->disc_id_work,
				 msecs_to_jiffies(DISC_ID_RETRY_MS));
		power_supply_changed(port->psy);

		break;

	/* Accessory states */
	case ACC_UNATTACHED:
		tcpm_acc_detach(port);
		tcpm_set_state(port, SRC_UNATTACHED, 0);
		break;
	case DEBUG_ACC_ATTACHED:
	case AUDIO_ACC_ATTACHED:
		ret = tcpm_acc_attach(port);
		if (ret < 0)
			tcpm_set_state(port, ACC_UNATTACHED, 0);
		break;
	case AUDIO_ACC_DEBOUNCE:
		tcpm_set_state(port, ACC_UNATTACHED, PD_T_CC_DEBOUNCE);
		break;

	/* Hard_Reset states */
	case HARD_RESET_SEND:
		if (port->ams != NONE_AMS)
			tcpm_ams_finish(port);
		/*
		 * State machine will be directed to HARD_RESET_START,
		 * thus set upcoming_state to INVALID_STATE.
		 */
		port->upcoming_state = INVALID_STATE;
		tcpm_ams_start(port, HARD_RESET);
		break;
	case HARD_RESET_START:
		tcpm_port_in_hard_reset(port, true);
		port->hard_reset_count++;
		port->tcpc->set_pd_rx(port->tcpc, false,
				      FRAME_FILTER_EN_HARD_RESET |
				      FRAME_FILTER_EN_SOP);
		tcpm_unregister_altmodes(port);
		port->send_discover = true;
		port->usb_comm_capable = false;
		if (port->chunk_rx_state != RCH_WAIT_FOR_MESSAGE)
			tcpm_set_chunk_state(port, RCH_WAIT_FOR_MESSAGE, 0);
		else if (port->chunk_tx_state != TCH_WAIT_FOR_MESSAGE_REQUEST)
			tcpm_set_chunk_state(port, TCH_WAIT_FOR_MESSAGE_REQUEST,
					     0);
		if (port->pwr_role == TYPEC_SOURCE)
			tcpm_set_state(port, SRC_HARD_RESET_VBUS_OFF,
				       PD_T_PS_HARD_RESET);
		else
			tcpm_set_state(port, SNK_HARD_RESET_SINK_OFF, 0);
		break;
	case SRC_HARD_RESET_VBUS_OFF:
		tcpm_set_vconn(port, false);
		tcpm_set_vbus(port, false);
		tcpm_set_roles(port, port->self_powered, TYPEC_SOURCE,
			       TYPEC_HOST);
		tcpm_set_state(port, SRC_HARD_RESET_VBUS_ON, PD_T_SRC_RECOVER);
		break;
	case SRC_HARD_RESET_VBUS_ON:
		tcpm_set_vconn(port, true);
		tcpm_set_vbus(port, true);
		if (port->ams == HARD_RESET)
			tcpm_ams_finish(port);
		port->tcpc->set_pd_rx(port->tcpc, true,
				      FRAME_FILTER_EN_HARD_RESET |
				      FRAME_FILTER_EN_SOP);
		tcpm_set_attached_state(port, true);
		tcpm_set_state(port, SRC_UNATTACHED, PD_T_PS_SOURCE_ON);
		break;
	case SNK_HARD_RESET_SINK_OFF:
		memset(&port->pps_data, 0, sizeof(port->pps_data));
		tcpm_set_vconn(port, false);
		if (port->pd_capable)
			tcpm_set_charge(port, false);
		tcpm_set_roles(port, port->self_powered, TYPEC_SINK,
			       TYPEC_DEVICE);
		/*
		 * VBUS may or may not toggle, depending on the adapter.
		 * If it doesn't toggle, transition to SNK_HARD_RESET_SINK_ON
		 * directly after timeout.
		 */
		tcpm_set_state(port, SNK_HARD_RESET_SINK_ON, PD_T_SAFE_0V);
		break;
	case SNK_HARD_RESET_WAIT_VBUS:
		if (port->ams == HARD_RESET)
			tcpm_ams_finish(port);
		/* Assume we're disconnected if VBUS doesn't come back. */
		tcpm_set_state(port, SNK_UNATTACHED,
			       PD_T_SRC_RECOVER_MAX + PD_T_SRC_TURN_ON);
		break;
	case SNK_HARD_RESET_SINK_ON:
		/* Note: There is no guarantee that VBUS is on in this state */
		/*
		 * XXX:
		 * The specification suggests that dual mode ports in sink
		 * mode should transition to state PE_SRC_Transition_to_default.
		 * See USB power delivery specification chapter 8.3.3.6.1.3.
		 * This would mean to to
		 * - turn off VCONN, reset power supply
		 * - request hardware reset
		 * - turn on VCONN
		 * - Transition to state PE_Src_Startup
		 * SNK only ports shall transition to state Snk_Startup
		 * (see chapter 8.3.3.3.8).
		 * Similar, dual-mode ports in source mode should transition
		 * to PE_SNK_Transition_to_default.
		 */
		if (port->pd_capable) {
			tcpm_set_current_limit(port,
					       tcpm_get_current_limit(port),
					       5000);
			tcpm_set_charge(port, true);
		}
		if (port->ams == HARD_RESET)
			tcpm_ams_finish(port);
		tcpm_set_attached_state(port, true);
		tcpm_set_state(port, SNK_STARTUP, 0);
		break;

	/* Soft_Reset states */
	case SOFT_RESET:
		port->message_id = 0;
		port->rx_msgid = -1;
		tcpm_pd_send_control(port, PD_CTRL_ACCEPT);
		tcpm_ams_finish(port);
		if (port->pwr_role == TYPEC_SOURCE) {
			port->upcoming_state = SRC_SEND_CAPABILITIES;
			tcpm_ams_start(port, POWER_NEGOTIATION);
		} else {
			tcpm_set_state(port, SNK_WAIT_CAPABILITIES, 0);
		}
		break;
	case SRC_SOFT_RESET_WAIT_SNK_TX:
	case SNK_SOFT_RESET:
		if (port->ams != NONE_AMS)
			tcpm_ams_finish(port);
		port->upcoming_state = SOFT_RESET_SEND;
		tcpm_ams_start(port, SOFT_RESET_AMS);
		break;
	case SOFT_RESET_SEND:
		port->message_id = 0;
		port->rx_msgid = -1;
		if (tcpm_pd_send_control(port, PD_CTRL_SOFT_RESET))
			tcpm_set_state_cond(port, hard_reset_state(port), 0);
		else
			tcpm_set_state_cond(port, hard_reset_state(port),
					    PD_T_SENDER_RESPONSE);
		break;

	/* DR_Swap states */
	case DR_SWAP_SEND:
		tcpm_pd_send_control(port, PD_CTRL_DR_SWAP);
		tcpm_set_state_cond(port, DR_SWAP_SEND_TIMEOUT,
				    PD_T_SENDER_RESPONSE);
		break;
	case DR_SWAP_ACCEPT:
		tcpm_pd_send_control(port, PD_CTRL_ACCEPT);
		/* Set VDM state machine running flag ASAP */
		if (port->data_role == TYPEC_DEVICE && port->send_discover)
			port->vdm_sm_running = true;
		tcpm_set_state_cond(port, DR_SWAP_CHANGE_DR, 0);
		break;
	case DR_SWAP_SEND_TIMEOUT:
		tcpm_swap_complete(port, -ETIMEDOUT);
		tcpm_ams_finish(port);
		tcpm_set_state(port, ready_state(port), 0);
		break;
	case DR_SWAP_CHANGE_DR:
		if (port->data_role == TYPEC_HOST) {
			tcpm_unregister_altmodes(port);
			tcpm_set_roles(port, true, port->pwr_role,
				       TYPEC_DEVICE);
		} else {
			tcpm_set_roles(port, true, port->pwr_role,
				       TYPEC_HOST);
			port->send_discover = true;
		}
		tcpm_ams_finish(port);
		tcpm_set_state(port, ready_state(port), 0);
		break;

	/* PR_Swap states */
	case PR_SWAP_ACCEPT:
		if (port->pwr_role == TYPEC_SOURCE)
			tcpm_set_cc(port, port->cc_req);
		else
			tcpm_set_cc(port, TYPEC_CC_RD);
		tcpm_log(port, "in PR_SWAP := true");
		port->tcpc->set_in_pr_swap(port->tcpc, true);
		tcpm_pd_send_control(port, PD_CTRL_ACCEPT);
		tcpm_set_state(port, PR_SWAP_START, 0);
		break;
	case PR_SWAP_SEND:
		if (port->pwr_role == TYPEC_SOURCE)
			tcpm_set_cc(port, port->cc_req);
		else
			tcpm_set_cc(port, TYPEC_CC_RD);
		tcpm_log(port, "in PR_SWAP := true");
		port->tcpc->set_in_pr_swap(port->tcpc, true);
		tcpm_pd_send_control(port, PD_CTRL_PR_SWAP);
		tcpm_set_state_cond(port, PR_SWAP_SEND_TIMEOUT,
				    PD_T_SENDER_RESPONSE);
		break;
	case PR_SWAP_SEND_TIMEOUT:
		tcpm_log(port, "in PR_SWAP := false");
		port->tcpc->set_in_pr_swap(port->tcpc, false);
		tcpm_swap_complete(port, -ETIMEDOUT);
		tcpm_set_state(port, ready_state(port), 0);
		break;
	case PR_SWAP_START:
		tcpm_log(port, "in PR_SWAP := true");
		if (port->pwr_role == TYPEC_SOURCE)
			tcpm_set_state(port, PR_SWAP_SRC_SNK_TRANSITION_OFF,
				       PD_T_SRC_TRANSITION);
		else
			tcpm_set_state(port, PR_SWAP_SNK_SRC_SINK_OFF, 0);
		break;
	case PR_SWAP_SRC_SNK_TRANSITION_OFF:
		tcpm_set_vbus(port, false);
		port->explicit_contract = false;
		/* allow time for Vbus discharge, must be < tSrcSwapStdby */
		tcpm_set_state(port, PR_SWAP_SRC_SNK_SOURCE_OFF,
			       PD_T_SRCSWAPSTDBY - PD_T_CC_DEBOUNCE);
		break;
	case PR_SWAP_SRC_SNK_SOURCE_OFF:
		tcpm_set_cc(port, TYPEC_CC_RD);
		/* allow CC debounce */
		tcpm_set_state(port, PR_SWAP_SRC_SNK_SOURCE_OFF_CC_DEBOUNCED,
			       PD_T_CC_DEBOUNCE);
		break;
	case PR_SWAP_SRC_SNK_SOURCE_OFF_CC_DEBOUNCED:
		/*
		 * USB-PD standard, 6.2.1.4, Port Power Role:
		 * "During the Power Role Swap Sequence, for the initial Source
		 * Port, the Port Power Role field shall be set to Sink in the
		 * PS_RDY Message indicating that the initial Source’s power
		 * supply is turned off"
		 */
		tcpm_set_pwr_role(port, TYPEC_SINK);
		if (tcpm_pd_send_control(port, PD_CTRL_PS_RDY)) {
			tcpm_set_state(port, ERROR_RECOVERY, 0);
			break;
		}
		tcpm_set_state(port, ERROR_RECOVERY, PD_T_PS_SOURCE_ON_PRS);
		break;
	case PR_SWAP_SRC_SNK_SINK_ON:
		tcpm_set_state(port, SNK_STARTUP, 0);
		break;
	case PR_SWAP_SNK_SRC_SINK_OFF:
		tcpm_set_charge(port, false);
		tcpm_set_state(port, hard_reset_state(port),
			       PD_T_PS_SOURCE_OFF);
		break;
	case PR_SWAP_SNK_SRC_SOURCE_ON:
		tcpm_set_cc(port, tcpm_rp_cc(port));
		tcpm_set_vbus(port, true);
		/*
		 * allow time VBUS ramp-up, must be < tNewSrc
		 * Also, this window overlaps with CC debounce as well.
		 * So, Wait for the max of two which is PD_T_NEWSRC
		 */
		tcpm_set_state(port, PR_SWAP_SNK_SRC_SOURCE_ON_VBUS_RAMPED_UP,
			       PD_T_NEWSRC);
		break;
	case PR_SWAP_SNK_SRC_SOURCE_ON_VBUS_RAMPED_UP:
		/*
		 * USB PD standard, 6.2.1.4:
		 * "Subsequent Messages initiated by the Policy Engine,
		 * such as the PS_RDY Message sent to indicate that Vbus
		 * is ready, will have the Port Power Role field set to
		 * Source."
		 */
		tcpm_set_pwr_role(port, TYPEC_SOURCE);
		tcpm_pd_send_control(port, PD_CTRL_PS_RDY);
		tcpm_set_state(port, SRC_STARTUP, PD_T_CC_DEBOUNCE);
		break;
	case VCONN_SWAP_ACCEPT:
		tcpm_pd_send_control(port, PD_CTRL_ACCEPT);
		tcpm_ams_finish(port);
		tcpm_set_state(port, VCONN_SWAP_START, 0);
		break;
	case VCONN_SWAP_SEND:
		tcpm_pd_send_control(port, PD_CTRL_VCONN_SWAP);
		tcpm_set_state(port, VCONN_SWAP_SEND_TIMEOUT,
			       PD_T_SENDER_RESPONSE);
		break;
	case VCONN_SWAP_SEND_TIMEOUT:
		tcpm_swap_complete(port, -ETIMEDOUT);
		if (port->data_role == TYPEC_HOST && port->send_discover)
			port->vdm_sm_running = true;
		tcpm_set_state(port, ready_state(port), 0);
		break;
	case VCONN_SWAP_START:
		if (port->vconn_role == TYPEC_SOURCE)
			tcpm_set_state(port, VCONN_SWAP_WAIT_FOR_VCONN, 0);
		else
			tcpm_set_state(port, VCONN_SWAP_TURN_ON_VCONN, 0);
		break;
	case VCONN_SWAP_WAIT_FOR_VCONN:
		tcpm_set_state(port, hard_reset_state(port),
			       PD_T_VCONN_SOURCE_ON);
		break;
	case VCONN_SWAP_TURN_ON_VCONN:
		tcpm_set_vconn(port, true);
		tcpm_pd_send_control(port, PD_CTRL_PS_RDY);
		if (port->data_role == TYPEC_HOST && port->send_discover)
			port->vdm_sm_running = true;
		tcpm_set_state(port, ready_state(port), 0);
		break;
	case VCONN_SWAP_TURN_OFF_VCONN:
		tcpm_set_vconn(port, false);
		if (port->data_role == TYPEC_HOST && port->send_discover)
			port->vdm_sm_running = true;
		tcpm_set_state(port, ready_state(port), 0);
		break;

	case PR_SWAP_CANCEL:
		tcpm_log(port, "in PR_SWAP := false");
		port->tcpc->set_in_pr_swap(port->tcpc, false);
	case DR_SWAP_CANCEL:
	case VCONN_SWAP_CANCEL:
		tcpm_swap_complete(port, port->swap_status);
		if (port->data_role == TYPEC_HOST && port->send_discover)
			port->vdm_sm_running = true;
		if (port->pwr_role == TYPEC_SOURCE)
			tcpm_set_state(port, SRC_READY, 0);
		else
			tcpm_set_state(port, SNK_READY, 0);
		break;

	case BIST_RX:
		switch (BDO_MODE_MASK(port->bist_request)) {
		case BDO_MODE_CARRIER2:
			// TODO: integrate this into chunk Tx
			tcpm_pd_transmit(port, TCPC_TX_BIST_MODE_2, NULL);
			break;
		default:
			break;
		}
		/* Always switch to unattached state */
		tcpm_set_state(port, unattached_state(port), 0);
		break;
	case GET_STATUS_SEND:
		tcpm_pd_send_control(port, PD_CTRL_GET_STATUS);
		tcpm_set_state(port, GET_STATUS_SEND_TIMEOUT,
			       PD_T_SENDER_RESPONSE);
		break;
	case GET_STATUS_SEND_TIMEOUT:
		tcpm_set_state(port, ready_state(port), 0);
		break;
	case GET_PPS_STATUS_SEND:
		tcpm_pd_send_control(port, PD_CTRL_GET_PPS_STATUS);
		tcpm_set_state(port, GET_PPS_STATUS_SEND_TIMEOUT,
			       PD_T_SENDER_RESPONSE);
		break;
	case GET_PPS_STATUS_SEND_TIMEOUT:
		tcpm_set_state(port, ready_state(port), 0);
		break;
	case ERROR_RECOVERY:
		tcpm_swap_complete(port, -EPROTO);
		tcpm_pps_complete(port, -EPROTO);
		tcpm_set_state(port, PORT_RESET, 0);
		break;
	case PORT_RESET:
		tcpm_reset_port(port);
		tcpm_set_cc(port, TYPEC_CC_OPEN);
		tcpm_set_state(port, PORT_RESET_WAIT_OFF,
			       PD_T_ERROR_RECOVERY);
		break;
	case PORT_RESET_WAIT_OFF:
		tcpm_set_state(port,
			       tcpm_default_state(port),
			       port->vbus_present ? PD_T_PS_SOURCE_OFF : 0);
		break;

	/* AMS intermediate state */
	case AMS_START:
		if (port->upcoming_state == INVALID_STATE) {
			tcpm_set_state(port, port->pwr_role == TYPEC_SOURCE ?
				       SRC_READY : SNK_READY, 0);
			break;
		}

		upcoming_state = port->upcoming_state;
		port->upcoming_state = INVALID_STATE;
		tcpm_set_state(port, upcoming_state, 0);
		break;

	/* Chunk state */
	case CHUNK_NOT_SUPP:
		tcpm_pd_send_control(port, PD_CTRL_NOT_SUPP);
		tcpm_set_state(port, port->pwr_role == TYPEC_SOURCE ?
			       SRC_READY : SNK_READY, 0);
		break;
	case CHUNK_RX:
		break;
	case CHUNK_RX_FINISH:
		tcpm_ams_finish(port);
		tcpm_set_state(port, ready_state(port), 0);
		break;
	case CHUNK_TX:
		tcpm_pd_send_extend(port);
		break;
	case CHUNK_TX_FINISH:
		tcpm_ams_finish(port);
		tcpm_set_state(port, ready_state(port), 0);
		break;
	default:
		WARN(1, "Unexpected port state %d\n", port->state);
		break;
	}
}

static void tcpm_state_machine_work(struct work_struct *work)
{
	struct tcpm_port *port = container_of(work, struct tcpm_port,
					      state_machine.work);
	enum tcpm_state prev_state;

	mutex_lock(&port->lock);
	port->state_machine_running = true;

	if (port->queued_message && tcpm_send_queued_message(port))
		goto done;

	/* If we were queued due to a delayed state change, update it now */
	if (port->delayed_state) {
		tcpm_log(port, "state change %s -> %s [delayed %ld ms]",
			 tcpm_states[port->state],
			 tcpm_states[port->delayed_state], port->delay_ms);
		port->prev_state = port->state;
		port->state = port->delayed_state;
		port->delayed_state = INVALID_STATE;
	}

	/*
	 * Continue running as long as we have (non-delayed) state changes
	 * to make.
	 */
	do {
		prev_state = port->state;
		run_state_machine(port);
		if (port->queued_message)
			tcpm_send_queued_message(port);
	} while (port->state != prev_state && !port->delayed_state);

done:
	port->state_machine_running = false;
	mutex_unlock(&port->lock);
}

static void _tcpm_cc_change(struct tcpm_port *port, enum typec_cc_status cc1,
			    enum typec_cc_status cc2)
{
	enum typec_cc_status old_cc1, old_cc2;
	enum tcpm_state new_state;

	old_cc1 = port->cc1;
	old_cc2 = port->cc2;
	port->cc1 = cc1;
	port->cc2 = cc2;

	tcpm_log_force(port,
		       "CC1: %u -> %u, CC2: %u -> %u [state %s, polarity %d, %s]",
		       old_cc1, cc1, old_cc2, cc2, tcpm_states[port->state],
		       port->polarity,
		       tcpm_port_is_disconnected(port) ? "disconnected"
						       : "connected");

	switch (port->state) {
	case TOGGLING:
		if (tcpm_port_is_debug(port) || tcpm_port_is_audio(port) ||
		    tcpm_port_is_source(port))
			tcpm_set_state(port, SRC_ATTACH_WAIT, 0);
		else if (tcpm_port_is_sink(port))
			tcpm_set_state(port, SNK_ATTACH_WAIT, 0);
		else if (tcpm_port_is_disconnected(port))
			port->hard_reset_count = 0;
		break;
	case SRC_UNATTACHED:
	case ACC_UNATTACHED:
		if (tcpm_port_is_debug(port) || tcpm_port_is_audio(port) ||
		    tcpm_port_is_source(port))
			tcpm_set_state(port, SRC_ATTACH_WAIT, 0);
		break;
	case SRC_ATTACH_WAIT:
		if (tcpm_port_is_disconnected(port) ||
		    tcpm_port_is_audio_detached(port))
			tcpm_set_state(port, SRC_UNATTACHED, 0);
		else if (cc1 != old_cc1 || cc2 != old_cc2)
			tcpm_set_state(port, SRC_ATTACH_WAIT, 0);
		break;
	case SRC_ATTACHED:
	case SRC_SEND_CAPABILITIES:
	case SRC_READY:
	case SRC_VPD_READY:
		if (tcpm_port_is_disconnected(port) ||
		    !tcpm_port_is_source(port))
			tcpm_set_state(port, SRC_UNATTACHED, 0);
		break;
	case SNK_UNATTACHED:
		if (tcpm_port_is_sink(port))
			tcpm_set_state(port, SNK_ATTACH_WAIT, 0);
		break;
	case SNK_ATTACH_WAIT:
		if ((port->cc1 == TYPEC_CC_OPEN &&
		     port->cc2 != TYPEC_CC_OPEN) ||
		    (port->cc1 != TYPEC_CC_OPEN &&
		     port->cc2 == TYPEC_CC_OPEN))
			new_state = SNK_DEBOUNCED;
		else if (tcpm_port_is_disconnected(port))
			new_state = SNK_UNATTACHED;
		else
			break;
		if (new_state != port->delayed_state)
			tcpm_set_state(port, SNK_ATTACH_WAIT, 0);
		break;
	case SNK_DEBOUNCED:
		if (tcpm_port_is_disconnected(port))
			new_state = SNK_UNATTACHED;
		else if (port->vbus_present)
			new_state = tcpm_try_src(port) ? SRC_TRY : SNK_ATTACHED;
		else
			new_state = SNK_UNATTACHED;
		if (new_state != port->delayed_state)
			tcpm_set_state(port, SNK_DEBOUNCED, 0);
		break;
	case SNK_READY:
		if (tcpm_port_is_disconnected(port))
			tcpm_set_state(port, unattached_state(port), 0);
		else if (!port->pd_capable &&
			 (cc1 != old_cc1 || cc2 != old_cc2))
			tcpm_set_current_limit(port,
					       tcpm_get_current_limit(port),
					       5000);
		if (tcpm_sink_tx_ok(port))
			mod_delayed_work(port->wq, &port->disc_id_work, 0);
		break;
	case AUDIO_ACC_ATTACHED:
		if (cc1 == TYPEC_CC_OPEN || cc2 == TYPEC_CC_OPEN)
			tcpm_set_state(port, AUDIO_ACC_DEBOUNCE, 0);
		break;
	case AUDIO_ACC_DEBOUNCE:
		if (tcpm_port_is_audio(port))
			tcpm_set_state(port, AUDIO_ACC_ATTACHED, 0);
		break;

	case DEBUG_ACC_ATTACHED:
		if (cc1 == TYPEC_CC_OPEN || cc2 == TYPEC_CC_OPEN)
			tcpm_set_state(port, ACC_UNATTACHED, 0);
		break;

	case SNK_TRY:
		/* Do nothing, waiting for timeout */
		break;

	case SNK_DISCOVERY:
		/* CC line is unstable, wait for debounce */
		if (tcpm_port_is_disconnected(port))
			tcpm_set_state(port, SNK_DISCOVERY_DEBOUNCE, 0);
		break;
	case SNK_DISCOVERY_DEBOUNCE:
		break;

	case SRC_TRYWAIT:
		/* Hand over to state machine if needed */
		if (!port->vbus_present && tcpm_port_is_source(port))
			tcpm_set_state(port, SRC_TRYWAIT_DEBOUNCE, 0);
		break;
	case SRC_TRYWAIT_DEBOUNCE:
		if (port->vbus_present || !tcpm_port_is_source(port))
			tcpm_set_state(port, SRC_TRYWAIT, 0);
		break;
	case SNK_TRY_WAIT_DEBOUNCE:
		if (!tcpm_port_is_sink(port)) {
			port->max_wait = 0;
			tcpm_set_state(port, SRC_TRYWAIT, 0);
		}
		break;
	case SRC_TRY_WAIT:
		if (tcpm_port_is_source(port))
			tcpm_set_state(port, SRC_TRY_DEBOUNCE, 0);
		break;
	case SRC_TRY_DEBOUNCE:
		tcpm_set_state(port, SRC_TRY_WAIT, 0);
		break;
	case SNK_TRYWAIT_DEBOUNCE:
		if (tcpm_port_is_sink(port))
			tcpm_set_state(port, SNK_TRYWAIT_VBUS, 0);
		break;
	case SNK_TRYWAIT_VBUS:
		if (!tcpm_port_is_sink(port))
			tcpm_set_state(port, SNK_TRYWAIT_DEBOUNCE, 0);
		break;
	case SNK_TRYWAIT:
		/* Do nothing, waiting for tCCDebounce */
		break;
	case PR_SWAP_SNK_SRC_SINK_OFF:
	case PR_SWAP_SRC_SNK_TRANSITION_OFF:
	case PR_SWAP_SRC_SNK_SOURCE_OFF:
	case PR_SWAP_SRC_SNK_SOURCE_OFF_CC_DEBOUNCED:
	case PR_SWAP_SNK_SRC_SOURCE_ON:
	case PR_SWAP_SNK_SRC_SOURCE_ON_VBUS_RAMPED_UP:
		/*
		 * CC state change is expected in PR_SWAP
		 * Ignore it.
		 */
		break;

	case PORT_RESET:
	case PORT_RESET_WAIT_OFF:
		/*
		 * State set back to default mode once the timer completes.
		 * Ignore CC changes here.
		*/
		break;
	default:
		if (tcpm_port_is_disconnected(port))
			tcpm_set_state(port, unattached_state(port), 0);
		break;
	}
}

static void _tcpm_pd_vbus_on(struct tcpm_port *port)
{
	tcpm_log_force(port, "VBUS on");
	port->vbus_present = true;
	/*
	 * When vbus_present is true i.e. Voltage at VBUS is greater than VSAFE5V implicitly
	 * states that vbus is not at VSAFE0V, hence clear the vbus_vsafe0v flag here.
	 */
	port->vbus_vsafe0v = false;

	switch (port->state) {
	case SNK_TRANSITION_SINK_VBUS:
		port->explicit_contract = true;
		/* Set the VDM flag ASAP */
		if (port->data_role == TYPEC_HOST && port->send_discover)
			port->vdm_sm_running = true;
		tcpm_set_state(port, SNK_READY, 0);
		break;
	case SNK_DISCOVERY:
		tcpm_set_state(port, SNK_DISCOVERY, 0);
		break;

	case SNK_DEBOUNCED:
		tcpm_set_state(port, tcpm_try_src(port) ? SRC_TRY
							: SNK_ATTACHED,
				       0);
		break;
	case SNK_HARD_RESET_WAIT_VBUS:
		tcpm_set_state(port, SNK_HARD_RESET_SINK_ON, 0);
		break;
	case SRC_ATTACHED:
		tcpm_set_state(port, SRC_STARTUP, 0);
		break;
	case SRC_HARD_RESET_VBUS_ON:
		tcpm_set_state(port, SRC_STARTUP, 0);
		break;

	case SNK_TRY:
		/* Do nothing, waiting for timeout */
		break;
	case SRC_TRYWAIT:
		/* Do nothing, Waiting for Rd to be detected */
		break;
	case SRC_TRYWAIT_DEBOUNCE:
		tcpm_set_state(port, SRC_TRYWAIT, 0);
		break;
	case SNK_TRY_WAIT_DEBOUNCE:
		/* Do nothing, waiting for PD_DEBOUNCE to do be done */
		break;
	case SNK_TRYWAIT:
		/* Do nothing, waiting for tCCDebounce */
		break;
	case SNK_TRYWAIT_VBUS:
		if (tcpm_port_is_sink(port))
			tcpm_set_state(port, SNK_ATTACHED, 0);
		break;
	case SNK_TRYWAIT_DEBOUNCE:
		/* Do nothing, waiting for Rp */
		break;
	case SRC_TRY_WAIT:
	case SRC_TRY_DEBOUNCE:
		/* Do nothing, waiting for sink detection */
		break;

	case PORT_RESET:
	case PORT_RESET_WAIT_OFF:
		/*
		 * State set back to default mode once the timer completes.
		 * Ignore vbus changes here.
		 */
		break;
	default:
		break;
	}
}

static void _tcpm_pd_vbus_off(struct tcpm_port *port)
{
	tcpm_log_force(port, "VBUS off");
	port->vbus_present = false;
	port->vbus_never_low = false;
	switch (port->state) {
	case SNK_HARD_RESET_SINK_OFF:
		tcpm_set_state(port, SNK_HARD_RESET_WAIT_VBUS, 0);
		break;
	case HARD_RESET_SEND:
		break;
	case SNK_TRY:
		/* Do nothing, waiting for timeout */
		break;
	case SRC_TRYWAIT:
		/* Hand over to state machine if needed */
		if (tcpm_port_is_source(port))
			tcpm_set_state(port, SRC_TRYWAIT_DEBOUNCE, 0);
		break;
	case SNK_TRY_WAIT_DEBOUNCE:
		/* Do nothing, waiting for PD_DEBOUNCE to do be done */
		break;
	case SNK_TRYWAIT:
	case SNK_TRYWAIT_VBUS:
	case SNK_TRYWAIT_DEBOUNCE:
		break;
	case SNK_ATTACH_WAIT:
	case SNK_DEBOUNCED:
		/* Do nothing, as TCPM is still waiting for vbus to reaach VSAFE5V to connect */
		break;

	case SNK_NEGOTIATE_CAPABILITIES:
		break;

	case PR_SWAP_SRC_SNK_TRANSITION_OFF:
		tcpm_set_state(port, PR_SWAP_SRC_SNK_SOURCE_OFF, 0);
		break;

	case PR_SWAP_SNK_SRC_SINK_OFF:
		/* Do nothing, expected */
		break;

	case PR_SWAP_SNK_SRC_SOURCE_ON:
		/*
		 * Do nothing when vbus off notification is received.
		 * TCPM can wait for PD_T_NEWSRC in PR_SWAP_SNK_SRC_SOURCE_ON
		 * for the vbus source to ramp up.
		 */
		break;

	case PORT_RESET_WAIT_OFF:
		tcpm_set_state(port, tcpm_default_state(port), 0);
		break;

	case SRC_TRY_WAIT:
	case SRC_TRY_DEBOUNCE:
		/* Do nothing, waiting for sink detection */
		break;

	case PORT_RESET:
		/*
		 * State set back to default mode once the timer completes.
		 * Ignore vbus changes here.
		 */
		break;
	default:
		if (port->pwr_role == TYPEC_SINK &&
		    port->attached)
			tcpm_set_state(port, SNK_UNATTACHED, 0);
		break;
	}
}

static void _tcpm_pd_vbus_vsafe0v(struct tcpm_port *port)
{
	tcpm_log_force(port, "VBUS VSAFE0V");
	port->vbus_vsafe0v = true;
	switch (port->state) {
	case SRC_HARD_RESET_VBUS_OFF:
		/*
		 * After establishing the vSafe0V voltage condition on VBUS, the Source Shall wait
		 * tSrcRecover before re-applying VCONN and restoring VBUS to vSafe5V.
		 */
		tcpm_set_state(port, SRC_HARD_RESET_VBUS_ON, PD_T_SRC_RECOVER);
		break;
	case SRC_ATTACH_WAIT:
		if (tcpm_port_is_source(port))
			tcpm_set_state(port, tcpm_try_snk(port) ? SNK_TRY : SRC_ATTACHED,
				       PD_T_CC_DEBOUNCE);
		break;
	default:
		break;
	}
}

static void _tcpm_pd_hard_reset(struct tcpm_port *port)
{
	tcpm_log_force(port, "Received hard reset");
	if (port->ams != NONE_AMS)
		port->ams = NONE_AMS;
	if (port->hard_reset_count < PD_N_HARD_RESET_COUNT)
		port->ams = HARD_RESET;
	/*
	 * If we keep receiving hard reset requests, executing the hard reset
	 * must have failed. Revert to error recovery if that happens.
	 */
	tcpm_set_state(port,
		       port->hard_reset_count < PD_N_HARD_RESET_COUNT ?
				HARD_RESET_START : ERROR_RECOVERY,
		       0);
}

static void tcpm_pd_event_handler(struct work_struct *work)
{
	struct tcpm_port *port = container_of(work, struct tcpm_port,
					      event_work);
	u32 events;

	mutex_lock(&port->lock);

	spin_lock(&port->pd_event_lock);
	while (port->pd_events) {
		events = port->pd_events;
		port->pd_events = 0;
		spin_unlock(&port->pd_event_lock);
		if (events & TCPM_RESET_EVENT)
			_tcpm_pd_hard_reset(port);
		if (events & TCPM_VBUS_EVENT) {
			bool vbus;

			vbus = port->tcpc->get_vbus(port->tcpc);
			if (vbus) {
				_tcpm_pd_vbus_on(port);
			} else {
				_tcpm_pd_vbus_off(port);
				/*
				 * When TCPC does not support detecting vsafe0v voltage level,
				 * treat vbus absent as vsafe0v. Else invoke is_vbus_vsafe0v
				 * to see if vbus has discharge to VSAFE0V.
				 */
				if (!port->tcpc->is_vbus_vsafe0v ||
				    port->tcpc->is_vbus_vsafe0v(port->tcpc))
					_tcpm_pd_vbus_vsafe0v(port);
			}
		}
		if (events & TCPM_CC_EVENT) {
			enum typec_cc_status cc1, cc2;

			if (port->tcpc->get_cc(port->tcpc, &cc1, &cc2) == 0)
				_tcpm_cc_change(port, cc1, cc2);
		}
		if (events & TCPM_PORT_RESET_EVENT)
			tcpm_set_state(port, PORT_RESET, 0);

		spin_lock(&port->pd_event_lock);
	}
	spin_unlock(&port->pd_event_lock);
	mutex_unlock(&port->lock);
}

void tcpm_port_reset(struct tcpm_port *port)
{
	spin_lock(&port->pd_event_lock);
	port->pd_events = TCPM_PORT_RESET_EVENT;
	spin_unlock(&port->pd_event_lock);
	queue_work(port->wq, &port->event_work);
}
EXPORT_SYMBOL_GPL(tcpm_port_reset);

void tcpm_cc_change(struct tcpm_port *port)
{
	spin_lock(&port->pd_event_lock);
	port->pd_events |= TCPM_CC_EVENT;
	spin_unlock(&port->pd_event_lock);
	queue_work(port->wq, &port->event_work);
}
EXPORT_SYMBOL_GPL(tcpm_cc_change);

void tcpm_vbus_change(struct tcpm_port *port)
{
	spin_lock(&port->pd_event_lock);
	port->pd_events |= TCPM_VBUS_EVENT;
	spin_unlock(&port->pd_event_lock);
	queue_work(port->wq, &port->event_work);
}
EXPORT_SYMBOL_GPL(tcpm_vbus_change);

void tcpm_pd_hard_reset(struct tcpm_port *port)
{
	spin_lock(&port->pd_event_lock);
	port->pd_events = TCPM_RESET_EVENT;
	spin_unlock(&port->pd_event_lock);
	queue_work(port->wq, &port->event_work);
}
EXPORT_SYMBOL_GPL(tcpm_pd_hard_reset);

static int tcpm_dr_set(const struct typec_capability *cap,
		       enum typec_data_role data)
{
	struct tcpm_port *port = typec_cap_to_tcpm(cap);
	int ret;

	mutex_lock(&port->swap_lock);
	mutex_lock(&port->lock);

	if (port->port_type != TYPEC_PORT_DRP || port->vpd_connected) {
		ret = -EINVAL;
		goto port_unlock;
	}
	if (port->state != SRC_READY && port->state != SNK_READY) {
		ret = -EAGAIN;
		goto port_unlock;
	}

	if (port->data_role == data) {
		ret = 0;
		goto port_unlock;
	}

	/*
	 * XXX
	 * 6.3.9: If an alternate mode is active, a request to swap
	 * alternate modes shall trigger a port reset.
	 * Reject data role swap request in this case.
	 */

	if (!port->pd_capable) {
		/*
		 * If the partner is not PD capable, reset the port to
		 * trigger a role change. This can only work if a preferred
		 * role is configured, and if it matches the requested role.
		 */
		if (port->try_role == TYPEC_NO_PREFERRED_ROLE ||
		    port->try_role == port->pwr_role) {
			ret = -EINVAL;
			goto port_unlock;
		}
		port->non_pd_role_swap = true;
		tcpm_set_state(port, PORT_RESET, 0);
	} else {
		port->upcoming_state = DR_SWAP_SEND;
		ret = tcpm_ams_start(port, DATA_ROLE_SWAP);
		if (ret == -EAGAIN) {
			port->upcoming_state = INVALID_STATE;
			goto port_unlock;
		}
	}

	port->swap_status = 0;
	port->swap_pending = true;
	reinit_completion(&port->swap_complete);
	mutex_unlock(&port->lock);

	if (!wait_for_completion_timeout(&port->swap_complete,
				msecs_to_jiffies(PD_ROLE_SWAP_TIMEOUT)))
		ret = -ETIMEDOUT;
	else
		ret = port->swap_status;

	port->non_pd_role_swap = false;
	goto swap_unlock;

port_unlock:
	mutex_unlock(&port->lock);
swap_unlock:
	mutex_unlock(&port->swap_lock);
	return ret;
}

static int tcpm_pr_set(const struct typec_capability *cap,
		       enum typec_role role)
{
	struct tcpm_port *port = typec_cap_to_tcpm(cap);
	int ret;

	mutex_lock(&port->swap_lock);
	mutex_lock(&port->lock);

	if (port->port_type != TYPEC_PORT_DRP || port->vpd_connected) {
		ret = -EINVAL;
		goto port_unlock;
	}
	if (port->state != SRC_READY && port->state != SNK_READY) {
		ret = -EAGAIN;
		goto port_unlock;
	}

	if (role == port->pwr_role) {
		ret = 0;
		goto port_unlock;
	}

	port->upcoming_state = PR_SWAP_SEND;
	ret = tcpm_ams_start(port, POWER_ROLE_SWAP);
	if (ret == -EAGAIN) {
		port->upcoming_state = INVALID_STATE;
		goto port_unlock;
	}

	port->swap_status = 0;
	port->swap_pending = true;
	reinit_completion(&port->swap_complete);
	mutex_unlock(&port->lock);

	if (!wait_for_completion_timeout(&port->swap_complete,
				msecs_to_jiffies(PD_ROLE_SWAP_TIMEOUT)))
		ret = -ETIMEDOUT;
	else
		ret = port->swap_status;

	goto swap_unlock;

port_unlock:
	mutex_unlock(&port->lock);
swap_unlock:
	mutex_unlock(&port->swap_lock);
	return ret;
}

static int tcpm_vconn_set(const struct typec_capability *cap,
			  enum typec_role role)
{
	struct tcpm_port *port = typec_cap_to_tcpm(cap);
	int ret;

	mutex_lock(&port->swap_lock);
	mutex_lock(&port->lock);

	if (port->vpd_connected) {
		ret = -EINVAL;
		goto port_unlock;
	}

	if (port->state != SRC_READY && port->state != SNK_READY) {
		ret = -EAGAIN;
		goto port_unlock;
	}

	if (role == port->vconn_role) {
		ret = 0;
		goto port_unlock;
	}

	port->upcoming_state = VCONN_SWAP_SEND;
	ret = tcpm_ams_start(port, VCONN_SWAP);
	if (ret == -EAGAIN) {
		port->upcoming_state = INVALID_STATE;
		goto port_unlock;
	}

	port->swap_status = 0;
	port->swap_pending = true;
	reinit_completion(&port->swap_complete);
	mutex_unlock(&port->lock);

	if (!wait_for_completion_timeout(&port->swap_complete,
				msecs_to_jiffies(PD_ROLE_SWAP_TIMEOUT)))
		ret = -ETIMEDOUT;
	else
		ret = port->swap_status;

	goto swap_unlock;

port_unlock:
	mutex_unlock(&port->lock);
swap_unlock:
	mutex_unlock(&port->swap_lock);
	return ret;
}

static int tcpm_try_role(const struct typec_capability *cap, int role)
{
	struct tcpm_port *port = typec_cap_to_tcpm(cap);
	struct tcpc_dev	*tcpc = port->tcpc;
	int ret = 0;

	mutex_lock(&port->lock);
	if (tcpc->try_role)
		ret = tcpc->try_role(tcpc, role);
	if (!ret && (!tcpc->config || !tcpc->config->try_role_hw))
		port->try_role = role;
	port->try_src_count = 0;
	port->try_snk_count = 0;
	mutex_unlock(&port->lock);

	return ret;
}

static int tcpm_pps_set_op_curr(struct tcpm_port *port, u16 req_op_curr)
{
	unsigned int target_mw;
	int ret;

	mutex_lock(&port->swap_lock);
	mutex_lock(&port->lock);

	if (!port->pps_data.active) {
		ret = -EOPNOTSUPP;
		goto port_unlock;
	}

	if (port->state != SNK_READY) {
		ret = -EAGAIN;
		goto port_unlock;
	}

	if (req_op_curr > port->pps_data.max_curr) {
		ret = -EINVAL;
		goto port_unlock;
	}

	target_mw = (req_op_curr * port->supply_voltage) / 1000;
	if (target_mw < port->operating_snk_mw) {
		ret = -EINVAL;
		goto port_unlock;
	}

	port->upcoming_state = SNK_NEGOTIATE_PPS_CAPABILITIES;
	ret = tcpm_ams_start(port, POWER_NEGOTIATION);
	if (ret == -EAGAIN) {
		port->upcoming_state = INVALID_STATE;
		goto port_unlock;
	}

	/* Round down operating current to align with PPS valid steps */
	req_op_curr = req_op_curr - (req_op_curr % RDO_PROG_CURR_MA_STEP);

	reinit_completion(&port->pps_complete);
	port->pps_data.req_op_curr = req_op_curr;
	port->pps_status = 0;
	port->pps_pending = true;
	mutex_unlock(&port->lock);

	if (!wait_for_completion_timeout(&port->pps_complete,
				msecs_to_jiffies(PD_PPS_CTRL_TIMEOUT)))
		ret = -ETIMEDOUT;
	else
		ret = port->pps_status;

	goto swap_unlock;

port_unlock:
	mutex_unlock(&port->lock);
swap_unlock:
	mutex_unlock(&port->swap_lock);

	return ret;
}

static int tcpm_pps_set_out_volt(struct tcpm_port *port, u16 req_out_volt)
{
	unsigned int target_mw;
	int ret;

	mutex_lock(&port->swap_lock);
	mutex_lock(&port->lock);

	if (!port->pps_data.active) {
		ret = -EOPNOTSUPP;
		goto port_unlock;
	}

	if (port->state != SNK_READY) {
		ret = -EAGAIN;
		goto port_unlock;
	}

	if (req_out_volt < port->pps_data.min_volt ||
	    req_out_volt > port->pps_data.max_volt) {
		ret = -EINVAL;
		goto port_unlock;
	}

	target_mw = (port->current_limit * req_out_volt) / 1000;
	if (target_mw < port->operating_snk_mw) {
		ret = -EINVAL;
		goto port_unlock;
	}

	port->upcoming_state = SNK_NEGOTIATE_PPS_CAPABILITIES;
	ret = tcpm_ams_start(port, POWER_NEGOTIATION);
	if (ret == -EAGAIN) {
		port->upcoming_state = INVALID_STATE;
		goto port_unlock;
	}

	/* Round down output voltage to align with PPS valid steps */
	req_out_volt = req_out_volt - (req_out_volt % RDO_PROG_VOLT_MV_STEP);

	reinit_completion(&port->pps_complete);
	port->pps_data.req_out_volt = req_out_volt;
	port->pps_status = 0;
	port->pps_pending = true;
	mutex_unlock(&port->lock);

	if (!wait_for_completion_timeout(&port->pps_complete,
				msecs_to_jiffies(PD_PPS_CTRL_TIMEOUT)))
		ret = -ETIMEDOUT;
	else
		ret = port->pps_status;

	goto swap_unlock;

port_unlock:
	mutex_unlock(&port->lock);
swap_unlock:
	mutex_unlock(&port->swap_lock);

	return ret;
}

static int tcpm_pps_activate(struct tcpm_port *port, bool activate)
{
	int ret = 0;

	mutex_lock(&port->swap_lock);
	mutex_lock(&port->lock);

	if (!port->pps_data.supported) {
		ret = -EOPNOTSUPP;
		goto port_unlock;
	}

	/* Trying to deactivate PPS when already deactivated so just bail */
	if (!port->pps_data.active && !activate)
		goto port_unlock;

	if (port->state != SNK_READY) {
		ret = -EAGAIN;
		goto port_unlock;
	}

	if (activate)
		port->upcoming_state = SNK_NEGOTIATE_PPS_CAPABILITIES;
	else
		port->upcoming_state = SNK_NEGOTIATE_CAPABILITIES;
	ret = tcpm_ams_start(port, POWER_NEGOTIATION);
	if (ret == -EAGAIN) {
		port->upcoming_state = INVALID_STATE;
		goto port_unlock;
	}

	reinit_completion(&port->pps_complete);
	port->pps_status = 0;
	port->pps_pending = true;

	/* Trigger PPS request or move back to standard PDO contract */
	if (activate) {
		port->pps_data.req_out_volt = port->supply_voltage;
		port->pps_data.req_op_curr = port->current_limit;
	}
	mutex_unlock(&port->lock);

	if (!wait_for_completion_timeout(&port->pps_complete,
				msecs_to_jiffies(PD_PPS_CTRL_TIMEOUT)))
		ret = -ETIMEDOUT;
	else
		ret = port->pps_status;

	goto swap_unlock;

port_unlock:
	mutex_unlock(&port->lock);
swap_unlock:
	mutex_unlock(&port->swap_lock);

	return ret;
}

static void tcpm_init(struct tcpm_port *port)
{
	enum typec_cc_status cc1, cc2;

	port->tcpc->init(port->tcpc);

	tcpm_reset_port(port);

	/*
	 * XXX
	 * Should possibly wait for VBUS to settle if it was enabled locally
	 * since tcpm_reset_port() will disable VBUS.
	 */
	port->vbus_present = port->tcpc->get_vbus(port->tcpc);
	if (port->vbus_present)
		port->vbus_never_low = true;

	/*
	 * 1. When vbus_present is true, voltage on VBUS is already at VSAFE5V.
	 * So implicitly vbus_vsafe0v = false.
	 *
	 * 2. When vbus_present is false and TCPC does NOT support querying
	 * vsafe0v status, then, it's best to assume vbus is at VSAFE0V i.e.
	 * vbus_vsafe0v is true.
	 *
	 * 3. When vbus_present is false and TCPC does support querying vsafe0v,
	 * then, query tcpc for vsafe0v status.
	 */
	if (port->vbus_present)
		port->vbus_vsafe0v = false;
	else if (!port->tcpc->is_vbus_vsafe0v)
		port->vbus_vsafe0v = true;
	else
		port->vbus_vsafe0v = port->tcpc->is_vbus_vsafe0v(port->tcpc);

	tcpm_set_state(port, tcpm_default_state(port), 0);

	if (port->tcpc->get_cc(port->tcpc, &cc1, &cc2) == 0)
		_tcpm_cc_change(port, cc1, cc2);

	/*
	 * Some adapters need a clean slate at startup, and won't recover
	 * otherwise. So do not try to be fancy and force a clean disconnect.
	 */
	tcpm_set_state(port, PORT_RESET, 0);
}

static int tcpm_port_type_set(const struct typec_capability *cap,
			      enum typec_port_type type)
{
	struct tcpm_port *port = typec_cap_to_tcpm(cap);

	mutex_lock(&port->lock);
	if (type == port->port_type)
		goto port_unlock;

	port->port_type = type;

	if (!port->connected) {
		tcpm_set_state(port, PORT_RESET, 0);
	} else if (type == TYPEC_PORT_SNK) {
		if (!(port->pwr_role == TYPEC_SINK &&
		      port->data_role == TYPEC_DEVICE))
			tcpm_set_state(port, PORT_RESET, 0);
	} else if (type == TYPEC_PORT_SRC) {
		if (!(port->pwr_role == TYPEC_SOURCE &&
		      port->data_role == TYPEC_HOST))
			tcpm_set_state(port, PORT_RESET, 0);
	}

port_unlock:
	mutex_unlock(&port->lock);
	return 0;
}

void tcpm_tcpc_reset(struct tcpm_port *port)
{
	mutex_lock(&port->lock);
	/* XXX: Maintain PD connection if possible? */
	tcpm_init(port);
	mutex_unlock(&port->lock);
}
EXPORT_SYMBOL_GPL(tcpm_tcpc_reset);

static int tcpm_copy_pdos(u32 *dest_pdo, const u32 *src_pdo,
			  unsigned int nr_pdo)
{
	unsigned int i;

	if (nr_pdo > PDO_MAX_OBJECTS)
		nr_pdo = PDO_MAX_OBJECTS;

	for (i = 0; i < nr_pdo; i++)
		dest_pdo[i] = src_pdo[i];

	return nr_pdo;
}

static int tcpm_copy_vdos(u32 *dest_vdo, const u32 *src_vdo,
			  unsigned int nr_vdo)
{
	unsigned int i;

	if (nr_vdo > VDO_MAX_OBJECTS)
		nr_vdo = VDO_MAX_OBJECTS;

	for (i = 0; i < nr_vdo; i++)
		dest_vdo[i] = src_vdo[i];

	return nr_vdo;
}

static int tcpm_fw_get_caps(struct tcpm_port *port,
			    struct fwnode_handle *fwnode)
{
	const char *cap_str;
	int ret;
	u32 mw;

	if (!fwnode)
		return -EINVAL;

	/* USB data support is optional */
	ret = fwnode_property_read_string(fwnode, "data-role", &cap_str);
	if (ret == 0) {
		ret = typec_find_port_data_role(cap_str);
		if (ret < 0)
			return ret;
		port->typec_caps.data = ret;
	}

	ret = fwnode_property_read_string(fwnode, "power-role", &cap_str);
	if (ret < 0)
		return ret;

	ret = typec_find_port_power_role(cap_str);
	if (ret < 0)
		return ret;
	port->typec_caps.type = ret;
	port->port_type = port->typec_caps.type;

	/* For now unchunked msg is not supported. */
	port->unchunk_supp = false;
	port->psnkstdby_after_accept = fwnode_property_read_bool(fwnode, "psnkstdby-after-accept");

	if (port->port_type == TYPEC_PORT_SNK)
		goto sink;

	/* Get source pdos */
	ret = fwnode_property_count_u32(fwnode, "source-pdos");
	if (ret <= 0)
		return -EINVAL;

	port->nr_src_pdo = min(ret, PDO_MAX_OBJECTS);
	ret = fwnode_property_read_u32_array(fwnode, "source-pdos",
					     port->src_pdo, port->nr_src_pdo);
	if ((ret < 0) || tcpm_validate_caps(port, port->src_pdo,
					    port->nr_src_pdo))
		return -EINVAL;

	if (port->port_type == TYPEC_PORT_SRC)
		return 0;

	/* Get the preferred power role for DRP */
	ret = fwnode_property_read_string(fwnode, "try-power-role", &cap_str);
	if (ret < 0)
		return ret;

	port->typec_caps.prefer_role = typec_find_power_role(cap_str);
	if (port->typec_caps.prefer_role < 0)
		return -EINVAL;
sink:
	/* Get sink pdos */
	ret = fwnode_property_count_u32(fwnode, "sink-pdos");
	if (ret <= 0)
		return -EINVAL;

	port->nr_snk_pdo = min(ret, PDO_MAX_OBJECTS);
	ret = fwnode_property_read_u32_array(fwnode, "sink-pdos",
					     port->snk_pdo, port->nr_snk_pdo);
	if ((ret < 0) || tcpm_validate_caps(port, port->snk_pdo,
					    port->nr_snk_pdo))
		return -EINVAL;

	if (fwnode_property_read_u32(fwnode, "op-sink-microwatt", &mw) < 0)
		return -EINVAL;
	port->operating_snk_mw = mw / 1000;

	port->self_powered = fwnode_property_read_bool(fwnode, "self-powered");

	ret = fwnode_property_read_u32_array(fwnode, "sink-vdos", NULL, 0);
	if (ret <= 0 && ret != -EINVAL) {
		return -EINVAL;
	} else if (ret > 0) {
		port->nr_snk_vdo = min(ret, VDO_MAX_OBJECTS);
		ret = fwnode_property_read_u32_array(fwnode, "sink-vdos",
						     port->snk_vdo,
						     port->nr_snk_vdo);
		if (ret < 0)
			return -EINVAL;
	}

	return 0;
}

int tcpm_update_source_capabilities(struct tcpm_port *port, const u32 *pdo,
				    unsigned int nr_pdo)
{
	int ret = 0;

	if (tcpm_validate_caps(port, pdo, nr_pdo))
		return -EINVAL;

	mutex_lock(&port->lock);
	port->nr_src_pdo = tcpm_copy_pdos(port->src_pdo, pdo, nr_pdo);
	switch (port->state) {
	case SRC_UNATTACHED:
	case SRC_ATTACH_WAIT:
	case SRC_TRYWAIT:
		tcpm_set_cc(port, tcpm_rp_cc(port));
		break;
	case SRC_SEND_CAPABILITIES:
	case SRC_NEGOTIATE_CAPABILITIES:
	case SRC_READY:
	case SRC_WAIT_NEW_CAPABILITIES:
		port->upcoming_state = SRC_SEND_CAPABILITIES;
		ret = tcpm_ams_start(port, POWER_NEGOTIATION);
		if (ret == -EAGAIN) {
			port->upcoming_state = INVALID_STATE;
			break;
		}
		break;
	default:
		break;
	}
	mutex_unlock(&port->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(tcpm_update_source_capabilities);

int tcpm_update_sink_capabilities(struct tcpm_port *port, const u32 *pdo,
				  unsigned int nr_pdo,
				  unsigned int operating_snk_mw)
{
	int ret = 0;

	if (tcpm_validate_caps(port, pdo, nr_pdo))
		return -EINVAL;

	mutex_lock(&port->lock);
	port->nr_snk_pdo = tcpm_copy_pdos(port->snk_pdo, pdo, nr_pdo);
	port->operating_snk_mw = operating_snk_mw;

	switch (port->state) {
	case SNK_NEGOTIATE_CAPABILITIES:
	case SNK_NEGOTIATE_PPS_CAPABILITIES:
	case SNK_READY:
	case SNK_TRANSITION_SINK:
	case SNK_TRANSITION_SINK_VBUS:
		if (port->pps_data.active)
			port->upcoming_state = SNK_NEGOTIATE_PPS_CAPABILITIES;
		else if (port->pd_capable)
			port->upcoming_state = SNK_NEGOTIATE_CAPABILITIES;
		else
			break;

		port->update_sink_caps = true;

		ret = tcpm_ams_start(port, POWER_NEGOTIATION);
		if (ret == -EAGAIN) {
			port->upcoming_state = INVALID_STATE;
			break;
		}

		tcpm_log(port, "in PR_SWAP := false");
		port->tcpc->set_in_pr_swap(port->tcpc, false);
		break;
	default:
		break;
	}
	mutex_unlock(&port->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(tcpm_update_sink_capabilities);

/*
 * Don't call this function in interrupt context. Caller needs to free the
 * memory by calling tcpm_put_partner_src_caps.
 */
int tcpm_get_partner_src_caps(struct tcpm_port *port, u32 **src_pdo)
{
	unsigned int nr_pdo;

	mutex_lock(&port->lock);
	if (port->nr_source_caps == 0) {
		mutex_unlock(&port->lock);
		return -ENODATA;
	}

	*src_pdo = kcalloc(port->nr_source_caps, sizeof(u32), GFP_KERNEL);
	if (!src_pdo) {
		mutex_unlock(&port->lock);
		return -ENOMEM;
	}

	nr_pdo = tcpm_copy_pdos(*src_pdo, port->source_caps,
				port->nr_source_caps);
	mutex_unlock(&port->lock);
	return nr_pdo;
}
EXPORT_SYMBOL_GPL(tcpm_get_partner_src_caps);

void tcpm_put_partner_src_caps(u32 **src_pdo)
{
	kfree(*src_pdo);
	*src_pdo = NULL;
}
EXPORT_SYMBOL_GPL(tcpm_put_partner_src_caps);

/* Power Supply access to expose source power information */
enum tcpm_psy_online_states {
	TCPM_PSY_OFFLINE = 0,
	TCPM_PSY_FIXED_ONLINE,
	TCPM_PSY_PROG_ONLINE,
};

static enum power_supply_property tcpm_psy_props[] = {
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static int tcpm_psy_get_online(struct tcpm_port *port,
			       union power_supply_propval *val)
{
	if (port->vbus_charge) {
		if (port->pps_data.active)
			val->intval = TCPM_PSY_PROG_ONLINE;
		else
			val->intval = TCPM_PSY_FIXED_ONLINE;
	} else {
		val->intval = TCPM_PSY_OFFLINE;
	}

	return 0;
}

static int tcpm_psy_get_voltage_min(struct tcpm_port *port,
				    union power_supply_propval *val)
{
	if (port->pps_data.active)
		val->intval = port->pps_data.min_volt * 1000;
	else
		val->intval = port->supply_voltage * 1000;

	return 0;
}

static int tcpm_psy_get_voltage_max(struct tcpm_port *port,
				    union power_supply_propval *val)
{
	if (port->pps_data.active)
		val->intval = port->pps_data.max_volt * 1000;
	else
		val->intval = port->supply_voltage * 1000;

	return 0;
}

static int tcpm_psy_get_voltage_now(struct tcpm_port *port,
				    union power_supply_propval *val)
{
	val->intval = port->supply_voltage * 1000;

	return 0;
}

static int tcpm_psy_get_current_max(struct tcpm_port *port,
				    union power_supply_propval *val)
{
	if (port->pps_data.active)
		val->intval = port->pps_data.max_curr * 1000;
	else
		val->intval = port->current_limit * 1000;

	return 0;
}

static int tcpm_psy_get_current_now(struct tcpm_port *port,
				    union power_supply_propval *val)
{
	val->intval = port->current_limit * 1000;

	return 0;
}

static int tcpm_psy_get_prop(struct power_supply *psy,
			     enum power_supply_property psp,
			     union power_supply_propval *val)
{
	struct tcpm_port *port = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = port->usb_type;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = tcpm_psy_get_online(port, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		ret = tcpm_psy_get_voltage_min(port, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		ret = tcpm_psy_get_voltage_max(port, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = tcpm_psy_get_voltage_now(port, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = tcpm_psy_get_current_max(port, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = tcpm_psy_get_current_now(port, val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int tcpm_psy_set_online(struct tcpm_port *port,
			       const union power_supply_propval *val)
{
	int ret;

	switch (val->intval) {
	case TCPM_PSY_FIXED_ONLINE:
		ret = tcpm_pps_activate(port, false);
		break;
	case TCPM_PSY_PROG_ONLINE:
		ret = tcpm_pps_activate(port, true);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int tcpm_psy_set_prop(struct power_supply *psy,
			     enum power_supply_property psp,
			     const union power_supply_propval *val)
{
	struct tcpm_port *port = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = tcpm_psy_set_online(port, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (val->intval < port->pps_data.min_volt * 1000 ||
		    val->intval > port->pps_data.max_volt * 1000)
			ret = -EINVAL;
		else
			ret = tcpm_pps_set_out_volt(port, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (val->intval > port->pps_data.max_curr * 1000)
			ret = -EINVAL;
		else
			ret = tcpm_pps_set_op_curr(port, val->intval / 1000);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int tcpm_psy_prop_writeable(struct power_supply *psy,
				   enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return 1;
	default:
		return 0;
	}
}

static enum power_supply_usb_type tcpm_psy_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_PPS,
};

static const char *tcpm_psy_name_prefix = "tcpm-source-psy-";

static int devm_tcpm_psy_register(struct tcpm_port *port)
{
	struct power_supply_config psy_cfg = {};
	const char *port_dev_name = dev_name(port->dev);
	size_t psy_name_len = strlen(tcpm_psy_name_prefix) +
				     strlen(port_dev_name) + 1;
	char *psy_name;

	psy_cfg.drv_data = port;
	psy_cfg.fwnode = dev_fwnode(port->dev);
	psy_name = devm_kzalloc(port->dev, psy_name_len, GFP_KERNEL);
	if (!psy_name)
		return -ENOMEM;

	snprintf(psy_name, psy_name_len, "%s%s", tcpm_psy_name_prefix,
		 port_dev_name);
	port->psy_desc.name = psy_name;
	port->psy_desc.type = POWER_SUPPLY_TYPE_USB,
	port->psy_desc.usb_types = tcpm_psy_usb_types;
	port->psy_desc.num_usb_types = ARRAY_SIZE(tcpm_psy_usb_types);
	port->psy_desc.properties = tcpm_psy_props,
	port->psy_desc.num_properties = ARRAY_SIZE(tcpm_psy_props),
	port->psy_desc.get_property = tcpm_psy_get_prop,
	port->psy_desc.set_property = tcpm_psy_set_prop,
	port->psy_desc.property_is_writeable = tcpm_psy_prop_writeable,

	port->usb_type = POWER_SUPPLY_USB_TYPE_C;

	port->psy = devm_power_supply_register(port->dev, &port->psy_desc,
					       &psy_cfg);

	return PTR_ERR_OR_ZERO(port->psy);
}

static int tcpm_copy_caps(struct tcpm_port *port,
			  const struct tcpc_config *tcfg)
{
	if (tcpm_validate_caps(port, tcfg->src_pdo, tcfg->nr_src_pdo) ||
	    tcpm_validate_caps(port, tcfg->snk_pdo, tcfg->nr_snk_pdo))
		return -EINVAL;

	port->nr_src_pdo = tcpm_copy_pdos(port->src_pdo, tcfg->src_pdo,
					  tcfg->nr_src_pdo);
	port->nr_snk_pdo = tcpm_copy_pdos(port->snk_pdo, tcfg->snk_pdo,
					  tcfg->nr_snk_pdo);

	port->nr_snk_vdo = tcpm_copy_vdos(port->snk_vdo, tcfg->snk_vdo,
					  tcfg->nr_snk_vdo);

	port->operating_snk_mw = tcfg->operating_snk_mw;

	port->typec_caps.prefer_role = tcfg->default_role;
	port->typec_caps.type = tcfg->type;
	port->typec_caps.data = tcfg->data;
	port->self_powered = tcfg->self_powered;

	return 0;
}

struct tcpm_port *tcpm_register_port(struct device *dev, struct tcpc_dev *tcpc)
{
	struct tcpm_port *port;
	int i, err;

	if (!dev || !tcpc ||
	    !tcpc->get_vbus || !tcpc->set_cc || !tcpc->get_cc ||
	    !tcpc->set_polarity || !tcpc->set_vconn || !tcpc->set_vbus ||
	    !tcpc->set_pd_rx || !tcpc->set_roles || !tcpc->pd_transmit)
		return ERR_PTR(-EINVAL);

	port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return ERR_PTR(-ENOMEM);

	port->dev = dev;
	port->tcpc = tcpc;

	mutex_init(&port->lock);
	mutex_init(&port->swap_lock);

	port->wq = create_singlethread_workqueue(dev_name(dev));
	if (!port->wq)
		return ERR_PTR(-ENOMEM);
	INIT_DELAYED_WORK(&port->state_machine, tcpm_state_machine_work);
	INIT_DELAYED_WORK(&port->vdm_state_machine, vdm_state_machine_work);
	INIT_WORK(&port->event_work, tcpm_pd_event_handler);
	INIT_DELAYED_WORK(&port->disc_id_work, disc_id_work);

	spin_lock_init(&port->pd_event_lock);

	init_completion(&port->tx_complete);
	init_completion(&port->swap_complete);
	init_completion(&port->pps_complete);
	tcpm_debugfs_init(port);

#ifndef CONFIG_DEBUG_FS
	port->log = logbuffer_register("tcpm");
	if (IS_ERR_OR_NULL(port->log)) {
		pr_err("tcpm: failed to obtain logbuffer instance\n");
		port->log = NULL;
	}
#endif

	err = tcpm_fw_get_caps(port, tcpc->fwnode);
	if ((err < 0) && tcpc->config)
		err = tcpm_copy_caps(port, tcpc->config);
	if (err < 0)
		goto out_destroy_wq;

	if (!tcpc->config || !tcpc->config->try_role_hw)
		port->try_role = port->typec_caps.prefer_role;
	else
		port->try_role = TYPEC_NO_PREFERRED_ROLE;

	if (tcpc->config && tcpc->port_type_override)
		port->typec_caps.type = tcpc->config->type;

	port->typec_caps.fwnode = tcpc->fwnode;
	port->typec_caps.revision = 0x0120;	/* Type-C spec release 1.2 */
	port->typec_caps.pd_revision = 0x0300;	/* USB-PD spec release 3.0 */
	port->typec_caps.dr_set = tcpm_dr_set;
	port->typec_caps.pr_set = tcpm_pr_set;
	port->typec_caps.vconn_set = tcpm_vconn_set;
	port->typec_caps.try_role = tcpm_try_role;
	port->typec_caps.port_type_set = tcpm_port_type_set;

	port->partner_desc.identity = &port->partner_ident;
	port->port_type = port->typec_caps.type;

	port->role_sw = usb_role_switch_get(port->dev);
	if (IS_ERR(port->role_sw)) {
		err = PTR_ERR(port->role_sw);
		goto out_destroy_wq;
	}

	err = devm_tcpm_psy_register(port);
	if (err)
		goto out_role_sw_put;

	port->typec_port = typec_register_port(port->dev, &port->typec_caps);
	if (IS_ERR(port->typec_port)) {
		err = PTR_ERR(port->typec_port);
		goto out_role_sw_put;
	}

	if (tcpc->config && tcpc->config->alt_modes) {
		const struct typec_altmode_desc *paltmode = tcpc->config->alt_modes;

		i = 0;
		while (paltmode->svid && i < ARRAY_SIZE(port->port_altmode)) {
			struct typec_altmode *alt;

			alt = typec_port_register_altmode(port->typec_port,
							  paltmode);
			if (IS_ERR(alt)) {
				tcpm_log(port,
					 "%s: failed to register port alternate mode 0x%x",
					 dev_name(dev), paltmode->svid);
				break;
			}
			typec_altmode_set_drvdata(alt, port);
			alt->ops = &tcpm_altmode_ops;
			port->port_altmode[i] = alt;
			i++;
			paltmode++;
		}
	}

	port->chunk_event = kzalloc(sizeof(struct pd_chunk_event), GFP_KERNEL);
	if (!port->chunk_event) {
		tcpm_log(port, "%s: failed to alloc chunk_event");
		goto out_destroy_wq;
	}

	INIT_DELAYED_WORK(&port->chunk_state_machine,
			  chunk_state_machine_work);

	mutex_lock(&port->lock);
	tcpm_init(port);
	mutex_unlock(&port->lock);

	tcpm_log(port, "%s: registered", dev_name(dev));
	return port;

out_role_sw_put:
	usb_role_switch_put(port->role_sw);
out_destroy_wq:
	tcpm_debugfs_exit(port);
	if (port->log)
		logbuffer_unregister(port->log);
	destroy_workqueue(port->wq);
	return ERR_PTR(err);
}
EXPORT_SYMBOL_GPL(tcpm_register_port);

void tcpm_unregister_port(struct tcpm_port *port)
{
	int i;

	tcpm_reset_port(port);
	for (i = 0; i < ARRAY_SIZE(port->port_altmode); i++)
		typec_unregister_altmode(port->port_altmode[i]);
	typec_unregister_port(port->typec_port);
	usb_role_switch_put(port->role_sw);
	tcpm_debugfs_exit(port);
	if (port->log)
		logbuffer_unregister(port->log);
	destroy_workqueue(port->wq);
	kfree(port->chunk_event);
}
EXPORT_SYMBOL_GPL(tcpm_unregister_port);

MODULE_AUTHOR("Guenter Roeck <groeck@chromium.org>");
MODULE_DESCRIPTION("USB Type-C Port Manager");
MODULE_LICENSE("GPL");
