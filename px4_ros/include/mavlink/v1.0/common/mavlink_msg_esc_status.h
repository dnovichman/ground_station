// MESSAGE ESC_STATUS PACKING

#define MAVLINK_MSG_ID_ESC_STATUS 116

typedef struct __mavlink_esc_status_t
{
 uint16_t current[4]; ///< Current in each ESC scaled to unit16(+-INT16_MAX)
 uint16_t rpm[4]; ///< RPM of each ESC scaled to uint16 (0..UINT16_MAX)
} mavlink_esc_status_t;

#define MAVLINK_MSG_ID_ESC_STATUS_LEN 16
#define MAVLINK_MSG_ID_116_LEN 16

#define MAVLINK_MSG_ID_ESC_STATUS_CRC 52
#define MAVLINK_MSG_ID_116_CRC 52

#define MAVLINK_MSG_ESC_STATUS_FIELD_CURRENT_LEN 4
#define MAVLINK_MSG_ESC_STATUS_FIELD_RPM_LEN 4

#define MAVLINK_MESSAGE_INFO_ESC_STATUS { \
	"ESC_STATUS", \
	2, \
	{  { "current", NULL, MAVLINK_TYPE_UINT16_T, 4, 0, offsetof(mavlink_esc_status_t, current) }, \
         { "rpm", NULL, MAVLINK_TYPE_UINT16_T, 4, 8, offsetof(mavlink_esc_status_t, rpm) }, \
         } \
}


/**
 * @brief Pack a esc_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param current Current in each ESC scaled to unit16(+-INT16_MAX)
 * @param rpm RPM of each ESC scaled to uint16 (0..UINT16_MAX)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_esc_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const uint16_t *current, const uint16_t *rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ESC_STATUS_LEN];

	_mav_put_uint16_t_array(buf, 0, current, 4);
	_mav_put_uint16_t_array(buf, 8, rpm, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESC_STATUS_LEN);
#else
	mavlink_esc_status_t packet;

	mav_array_memcpy(packet.current, current, sizeof(uint16_t)*4);
	mav_array_memcpy(packet.rpm, rpm, sizeof(uint16_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESC_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ESC_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ESC_STATUS_LEN, MAVLINK_MSG_ID_ESC_STATUS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ESC_STATUS_LEN);
#endif
}

/**
 * @brief Pack a esc_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param current Current in each ESC scaled to unit16(+-INT16_MAX)
 * @param rpm RPM of each ESC scaled to uint16 (0..UINT16_MAX)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_esc_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const uint16_t *current,const uint16_t *rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ESC_STATUS_LEN];

	_mav_put_uint16_t_array(buf, 0, current, 4);
	_mav_put_uint16_t_array(buf, 8, rpm, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESC_STATUS_LEN);
#else
	mavlink_esc_status_t packet;

	mav_array_memcpy(packet.current, current, sizeof(uint16_t)*4);
	mav_array_memcpy(packet.rpm, rpm, sizeof(uint16_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESC_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ESC_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ESC_STATUS_LEN, MAVLINK_MSG_ID_ESC_STATUS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ESC_STATUS_LEN);
#endif
}

/**
 * @brief Encode a esc_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param esc_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_esc_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_esc_status_t* esc_status)
{
	return mavlink_msg_esc_status_pack(system_id, component_id, msg, esc_status->current, esc_status->rpm);
}

/**
 * @brief Encode a esc_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param esc_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_esc_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_esc_status_t* esc_status)
{
	return mavlink_msg_esc_status_pack_chan(system_id, component_id, chan, msg, esc_status->current, esc_status->rpm);
}

/**
 * @brief Send a esc_status message
 * @param chan MAVLink channel to send the message
 *
 * @param current Current in each ESC scaled to unit16(+-INT16_MAX)
 * @param rpm RPM of each ESC scaled to uint16 (0..UINT16_MAX)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_esc_status_send(mavlink_channel_t chan, const uint16_t *current, const uint16_t *rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ESC_STATUS_LEN];

	_mav_put_uint16_t_array(buf, 0, current, 4);
	_mav_put_uint16_t_array(buf, 8, rpm, 4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_STATUS, buf, MAVLINK_MSG_ID_ESC_STATUS_LEN, MAVLINK_MSG_ID_ESC_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_STATUS, buf, MAVLINK_MSG_ID_ESC_STATUS_LEN);
#endif
#else
	mavlink_esc_status_t packet;

	mav_array_memcpy(packet.current, current, sizeof(uint16_t)*4);
	mav_array_memcpy(packet.rpm, rpm, sizeof(uint16_t)*4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ESC_STATUS_LEN, MAVLINK_MSG_ID_ESC_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ESC_STATUS_LEN);
#endif
#endif
}

#endif

// MESSAGE ESC_STATUS UNPACKING


/**
 * @brief Get field current from esc_status message
 *
 * @return Current in each ESC scaled to unit16(+-INT16_MAX)
 */
static inline uint16_t mavlink_msg_esc_status_get_current(const mavlink_message_t* msg, uint16_t *current)
{
	return _MAV_RETURN_uint16_t_array(msg, current, 4,  0);
}

/**
 * @brief Get field rpm from esc_status message
 *
 * @return RPM of each ESC scaled to uint16 (0..UINT16_MAX)
 */
static inline uint16_t mavlink_msg_esc_status_get_rpm(const mavlink_message_t* msg, uint16_t *rpm)
{
	return _MAV_RETURN_uint16_t_array(msg, rpm, 4,  8);
}

/**
 * @brief Decode a esc_status message into a struct
 *
 * @param msg The message to decode
 * @param esc_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_esc_status_decode(const mavlink_message_t* msg, mavlink_esc_status_t* esc_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_esc_status_get_current(msg, esc_status->current);
	mavlink_msg_esc_status_get_rpm(msg, esc_status->rpm);
#else
	memcpy(esc_status, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ESC_STATUS_LEN);
#endif
}
