#pragma once
// MESSAGE UI_TO_PX4_MODE PACKING

#define MAVLINK_MSG_ID_UI_TO_PX4_MODE 50007


typedef struct __mavlink_ui_to_px4_mode_t {
 uint64_t timestamp; /*<  Timestamp in milliseconds since system boot*/
 uint8_t mode; /*<  Control mode (0: Manual, 1: Auto 2:, Remote)*/
} mavlink_ui_to_px4_mode_t;

#define MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN 9
#define MAVLINK_MSG_ID_UI_TO_PX4_MODE_MIN_LEN 9
#define MAVLINK_MSG_ID_50007_LEN 9
#define MAVLINK_MSG_ID_50007_MIN_LEN 9

#define MAVLINK_MSG_ID_UI_TO_PX4_MODE_CRC 61
#define MAVLINK_MSG_ID_50007_CRC 61



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UI_TO_PX4_MODE { \
    50007, \
    "UI_TO_PX4_MODE", \
    2, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ui_to_px4_mode_t, timestamp) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_ui_to_px4_mode_t, mode) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UI_TO_PX4_MODE { \
    "UI_TO_PX4_MODE", \
    2, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ui_to_px4_mode_t, timestamp) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_ui_to_px4_mode_t, mode) }, \
         } \
}
#endif

/**
 * @brief Pack a ui_to_px4_mode message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Timestamp in milliseconds since system boot
 * @param mode  Control mode (0: Manual, 1: Auto 2:, Remote)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ui_to_px4_mode_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN);
#else
    mavlink_ui_to_px4_mode_t packet;
    packet.timestamp = timestamp;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UI_TO_PX4_MODE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UI_TO_PX4_MODE_MIN_LEN, MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN, MAVLINK_MSG_ID_UI_TO_PX4_MODE_CRC);
}

/**
 * @brief Pack a ui_to_px4_mode message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Timestamp in milliseconds since system boot
 * @param mode  Control mode (0: Manual, 1: Auto 2:, Remote)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ui_to_px4_mode_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN);
#else
    mavlink_ui_to_px4_mode_t packet;
    packet.timestamp = timestamp;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UI_TO_PX4_MODE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UI_TO_PX4_MODE_MIN_LEN, MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN, MAVLINK_MSG_ID_UI_TO_PX4_MODE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UI_TO_PX4_MODE_MIN_LEN, MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN);
#endif
}

/**
 * @brief Pack a ui_to_px4_mode message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Timestamp in milliseconds since system boot
 * @param mode  Control mode (0: Manual, 1: Auto 2:, Remote)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ui_to_px4_mode_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN);
#else
    mavlink_ui_to_px4_mode_t packet;
    packet.timestamp = timestamp;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UI_TO_PX4_MODE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UI_TO_PX4_MODE_MIN_LEN, MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN, MAVLINK_MSG_ID_UI_TO_PX4_MODE_CRC);
}

/**
 * @brief Encode a ui_to_px4_mode struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ui_to_px4_mode C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ui_to_px4_mode_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ui_to_px4_mode_t* ui_to_px4_mode)
{
    return mavlink_msg_ui_to_px4_mode_pack(system_id, component_id, msg, ui_to_px4_mode->timestamp, ui_to_px4_mode->mode);
}

/**
 * @brief Encode a ui_to_px4_mode struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ui_to_px4_mode C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ui_to_px4_mode_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ui_to_px4_mode_t* ui_to_px4_mode)
{
    return mavlink_msg_ui_to_px4_mode_pack_chan(system_id, component_id, chan, msg, ui_to_px4_mode->timestamp, ui_to_px4_mode->mode);
}

/**
 * @brief Encode a ui_to_px4_mode struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param ui_to_px4_mode C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ui_to_px4_mode_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_ui_to_px4_mode_t* ui_to_px4_mode)
{
    return mavlink_msg_ui_to_px4_mode_pack_status(system_id, component_id, _status, msg,  ui_to_px4_mode->timestamp, ui_to_px4_mode->mode);
}

/**
 * @brief Send a ui_to_px4_mode message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Timestamp in milliseconds since system boot
 * @param mode  Control mode (0: Manual, 1: Auto 2:, Remote)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ui_to_px4_mode_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UI_TO_PX4_MODE, buf, MAVLINK_MSG_ID_UI_TO_PX4_MODE_MIN_LEN, MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN, MAVLINK_MSG_ID_UI_TO_PX4_MODE_CRC);
#else
    mavlink_ui_to_px4_mode_t packet;
    packet.timestamp = timestamp;
    packet.mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UI_TO_PX4_MODE, (const char *)&packet, MAVLINK_MSG_ID_UI_TO_PX4_MODE_MIN_LEN, MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN, MAVLINK_MSG_ID_UI_TO_PX4_MODE_CRC);
#endif
}

/**
 * @brief Send a ui_to_px4_mode message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ui_to_px4_mode_send_struct(mavlink_channel_t chan, const mavlink_ui_to_px4_mode_t* ui_to_px4_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ui_to_px4_mode_send(chan, ui_to_px4_mode->timestamp, ui_to_px4_mode->mode);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UI_TO_PX4_MODE, (const char *)ui_to_px4_mode, MAVLINK_MSG_ID_UI_TO_PX4_MODE_MIN_LEN, MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN, MAVLINK_MSG_ID_UI_TO_PX4_MODE_CRC);
#endif
}

#if MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ui_to_px4_mode_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint8_t(buf, 8, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UI_TO_PX4_MODE, buf, MAVLINK_MSG_ID_UI_TO_PX4_MODE_MIN_LEN, MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN, MAVLINK_MSG_ID_UI_TO_PX4_MODE_CRC);
#else
    mavlink_ui_to_px4_mode_t *packet = (mavlink_ui_to_px4_mode_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UI_TO_PX4_MODE, (const char *)packet, MAVLINK_MSG_ID_UI_TO_PX4_MODE_MIN_LEN, MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN, MAVLINK_MSG_ID_UI_TO_PX4_MODE_CRC);
#endif
}
#endif

#endif

// MESSAGE UI_TO_PX4_MODE UNPACKING


/**
 * @brief Get field timestamp from ui_to_px4_mode message
 *
 * @return  Timestamp in milliseconds since system boot
 */
static inline uint64_t mavlink_msg_ui_to_px4_mode_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field mode from ui_to_px4_mode message
 *
 * @return  Control mode (0: Manual, 1: Auto 2:, Remote)
 */
static inline uint8_t mavlink_msg_ui_to_px4_mode_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Decode a ui_to_px4_mode message into a struct
 *
 * @param msg The message to decode
 * @param ui_to_px4_mode C-struct to decode the message contents into
 */
static inline void mavlink_msg_ui_to_px4_mode_decode(const mavlink_message_t* msg, mavlink_ui_to_px4_mode_t* ui_to_px4_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ui_to_px4_mode->timestamp = mavlink_msg_ui_to_px4_mode_get_timestamp(msg);
    ui_to_px4_mode->mode = mavlink_msg_ui_to_px4_mode_get_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN? msg->len : MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN;
        memset(ui_to_px4_mode, 0, MAVLINK_MSG_ID_UI_TO_PX4_MODE_LEN);
    memcpy(ui_to_px4_mode, _MAV_PAYLOAD(msg), len);
#endif
}
