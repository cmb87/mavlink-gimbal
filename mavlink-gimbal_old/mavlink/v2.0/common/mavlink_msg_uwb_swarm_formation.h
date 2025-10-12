#pragma once
// MESSAGE UWB_SWARM_FORMATION PACKING

#define MAVLINK_MSG_ID_UWB_SWARM_FORMATION 303


typedef struct __mavlink_uwb_swarm_formation_t {
 uint64_t usec; /*< [us] Timestamp (UNIX time or since system boot)*/
 float x; /*< [m] X Position*/
 float y; /*< [m] Y Position*/
 float z; /*< [m] Z Position*/
 float roll; /*< [rad] Roll angle (-pi..+pi)*/
 float pitch; /*< [rad] Pitch angle (-pi..+pi)*/
 float yaw; /*< [rad] Yaw angle (-pi..+pi)*/
 uint8_t id; /*<  Swarm ID of swarm member*/
 uint8_t formation_type; /*<  Type of formation strategy*/
} mavlink_uwb_swarm_formation_t;

#define MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN 34
#define MAVLINK_MSG_ID_UWB_SWARM_FORMATION_MIN_LEN 34
#define MAVLINK_MSG_ID_303_LEN 34
#define MAVLINK_MSG_ID_303_MIN_LEN 34

#define MAVLINK_MSG_ID_UWB_SWARM_FORMATION_CRC 11
#define MAVLINK_MSG_ID_303_CRC 11



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UWB_SWARM_FORMATION { \
    303, \
    "UWB_SWARM_FORMATION", \
    9, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_uwb_swarm_formation_t, usec) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_uwb_swarm_formation_t, id) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_uwb_swarm_formation_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_uwb_swarm_formation_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_uwb_swarm_formation_t, z) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_uwb_swarm_formation_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_uwb_swarm_formation_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_uwb_swarm_formation_t, yaw) }, \
         { "formation_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_uwb_swarm_formation_t, formation_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UWB_SWARM_FORMATION { \
    "UWB_SWARM_FORMATION", \
    9, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_uwb_swarm_formation_t, usec) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_uwb_swarm_formation_t, id) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_uwb_swarm_formation_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_uwb_swarm_formation_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_uwb_swarm_formation_t, z) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_uwb_swarm_formation_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_uwb_swarm_formation_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_uwb_swarm_formation_t, yaw) }, \
         { "formation_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_uwb_swarm_formation_t, formation_type) }, \
         } \
}
#endif

/**
 * @brief Pack a uwb_swarm_formation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec [us] Timestamp (UNIX time or since system boot)
 * @param id  Swarm ID of swarm member
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param roll [rad] Roll angle (-pi..+pi)
 * @param pitch [rad] Pitch angle (-pi..+pi)
 * @param yaw [rad] Yaw angle (-pi..+pi)
 * @param formation_type  Type of formation strategy
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uwb_swarm_formation_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t usec, uint8_t id, float x, float y, float z, float roll, float pitch, float yaw, uint8_t formation_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, yaw);
    _mav_put_uint8_t(buf, 32, id);
    _mav_put_uint8_t(buf, 33, formation_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN);
#else
    mavlink_uwb_swarm_formation_t packet;
    packet.usec = usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.id = id;
    packet.formation_type = formation_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UWB_SWARM_FORMATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_MIN_LEN, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_CRC);
}

/**
 * @brief Pack a uwb_swarm_formation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec [us] Timestamp (UNIX time or since system boot)
 * @param id  Swarm ID of swarm member
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param roll [rad] Roll angle (-pi..+pi)
 * @param pitch [rad] Pitch angle (-pi..+pi)
 * @param yaw [rad] Yaw angle (-pi..+pi)
 * @param formation_type  Type of formation strategy
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uwb_swarm_formation_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t usec, uint8_t id, float x, float y, float z, float roll, float pitch, float yaw, uint8_t formation_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, yaw);
    _mav_put_uint8_t(buf, 32, id);
    _mav_put_uint8_t(buf, 33, formation_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN);
#else
    mavlink_uwb_swarm_formation_t packet;
    packet.usec = usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.id = id;
    packet.formation_type = formation_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UWB_SWARM_FORMATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_MIN_LEN, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_MIN_LEN, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN);
#endif
}

/**
 * @brief Pack a uwb_swarm_formation message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec [us] Timestamp (UNIX time or since system boot)
 * @param id  Swarm ID of swarm member
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param roll [rad] Roll angle (-pi..+pi)
 * @param pitch [rad] Pitch angle (-pi..+pi)
 * @param yaw [rad] Yaw angle (-pi..+pi)
 * @param formation_type  Type of formation strategy
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uwb_swarm_formation_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t usec,uint8_t id,float x,float y,float z,float roll,float pitch,float yaw,uint8_t formation_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, yaw);
    _mav_put_uint8_t(buf, 32, id);
    _mav_put_uint8_t(buf, 33, formation_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN);
#else
    mavlink_uwb_swarm_formation_t packet;
    packet.usec = usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.id = id;
    packet.formation_type = formation_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UWB_SWARM_FORMATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_MIN_LEN, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_CRC);
}

/**
 * @brief Encode a uwb_swarm_formation struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uwb_swarm_formation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uwb_swarm_formation_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uwb_swarm_formation_t* uwb_swarm_formation)
{
    return mavlink_msg_uwb_swarm_formation_pack(system_id, component_id, msg, uwb_swarm_formation->usec, uwb_swarm_formation->id, uwb_swarm_formation->x, uwb_swarm_formation->y, uwb_swarm_formation->z, uwb_swarm_formation->roll, uwb_swarm_formation->pitch, uwb_swarm_formation->yaw, uwb_swarm_formation->formation_type);
}

/**
 * @brief Encode a uwb_swarm_formation struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uwb_swarm_formation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uwb_swarm_formation_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uwb_swarm_formation_t* uwb_swarm_formation)
{
    return mavlink_msg_uwb_swarm_formation_pack_chan(system_id, component_id, chan, msg, uwb_swarm_formation->usec, uwb_swarm_formation->id, uwb_swarm_formation->x, uwb_swarm_formation->y, uwb_swarm_formation->z, uwb_swarm_formation->roll, uwb_swarm_formation->pitch, uwb_swarm_formation->yaw, uwb_swarm_formation->formation_type);
}

/**
 * @brief Encode a uwb_swarm_formation struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param uwb_swarm_formation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uwb_swarm_formation_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_uwb_swarm_formation_t* uwb_swarm_formation)
{
    return mavlink_msg_uwb_swarm_formation_pack_status(system_id, component_id, _status, msg,  uwb_swarm_formation->usec, uwb_swarm_formation->id, uwb_swarm_formation->x, uwb_swarm_formation->y, uwb_swarm_formation->z, uwb_swarm_formation->roll, uwb_swarm_formation->pitch, uwb_swarm_formation->yaw, uwb_swarm_formation->formation_type);
}

/**
 * @brief Send a uwb_swarm_formation message
 * @param chan MAVLink channel to send the message
 *
 * @param usec [us] Timestamp (UNIX time or since system boot)
 * @param id  Swarm ID of swarm member
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param roll [rad] Roll angle (-pi..+pi)
 * @param pitch [rad] Pitch angle (-pi..+pi)
 * @param yaw [rad] Yaw angle (-pi..+pi)
 * @param formation_type  Type of formation strategy
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uwb_swarm_formation_send(mavlink_channel_t chan, uint64_t usec, uint8_t id, float x, float y, float z, float roll, float pitch, float yaw, uint8_t formation_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, yaw);
    _mav_put_uint8_t(buf, 32, id);
    _mav_put_uint8_t(buf, 33, formation_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UWB_SWARM_FORMATION, buf, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_MIN_LEN, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_CRC);
#else
    mavlink_uwb_swarm_formation_t packet;
    packet.usec = usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.id = id;
    packet.formation_type = formation_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UWB_SWARM_FORMATION, (const char *)&packet, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_MIN_LEN, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_CRC);
#endif
}

/**
 * @brief Send a uwb_swarm_formation message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uwb_swarm_formation_send_struct(mavlink_channel_t chan, const mavlink_uwb_swarm_formation_t* uwb_swarm_formation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uwb_swarm_formation_send(chan, uwb_swarm_formation->usec, uwb_swarm_formation->id, uwb_swarm_formation->x, uwb_swarm_formation->y, uwb_swarm_formation->z, uwb_swarm_formation->roll, uwb_swarm_formation->pitch, uwb_swarm_formation->yaw, uwb_swarm_formation->formation_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UWB_SWARM_FORMATION, (const char *)uwb_swarm_formation, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_MIN_LEN, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uwb_swarm_formation_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t usec, uint8_t id, float x, float y, float z, float roll, float pitch, float yaw, uint8_t formation_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, yaw);
    _mav_put_uint8_t(buf, 32, id);
    _mav_put_uint8_t(buf, 33, formation_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UWB_SWARM_FORMATION, buf, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_MIN_LEN, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_CRC);
#else
    mavlink_uwb_swarm_formation_t *packet = (mavlink_uwb_swarm_formation_t *)msgbuf;
    packet->usec = usec;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->id = id;
    packet->formation_type = formation_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UWB_SWARM_FORMATION, (const char *)packet, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_MIN_LEN, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_CRC);
#endif
}
#endif

#endif

// MESSAGE UWB_SWARM_FORMATION UNPACKING


/**
 * @brief Get field usec from uwb_swarm_formation message
 *
 * @return [us] Timestamp (UNIX time or since system boot)
 */
static inline uint64_t mavlink_msg_uwb_swarm_formation_get_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field id from uwb_swarm_formation message
 *
 * @return  Swarm ID of swarm member
 */
static inline uint8_t mavlink_msg_uwb_swarm_formation_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field x from uwb_swarm_formation message
 *
 * @return [m] X Position
 */
static inline float mavlink_msg_uwb_swarm_formation_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field y from uwb_swarm_formation message
 *
 * @return [m] Y Position
 */
static inline float mavlink_msg_uwb_swarm_formation_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field z from uwb_swarm_formation message
 *
 * @return [m] Z Position
 */
static inline float mavlink_msg_uwb_swarm_formation_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field roll from uwb_swarm_formation message
 *
 * @return [rad] Roll angle (-pi..+pi)
 */
static inline float mavlink_msg_uwb_swarm_formation_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pitch from uwb_swarm_formation message
 *
 * @return [rad] Pitch angle (-pi..+pi)
 */
static inline float mavlink_msg_uwb_swarm_formation_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field yaw from uwb_swarm_formation message
 *
 * @return [rad] Yaw angle (-pi..+pi)
 */
static inline float mavlink_msg_uwb_swarm_formation_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field formation_type from uwb_swarm_formation message
 *
 * @return  Type of formation strategy
 */
static inline uint8_t mavlink_msg_uwb_swarm_formation_get_formation_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Decode a uwb_swarm_formation message into a struct
 *
 * @param msg The message to decode
 * @param uwb_swarm_formation C-struct to decode the message contents into
 */
static inline void mavlink_msg_uwb_swarm_formation_decode(const mavlink_message_t* msg, mavlink_uwb_swarm_formation_t* uwb_swarm_formation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uwb_swarm_formation->usec = mavlink_msg_uwb_swarm_formation_get_usec(msg);
    uwb_swarm_formation->x = mavlink_msg_uwb_swarm_formation_get_x(msg);
    uwb_swarm_formation->y = mavlink_msg_uwb_swarm_formation_get_y(msg);
    uwb_swarm_formation->z = mavlink_msg_uwb_swarm_formation_get_z(msg);
    uwb_swarm_formation->roll = mavlink_msg_uwb_swarm_formation_get_roll(msg);
    uwb_swarm_formation->pitch = mavlink_msg_uwb_swarm_formation_get_pitch(msg);
    uwb_swarm_formation->yaw = mavlink_msg_uwb_swarm_formation_get_yaw(msg);
    uwb_swarm_formation->id = mavlink_msg_uwb_swarm_formation_get_id(msg);
    uwb_swarm_formation->formation_type = mavlink_msg_uwb_swarm_formation_get_formation_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN? msg->len : MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN;
        memset(uwb_swarm_formation, 0, MAVLINK_MSG_ID_UWB_SWARM_FORMATION_LEN);
    memcpy(uwb_swarm_formation, _MAV_PAYLOAD(msg), len);
#endif
}
