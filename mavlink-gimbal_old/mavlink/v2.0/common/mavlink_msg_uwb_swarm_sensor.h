#pragma once
// MESSAGE UWB_SWARM_SENSOR PACKING

#define MAVLINK_MSG_ID_UWB_SWARM_SENSOR 302


typedef struct __mavlink_uwb_swarm_sensor_t {
 uint64_t usec; /*< [us] Timestamp (UNIX time or since system boot)*/
 float d; /*< [m] Measured raw distance from vehicle to swarm member id*/
 float x; /*< [m] X Position*/
 float y; /*< [m] Y Position*/
 float z; /*< [m] Z Position*/
 float vx; /*< [m/s] X Speed*/
 float vy; /*< [m/s] Y Speed*/
 float vz; /*< [m/s] Z Speed*/
 float roll; /*< [rad] Roll*/
 float pitch; /*< [rad] Pitch*/
 float yaw; /*< [rad] Yaw*/
 float RXQuality; /*< [dB] RX_Quality*/
 float RXPower; /*< [dB] RX_Power*/
 float RXFPPower; /*< [Db] RXFPPower*/
 uint8_t id; /*<  Swarm ID of swarm member*/
} mavlink_uwb_swarm_sensor_t;

#define MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN 61
#define MAVLINK_MSG_ID_UWB_SWARM_SENSOR_MIN_LEN 61
#define MAVLINK_MSG_ID_302_LEN 61
#define MAVLINK_MSG_ID_302_MIN_LEN 61

#define MAVLINK_MSG_ID_UWB_SWARM_SENSOR_CRC 136
#define MAVLINK_MSG_ID_302_CRC 136



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UWB_SWARM_SENSOR { \
    302, \
    "UWB_SWARM_SENSOR", \
    15, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_uwb_swarm_sensor_t, usec) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 60, offsetof(mavlink_uwb_swarm_sensor_t, id) }, \
         { "d", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_uwb_swarm_sensor_t, d) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_uwb_swarm_sensor_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_uwb_swarm_sensor_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_uwb_swarm_sensor_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_uwb_swarm_sensor_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_uwb_swarm_sensor_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_uwb_swarm_sensor_t, vz) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_uwb_swarm_sensor_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_uwb_swarm_sensor_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_uwb_swarm_sensor_t, yaw) }, \
         { "RXQuality", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_uwb_swarm_sensor_t, RXQuality) }, \
         { "RXPower", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_uwb_swarm_sensor_t, RXPower) }, \
         { "RXFPPower", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_uwb_swarm_sensor_t, RXFPPower) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UWB_SWARM_SENSOR { \
    "UWB_SWARM_SENSOR", \
    15, \
    {  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_uwb_swarm_sensor_t, usec) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 60, offsetof(mavlink_uwb_swarm_sensor_t, id) }, \
         { "d", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_uwb_swarm_sensor_t, d) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_uwb_swarm_sensor_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_uwb_swarm_sensor_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_uwb_swarm_sensor_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_uwb_swarm_sensor_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_uwb_swarm_sensor_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_uwb_swarm_sensor_t, vz) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_uwb_swarm_sensor_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_uwb_swarm_sensor_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_uwb_swarm_sensor_t, yaw) }, \
         { "RXQuality", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_uwb_swarm_sensor_t, RXQuality) }, \
         { "RXPower", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_uwb_swarm_sensor_t, RXPower) }, \
         { "RXFPPower", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_uwb_swarm_sensor_t, RXFPPower) }, \
         } \
}
#endif

/**
 * @brief Pack a uwb_swarm_sensor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec [us] Timestamp (UNIX time or since system boot)
 * @param id  Swarm ID of swarm member
 * @param d [m] Measured raw distance from vehicle to swarm member id
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param vx [m/s] X Speed
 * @param vy [m/s] Y Speed
 * @param vz [m/s] Z Speed
 * @param roll [rad] Roll
 * @param pitch [rad] Pitch
 * @param yaw [rad] Yaw
 * @param RXQuality [dB] RX_Quality
 * @param RXPower [dB] RX_Power
 * @param RXFPPower [Db] RXFPPower
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uwb_swarm_sensor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t usec, uint8_t id, float d, float x, float y, float z, float vx, float vy, float vz, float roll, float pitch, float yaw, float RXQuality, float RXPower, float RXFPPower)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, d);
    _mav_put_float(buf, 12, x);
    _mav_put_float(buf, 16, y);
    _mav_put_float(buf, 20, z);
    _mav_put_float(buf, 24, vx);
    _mav_put_float(buf, 28, vy);
    _mav_put_float(buf, 32, vz);
    _mav_put_float(buf, 36, roll);
    _mav_put_float(buf, 40, pitch);
    _mav_put_float(buf, 44, yaw);
    _mav_put_float(buf, 48, RXQuality);
    _mav_put_float(buf, 52, RXPower);
    _mav_put_float(buf, 56, RXFPPower);
    _mav_put_uint8_t(buf, 60, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN);
#else
    mavlink_uwb_swarm_sensor_t packet;
    packet.usec = usec;
    packet.d = d;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.RXQuality = RXQuality;
    packet.RXPower = RXPower;
    packet.RXFPPower = RXFPPower;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UWB_SWARM_SENSOR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_MIN_LEN, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_CRC);
}

/**
 * @brief Pack a uwb_swarm_sensor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec [us] Timestamp (UNIX time or since system boot)
 * @param id  Swarm ID of swarm member
 * @param d [m] Measured raw distance from vehicle to swarm member id
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param vx [m/s] X Speed
 * @param vy [m/s] Y Speed
 * @param vz [m/s] Z Speed
 * @param roll [rad] Roll
 * @param pitch [rad] Pitch
 * @param yaw [rad] Yaw
 * @param RXQuality [dB] RX_Quality
 * @param RXPower [dB] RX_Power
 * @param RXFPPower [Db] RXFPPower
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uwb_swarm_sensor_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t usec, uint8_t id, float d, float x, float y, float z, float vx, float vy, float vz, float roll, float pitch, float yaw, float RXQuality, float RXPower, float RXFPPower)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, d);
    _mav_put_float(buf, 12, x);
    _mav_put_float(buf, 16, y);
    _mav_put_float(buf, 20, z);
    _mav_put_float(buf, 24, vx);
    _mav_put_float(buf, 28, vy);
    _mav_put_float(buf, 32, vz);
    _mav_put_float(buf, 36, roll);
    _mav_put_float(buf, 40, pitch);
    _mav_put_float(buf, 44, yaw);
    _mav_put_float(buf, 48, RXQuality);
    _mav_put_float(buf, 52, RXPower);
    _mav_put_float(buf, 56, RXFPPower);
    _mav_put_uint8_t(buf, 60, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN);
#else
    mavlink_uwb_swarm_sensor_t packet;
    packet.usec = usec;
    packet.d = d;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.RXQuality = RXQuality;
    packet.RXPower = RXPower;
    packet.RXFPPower = RXFPPower;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UWB_SWARM_SENSOR;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_MIN_LEN, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_MIN_LEN, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN);
#endif
}

/**
 * @brief Pack a uwb_swarm_sensor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec [us] Timestamp (UNIX time or since system boot)
 * @param id  Swarm ID of swarm member
 * @param d [m] Measured raw distance from vehicle to swarm member id
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param vx [m/s] X Speed
 * @param vy [m/s] Y Speed
 * @param vz [m/s] Z Speed
 * @param roll [rad] Roll
 * @param pitch [rad] Pitch
 * @param yaw [rad] Yaw
 * @param RXQuality [dB] RX_Quality
 * @param RXPower [dB] RX_Power
 * @param RXFPPower [Db] RXFPPower
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uwb_swarm_sensor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t usec,uint8_t id,float d,float x,float y,float z,float vx,float vy,float vz,float roll,float pitch,float yaw,float RXQuality,float RXPower,float RXFPPower)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, d);
    _mav_put_float(buf, 12, x);
    _mav_put_float(buf, 16, y);
    _mav_put_float(buf, 20, z);
    _mav_put_float(buf, 24, vx);
    _mav_put_float(buf, 28, vy);
    _mav_put_float(buf, 32, vz);
    _mav_put_float(buf, 36, roll);
    _mav_put_float(buf, 40, pitch);
    _mav_put_float(buf, 44, yaw);
    _mav_put_float(buf, 48, RXQuality);
    _mav_put_float(buf, 52, RXPower);
    _mav_put_float(buf, 56, RXFPPower);
    _mav_put_uint8_t(buf, 60, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN);
#else
    mavlink_uwb_swarm_sensor_t packet;
    packet.usec = usec;
    packet.d = d;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.RXQuality = RXQuality;
    packet.RXPower = RXPower;
    packet.RXFPPower = RXFPPower;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UWB_SWARM_SENSOR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_MIN_LEN, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_CRC);
}

/**
 * @brief Encode a uwb_swarm_sensor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uwb_swarm_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uwb_swarm_sensor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uwb_swarm_sensor_t* uwb_swarm_sensor)
{
    return mavlink_msg_uwb_swarm_sensor_pack(system_id, component_id, msg, uwb_swarm_sensor->usec, uwb_swarm_sensor->id, uwb_swarm_sensor->d, uwb_swarm_sensor->x, uwb_swarm_sensor->y, uwb_swarm_sensor->z, uwb_swarm_sensor->vx, uwb_swarm_sensor->vy, uwb_swarm_sensor->vz, uwb_swarm_sensor->roll, uwb_swarm_sensor->pitch, uwb_swarm_sensor->yaw, uwb_swarm_sensor->RXQuality, uwb_swarm_sensor->RXPower, uwb_swarm_sensor->RXFPPower);
}

/**
 * @brief Encode a uwb_swarm_sensor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uwb_swarm_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uwb_swarm_sensor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uwb_swarm_sensor_t* uwb_swarm_sensor)
{
    return mavlink_msg_uwb_swarm_sensor_pack_chan(system_id, component_id, chan, msg, uwb_swarm_sensor->usec, uwb_swarm_sensor->id, uwb_swarm_sensor->d, uwb_swarm_sensor->x, uwb_swarm_sensor->y, uwb_swarm_sensor->z, uwb_swarm_sensor->vx, uwb_swarm_sensor->vy, uwb_swarm_sensor->vz, uwb_swarm_sensor->roll, uwb_swarm_sensor->pitch, uwb_swarm_sensor->yaw, uwb_swarm_sensor->RXQuality, uwb_swarm_sensor->RXPower, uwb_swarm_sensor->RXFPPower);
}

/**
 * @brief Encode a uwb_swarm_sensor struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param uwb_swarm_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uwb_swarm_sensor_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_uwb_swarm_sensor_t* uwb_swarm_sensor)
{
    return mavlink_msg_uwb_swarm_sensor_pack_status(system_id, component_id, _status, msg,  uwb_swarm_sensor->usec, uwb_swarm_sensor->id, uwb_swarm_sensor->d, uwb_swarm_sensor->x, uwb_swarm_sensor->y, uwb_swarm_sensor->z, uwb_swarm_sensor->vx, uwb_swarm_sensor->vy, uwb_swarm_sensor->vz, uwb_swarm_sensor->roll, uwb_swarm_sensor->pitch, uwb_swarm_sensor->yaw, uwb_swarm_sensor->RXQuality, uwb_swarm_sensor->RXPower, uwb_swarm_sensor->RXFPPower);
}

/**
 * @brief Send a uwb_swarm_sensor message
 * @param chan MAVLink channel to send the message
 *
 * @param usec [us] Timestamp (UNIX time or since system boot)
 * @param id  Swarm ID of swarm member
 * @param d [m] Measured raw distance from vehicle to swarm member id
 * @param x [m] X Position
 * @param y [m] Y Position
 * @param z [m] Z Position
 * @param vx [m/s] X Speed
 * @param vy [m/s] Y Speed
 * @param vz [m/s] Z Speed
 * @param roll [rad] Roll
 * @param pitch [rad] Pitch
 * @param yaw [rad] Yaw
 * @param RXQuality [dB] RX_Quality
 * @param RXPower [dB] RX_Power
 * @param RXFPPower [Db] RXFPPower
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uwb_swarm_sensor_send(mavlink_channel_t chan, uint64_t usec, uint8_t id, float d, float x, float y, float z, float vx, float vy, float vz, float roll, float pitch, float yaw, float RXQuality, float RXPower, float RXFPPower)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN];
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, d);
    _mav_put_float(buf, 12, x);
    _mav_put_float(buf, 16, y);
    _mav_put_float(buf, 20, z);
    _mav_put_float(buf, 24, vx);
    _mav_put_float(buf, 28, vy);
    _mav_put_float(buf, 32, vz);
    _mav_put_float(buf, 36, roll);
    _mav_put_float(buf, 40, pitch);
    _mav_put_float(buf, 44, yaw);
    _mav_put_float(buf, 48, RXQuality);
    _mav_put_float(buf, 52, RXPower);
    _mav_put_float(buf, 56, RXFPPower);
    _mav_put_uint8_t(buf, 60, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UWB_SWARM_SENSOR, buf, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_MIN_LEN, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_CRC);
#else
    mavlink_uwb_swarm_sensor_t packet;
    packet.usec = usec;
    packet.d = d;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.RXQuality = RXQuality;
    packet.RXPower = RXPower;
    packet.RXFPPower = RXFPPower;
    packet.id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UWB_SWARM_SENSOR, (const char *)&packet, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_MIN_LEN, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_CRC);
#endif
}

/**
 * @brief Send a uwb_swarm_sensor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uwb_swarm_sensor_send_struct(mavlink_channel_t chan, const mavlink_uwb_swarm_sensor_t* uwb_swarm_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uwb_swarm_sensor_send(chan, uwb_swarm_sensor->usec, uwb_swarm_sensor->id, uwb_swarm_sensor->d, uwb_swarm_sensor->x, uwb_swarm_sensor->y, uwb_swarm_sensor->z, uwb_swarm_sensor->vx, uwb_swarm_sensor->vy, uwb_swarm_sensor->vz, uwb_swarm_sensor->roll, uwb_swarm_sensor->pitch, uwb_swarm_sensor->yaw, uwb_swarm_sensor->RXQuality, uwb_swarm_sensor->RXPower, uwb_swarm_sensor->RXFPPower);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UWB_SWARM_SENSOR, (const char *)uwb_swarm_sensor, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_MIN_LEN, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_CRC);
#endif
}

#if MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uwb_swarm_sensor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t usec, uint8_t id, float d, float x, float y, float z, float vx, float vy, float vz, float roll, float pitch, float yaw, float RXQuality, float RXPower, float RXFPPower)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, usec);
    _mav_put_float(buf, 8, d);
    _mav_put_float(buf, 12, x);
    _mav_put_float(buf, 16, y);
    _mav_put_float(buf, 20, z);
    _mav_put_float(buf, 24, vx);
    _mav_put_float(buf, 28, vy);
    _mav_put_float(buf, 32, vz);
    _mav_put_float(buf, 36, roll);
    _mav_put_float(buf, 40, pitch);
    _mav_put_float(buf, 44, yaw);
    _mav_put_float(buf, 48, RXQuality);
    _mav_put_float(buf, 52, RXPower);
    _mav_put_float(buf, 56, RXFPPower);
    _mav_put_uint8_t(buf, 60, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UWB_SWARM_SENSOR, buf, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_MIN_LEN, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_CRC);
#else
    mavlink_uwb_swarm_sensor_t *packet = (mavlink_uwb_swarm_sensor_t *)msgbuf;
    packet->usec = usec;
    packet->d = d;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->RXQuality = RXQuality;
    packet->RXPower = RXPower;
    packet->RXFPPower = RXFPPower;
    packet->id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UWB_SWARM_SENSOR, (const char *)packet, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_MIN_LEN, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_CRC);
#endif
}
#endif

#endif

// MESSAGE UWB_SWARM_SENSOR UNPACKING


/**
 * @brief Get field usec from uwb_swarm_sensor message
 *
 * @return [us] Timestamp (UNIX time or since system boot)
 */
static inline uint64_t mavlink_msg_uwb_swarm_sensor_get_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field id from uwb_swarm_sensor message
 *
 * @return  Swarm ID of swarm member
 */
static inline uint8_t mavlink_msg_uwb_swarm_sensor_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  60);
}

/**
 * @brief Get field d from uwb_swarm_sensor message
 *
 * @return [m] Measured raw distance from vehicle to swarm member id
 */
static inline float mavlink_msg_uwb_swarm_sensor_get_d(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field x from uwb_swarm_sensor message
 *
 * @return [m] X Position
 */
static inline float mavlink_msg_uwb_swarm_sensor_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field y from uwb_swarm_sensor message
 *
 * @return [m] Y Position
 */
static inline float mavlink_msg_uwb_swarm_sensor_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field z from uwb_swarm_sensor message
 *
 * @return [m] Z Position
 */
static inline float mavlink_msg_uwb_swarm_sensor_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vx from uwb_swarm_sensor message
 *
 * @return [m/s] X Speed
 */
static inline float mavlink_msg_uwb_swarm_sensor_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field vy from uwb_swarm_sensor message
 *
 * @return [m/s] Y Speed
 */
static inline float mavlink_msg_uwb_swarm_sensor_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field vz from uwb_swarm_sensor message
 *
 * @return [m/s] Z Speed
 */
static inline float mavlink_msg_uwb_swarm_sensor_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field roll from uwb_swarm_sensor message
 *
 * @return [rad] Roll
 */
static inline float mavlink_msg_uwb_swarm_sensor_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field pitch from uwb_swarm_sensor message
 *
 * @return [rad] Pitch
 */
static inline float mavlink_msg_uwb_swarm_sensor_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field yaw from uwb_swarm_sensor message
 *
 * @return [rad] Yaw
 */
static inline float mavlink_msg_uwb_swarm_sensor_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field RXQuality from uwb_swarm_sensor message
 *
 * @return [dB] RX_Quality
 */
static inline float mavlink_msg_uwb_swarm_sensor_get_RXQuality(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field RXPower from uwb_swarm_sensor message
 *
 * @return [dB] RX_Power
 */
static inline float mavlink_msg_uwb_swarm_sensor_get_RXPower(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field RXFPPower from uwb_swarm_sensor message
 *
 * @return [Db] RXFPPower
 */
static inline float mavlink_msg_uwb_swarm_sensor_get_RXFPPower(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Decode a uwb_swarm_sensor message into a struct
 *
 * @param msg The message to decode
 * @param uwb_swarm_sensor C-struct to decode the message contents into
 */
static inline void mavlink_msg_uwb_swarm_sensor_decode(const mavlink_message_t* msg, mavlink_uwb_swarm_sensor_t* uwb_swarm_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uwb_swarm_sensor->usec = mavlink_msg_uwb_swarm_sensor_get_usec(msg);
    uwb_swarm_sensor->d = mavlink_msg_uwb_swarm_sensor_get_d(msg);
    uwb_swarm_sensor->x = mavlink_msg_uwb_swarm_sensor_get_x(msg);
    uwb_swarm_sensor->y = mavlink_msg_uwb_swarm_sensor_get_y(msg);
    uwb_swarm_sensor->z = mavlink_msg_uwb_swarm_sensor_get_z(msg);
    uwb_swarm_sensor->vx = mavlink_msg_uwb_swarm_sensor_get_vx(msg);
    uwb_swarm_sensor->vy = mavlink_msg_uwb_swarm_sensor_get_vy(msg);
    uwb_swarm_sensor->vz = mavlink_msg_uwb_swarm_sensor_get_vz(msg);
    uwb_swarm_sensor->roll = mavlink_msg_uwb_swarm_sensor_get_roll(msg);
    uwb_swarm_sensor->pitch = mavlink_msg_uwb_swarm_sensor_get_pitch(msg);
    uwb_swarm_sensor->yaw = mavlink_msg_uwb_swarm_sensor_get_yaw(msg);
    uwb_swarm_sensor->RXQuality = mavlink_msg_uwb_swarm_sensor_get_RXQuality(msg);
    uwb_swarm_sensor->RXPower = mavlink_msg_uwb_swarm_sensor_get_RXPower(msg);
    uwb_swarm_sensor->RXFPPower = mavlink_msg_uwb_swarm_sensor_get_RXFPPower(msg);
    uwb_swarm_sensor->id = mavlink_msg_uwb_swarm_sensor_get_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN? msg->len : MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN;
        memset(uwb_swarm_sensor, 0, MAVLINK_MSG_ID_UWB_SWARM_SENSOR_LEN);
    memcpy(uwb_swarm_sensor, _MAV_PAYLOAD(msg), len);
#endif
}
