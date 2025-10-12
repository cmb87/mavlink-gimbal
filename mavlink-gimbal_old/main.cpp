#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

// Include MAVLink headers (adjust path to your MAVLink include directory)
#include "mavlink/v2.0/common/mavlink.h"
#include "mavlink/v2.0/common/mavlink_msg_gimbal_device_information.h"
#include "mavlink/v2.0/common/mavlink_msg_gimbal_device_attitude_status.h"

using namespace std;
using namespace std::chrono_literals;

// Gimbal parameters
const uint8_t SYSTEM_ID = 1;        // System ID for gimbal
const uint8_t COMPONENT_ID = MAV_COMP_ID_GIMBAL; // 154
const float MAX_PITCH = 90.0f;      // Max pitch angle (degrees)
const float MIN_PITCH = -90.0f;     // Min pitch angle (degrees)
const float MAX_YAW = 180.0f;       // Max yaw angle (degrees)
const float MIN_YAW = -180.0f;      // Min yaw angle (degrees)

// Serial port configuration
int open_serial(const string& port, int baudrate) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        cerr << "Error opening serial port " << port << ": " << strerror(errno) << endl;
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

// Send MAVLink message over serial
void send_mavlink_message(int fd, const mavlink_message_t& msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &msg);
    if (write(fd, buffer, len) != len) {
        cerr << "Error sending MAVLink message: " << strerror(errno) << endl;
    }
}

int main(int argc, char** argv) {
    // Default serial port (adjust for your setup)
    string serial_port = "/dev/ttyACM0";
    int baudrate = B115200;
    if (argc > 1) serial_port = argv[1];

    // Open serial port
    int fd = open_serial(serial_port, baudrate);
    if (fd == -1) return 1;

    cout << "Connected to serial port: " << serial_port << endl;

    // Track current gimbal angles (replace with actual hardware control)
    float current_pitch = 0.0f;
    float current_yaw = 0.0f;

    // Send HEARTBEAT periodically
    thread heartbeat_thread([fd]() {
        while (true) {
            mavlink_message_t msg;
            mavlink_heartbeat_t heartbeat;
            heartbeat.type = MAV_TYPE_GIMBAL;
            heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
            heartbeat.base_mode = 0;
            heartbeat.custom_mode = 0;
            heartbeat.system_status = MAV_STATE_ACTIVE;
            mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &msg, 
                                       heartbeat.type, 
                                       heartbeat.autopilot, 
                                       heartbeat.base_mode, 
                                       heartbeat.custom_mode, 
                                       heartbeat.system_status);
            send_mavlink_message(fd, msg);
            this_thread::sleep_for(1s);
        }
    });
    heartbeat_thread.detach();

    // Broadcast GIMBAL_DEVICE_INFORMATION
    auto send_gimbal_info = [fd]() {
        mavlink_message_t msg;
        mavlink_gimbal_device_information_t info = {};
        strncpy((char*)info.vendor_name, "CustomGimbal", sizeof(info.vendor_name));
        strncpy((char*)info.model_name, "MAVLinkGimbal", sizeof(info.model_name));
        info.firmware_version = 100; // v1.00
        info.cap_flags = GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH | GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW;
        info.pitch_max = MAX_PITCH;
        info.pitch_min = MIN_PITCH;
        info.yaw_max = MAX_YAW;
        info.yaw_min = MIN_YAW;
        mavlink_msg_gimbal_device_information_pack(SYSTEM_ID, COMPONENT_ID, &msg, 
                                                  info.time_boot_ms, 
                                                  info.vendor_name, 
                                                  info.model_name, 
                                                  info.firmware_version, 
                                                  info.hardware_version, 
                                                  info.uid, 
                                                  info.cap_flags, 
                                                  info.custom_cap_flags, 
                                                  info.roll_min, 
                                                  info.roll_max, 
                                                  info.pitch_min, 
                                                  info.pitch_max, 
                                                  info.yaw_min, 
                                                  info.yaw_max);
        send_mavlink_message(fd, msg);
    };
    send_gimbal_info();
    thread gimbal_info_thread([send_gimbal_info]() {
        while (true) {
            send_gimbal_info();
            this_thread::sleep_for(10s);
        }
    });
    gimbal_info_thread.detach();

    // Send GIMBAL_DEVICE_ATTITUDE_STATUS periodically
    thread attitude_thread([fd, &current_pitch, &current_yaw]() {
        while (true) {
            mavlink_message_t msg;
            mavlink_gimbal_device_attitude_status_t status = {};
            status.flags = GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME;
            status.q[0] = 1.0f; // Simplified quaternion
            status.angular_velocity_x = 0.0f;
            status.angular_velocity_y = 0.0f;
            status.angular_velocity_z = 0.0f;
            status.pitch = current_pitch;
            status.yaw = current_yaw;
            mavlink_msg_gimbal_device_attitude_status_pack(SYSTEM_ID, COMPONENT_ID, &msg, 
                                                          status.time_boot_ms, 
                                                          status.flags, 
                                                          status.q, 
                                                          status.angular_velocity_x, 
                                                          status.angular_velocity_y, 
                                                          status.angular_velocity_z, 
                                                          status.yaw, 
                                                          status.pitch, 
                                                          status.roll);
            send_mavlink_message(fd, msg);
            this_thread::sleep_for(100ms);
        }
    });
    attitude_thread.detach();

    // Process incoming MAVLink messages
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;
    mavlink_status_t status;
    while (true) {
        int bytes_read = read(fd, buffer, sizeof(buffer));
        if (bytes_read < 0 && errno != EAGAIN) {
            cerr << "Error reading from serial: " << strerror(errno) << endl;
            break;
        }
        for (int i = 0; i < bytes_read; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                    mavlink_command_long_t cmd;
                    mavlink_msg_command_long_decode(&msg, &cmd);
                    if (cmd.command == MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW && 
                        cmd.target_component == COMPONENT_ID) {
                        float pitch = cmd.param1;
                        float yaw = cmd.param2;
                        current_pitch = max(MIN_PITCH, min(MAX_PITCH, pitch));
                        current_yaw = max(MIN_YAW, min(MAX_YAW, yaw));
                        cout << "Received command: Pitch=" << current_pitch 
                             << ", Yaw=" << current_yaw << endl;

                        // Send ACK
                        mavlink_message_t ack_msg;
                        mavlink_command_ack_t ack;
                        ack.command = MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW;
                        ack.result = MAV_RESULT_ACCEPTED;
                        mavlink_msg_command_ack_pack(SYSTEM_ID, COMPONENT_ID, &ack_msg, 
                                                     ack.command, 
                                                     ack.result, 
                                                     0, 0, 
                                                     cmd.target_system, 
                                                     cmd.target_component);
                        send_mavlink_message(fd, ack_msg);
                    }
                } else if (msg.msgid == MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE) {
                    mavlink_autopilot_state_for_gimbal_device_t state;
                    mavlink_msg_autopilot_state_for_gimbal_device_decode(&msg, &state);
                    cout << "Received vehicle state: Roll=" << state.roll 
                         << ", Pitch=" << state.pitch << ", Yaw=" << state.yaw << endl;
                }
            }
        }
        this_thread::sleep_for(10ms);
    }

    close(fd);
    return 0;