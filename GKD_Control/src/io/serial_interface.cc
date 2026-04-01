#include "serial_interface.hpp"

#include "user_lib.hpp"

namespace IO
{
    Serial_interface::Serial_interface(std::string port_name, int baudrate, int simple_timeout)
        : serial::Serial(port_name, baudrate, serial::Timeout::simpleTimeout(simple_timeout)),
          name(port_name) {
    }

    Serial_interface::~Serial_interface() = default;

    inline void Serial_interface::enumerate_ports() {
        std::vector<serial::PortInfo> devices_found = serial::list_ports();
        auto iter = devices_found.begin();

        while (iter != devices_found.end()) {
            serial::PortInfo device = *iter++;
            LOG_INFO("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str());
        }
    }

    inline int Serial_interface::unpack(uint8_t pkg_id) {
        
        if (pkg_id == 1) {
            memcpy(buffer, read(sizeof(Types::ReceivePacket_IMU)).c_str(), sizeof(Types::ReceivePacket_IMU));
            UserLib::fromVector(buffer, &imu_pkg);
            callback(imu_pkg);
            //LOG_INFO("IMU_ID_OK");
        } else if (pkg_id == 2) {
            memcpy(buffer, read(sizeof(Types::ReceivePacket_RC_CTRL)).c_str(), sizeof(Types::ReceivePacket_RC_CTRL));
            UserLib::fromVector(buffer, &rc_pkg);
            callback(rc_pkg);
            // LOG_INFO("RC_ID_OK");
        } else if(pkg_id == 3){   //兼容旧协议，接上裁判系统实际没用
            memcpy(buffer, read(sizeof(Types::ReceivePacket_ARM)).c_str(), sizeof(Types::ReceivePacket_ARM));
            UserLib::fromVector(buffer, &arm_pkg);
            callback(arm_pkg);
            //LOG_INFO("ARM_ID_OK");
        } 
        return 0;
    }

    void Serial_interface::task() {
        while (true) {
            try {
                if (isOpen()) {

                // arm_rx&referee new unpakg
                if (name == "/dev/ARM_RX") {
                        uint8_t frame_header[7] = {0};
                        read(frame_header, 1);
                        if (frame_header[0] != 0xA5) {
                            continue;
                        }
                        read(frame_header + 1, 6);

                        uint8_t custom_data[30] = {0};
                        read(custom_data, sizeof(custom_data));

                        uint8_t frame_tail[2] = {0};
                        read(frame_tail, sizeof(frame_tail));

                        memset(&arm_pkg, 0, sizeof(arm_pkg));
                        memcpy(&arm_pkg, custom_data, sizeof(custom_data));
                        callback(arm_pkg);
                        continue;
                    }

                    // other unpack
                    read((uint8_t *)&header, 2);
                    if (header == 0xAA55) {
                        read((uint8_t *)&header, 1);
                        unpack(header);
                    }



                } else {
                    enumerate_ports();
                    return;
                }
            } catch (serial::IOException &e) {
                LOG_ERR("serail offline! end program now\n");
                exit(-1);
            }
        }
    }



//     void Serial_interface::task() {
//     // 假设数据包体最大为 64 字节，请根据你的实际情况修改 MAX_PKG_SIZE
//     // 更好的做法是定义一个常量，例如：
//     // #define MAX_PACKET_DATA_SIZE 64 
//     const size_t MAX_PACKET_DATA_SIZE = 64; 
//     uint8_t raw_data[MAX_PACKET_DATA_SIZE];
//     uint8_t pkg_id;
    
//     while (true) {
//         try {
//             if (isOpen()) {
//                 // 1. 读取帧头 (0xAA55)
//                 read((uint8_t *)&header, 2);
                
//                 if (header == 0xAA55) {
//                     // 2. 读取数据包 ID
//                     read(&pkg_id, 1); // 读取 pkg_id 到局部变量
                    
//                     // 3. 读取数据包体（忽略 ID，读取最大长度）
//                     // 我们读取 MAX_PACKET_DATA_SIZE 字节作为原始数据包内容
//                     size_t bytes_read = read(raw_data, MAX_PACKET_DATA_SIZE);

//                     // 4. 打印原始数据和 ID
//                     LOG_INFO("--- 收到数据包 ---");
//                     LOG_INFO("数据包 ID: %d", pkg_id); // 打印 ID
//                     LOG_INFO("读取字节数: %zu", bytes_read); // 打印实际读取的字节数

//                     printf("原始数据 (Hex): ");
//                     for (size_t i = 0; i < bytes_read; ++i) {
//                         printf("%02X ", raw_data[i]);
//                     }
//                     printf("\n");
                    
//                     // 原始代码中的 unpack(header) 逻辑被替换/暂时注释
//                     // unpack(header); // 暂时注释或删除此行

//                 }
//             } else {
//                 enumerate_ports();
//                 return;
//             }
//         } catch (serial::IOException &e) {
//             LOG_ERR("serail offline! end program now\n");
//             //exit(-1);
//         }
//     }
// }



}  // namespace IO
