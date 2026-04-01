#include "device/arm.hpp"
#include "io.hpp"
#include "serial_interface.hpp"
#include "user_lib.hpp"
#include "types.hpp"


namespace Device
{
    Arm::Arm(const std::string &serial_rx,const std::string &serial_tx) : serial_rx(serial_rx), serial_tx(serial_tx){
    }

    void Arm::init(const std::shared_ptr<Robot::Robot_set> &robot) {
            robot_set = robot;

            auto serial_interface = IO::io<SERIAL>[serial_rx];
            if (serial_interface == nullptr) {
                LOG_ERR("ARM Error: no serial named %s\n", serial_rx.c_str());
                return;
            }
            serial_interface->register_callback<Types::ReceivePacket_ARM>(
                [&](const Types::ReceivePacket_ARM &rp) { unpack(rp); });


            
            serial_send = IO::io<SERIAL>[serial_tx];
            if (serial_send == nullptr) {
                LOG_ERR("ARM Error: no serial named %s\n", serial_tx.c_str());
                return;
            }


            // // Test
            // uint8_t data1[4] = {0x55, 0xAA, 0x03, 0x01};
            //send_arm_data(data);
            //LOG_INFO("Receive_ok");
        }

    void Arm::send_arm_data(const uint8_t *data) {
        
        // data need to send to mcu
        for (size_t i = 0; i < kArmCustomDataLen; i++) {
            serial_send->send(data[i]);
        }

    }

    void Arm::unpack(const Types::ReceivePacket_ARM &pkg) {
        
        memcpy(data, &pkg, kArmCustomDataLen);
        data[25] = robot_set->clamp_open ? 0x01 : 0x00;
        send_arm_data(data);

        float angle1 = 0.0f;
        float angle2 = 0.0f;
        float angle3 = 0.0f;
        float angle4 = 0.0f;
        float angle5 = 0.0f;
        float angle6 = 0.0f;

        memcpy(&angle1, &data[1], sizeof(float));
        memcpy(&angle2, &data[5], sizeof(float));
        memcpy(&angle3, &data[9], sizeof(float));
        memcpy(&angle4, &data[13], sizeof(float));
        memcpy(&angle5, &data[17], sizeof(float));
        memcpy(&angle6, &data[21], sizeof(float));
    

        LOG_INFO("Receive_ok:%f,%f,%f,%f,%f,%f\n", angle1, angle2, angle3, angle4,angle5,angle6);
        LOG_INFO("last_2 byte:%d,%d,%d\n", data[0],data[25],data[26]);
        
        update_time();
    }

}  // namespace Device
