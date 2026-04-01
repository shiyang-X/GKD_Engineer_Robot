#pragma once

#include <memory>

#include "device/deviece_base.hpp"
#include "memory"
#include "robot.hpp"
#include "types.hpp"
#include "io.hpp"
#include "io/serial_interface.hpp"

namespace Device
{
    class Arm : public DeviceBase
    {
       public:
        explicit Arm(const std::string& serial_rx,const std::string& serial_tx);

        void init(const std::shared_ptr<Robot::Robot_set>& robot);
        void unpack(const Types::ReceivePacket_ARM& pkg);
        void send_arm_data(const uint8_t *data);

        template<typename T>
        void send(T &val) {
           write(&val, sizeof(T));
        }

        
        
       private:
       
        uint8_t head_data=0xBB;
        static constexpr size_t kArmCustomDataLen = 30;
        uint8_t data[kArmCustomDataLen] = {0};

        //float angle_j1,angle_j2,angle_j3,angle_j4,angle_j5,angle_j6;

        std::string serial_rx;
        std::string serial_tx;

        IO::Serial_interface* serial_send;
        std::shared_ptr<Robot::Robot_set> robot_set;
    };
}  // namespace Device
