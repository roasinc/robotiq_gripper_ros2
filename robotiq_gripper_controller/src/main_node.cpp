#include "rclcpp/rclcpp.hpp"
#include <modbus/modbus.h>

using namespace std::chrono_literals;

class RobotiqGripperController: public rclcpp::Node
{
    public:
        RobotiqGripperController() : Node("robotiq_gripper_controller")
        {
            this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
            this->declare_parameter<int>("baudrate", 115200);
            this->declare_parameter<double>("rate", 20.0);

            auto port_name = this->get_parameter("port_name").get_parameter_value().get<std::string>();
            auto baudrate = this->get_parameter("baudrate").get_parameter_value().get<long int>();

            RCLCPP_INFO(this->get_logger(), "Port: [%s] and baudrate %ld", port_name.c_str(), baudrate);

            modbus_ = modbus_new_rtu(port_name.c_str(), baudrate, 'N', 8, 1);
            assert(modbus_ != NULL);
            // assert(modbus_connect(modbus_) != -1);

            auto period = std::chrono::duration<double>(1.0 / this->get_parameter("rate").as_double());
            timer_ = this->create_wall_timer(period, std::bind(&RobotiqGripperController::timer_callback, this));

            RCLCPP_INFO(this->get_logger(), "Initialized...");
        }

        ~RobotiqGripperController() {}

    private:
        void timer_callback()
        {
            // Send command to Gripper


            // Receive status from Gripper
            uint16_t recv_registers[3] = {0, };
            modbus_read_registers(0x7D0, 3, recv_registers);
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        modbus_t *modbus_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotiqGripperController>());
    rclcpp::shutdown();
    return 0;
}