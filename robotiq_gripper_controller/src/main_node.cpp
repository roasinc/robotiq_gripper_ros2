#include "rclcpp/rclcpp.hpp"
#include <modbus/modbus.h>
#include "robotiq_gripper_interfaces/msg/gripper_status.hpp"
#include "robotiq_gripper_interfaces/msg/gripper_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class RobotiqGripperController: public rclcpp::Node
{
    public:
        RobotiqGripperController() : Node("robotiq_gripper_controller")
        {
            req_gripper_pos_ = 0.0;
            req_gripper_speed_= 0.0;
            req_gripper_force_ = 0.0;

            this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
            this->declare_parameter<int>("baudrate", 115200);
            this->declare_parameter<double>("rate", 20.0);

            auto port_name = this->get_parameter("port_name").get_parameter_value().get<std::string>();
            auto baudrate = this->get_parameter("baudrate").get_parameter_value().get<long int>();

            RCLCPP_INFO(this->get_logger(), "Port: [%s] and baudrate %ld", port_name.c_str(), baudrate);

            modbus_ = modbus_new_rtu(port_name.c_str(), baudrate, 'N', 8, 1);
            assert(modbus_ != NULL);
            modbus_set_slave(modbus_, 9);
            if(modbus_connect(modbus_) == -1)
            {
                modbus_free(modbus_);
                assert(false);
            }

            uint16_t initial_registers[3] = { 0x0800, 0x0000, 0x0000 };
            modbus_write_registers(modbus_, 0x3E8, 3, initial_registers);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            sub_gripper_command_ = this->create_subscription<robotiq_gripper_interfaces::msg::GripperCommand>(
                "gripper_command", 10, std::bind(&RobotiqGripperController::gripper_command_callback, this, _1));
            pub_gripper_status_ = this->create_publisher<robotiq_gripper_interfaces::msg::GripperStatus>("gripper_status", 10);
            pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

            auto period = std::chrono::duration<double>(1.0 / this->get_parameter("rate").as_double());
            timer_ = this->create_wall_timer(period, std::bind(&RobotiqGripperController::timer_callback, this));

            RCLCPP_INFO(this->get_logger(), "Initialized...");
        }

        ~RobotiqGripperController() {}

    private:
        void timer_callback()
        {
            // Send command to Gripper
            uint16_t send_registers[3] = {0, 0, 0};

            send_registers[0] = 0x0900;  // gACT and gGTO is always on
            send_registers[1] = req_gripper_pos_ & 0xFF;
            send_registers[2] = (req_gripper_speed_ << 8) + (req_gripper_force_);

            modbus_write_registers(modbus_, 0x3E8, 3, send_registers);

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            // Receive status from Gripper
            uint16_t recv_registers[3] = {0, };
            modbus_read_registers(modbus_, 0x7D0, 3, recv_registers);

            auto msg = robotiq_gripper_interfaces::msg::GripperStatus();
            msg.g_act = (uint8_t)(recv_registers[0] >> 8) & 0x01;
            msg.g_gto = (uint8_t)(recv_registers[0] >> 11) & 0x01;
            msg.g_sta = (uint8_t)(recv_registers[0] >> 12) & 0x03;
            msg.g_obj = (uint8_t)(recv_registers[0] >> 14) & 0x03;
            msg.g_flt = (uint8_t)(recv_registers[1] >> 8) & 0x0F;
            msg.g_pr = (uint8_t)(recv_registers[1]) & 0xFF;
            msg.g_po = (uint8_t)(recv_registers[2] >> 8) & 0xFF;
            msg.g_cu = (uint8_t)(recv_registers[2]) & 0xFF;

            pub_gripper_status_->publish(msg);

            auto js_msg = sensor_msgs::msg::JointState();
            js_msg.header.stamp = this->get_clock()->now();
            js_msg.name.push_back("gripper_finger_joint");
            js_msg.position.push_back(msg.g_po / 255.0);
            js_msg.velocity.push_back(0.0);
            js_msg.effort.push_back(msg.g_cu / 255.0);

            pub_joint_states_->publish(js_msg);
        }

        void gripper_command_callback(const robotiq_gripper_interfaces::msg::GripperCommand::SharedPtr msg)
        {
            if(msg->position > 1.0 && msg->position < 0.0)
            {
                RCLCPP_WARN(this->get_logger(), "Gripper positon is in [0...1]...");
                return;
            }

            if(msg->speed > 1.0 && msg->speed < 0.0)
            {
                RCLCPP_WARN(this->get_logger(), "Gripper speed is in [0...1]...");
                return;
            }

            if(msg->force > 1.0 && msg->force < 0.0)
            {
                RCLCPP_WARN(this->get_logger(), "Gripper force is in [0...1]...");
                return;
            }

            req_gripper_pos_ = (uint8_t)(msg->position * 255.0);
            req_gripper_speed_ = (uint8_t)(msg->speed * 255.0);
            req_gripper_force_ = (uint8_t)(msg->force * 255.0);
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        modbus_t *modbus_;

        uint8_t req_gripper_pos_;
        uint8_t req_gripper_speed_;
        uint8_t req_gripper_force_;

        rclcpp::Publisher<robotiq_gripper_interfaces::msg::GripperStatus>::SharedPtr pub_gripper_status_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
        rclcpp::Subscription<robotiq_gripper_interfaces::msg::GripperCommand>::SharedPtr sub_gripper_command_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotiqGripperController>());
    rclcpp::shutdown();
    return 0;
}