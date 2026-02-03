#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

serial::Serial _serial;				// serial object

class MirobotWriteNode : public rclcpp::Node
{
public:
    MirobotWriteNode()
        : Node("mirobot_write_node")
    {
        this->declare_parameter("joint_states_topic_name", "joint_states");
        joint_states_topic_name = this->get_parameter("joint_states_topic_name").as_string();
        RCLCPP_INFO(this->get_logger(), "Joint States Topic Name : %s", joint_states_topic_name.c_str());

        js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic_name, 10, std::bind(&MirobotWriteNode::joint_state_callback, this, std::placeholders::_1));

        request_exit_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "request_normal_exit", 10, std::bind(&MirobotWriteNode::request_normal_exit_callback, this, std::placeholders::_1));
    }

    void request_normal_exit_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/)
    {
        RCLCPP_INFO(this->get_logger(), "Received normal exit request, closing serial and shutting down");
        if (_serial.isOpen()) {
            _serial.close();
            RCLCPP_INFO(this->get_logger(), "Serial port closed");
        }
        rclcpp::shutdown();
    }

    void homing(){
        //TODO: home position reset
        // https://document.wlkata.com/?doc=/wlkata-mirobot-user-manual-platinum/18-g-code-instruction-set/
        std::string HomingGcode = (std::string)"$H" + "\r\n";

        _serial.write(HomingGcode.c_str());
        result.data = _serial.read(_serial.available());

        RCLCPP_INFO(this->get_logger(), "%s", result.data.c_str());
        RCLCPP_INFO(this->get_logger(), "Wait for seconds, Mirobot is Homing now...");
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!_serial.isOpen()) {
            return;
        }
        std::string Gcode = "";
        char angle0[10];
        char angle1[10];
        char angle2[10];
        char angle3[10];
        char angle4[10];
        char angle5[10];

        sprintf(angle0, "%.2f", msg->position[0]*57.296);
        sprintf(angle1, "%.2f", msg->position[1]*57.296);
        sprintf(angle2, "%.2f", msg->position[2]*57.296);
        sprintf(angle3, "%.2f", msg->position[3]*57.296);
        sprintf(angle4, "%.2f", msg->position[4]*57.296);
        sprintf(angle5, "%.2f", msg->position[5]*57.296);
        Gcode = (std::string)"M50 G0 X" + angle0 + " Y" + angle1 + " Z" + angle2 + " A" + angle3 + "B" + angle4 + "C" + angle5 + " F3000" + "\r\n";
        
        RCLCPP_INFO(this->get_logger(), "%s", Gcode.c_str());

        _serial.write(Gcode.c_str());
        result.data = _serial.read(_serial.available());
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr request_exit_sub_;
    std::string joint_states_topic_name;
    std_msgs::msg::String result;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto mirobot_gcode_write_node = std::make_shared<MirobotWriteNode>();
    
    mirobot_gcode_write_node->declare_parameter("port_name", "/dev/ttyUSB0");
    mirobot_gcode_write_node->declare_parameter("baud_rate", 115200);

    auto port_name = mirobot_gcode_write_node->get_parameter("port_name").as_string();
    auto baud_rate = mirobot_gcode_write_node->get_parameter("baud_rate").as_int();

    RCLCPP_INFO(mirobot_gcode_write_node->get_logger(), "Port Name : %s", port_name.c_str());
    RCLCPP_INFO(mirobot_gcode_write_node->get_logger(), "Baudrate : %ld", static_cast<long>(baud_rate));

	try{
		_serial.setPort(port_name);
		_serial.setBaudrate(baud_rate);

		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		_serial.setTimeout(to);

		// 启动时重置端口：先打开再关闭并等待，然后重新打开，清除上次未正确退出遗留的状态
		_serial.open();
		_serial.close();
		using namespace std::chrono_literals;
		rclcpp::sleep_for(400ms);
		_serial.open();

		// DTR/RTS 翻转，让机械臂端串口状态复位
		_serial.setDTR(false);
		_serial.setRTS(false);
		rclcpp::sleep_for(100ms);
		_serial.setDTR(true);
		_serial.setRTS(true);
		rclcpp::sleep_for(100ms);

		_serial.write("M50\r\n");

		RCLCPP_INFO(mirobot_gcode_write_node->get_logger(), "Port has been open successfully");
	}
	catch (serial::IOException& e){
        RCLCPP_FATAL(mirobot_gcode_write_node->get_logger(), "Unable to open port");
		return -1;
	}

	// 在 shutdown 时显式关闭串口，避免 Ctrl+C 退出后端口处于异常状态导致再次启动无法收发
	rclcpp::on_shutdown([&]() {
		if (_serial.isOpen()) {
			_serial.close();
			RCLCPP_INFO(mirobot_gcode_write_node->get_logger(), "Serial port closed on shutdown");
		}
	});

	if (_serial.isOpen()){
        using namespace std::chrono_literals;
        rclcpp::sleep_for(1s);
        RCLCPP_INFO(mirobot_gcode_write_node->get_logger(), "Attach and wait for commands");
	}

    mirobot_gcode_write_node->homing();
    rclcpp::sleep_for(std::chrono::seconds(13));
    RCLCPP_INFO(mirobot_gcode_write_node->get_logger(), "Homing Done!!!");

    rclcpp::spin(mirobot_gcode_write_node);
    rclcpp::shutdown();

	if (_serial.isOpen()) {
		_serial.close();
	}

    return 0;
}
