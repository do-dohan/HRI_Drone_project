#include <chrono>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class TestSensorNode : public rclcpp::Node {
public:
    TestSensorNode() : Node("test_sensor_node"), counter_(0.0) {
        // [팩트] 수신부와 똑같은 "/imu_data" 이름을 써야 합니다.
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_data", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&TestSensorNode::timer_callback, this));
    }

private:
    void timer_callback() {
        auto message = sensor_msgs::msg::Imu();
        counter_ += 0.1;

        // 자이로 값을 sin, cos으로 변화시켜서 Roll/Pitch가 움직이게 만듭니다.
        message.angular_velocity.x = std::sin(counter_);
        message.angular_velocity.y = std::cos(counter_);
        message.linear_acceleration.z = 9.8; // 기본 중력값

        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Sending Fake IMU -> gx: %.2f, gy: %.2f", message.angular_velocity.x, message.angular_velocity.y);
    }
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double counter_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestSensorNode>());
    rclcpp::shutdown();
    return 0;
}