#include <memory>              // 스마트 포인터 사용 (변경 불가)
#include <cmath>               // 수학 연산 사용 (변경 불가)
#include "rclcpp/rclcpp.hpp"   // ROS2 라이브러리 (변경 불가)
#include "sensor_msgs/msg/imu.hpp" // IMU 메시지 (변경 불가)

// [Fixed] Madgwick 필터 클래스: 내부 로직은 수학적 팩트입니다.
class MadgwickFilter {
public:
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; 
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
        // 간략화된 쿼터니언 업데이트 로직
        q0 += (-q1 * gx - q2 * gy - q3 * gz) * (0.5f * dt);
        q1 += (q0 * gx + q2 * gz - q3 * gy) * (0.5f * dt);
        q2 += (q0 * gy - q1 * gz + q3 * gx) * (0.5f * dt);
        q3 += (q0 * gz + q1 * gy - q2 * gx) * (0.5f * dt);
        float norm = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
    }
};

class SignalProcessorTest : public rclcpp::Node {
public:
    SignalProcessorTest() : Node("Signal_Processor_Test") {
        // [팩트] 반드시 앞에 슬래시(/)가 있어야 전역 토픽을 잡습니다.
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_data", 10, std::bind(&SignalProcessorTest::topic_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Signal Processor Test Node Started!");
    }

private:
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // 1. 필터 업데이트 (주기 0.1s)
        filter_.update(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
                       msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z, 0.1f);

        // 2. 오일러 각도 변환 (이름을 바꾸면 안 되는 핵심 수식)
        float q0 = filter_.q0; float q1 = filter_.q1; float q2 = filter_.q2; float q3 = filter_.q3;
        float roll = std::atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.2958f;
        float pitch = std::asin(2.0f * (q0 * q2 - q3 * q1)) * 57.2958f;

        // 3. 결과 출력
        RCLCPP_INFO(this->get_logger(), "==> [Roll]: %2.1f | [Pitch]: %2.1f", roll, pitch);
    }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    MadgwickFilter filter_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalProcessorTest>());
    rclcpp::shutdown();
    return 0;
}