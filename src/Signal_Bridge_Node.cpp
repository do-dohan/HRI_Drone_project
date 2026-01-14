#include "HRI_Drone_HPP/Signal_Bridge_Node.hpp"


Signal_Bridge::Signal_Bridge() : Node("Signal_Bridge_Node"){
    raw_data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    raw_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("raw_data",1);
    raw_timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Signal_Bridge::raw_data_pub, this));
    RCLCPP_INFO(this->get_logger(), "센서 브릿지 노드가 성공적으로 시작되었습니다.");
}

void Signal_Bridge::raw_data_pub(){
    for (int i = 0; i < 10; ++i){
        float new_val = static_cast<int>((this->raw_data[i] + 0.1f) * 10.0f + 0.5f) * 0.1f;
        if (new_val < 10.0f){
            this->raw_data[i] = new_val;
        }
        else{
            this->raw_data[i] = 0.0f;
        }
    }
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = this->raw_data;
    raw_pub->publish(msg);
    RCLCPP_INFO(this->get_logger(), "데이터 발행 중: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
        msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4],
        msg.data[5], msg.data[6], msg.data[7], msg.data[8], msg.data[9]);
}