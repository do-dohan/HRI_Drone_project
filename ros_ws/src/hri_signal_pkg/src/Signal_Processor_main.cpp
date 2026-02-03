#include "rclcpp/rclcpp.hpp"
#include "hri_signal_pkg/Signal_Processor_Node.hpp"

int main(int argc, char *argv[]){
    // 1. ROS 2 통신 시스템을 초기화하고 커맨드라인 인수를 처리합니다.
    // 1. Initialize ROS 2 communication and handle command line args.
    rclcpp::init(argc, argv);

    // 2. 우리가 만든 Signal_Processor 클래스의 객체를 메모리에 생성합니다.
    // 2. Create an instance of Signal_Processor class in memory.
    auto signal_processor = std::make_shared<Signal_Processor>();

    // 3. 노드가 종료될 때까지 타이머와 콜백 함수들이 계속 작동하도록 대기합니다.
    // 3. Spin the node to keep callbacks active until shutdown.
    rclcpp::spin(signal_processor);

    // 4. 프로그램 종료 시(Ctrl+C) ROS 2 리소스를 정리하고 종료합니다.
    // 4. Clean up ROS 2 resources upon exit.
    rclcpp::shutdown();
    
    return 0;
}