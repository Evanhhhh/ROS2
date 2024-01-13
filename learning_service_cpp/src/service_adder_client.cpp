/***
@作者: 古月居(www.guyuehome.com)
@说明: ROS2服务示例-发送两个加数，请求加法器计算
***/

#include "rclcpp/rclcpp.hpp"                          // ROS2 C++接口库
#include "learning_interface/srv/add_two_ints.hpp"    // 自定义的服务接口

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                      // ROS2 C++接口初始化

    if (argc != 3) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: service_adder_client X Y");
        return 1;
    }

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_adder_client");   // 创建ROS2节点对象并进行初始化
    rclcpp::Client<learning_interface::srv::AddTwoInts>::SharedPtr client =
        node->create_client<learning_interface::srv::AddTwoInts>("add_two_ints");             // 创建服务客户端对象（服务接口类型，服务名）

    auto request = std::make_shared<learning_interface::srv::AddTwoInts::Request>();          // 创建服务接口数据
    request->a = atoll(argv[1]);
    request->b = atoll(argv[2]);

    while (!client->wait_for_service(1s)) {                                                   // 循环等待服务器端成功启动
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);                                        // 异步方式发送服务请求
    

    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)                                                    // 接收服务器端的反馈数据
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);             // 将收到的反馈信息打印输出
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }

    rclcpp::shutdown();                                                                       // 关闭ROS2 C++接口
    return 0;
}
