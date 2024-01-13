/***
@作者: 古月居(www.guyuehome.com)
@说明: ROS2话题示例-发布图像话题
***/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"          // ROS2 C++接口库
#include "std_msgs/msg/string.hpp"    // 字符串消息类型

using namespace std::chrono_literals;


class PublisherNode : public rclcpp::Node
{
    public:
        PublisherNode()
        : Node("topic_helloworld_pub") // ROS2节点父类初始化
        {
            publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10); // 创建发布者对象（消息类型、话题名、队列长度）
            timer_ = this->create_wall_timer(
                500ms, std::bind(&PublisherNode::timer_callback, this));            // 创建一个定时器,定时执行回调函数
        }

    private:
        void timer_callback()                                                       // 创建定时器周期执行的回调函数
        {
          auto msg = std_msgs::msg::String();                                       // 创建一个String类型的消息对象
          msg.data = "Hello World";                                                 // 填充消息对象中的消息数据
          RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());    // 发布话题消息
          publisher_->publish(msg);                                                 // 输出日志信息，提示已经完成话题发布
        }
        
        rclcpp::TimerBase::SharedPtr timer_;                                        // 定时器指针
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;             // 发布者指针
};

int main(int argc, char * argv[])                      // ROS2节点主入口main函数
{
    rclcpp::init(argc, argv);                          // ROS2 C++接口初始化
    rclcpp::spin(std::make_shared<PublisherNode>());   // 创建ROS2节点对象并进行初始化
    rclcpp::shutdown();                                // 关闭ROS2 C++接口

    return 0;
}
