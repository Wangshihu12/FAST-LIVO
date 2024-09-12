#include <ros/ros.h>
#include <mutex>
#include <std_msgs/String.h>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <atomic>
#include <deque>

#include <livox_ros_driver2/CustomMsg.h>

using namespace std;

std::mutex mtx_buffer;
std::deque<double> time_buffer;
std::deque<double> current_time_buffer;

// 原子布尔值，用于线程间同步
std::atomic<bool> gpio_signal_needed(false);

// 在单独线程中控制GPIO引脚
void gpioControlThread()
{
    while (ros::ok())
    {
        if (gpio_signal_needed)
        {
            system("gpio write 7 1");
            cout << "发送高电平....." << endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(30)); // 延时30ms
            system("gpio write 7 0"); // 发送低电平信号
            cout << "发送低电平....." << endl;

            // 重置信号标志
            gpio_signal_needed = false;
        }
        else
        {
            // 休眠一段时间
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

// 回调函数，用于处理接收到的消息
void triggerCallback(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
{
    // mtx_buffer.lock();
    // time_buffer.push_back(msg->header.stamp.toSec());
    // current_time_buffer.push_back(ros::Time::now().toSec());
    // mtx_buffer.unlock();

    // 获取当前时间
    double current_time = ros::Time::now().toSec();

    // 这里需要根据消息实际类型来访问时间戳
    double message_time = msg->header.stamp.toSec();

    // 当前时间与消息时间戳的时间差（毫秒）
    double time_diff = (current_time - message_time) * 1000;
    cout << "时间差为: " << time_diff << endl;

    // 等待 time_diff 后设置信号标志
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(time_diff)));
    gpio_signal_needed = true;
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "listener");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 设置GPIO引脚为输出模式
    system("gpio mode 7 out");

    // 启动GPIO控制线程
    std::thread gpio_thread(gpioControlThread);

    // 订阅话题
    ros::Subscriber sub = nh.subscribe("trigger_topic", 1000, triggerCallback);

    // 循环等待消息
    ros::spin();

    // 等待GPIO控制线程结束（通常在程序结束时）
    gpio_thread.join();

    return 0;
}