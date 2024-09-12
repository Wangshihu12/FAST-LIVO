#include <iostream>
// #include <gpiod.h>  // 包含GPIO库头文件
// #include <wiringPi.h>
#include <chrono>   // 提供高精度计时功能
#include <thread>   // 提供延时功能
#include <cstdlib>  // 提供 system 函数

#define GPIO_CHIP "gpiochip0" // GPIO芯片名，通常为gpiochip0
#define GPIO_LINE 7          // GPIO引脚编号

// int main() {
//     // 打开GPIO芯片
//     gpiod::chip chip(GPIO_CHIP);
//     gpiod::line line = chip.get_line(GPIO_LINE);
    
//     // 请求GPIO引脚，设置为输出模式
//     line.request({"gpio_control", gpiod::line_request::DIRECTION_OUTPUT});

//     std::cout << "Triggering camera..." << std::endl;

//     // 发送高电平信号
//     line.set_value(1);
//     std::this_thread::sleep_for(std::chrono::milliseconds(30)); // 延时30ms

//     // 发送低电平信号
//     line.set_value(0);
//     std::this_thread::sleep_for(std::chrono::milliseconds(30)); // 延时70ms

//     std::cout << "Camera triggered." << std::endl;

//     return 0;
// }



int main() {

    // 设置GPIO引脚为输出模式
    system("gpio mode 7 out");

    while (true)
    {
        // 发送高电平信号
        system("gpio write 7 1");
        std::this_thread::sleep_for(std::chrono::milliseconds(30)); // 延时30ms

        // 发送低电平信号
        system("gpio write 7 0");
        std::this_thread::sleep_for(std::chrono::milliseconds(70));
    }
    

    return 0;
}