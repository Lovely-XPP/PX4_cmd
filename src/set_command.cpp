#include <ros/ros.h>

#include <iostream>
#include <string>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <px4_cmd/Command.h>

#include <utility/printf_utility.h>

using namespace std;


// 发布消息初始化
ros::Publisher cmd_pub;

// 初始化命令
px4_cmd::Command cmd;

int main(int argc, char **argv)
{
    // 节点初始化
    ros::init(argc, argv, "set_command");
    ros::NodeHandle nh;

    cmd_pub = nh.advertise<px4_cmd::Command>("/px4_cmd/control_command", 10);

    // 定义列表储存所有模式
    std::vector<string> command_list = {
        "Idle",    //怠速
        "Takeoff", //起飞到指定高度
        "Move",    //移动
        "Land",    //降落
        "Disarm",  //上锁
    };

    std::vector<string> frame_list = {
        "ENU",    //惯性坐标系，东北天
        "Body",   //机体坐标系
    };

    std::vector<string> move_list = {
        "Position (XYZ)",            //三位置
        "Velocity (XY) + Alltitude", //定高两速度
        "Velocity (XYZ)"             //定高两速度
    };

    return 0;
}
