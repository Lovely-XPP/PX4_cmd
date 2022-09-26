#include <ros/ros.h>

#include <iostream>
#include <string>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <px4_cmd/Command.h>

#include <utility/printf_utility.h>

using namespace std;

// 初始化信息用于接受设置命令,发送指定位置信息
px4_cmd::Command set_cmd;
mavros_msgs::PositionTarget pos_setpoint;
mavros_msgs::SetMode mode_cmd;
geometry_msgs::PoseStamped current_pos;

// 初始化订阅和广播
ros::Subscriber set_cmd_sub;
ros::Subscriber current_pos_sub;
ros::Publisher setpoint_raw_local_pub;

// 声明回调函数
void sub_set_cmd_cb(const px4_cmd::Command::ConstPtr &msg);
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

int main(int argc, char **argv)
{
    // 节点初始化
    ros::init(argc, argv, "send_command");
    ros::NodeHandle nh;

    // 广播和节点
    set_cmd_sub = nh.subscribe<px4_cmd::Command>("/px4_cmd/control_command", 10, sub_set_cmd_cb);
    current_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);
    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    // 服务
    ros::ServiceClient mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // 等待节点初始化完成
    sleep(1);
    ros::Rate rate(50.0);

    // 输出标题（提示）
    system("clear");
    print_head("PX4 Command Sender");
    Info("PX4 Command Sender is Running...");

    // 主循环
    while (ros::ok())
    {
        setpoint_raw_local_pub.publish(pos_setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

// 订阅回调函数,获取设置的指令信息
void sub_set_cmd_cb(const px4_cmd::Command::ConstPtr &msg)
{
    set_cmd = *msg;

    // 设定坐标系
    switch (set_cmd.Move_frame)
    {
        case px4_cmd::Command::ENU:
        {
            pos_setpoint.coordinate_frame = 1;
            break;
        }

        case px4_cmd::Command::BODY:
        {
            pos_setpoint.coordinate_frame = 8;
            break;
        }
    }

    // 设定输入值
    // Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    // Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    // Bit 10 should set to 0, means is not force sp
    switch (set_cmd.Move_mode)
    {
        case px4_cmd::Command::XYZ_POS:
        {
            pos_setpoint.type_mask = 0b100111111000;
            if(set_cmd.Mode == px4_cmd::Command::Hover)
            {
                pos_setpoint.position.x = current_pos.pose.position.x;
                pos_setpoint.position.y = current_pos.pose.position.y;
                pos_setpoint.position.z = current_pos.pose.position.z;
            }
            else
            {
                pos_setpoint.position.x = set_cmd.desire_cmd[0];
                pos_setpoint.position.y = set_cmd.desire_cmd[1];
                pos_setpoint.position.z = set_cmd.desire_cmd[2];
            }
            break;
        }

        case px4_cmd::Command::XY_VEL_Z_POS:
        {
            pos_setpoint.type_mask = 0b100111100011;
            pos_setpoint.velocity.x = set_cmd.desire_cmd[0];
            pos_setpoint.velocity.y = set_cmd.desire_cmd[1];
            pos_setpoint.position.z = set_cmd.desire_cmd[2];
            break;
        }

        case px4_cmd::Command::XYZ_VEL:
        {
            pos_setpoint.type_mask = 0b100111000111;
            pos_setpoint.velocity.x = set_cmd.desire_cmd[0];
            pos_setpoint.velocity.y = set_cmd.desire_cmd[1];
            pos_setpoint.velocity.z = set_cmd.desire_cmd[2];
            break;
        }

        case px4_cmd::Command::XYZ_REL_POS:
        {
            pos_setpoint.type_mask = 0b100111111000;
            pos_setpoint.position.x = set_cmd.desire_cmd[0];
            pos_setpoint.position.y = set_cmd.desire_cmd[1];
            pos_setpoint.position.z = set_cmd.desire_cmd[2];
            break;
        }
    }
    
    pos_setpoint.header.frame_id = 1;

    pos_setpoint.yaw = set_cmd.yaw_cmd;
        
}

// 订阅回调返回状态信息
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pos = *msg;
}
