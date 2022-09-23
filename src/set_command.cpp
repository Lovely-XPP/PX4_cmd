#include <ros/ros.h>

#include <thread>
#include <iostream>
#include <string>

#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <px4_cmd/Command.h>

#include <utility/printf_utility.h>

using namespace std;


// 发布消息初始化
ros::Publisher cmd_pub;
ros::Subscriber state_sub;

// 订阅信息
geometry_msgs::PoseStamped current_state;
void state_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

// 初始化命令
px4_cmd::Command cmd;

// 定义列表储存所有模式
std::vector<string> command_list = {
    "Idle",    //怠速
    "Takeoff", //起飞到指定高度
    "Move",    //移动
};

// 坐标系
std::vector<string> frame_list = {
    "ENU", //惯性坐标系，东北天
    "Body" //机体坐标系
};

// 指令方式
std::vector<string> move_list = {
    "Position (XYZ)",            //三位置
    "Velocity (XY) + Alltitude", //定高两速度
    "Velocity (XYZ)"             //定高两速度
};

// 子函数声明
void print_current_cmd(px4_cmd::Command cmd);
void pub_thread_fun();

/*     主函数      */
int main(int argc, char **argv)
{
    // 节点初始化
    ros::init(argc, argv, "set_command");
    ros::NodeHandle nh;

    // 订阅
    state_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, state_cb);

    // 广播初始化
    cmd_pub = nh.advertise<px4_cmd::Command>("/px4_cmd/control_command", 10);

    // 用户输入
    int switch_cmd = 0;
    int switch_frame = 0;
    int switch_cmd_mode = 0;

    // 初始化命令信息
    float desire_cmd_value[3];
    float yaw_value = 0.0;
    desire_cmd_value[0] = 0.0;
    desire_cmd_value[1] = 0.0;
    desire_cmd_value[2] = 0.0;
    cmd.Mode = px4_cmd::Command::Idle;
    cmd.Move_frame = px4_cmd::Command::ENU;
    cmd.Move_mode = px4_cmd::Command::XYZ_POS;
    cmd.desire_cmd[0] = desire_cmd_value[0];
    cmd.desire_cmd[1] = desire_cmd_value[1];
    cmd.desire_cmd[2] = desire_cmd_value[2];
    cmd.yaw_cmd = yaw_value;

    // 开启广播线程
    sleep(1);
    std::thread pub_thread(pub_thread_fun);
    pub_thread.detach();

    // 主循环
    while (ros::ok())
    {
        // 清屏及初始化
        system("clear");
        cout << POINTER;

        // 输出标题及选项
        print_title("PX4 Offboard Command", command_list);
        print_current_cmd(cmd);

        // 获取用户输入
        cout << "\n" << "Input Command Number: ";
        cin >> switch_cmd;

        // 判断输入正确性
        if (switch_cmd >= command_list.size() || switch_cmd < 0)
        {
            cout << "\n";
            Error("Please Input int 0 ~ " + to_string(command_list.size() - 1));
            sleep(2);
            continue;
        }

        // 更改模式
        cmd.Mode = switch_cmd;

        switch (cmd.Mode)
        {
            case px4_cmd::Command::Idle:
            {
                cmd.Move_frame = px4_cmd::Command::ENU;
                cmd.Move_mode = px4_cmd::Command::XYZ_POS;
                cmd.desire_cmd[0] = 0.0;
                cmd.desire_cmd[1] = 0.0;
                cmd.desire_cmd[2] = 0.0;
                cmd.yaw_cmd = 0.0;
                break;
            }

            case px4_cmd::Command::Takeoff:
            {
                cmd.Move_frame = px4_cmd::Command::ENU;
                cmd.Move_mode = px4_cmd::Command::XYZ_POS;
                cmd.desire_cmd[0] = current_state.pose.position.x;
                cmd.desire_cmd[1] = current_state.pose.position.y;
                cmd.desire_cmd[2] = 2.0;
                cmd.yaw_cmd = 0.0;
                break;
            }

            case px4_cmd::Command::Move:
            {
                // 输入坐标系
                system("clear");
                print_title("PX4 Command Center", frame_list);
                cout << WHITE << "Input frame id: ";
                cin >> switch_frame;
                // 判断输入正确性
                if (switch_frame >= frame_list.size() || switch_frame < 0)
                {
                    cout << "\n" << NO_POINTER;
                    Error("Please Input int 0 ~ " + to_string(frame_list.size() - 1));
                    sleep(2);
                    continue;
                }

                // 机体坐标系仅支持速度控制
                if (switch_frame == px4_cmd::Command::BODY)
                {
                    system("clear");
                    print_head("PX4 Command Center");
                    cout << YELLOW << "Tip: Body Frame Only Support [" << GREEN << "Velocity (XYZ)" << YELLOW << "] Control!" << WHITE << endl;
                    cout << "\n" << "X Velocity [m/s]: ";
                    cin >> desire_cmd_value[0];
                    cout << "\n" << "Y Velocity [m/s]: ";
                    cin >> desire_cmd_value[1];
                    cout << "\n" << "Z Velocity [m/s]: ";
                    cin >> desire_cmd_value[2];
                    // yaw指令输入
                    cout << "\n" << "Yaw Command [rad]: ";
                    cin >> yaw_value;
                    // 修改命令
                    cmd.Move_frame = switch_frame;
                    cmd.Move_mode = px4_cmd::Command::XYZ_VEL;
                    cmd.desire_cmd[0] = desire_cmd_value[0];
                    cmd.desire_cmd[1] = desire_cmd_value[1];
                    cmd.desire_cmd[2] = desire_cmd_value[2];
                    cmd.yaw_cmd = yaw_value;
                    break;
                }

                // 输入命令类型
                system("clear");
                print_title("PX4 Command Center", move_list);
                cout << WHITE << "Input Move Mode Number: ";
                cin >> switch_cmd_mode;
                // 判断输入正确性
                if (switch_cmd_mode >= move_list.size() || switch_cmd_mode < 0)
                {
                    cout << "\n" << NO_POINTER;
                    Error("Please Input int 0 ~ " + to_string(move_list.size() - 1));
                    sleep(2);
                    continue;
                }

                // 输入相应模式的值
                switch (switch_cmd_mode)
                {
                    case px4_cmd::Command::XYZ_POS:
                    {
                        cout << "\n" << "X Position [m]: ";
                        cin >> desire_cmd_value[0];
                        cout << "\n" << "Y Position [m]: ";
                        cin >> desire_cmd_value[1];
                        cout << "\n" << "Z Position [m]: ";
                        cin >> desire_cmd_value[2];
                        break;
                    }

                    case px4_cmd::Command::XY_VEL_Z_POS:
                    {
                        cout << "\n" << "X Velocity [m/s]: ";
                        cin >> desire_cmd_value[0];
                        cout << "\n" << "Y Velocity [m/s]: ";
                        cin >> desire_cmd_value[1];
                        cout << "\n" << "Z Position [m]: ";
                        cin >> desire_cmd_value[2];
                        break;
                    }

                    case px4_cmd::Command::XYZ_VEL:
                    {
                        cout << "\n" << "X Velocity [m/s]: ";
                        cin >> desire_cmd_value[0];
                        cout << "\n" << "Y Velocity [m/s]: ";
                        cin >> desire_cmd_value[1];
                        cout << "\n" << "Z Velocity [m/s]:: ";
                        cin >> desire_cmd_value[2];
                        break;
                    }
                }
                // yaw指令输入
                cout << "\n" << "Yaw Command [rad]: ";
                cin >> yaw_value;

                // 修改命令
                cmd.Move_frame = switch_frame;
                cmd.Move_mode = switch_cmd_mode;
                cmd.desire_cmd[0] = desire_cmd_value[0];
                cmd.desire_cmd[1] = desire_cmd_value[1];
                cmd.desire_cmd[2] = desire_cmd_value[2];
                cmd.yaw_cmd = yaw_value;
                break;
            }
        }
    }
    return 0;
}

// 打印当前命令
void print_current_cmd(px4_cmd::Command cmd)
{
    cout << WHITE << "Current Command: [" << GREEN << command_list[cmd.Mode] << WHITE << "]    ";
    cout << WHITE << "Frame: [" << GREEN << frame_list[cmd.Move_frame] << WHITE << "]" << endl;
    cout << WHITE << "Mode: [" << GREEN << move_list[cmd.Move_mode] << WHITE << "]" << endl;
    cout << WHITE << "Value: " << setprecision(2) << cmd.desire_cmd[0]
         << "  " << cmd.desire_cmd[1] << "  " << cmd.desire_cmd[2] << "    ";
    cout << WHITE << "Yaw: " << setprecision(2) << cmd.yaw_cmd << endl;
}

// 广播线程
void pub_thread_fun()
{
    ros::Rate rate(50.0);
    while (ros::ok())
    {
        cmd_pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();
    }
}

// 订阅回调返回状态信息
void state_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_state = *msg;
}
