std_msgs/Header header

## 控制命令模式 
uint8 Mode

# Mode 控制模式枚举
uint8 Idle=0
uint8 Takeoff=1
uint8 Move=2
uint8 Hover=3
uint8 Trajectory=4

## 移动命令坐标系
uint8 Move_frame
## 坐标系枚举
uint8 ENU = 0           ### 惯性系
uint8 BODY = 1          ### 机体系

## 移动命令下的子模式
uint8 Move_mode
## 移动命令下的子模式枚举
uint8 XYZ_POS = 0       ### 定点控制
uint8 XY_VEL_Z_POS = 1  ### 定高速度控制
uint8 XYZ_VEL = 2       ### 速度控制
uint8 XYZ_REL_POS = 3   ### 定点控制

## 指令输入
float32[3] desire_cmd          ## [m] [m/s] [m/s^2]
float32 yaw_cmd                ## [rad]