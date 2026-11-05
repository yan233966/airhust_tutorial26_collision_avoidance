#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>

//全局变量
//--------------------------------------------雷达（三维点云）--------------------------------------------------
#define BIN_NUM      360                                       //水平方向 1[度] 一个 bin
#define NO_OBSTACLE  100.0f                                    //该方向无有效点时的哨兵距离 [m]

float bin_range[BIN_NUM];                                       //每个 bin 内最近障碍物的水平距离 [m]
int   bin_valid = 0;                                            //本帧有数据的 bin 个数（调试用）

int range_min;                                                  //参与搜索的最小 bin [度]
int range_max;                                                  //参与搜索的最大 bin [度]

float z_band             = 0.30f;                               //"0 度平面"上下取点的高度带 [m]
float min_obstacle_range = 0.15f;                               //小于该距离的点丢弃（自车/地面噪点） [m]
float max_obstacle_range = 8.00f;                               //大于该距离的点丢弃 [m]

//--------------------------------------------算法相关--------------------------------------------------
float R_outside,R_inside;                                       //安全半径 [避障算法相关参数]
float p_R;                                                      //大圈比例参数
float p_r;                                                      //小圈比例参数
float distance_c = NO_OBSTACLE, angle_c = 0;                    //最近障碍物距离[m] 角度[度]（初值=无障碍，避免首帧 0/0）
float distance_cx,distance_cy;                                  //最近障碍物距离XY
float vel_collision[2];                                         //躲避障碍部分速度
float vel_collision_ENU[2];                                     //躲避障碍部分世界速度
float vel_collision_max;                                        //躲避障碍部分速度限幅
float p_xy;                                                     //追踪部分位置环P
float vel_track[2];                                             //追踪部分速度
float vel_track_max;                                            //追踪部分速度限幅

//--------------------------------------------输出--------------------------------------------------
std_msgs::Bool flag_collision_avoidance;                       //是否进入避障模式标志位
float vel_sp_ENU[2];                                            //ENU下的总速度
float vel_sp_max;                                               //总速度限幅


void rotation_yaw(float yaw_angle, float input[2], float output[2])
{
    
}

//在 360 个 bin 里找最近的障碍物：distance_c 为水平距离[m]，angle_c 为机体系角度[度]，0 度 = 机头方向
void cal_min_distance()
{
    distance_c = NO_OBSTACLE;
    angle_c    = 0;
    bin_valid  = 0;

    for (int i = range_min; i <= range_max; i++)
    {
        if (i < 0 || i >= BIN_NUM) continue;            //参数写错了就跳过
        if (bin_range[i] >= NO_OBSTACLE) continue;      //这个方向本帧没有点
        bin_valid++;
        if (bin_range[i] < distance_c)
        {
            distance_c = bin_range[i];
            angle_c    = i;
        }
    }
}



//避障函数
void collision_avoidance(float target_x,float target_y)
{
    //2. 根据最小距离判断：是否启用避障策略
    if (distance_c >= R_outside )
    {
        flag_collision_avoidance.data = false;
    }
    else
    {
        flag_collision_avoidance.data = true;
    }

    //3. 计算追踪速度
    vel_track[0] = p_xy * (target_x - local_pos.pose.pose.position.x);
    vel_track[1] = p_xy * (target_y - local_pos.pose.pose.position.y);

    //速度限幅
    float vel_max = (fabs(vel_track[0])>=fabs(vel_track[1]))?fabs(vel_track[0]):fabs(vel_track[1]);
    if (vel_max > vel_track_max){
        for (int i = 0; i < 2; i++)
        {
            vel_track[i] = vel_track[i] * vel_track_max / vel_max;
        }
    }

    vel_collision[0]= 0;
    vel_collision[1]= 0;

    //4. 避障策略
    if(flag_collision_avoidance.data == true)
    {
        distance_cx = distance_c * cos(angle_c/180*3.1415926);
        distance_cy = distance_c * sin(angle_c/180*3.1415926);

        float F_c = 0;

        //小幅度抑制移动速度
        

        //大幅度抑制移动速度
        

        //安全性检查 7：距离为 0 时"障碍物在哪个方向"没有意义，跳过除法，避免 0/0 = nan
        if (distance_c > 0.01f)
        {
            vel_collision[0] = vel_collision[0] - F_c * distance_cx / distance_c;
            vel_collision[1] = vel_collision[1] - F_c * distance_cy / distance_c;
        }
        //避障速度限幅到设定最大值（需要等比例修改）
        

    }

    //坐标系转换
    

    vel_sp_ENU[0] = vel_track[0] + vel_collision_ENU[0];
    vel_sp_ENU[1] = vel_track[1] + vel_collision_ENU[1];


    //速度限幅
    

}

//接收处理三维点云（世界系 ENU）：
//   只保留与机体同高（"0 度平面"）这个水平切片里的点
void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    //第一步：先把 360 个 bin 全部清空，表示"这些方向都没看到障碍物"
    for (int i = 0; i < BIN_NUM; i++)
    {
        bin_range[i] = NO_OBSTACLE;
    }

    //把 ROS 的点云消息转成 pcl 点云，方便一个点一个点地读
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    //这一帧点云是空的，直接认为"没有障碍物"
    if (cloud.points.empty())
    {
        cal_min_distance();
        return;
    }

    //机体当前位置（世界系 ENU）
    float px = local_pos.pose.pose.position.x;
    float py = local_pos.pose.pose.position.y;
    float pz = local_pos.pose.pose.position.z;

    //机体位置本身是 nan/inf 时，这一帧不处理
    if (!isfinite(px) || !isfinite(py) || !isfinite(pz))
    {
        cal_min_distance();
        return;
    }

    int count = static_cast<int>(cloud.points.size());

    for (int i = 0; i < count; i++)
    {
        float x = cloud.points[i].x;
        float y = cloud.points[i].y;
        float z = cloud.points[i].z;

        //无效点（nan / inf）跳过，否则后面的除法会算出 nan
        if (!isfinite(x) || !isfinite(y) || !isfinite(z)) continue;

        //只取"0 度平面"附近的点：和机体当前高度相差不超过 z_band
        if (fabs(z - pz) > z_band) continue;

        float dx = x - px;
        float dy = y - py;
        float r  = sqrt(dx * dx + dy * dy);     //水平距离 [m]

        //太近的点是自机噪声，太远的点不参与避障；
        //这个检查同时保证了 r > 0，后面算方向不会出现 0/0
        if (r < min_obstacle_range || r > max_obstacle_range) continue;

        //世界系方位角 - 机体偏航角 = 机体系角度（0 度 = 机头方向，逆时针为正）
        float bearing_world = atan2(dy, dx) * 180.0 / M_PI;     //世界系方位角 [度]
        float yaw_degree    = yaw * 180.0 / M_PI;               //机体偏航角 [度]
        float bearing       = bearing_world - yaw_degree;       //机体系角度 [度]

        //角度算出来不是正常数字就跳过，别拿去做下标
        if (!isfinite(bearing)) continue;

        //换算到 [0, 360) 度
        while (bearing < 0)    bearing += 360;
        while (bearing >= 360) bearing -= 360;

        int bin = static_cast<int>(bearing);

        //防止下标越界（正常算不会发生，保险起见）
        if (bin < 0)        bin = 0;
        if (bin >= BIN_NUM) bin = BIN_NUM - 1;

        //同一个方向上取最近的那个点
        if (r < bin_range[bin])
        {
            bin_range[bin] = r;
        }
    }

    cal_min_distance();
}
