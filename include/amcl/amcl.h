#ifndef _AMCL_H
#define _AMCL_H

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include <signal.h>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/foreach.hpp>

#include "amcl/map/map.h"
#include "amcl/pf/pf.h"
#include "amcl/sensors/amcl_odom.h"
#include "amcl/sensors/amcl_laser.h"

#include "ros/assert.h"
#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "std_srvs/Empty.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"

#include "dynamic_reconfigure/server.h"
#include "amcl/AMCLConfig.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace amcl;

// Pose hypothesis
typedef struct
{
  // Total weight (weights sum to 1)
  double weight;

  // Mean of pose esimate
  pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

class AmclNode
{
  public:
    AmclNode();
    ~AmclNode();

    /**
     * @brief Uses TF and LaserScan messages from bag file to drive AMCL instead
     */
    void runFromBag(const std::string &in_bag_fn);

    int process();

    void savePoseToServer();

    static inline double normalize(double z)
    {
      return atan2(sin(z),cos(z));
    }

    static inline double angle_diff(double a, double b)
    {
      double d1, d2;
      a = normalize(a);
      b = normalize(b);
      d1 = a-b;
      d2 = 2*M_PI - fabs(d1);
      if(d1 > 0)
        d2 *= -1.0;
      if(fabs(d1) < fabs(d2))
        return(d1);
      else
        return(d2);
    }

  private:
    tf::TransformBroadcaster* tfb_;

    // Use a child class to get access to tf2::Buffer class inside of tf_
    struct TransformListenerWrapper : public tf::TransformListener
    {
      inline tf2_ros::Buffer &getBuffer() {return tf2_buffer_;}
    };

    TransformListenerWrapper* tf_;

    bool sent_first_transform_;

    tf::Transform latest_tf_;
    bool latest_tf_valid_;

    // Pose-generating function used to uniformly distribute particles over
    // the map
    static pf_vector_t uniformPoseGenerator(void* arg);

    static std::vector<std::pair<int,int> > free_space_indices;

    // Callbacks
    bool global_localization_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    bool nomotionUpdateCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    bool setMapCallback(nav_msgs::SetMap::Request& req, nav_msgs::SetMap::Response& res);

    void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);

    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
    void freeMapDependentMemory();
    map_t* convertMap( const nav_msgs::OccupancyGrid& map_msg );
    void updatePoseFromServer();
    void applyInitialPose();

    double getYaw(tf::Pose& t);

    //parameter for what odom to use
    std::string odom_frame_id_;

    //paramater to store latest odom pose
    tf::Stamped<tf::Pose> latest_odom_pose_;

    //parameter for what base to use
    std::string base_frame_id_;
    std::string global_frame_id_;

    bool use_map_topic_;
    bool first_map_only_;

    ros::Duration gui_publish_period;
    ros::Time save_pose_last_time;
    ros::Duration save_pose_period;

    geometry_msgs::PoseWithCovarianceStamped last_published_pose;

    map_t* map_;
    char* mapdata;
    int sx, sy;
    double resolution;

    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;
    ros::Subscriber initial_pose_sub_;
    std::vector< AMCLLaser* > lasers_;
    std::vector< bool > lasers_update_;
    std::map< std::string, int > frame_to_laser_;

    // Particle filter
    pf_t *pf_;
    double pf_err_, pf_z_;
    bool pf_init_;
    pf_vector_t pf_odom_pose_;
    double d_thresh_, a_thresh_;
    int resample_interval_;
    int resample_count_;
    double laser_min_range_;
    double laser_max_range_;

    //Nomotion update control
    bool m_force_update;  // used to temporarily let amcl update samples even when no motion occurs...

    AMCLOdom* odom_;
    AMCLLaser* laser_;

    ros::Duration cloud_pub_interval;
    ros::Time last_cloud_pub_time;

    // For slowing play-back when reading directly from a bag file
    ros::WallDuration bag_scan_period_;

    void requestMap();

    // Helper to get odometric pose from transform system
    bool getOdomPose(tf::Stamped<tf::Pose>& pose,
                     double& x, double& y, double& yaw,
                     const ros::Time& t, const std::string& f);

    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    ros::Duration transform_tolerance_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pose_pub_;
    ros::Publisher particlecloud_pub_;
    ros::ServiceServer global_loc_srv_;
    ros::ServiceServer nomotion_update_srv_; //to let amcl update samples without requiring motion
    ros::ServiceServer set_map_srv_;
    ros::Subscriber initial_pose_sub_old_;
    ros::Subscriber map_sub_;

    amcl_hyp_t* initial_pose_hyp_;
    bool first_map_received_;
    bool first_reconfigure_call_;

    boost::recursive_mutex configuration_mutex_;
    dynamic_reconfigure::Server<amcl::AMCLConfig> *dsrv_;
    amcl::AMCLConfig default_config_;
    ros::Timer check_laser_timer_;

    int max_beams_, min_particles_, max_particles_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double alpha_slow_, alpha_fast_;
    double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;

    //beam skip related params
    bool do_beamskip_;
    double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
    double laser_likelihood_max_dist_;
    odom_model_t odom_model_type_;
    double init_pose_[3];
    double init_cov_[3];
    laser_model_t laser_model_type_;
    bool tf_broadcast_;

    void reconfigureCB(amcl::AMCLConfig &config, uint32_t level);

    ros::Time last_laser_received_ts_;
    ros::Duration laser_check_interval_;
    void checkLaserReceived(const ros::TimerEvent& event);
};

////后为wiki官网的参数说明
//（）中为粗读算法参数说明及理解
//×××面临的问题

//常用地图有2种：
//    1.基于特征，仅指明在指定位置（地图中包含的对象的位置）的环境的形状。特征表示使得调节对象的位置变得简单，作为附加的检测结果。这样的地图在地图构建领域很受欢迎。
//    2.基于位置，这样的地图是有体积的，它们为环境中的许多位置都提供标签。不仅包括环境物体的信息，也包括了对象没有物体的信息（如空闲空间），比较经典的占用栅格地图就是基于位置的。

//根据官网信息，amcl采用的是结合自适应（增强蒙特卡洛Augmented_MCL）和库尔贝克-莱不勒散度采样KLD_Sampling_MCL（蒙特卡洛定位的一个变种）。
//    1.KLD_Sampling_MCL随时间改变粒子数，改良了度过初期后的蒙特卡洛大样本集合的资源浪费。两个kld_配置参数就是KLD的参数。对于每次粒子滤波迭代，KLD采样以概率1-δ确定样本数（1-δ就是kld_z配置参数），使得真实的后验与基于采样的近似之间的误差小于ε（ε就是kld_err配置参数）.kld_z=0.99,kld_err=0.05,直方图位大小为15cm*15cm*15°就能取得良好的结果。
//    2.Augmented_MCL解决的是从机器人绑架或全局定位失效中恢复的问题。两个recovery_alpha_配置参数就是用于失效恢复的。随机采样以max{0.0,1.0-w(fast)/w(slow)}概率增加。如果短期似然劣于长期似然则增加随机采样，这种方法，测量似然的一个突然衰减将引起随机采样的数目增加。w=w+α(Wavg-w)--Wavg当前测量模型的权重，w为短期（w(fast)）或长期(w(slow))平滑估计，α为与w对应的recovery_alpha_参数。

//-->
//  <node pkg="amcl" type="amcl" name="amcl" output="screen">
//    <!-- Publish scans from best pose at a max of 10 Hz -->

//    //全部滤波器参数
//    <param name="min_particles" value="500"/>   //允许的粒子数量的最小值，默认100
//    <param name="max_particles" value="5000"/> //允许的例子数量的最大值，默认5000
//    <param name="kld_err" value="0.05"/>    //真实分布和估计分布之间的最大误差，默认0.01
//    <param name="kld_z" value="0.99"/>   //上标准分位数（1-p），其中p是估计分布上误差小于kld_err的概率，默认0.99
//    <param name="update_min_d" value="0.2"/>   //在执行滤波更新前平移运动的距离，默认0.2m(对于里程计模型有影响，模型中根据运动和地图求最终位姿的释然时丢弃了路径中的相关所有信息，已知的只有最终位姿，为了规避不合理的穿过障碍物后的非零似然，这个值建议不大于机器人半径。否则因更新频率的不同可能产生完全不同的结果)
//    <param name="update_min_a" value="0.5"/>   //执行滤波更新前旋转的角度，默认pi/6 rad
//    <param name="resample_interval" value="1"/>   //在重采样前需要的滤波更新的次数,默认2
//    <param name="transform_tolerance" value="0.1"/>  //tf变换发布推迟的时间，为了说明tf变换在未来时间内是可用的
//    <param name="recovery_alpha_slow" value="0.0"/> //慢速的平均权重滤波的指数衰减频率，用作决定什么时候通过增加随机位姿来recover，默认0（disable），可能0.001是一个不错的值
//    <param name="recovery_alpha_fast" value="0.0"/>  //快速的平均权重滤波的指数衰减频率，用作决定什么时候通过增加随机位姿来recover，默认0（disable），可能0.1是个不错的值
//    <param name="gui_publish_rate" value="10.0"/>  //扫描和路径发布到可视化软件的最大频率，设置参数为-1.0意为失能此功能，默认-1.0
//    <param name="save_pose_rate" value="0.5"/>  //存储上一次估计的位姿和协方差到参数服务器的最大速率。被保存的位姿将会用在连续的运动上来初始化滤波器。-1.0失能。
//    <param name="use_map_topic" value="false"/>  //当设置为true时，AMCL将会订阅map话题，而不是调用服务返回地图。也就是说，当设置为true时，有另外一个节点实时的发布map话题，也就是机器人在实时的进行地图构建，并供给amcl话题使用；当设置为false时，通过map server，也就是调用已经构建完成的地图。在navigation 1.4.2中新加入的参数。
//    <param name="first_map_only" value="false"/>  //当设置为true时，AMCL将仅仅使用订阅的第一个地图，而不是每次接收到新的时更新为一个新的地图，在navigation 1.4.2中新加入的参数。

//    //激光模型参数
//    <param name="laser_min_range" value="-1.0"/>  //被考虑的最小扫描范围；参数设置为-1.0时，将会使用激光上报的最小扫描范围
//    <param name="laser_max_range" value="-1.0"/>  //被考虑的最大扫描范围；参数设置为-1.0时，将会使用激光上报的最大扫描范围
//    <param name="laser_max_beams" value="30"/>   //更新滤波器时，每次扫描中多少个等间距的光束被使用（减小计算量，测距扫描中相邻波束往往不是独立的可以减小噪声影响，太小也会造成信息量少定位不准）
//<!--
//这4个laser_z参数，在动态环境下的定位时用于异常值去除技术（还有一种状态增广技术-将隐藏状态包含进状态估计，缺点是计算复杂，acml定位未使用这种定位）
//这种定位思想是环境中的动态物体总是会获得比静态地图障碍物更短的读数（人在障碍物后面是扫描不到的-假如不考虑体积，比如单个激光光束不用考虑体积），利用这样的不对称性去除异常值
//缺点是：在其他可改变环境的其他类型情景（如去除障碍物）时，这样的非对称性可能不存在，但相同概率分析通常是可适用的。因为每一个异常值都被舍弃了，缺少对称性的缺点可能是从全局定位失效中恢复变得不可能。这种情况下，×××强加额外约束（如限制部分可能已被破坏的测量值）是有意义的（×××怎么约束）。（这里说的舍弃与likelihood_field模型的舍弃有区别，这里定位是先计算测量值对应非预期物体的概率（意外对象概率/混合概率）大于用户设定的阀值（amcl配置参数里貌似没有这个？）舍弃，而似然域概率是舍弃的超出最大测量范围的值，不计算概率。）
//（针对这个缺点不想改代码的粗暴又好用省心的处理方式可能是构图的时候将可移动的障碍物搬走，还有更直接的办法就是PS）
//最后，概率由这4个权重乘他们对应的概率然后相加，算法中4个权重相加等于1(这里默认值不等于1，估计做了归一化)。
//这6个laser_参数可以用learn_intrinsic_parameters算法计算，该算法是期望值极大化算法，是估计极大似然参数的迭代过程。（×××好吧，amcl好像并没有做这个工作）
//-->
//    <param name="laser_z_hit" value="0.5"/> //模型的z_hit部分的混合权值，默认0.95(混合权重1.具有局部测量噪声的正确范围--以测量距离近似真实距离为均值，其后laser_sigma_hit为标准偏差的高斯分布的权重)
//    <param name="laser_z_short" value="0.05"/> //模型的z_short部分的混合权值，默认0.1（混合权重2.意外对象权重（类似于一元指数关于y轴对称0～测量距离（非最大距离）的部分：--ηλe^(-λz)，其余部分为0，其中η为归一化参数，λ为laser_lambda_short,z为t时刻的一个独立测量值（一个测距值，测距传感器一次测量通常产生一系列的测量值）），动态的环境，如人或移动物体）
//    <param name="laser_z_max" value="0.05"/> //模型的z_max部分的混合权值，默认0.05（混合权重3.测量失败权重（最大距离时为1，其余为0），如声呐镜面反射，激光黑色吸光对象或强光下的测量，最典型的是超出最大距离）
//    <param name="laser_z_rand" value="0.5"/> //模型的z_rand部分的混合权值，默认0.05（混合权重4.随机测量权重--均匀分布（1平均分布到0～最大测量范围），完全无法解释的测量，如声呐的多次反射，传感器串扰）
//    <param name="laser_sigma_hit" value="0.2"/> //被用在模型的z_hit部分的高斯模型的标准差，默认0.2m
//    <param name="laser_lambda_short" value="0.1"/> //模型z_short部分的指数衰减参数，默认0.1（根据ηλe^(-λz)，λ越大随距离增大意外对象概率衰减越快）
//    <param name="laser_likelihood_max_dist" value="2.0"/> //地图上做障碍物膨胀的最大距离，用作likelihood_field模型（likelihood_field_range_finder_model只描述了最近障碍物的距离，（目前理解应该是在这个距离内的障碍物膨胀处理,但是算法里又没有提到膨胀，不明确是什么意思）.这里算法用到上面的laser_sigma_hit。似然域计算测量概率的算法是将t时刻的各个测量（舍去达到最大测量范围的测量值）的概率相乘，单个测量概率：Zh * prob(dist,σ) +avg，Zh为laser_z_hit,avg为均匀分布概率，dist最近障碍物的距离，prob为0为中心标准方差为σ（laser_sigma_hit）的高斯分布的距离概率）
//    <param name="laser_model_type" value="likelihood_field"/> //模型使用，可以是beam, likehood_field, likehood_field_prob（和likehood_field一样但是融合了beamskip特征--官网的注释），默认是“likehood_field”

//    //里程计模型参数
//    <!--
//    ×××里程计模型并没有涉及机器人漂移或打滑的情况，一旦出现这样的情况，后续定位基本废了，虽然Augmented_MCL有失效恢复，但是实际运行中耗时太长且结果不太理想（位置居然跳，这很不合理，可能参数配置不太好）
//    -->
//    <param name="odom_model_type" value="diff"/> //模型使用，可以是"diff", "omni", "diff-corrected", "omni-corrected",后面两  个是对老版本里程计模型的矫正，相应的里程计参数需要做一定的减小
//    <param name="odom_alpha1" value="0.2"/> //指定由机器人运动部分的旋转分量估计的里程计旋转的期望噪声，默认0.2（旋转存在旋转噪声）
//    <param name="odom_alpha2" value="0.2"/> //制定由机器人运动部分的平移分量估计的里程计旋转的期望噪声，默认0.2（旋转中可能出现平移噪声）
//    <!-- translation std dev, m -->
//    <param name="odom_alpha3" value="0.8"/> //指定由机器人运动部分的平移分量估计的里程计平移的期望噪声，默认0.2（类似上）
//    <param name="odom_alpha4" value="0.2"/> //指定由机器人运动部分的旋转分量估计的里程计平移的期望噪声，默认0.2（类似上）
//    <param name="odom_alpha5" value="0.1"/> //平移相关的噪声参数（仅用于模型是“omni”的情况--wiki官网的注释）
//    <param name="odom_frame_id" value="odom"/>  //里程计默认使用的坐标系
//    <param name="base_frame_id" value="base_link"/>  //用作机器人的基坐标系
//    <param name="global_frame_id" value="map"/>  //由定位系统发布的坐标系名称
//    <param name="tf_broadcast" value="true"/>  //设置为false阻止amcl发布全局坐标系和里程计坐标系之间的tf变换

//    //机器人初始化数据设置
//    <param name="initial_pose_x" value="0.0"/> //初始位姿均值（x），用于初始化高斯分布滤波器。（initial_pose_参数决定撒出去的初始位姿粒子集范围中心）
//    <param name="initial_pose_y" value="0.0"/> //初始位姿均值（y），用于初始化高斯分布滤波器。（同上）
//    <param name="initial_pose_a" value="0.0"/> //初始位姿均值（yaw），用于初始化高斯分布滤波器。（粒子朝向）
//    <param name="initial_cov_xx" value="0.5*0.5"/> //初始位姿协方差（x*x），用于初始化高斯分布滤波器。（initial_cov_参数决定初始粒子集的范围）
//    <param name="initial_cov_yy" value="0.5*0.5"/> //初始位姿协方差（y*y），用于初始化高斯分布滤波器。（同上）
//    <param name="initial_cov_aa" value="(π/12)*(π/12)"/> //初始位姿协方差（yaw*yaw），用于初始化高斯分布滤波器。（粒子朝向的偏差）
//  </node>
//</launch>



#endif
