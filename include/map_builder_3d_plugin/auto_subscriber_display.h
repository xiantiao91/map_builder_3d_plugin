#pragma once
#include <rviz/display.h>
#include <rviz/message_filter_display.h>
#include <sensor_msgs/PointCloud2.h>
#include <rviz/default_plugin/point_cloud_common.h>
#include <custom_point_cloud_common.h>
#include <cartographer_ros_msgs/SubmapList.h>
#include <eigen3/Eigen/Eigen>
#include <chrono>
#include <pcl/point_types.h>
#include <rviz/properties/enum_property.h>
#include <cartographer_ros_msgs/GetSubmapPoints.h>
#include "custom_point_cloud_common.h"
#include <rviz/tool.h>

namespace map_builder_3d_plugin
{


struct submapPointsInfo
{
    cartographer_ros_msgs::GetSubmapPoints::Response::_points_type local_points;
    sensor_msgs::PointCloud2 global_points;
};

class AutoSubscriberDisplay : public rviz::/*Display*/MessageFilterDisplay<sensor_msgs::PointCloud2>
{

    Q_OBJECT
public:
    AutoSubscriberDisplay();
    virtual ~AutoSubscriberDisplay();
    void updateAutoSubscriberTopic();
public Q_SLOTS:
  void updateStyle();
protected:
    virtual void onInitialize();
    virtual void update(float wall_dt, float ros_dt);
    virtual void processMessage(const sensor_msgs::PointCloud2ConstPtr& msg);
    double update_rate_;//update frenquency
    double last_update_time_;
private:
    ros::NodeHandle nh_;
    int submap_count_;
    void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg);private:
    double pointDiffNorm(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    double computeAngleChange(const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2);
    void colorizePoint(pcl::PointXYZRGB& p, double min_z, double max_z);
    double getSystemTime()
    {
        // 获取当前时间点
        auto now = std::chrono::high_resolution_clock::now();

        // 计算自Epoch以来的纳秒数
        auto nanoseconds_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

        // 将纳秒转换为秒
        double seconds_since_epoch = nanoseconds_since_epoch / 1e9;
        return seconds_since_epoch;
    }

    std::map<int,bool> getNeedUpdateSubmap(const std::map<int,cartographer_ros_msgs::SubmapEntry>& prev, const cartographer_ros_msgs::SubmapList& cur);
    cartographer_ros_msgs::GetSubmapPoints::Response::_points_type getSubmapPoints(int trajectory_id, int submap_idx, double min_score);
    void localPointsToWor(const cartographer_ros_msgs::GetSubmapPointsResponse::_points_type& loc_points,
                          sensor_msgs::PointCloud2& wor_points, const cartographer_ros_msgs::SubmapEntry::_pose_type& gp);
    std::map<int,submapPointsInfo> map_points_;
//    rviz::PointCloudCommon* point_cloud_common_;
    ros::Subscriber submap_list_subscriber_;
    cartographer_ros_msgs::SubmapList submap_list_;
//    cartographer_ros_msgs::SubmapList updated_submap_list_;
    std::map<int,cartographer_ros_msgs::SubmapEntry> updated_submap_list_;
    boost::mutex mutex_submap_list_;
    std::unique_ptr<CustomPointCloudCommon> point_cloud_common_;
    double min_points_score_ = 0.75;
    double min_z_color_ = -5.0;
    double max_z_color_ = 10.0;

private:
  ros::NodeHandle update_nh_;
  ros::CallbackQueue update_queue_;
  boost::recursive_mutex update_mutex_;
//  rviz::EnumProperty* style_property_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_;
//  AutoSubscriberKeyboardTool auto_subscriber_key_board_tool_;
};

}
