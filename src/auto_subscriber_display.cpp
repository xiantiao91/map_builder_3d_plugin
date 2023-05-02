#include "auto_subscriber_display.h"
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/UInt32.h>
//using namespace rviz;
namespace map_builder_3d_plugin
{

AutoSubscriberDisplay::AutoSubscriberDisplay() : submap_count_(0)
{
    ROS_INFO_STREAM("AutoSubscriberDisplay plugin loaded.");
}

AutoSubscriberDisplay::~AutoSubscriberDisplay()
{
}
void AutoSubscriberDisplay::updateStyle()
{

}
void AutoSubscriberDisplay::onInitialize()
{
    rviz::Display::onInitialize();


    point_cloud_common_ = std::make_unique<CustomPointCloudCommon>(this, context_);
    last_update_time_ = getSystemTime();
    ROS_INFO(__FUNCTION__);
    MessageFilterDisplay::onInitialize();
    update_rate_ = 1.0;

    submap_list_subscriber_ = nh_.subscribe<cartographer_ros_msgs::SubmapList>("submap_list", 1, [this](const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
        // 在此处处理收到的 submap_count 消息
        // 例如：
        mutex_submap_list_.lock();
        submap_list_ = *msg;
        //        submap_count_ = msg->data;
        //        ROS_INFO("Received submap count: %d", submap_list_.submap.size());
        mutex_submap_list_.unlock();
    });
}

double AutoSubscriberDisplay::pointDiffNorm(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
    Eigen::Vector3d diff(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
    return diff.norm();
}
double AutoSubscriberDisplay::computeAngleChange(const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2)
{
    // Normalize the quaternions
    geometry_msgs::Quaternion q1_norm = q1;
    geometry_msgs::Quaternion q2_norm = q2;
    double norm_q1 = std::sqrt(q1_norm.x*q1_norm.x + q1_norm.y*q1_norm.y + q1_norm.z*q1_norm.z + q1_norm.w*q1_norm.w);
    double norm_q2 = std::sqrt(q2_norm.x*q2_norm.x + q2_norm.y*q2_norm.y + q2_norm.z*q2_norm.z + q2_norm.w*q2_norm.w);
    q1_norm.x /= norm_q1;
    q1_norm.y /= norm_q1;
    q1_norm.z /= norm_q1;
    q1_norm.w /= norm_q1;
    q2_norm.x /= norm_q2;
    q2_norm.y /= norm_q2;
    q2_norm.z /= norm_q2;
    q2_norm.w /= norm_q2;

    // Compute the dot product of the two quaternions
    double dot_product = q1_norm.x*q2_norm.x + q1_norm.y*q2_norm.y + q1_norm.z*q2_norm.z + q1_norm.w*q2_norm.w;

    // Convert the dot product to an angle in degrees
    double angle = std::acos(2*dot_product*dot_product - 1) * 180 / M_PI;

    return angle;
}

std::map<int,bool> AutoSubscriberDisplay::getNeedUpdateSubmap(const std::map<int,cartographer_ros_msgs::SubmapEntry>& prev, const cartographer_ros_msgs::SubmapList& cur)
{
    std::map<int,bool> ret;
    for(const cartographer_ros_msgs::SubmapEntry entry : cur.submap)
    {
        if(prev.find(entry.submap_index) == prev.end())
        {
            ret[entry.submap_index] = true;
            continue;
        }

        const cartographer_ros_msgs::SubmapEntry& pre = prev.at(entry.submap_index);
        if(pre.submap_version != entry.submap_version)
        {
            ret[entry.submap_index] = true;
            continue;
        }

        if(pointDiffNorm(pre.pose.position, entry.pose.position) > 0.001 || computeAngleChange(pre.pose.orientation, entry.pose.orientation) > 0.01)//large than 0.001m
        {
            ret[entry.submap_index] = true;
            continue;
        }
        ret[entry.submap_index] = false;
        //        std::find_if()
        //        if(std::find prev.submap)
    }
    if(ret.size() > 1)
    {
        auto max_it = std::max_element(ret.begin(), ret.end(),
                                       [](const auto& p1, const auto& p2) { return p1.first < p2.first; });
        if (max_it != ret.end()) {
            max_it->second = false; //active submap is two,ingore newest
        }
    }
    return ret;
}

void AutoSubscriberDisplay::processPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
//    processMessage(msg);
}
void AutoSubscriberDisplay::processMessage(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    //    sensor_msgs::PointCloud2 display_msg = *msg;

    // Call the method of the point_cloud_common_ object to actually display the point cloud
//    point_cloud_common_->addMessage(msg);
}

void AutoSubscriberDisplay::colorizePoint(pcl::PointXYZRGB& p, double min_z, double max_z)
{
    double height = p.z;
    double ratio = (height - min_z) / (max_z - min_z);
    double r, g, b;
    // Linear mapping of height to RGB color space
    r = std::min(2 * ratio, 1.0);
    g = 0.0;
    b = std::min(2 * (1 - ratio), 1.0);
    // Set the color of the output point
    p.r = r * 255;
    p.g = g * 255;
    p.b = b * 255;
//    double height = p.z;
//    double ratio = (height - min_z) / (max_z - min_z);
//    double r, g, b;
//    // Linear mapping of height to RGB color space
//    r = std::min(2 * (1 - ratio), 1.0);
//    g = std::min(2 * ratio, 1.0);
//    b = 0.0;
//    // Set the color of the output point
//    p.r = r * 255;
//    p.g = g * 255;
//    p.b = b * 255;
}

void AutoSubscriberDisplay::localPointsToWor(const cartographer_ros_msgs::GetSubmapPointsResponse::_points_type& loc_points,
                                             sensor_msgs::PointCloud2& wor_points, const cartographer_ros_msgs::SubmapEntry::_pose_type& gp)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_clouds(new pcl::PointCloud<pcl::PointXYZRGB>());
    Eigen::Vector3d trans(gp.position.x,gp.position.y,gp.position.z);
    Eigen::Quaterniond rotation(gp.orientation.w,gp.orientation.x,gp.orientation.y,gp.orientation.z);
    for(const cartographer_ros_msgs::PointXYZ& p : loc_points)
    {
        Eigen::Vector3d tp(p.x,p.y,p.z);
        Eigen::Vector3d gp = rotation * tp + trans;
        pcl::PointXYZRGB rgbp;
        rgbp.x = gp.data()[0];
        rgbp.y = gp.data()[1];
        rgbp.z = gp.data()[2];
        colorizePoint(rgbp,min_z_color_, max_z_color_);
        //        color_.setColor(rgbp, min_z_color_, max_z_color_);
        //                        if(rgbp.z < max_z)
        input_clouds->push_back(rgbp);
    }
    pcl::toROSMsg(*input_clouds,wor_points);
    wor_points.header.frame_id = "map";
    wor_points.header.stamp = ros::Time::now();
}

cartographer_ros_msgs::GetSubmapPointsResponse::_points_type AutoSubscriberDisplay::getSubmapPoints(int trajectory_id, int submap_idx, double min_score)
{
    ROS_WARN("service call get map: %d", submap_idx);
    cartographer_ros_msgs::GetSubmapPoints::Request req;
    req.min_score = min_score;
    req.trajectory_id = trajectory_id;
    req.submap_index = submap_idx;
    cartographer_ros_msgs::GetSubmapPoints::Response res;
    bool ret = ros::service::call("get_submap_points",req,res);
    //    sensor_msgs::PointCloud2 cloud;//
    return res.points;
}

void AutoSubscriberDisplay::update(float wall_dt, float ros_dt)
{
    //    ROS_INFO_STREAM("AutoSubscriberDisplay::update");
    mutex_submap_list_.lock();
    cartographer_ros_msgs::SubmapList submap_list = submap_list_;
    mutex_submap_list_.unlock();

    double current_time = getSystemTime();
    double elapsed_time = current_time - last_update_time_;
    if (elapsed_time >= 1.0 / update_rate_)
    {

        //        ROS_INFO("AutoSubscriberDisplay::update elapsed_time: %.3f current time: %.3f, last update time: %.3f",
        //                 elapsed_time.toSec(), current_time.toSec(), last_update_time_.toSec());

        std::map<int,bool> need_update_submap = getNeedUpdateSubmap(updated_submap_list_,submap_list);

        if(!need_update_submap.empty())
        {
            auto max_it = std::max_element(need_update_submap.begin(), need_update_submap.end(),
                                           [](const auto& p1, const auto& p2) { return p1.first < p2.first; });
            int max_submap_id;
            if (max_it != need_update_submap.end()) {
                max_submap_id = max_it->first; //active submap is two,ingore newest
            }
            for(auto& data : need_update_submap)
            {
                if(data.second)
                {
                    int id = data.first;
                    const auto it = std::find_if(submap_list.submap.begin(), submap_list.submap.end(),
                                                 [id](const cartographer_ros_msgs::SubmapEntry& e){return e.submap_index == id;});
                    const cartographer_ros_msgs::SubmapEntry& entry = *it;
                    //Determine if submap points exist
                    if(map_points_.find(id) != map_points_.end() && data.first != max_submap_id)
                    {
                        //pose change ,update submap with new pose
                        ROS_INFO("pose change ,update submap with new pose: %d", data.first);
                        point_cloud_common_->mutex_map_points_.lock();
                        localPointsToWor(map_points_[id].local_points,
                                         point_cloud_common_->getMapPointsById(id)/*map_points_[id].global_points*/, entry.pose);
                        point_cloud_common_->update_flag_[id] = true;
                        point_cloud_common_->mutex_map_points_.unlock();
                    }
                    else
                    {
                        //get map points
                        map_points_[id].local_points = getSubmapPoints(0,id,min_points_score_);
                        point_cloud_common_->mutex_map_points_.lock();
                        localPointsToWor(map_points_[id].local_points,
                                         point_cloud_common_->getMapPointsById(id)/*map_points_[id].global_points*/, entry.pose);
                        point_cloud_common_->update_flag_[id] = true;
                        point_cloud_common_->mutex_map_points_.unlock();
//                        const sensor_msgs::PointCloud2& global_points = ;
                        const cartographer_ros_msgs::GetSubmapPoints::Response::_points_type& local_points = map_points_[id].local_points;
                        ROS_INFO("submap id: %d  points size: %d", id, local_points.size());
                    }
                }
            }
            {
              point_cloud_common_->update(wall_dt, ros_dt);
            }

        }
        MessageFilterDisplay::update(wall_dt, ros_dt);
        // 更新上次更新时间
        last_update_time_ = current_time;
        for(const cartographer_ros_msgs::SubmapEntry& entry : submap_list.submap)
        {
            updated_submap_list_[entry.submap_index] = entry;
        }
    }
}

} // namespace map_builder_3d_plugin

PLUGINLIB_EXPORT_CLASS(map_builder_3d_plugin::AutoSubscriberDisplay, rviz::Display)
//PLUGINLIB_EXPORT_CLASS(map_builder_3d_plugin::AutoSubscriberDisplay, rviz::MessageFilterDisplay<sensor_msgs::PointCloud2>)
