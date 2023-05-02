#pragma once

#include <rviz/default_plugin/point_cloud_common.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <rviz/display_context.h>
namespace map_builder_3d_plugin
{
typedef sensor_msgs::PointCloud2 cache_point_type;
class CustomPointCloudCommon : public rviz::PointCloudCommon
{
public:
    CustomPointCloudCommon(rviz::Display* display,rviz::DisplayContext* display_context);
    void addSubmap(int submap_id, const sensor_msgs::PointCloud2ConstPtr& msg) {
        submap_cloud_map_[submap_id] = *msg;
    }
    void setPartialPointCloud(int submap_id, const sensor_msgs::PointCloud2& cloud);
    cache_point_type& getMapPointsById(int submap_idx);
    //  void update(const std::string& topic, const sensor_msgs::PointCloud2ConstPtr& cloud_msg) override;
    std::map<int, sensor_msgs::PointCloud2> submap_cloud_map_;
    // 在此处添加新的方法或重写基类方法
//    void update();
    void update(float wall_dt, float ros_dt);
    rviz::PointCloud::RenderMode pre_modle_ = rviz::PointCloud::RenderMode::RM_POINTS;
    float pre_size_ = 0.01;
//    std::map<int, Ogre::ManualObject*> map_renderers_;
    std::map<int,rviz::PointCloud*> point_cloud_renderers_;
    boost::mutex mutex_map_points_;
    std::map<int,cache_point_type>  map_points_;
    std::map<int,bool> update_flag_;
    rviz::DisplayContext* display_context_;
protected:
//    virtual void updateStyle() override;
private:
    Ogre::SceneNode* custom_scene_node_;
};

}  // namespace rviz_plugin_tutorials

