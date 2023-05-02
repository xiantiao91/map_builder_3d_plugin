#include "custom_point_cloud_common.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <QColor>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreWireBoundingBox.h>

#include <ros/time.h>

#include <pluginlib/class_loader.hpp>

#include "rviz/default_plugin/point_cloud_transformer.h"
#include "rviz/default_plugin/point_cloud_transformers.h"
#include "rviz/display.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/point_cloud.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/uniform_string_stream.h"
#include "rviz/validate_floats.h"
#include "rviz/default_plugin/point_cloud_common.h"

using namespace rviz;
namespace map_builder_3d_plugin
{

CustomPointCloudCommon::CustomPointCloudCommon(rviz::Display* display, rviz::DisplayContext *display_context) :
    rviz::PointCloudCommon(display),display_context_(display_context)
{
    //    void initialize(DisplayContext* context, Ogre::SceneNode* scene_node);
    rviz::PointCloudCommon::initialize(display_context,display_context_->getSceneManager()->getRootSceneNode()->createChildSceneNode());
    custom_scene_node_ = display_context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();

}



cache_point_type& CustomPointCloudCommon::getMapPointsById(int submap_idx)
{
    return map_points_[submap_idx];
}
void CustomPointCloudCommon::setPartialPointCloud(int submap_id, const sensor_msgs::PointCloud2 &cloud)
{
}

// 在此处实现新的方法或重写基类方法

void CustomPointCloudCommon::update(float wall_dt, float ros_dt)
{
    //    ROS_INFO("CustomPointCloudCommon::update");

    PointCloud::RenderMode mode = (PointCloud::RenderMode)style_property_->getOptionInt();

    if (needs_retransform_)
    {
        retransform();
        needs_retransform_ = false;
    }

    {
        float size;
        if (mode == PointCloud::RM_POINTS)
        {
            size = point_pixel_size_property_->getFloat();
        }
        else
        {
            size = point_world_size_property_->getFloat();
        }
        bool per_point_alpha = true;/*findChannelIndex(cloud_info->message_, "rgba") != -1;*/
        mutex_map_points_.lock();
        for (auto& flag : update_flag_)
        {
            int submap_id = flag.first;
            if (flag.second) // 如果需要更新
            {
                //            sensor_msgs::PointCloud2& cloud = map_points_[submap_id];
                // 更新子地图的点云渲染器
                auto it = point_cloud_renderers_.find(submap_id);

                // 如果找不到与子图 ID 关联的 rviz::PointCloud 对象，则创建一个新的 rviz::PointCloud 对象
                if (it == point_cloud_renderers_.end())
                {
                    rviz::PointCloud *new_point_cloud = new rviz::PointCloud();
                    //                std::string point_name = ;
                    new_point_cloud->setName("PointCloud" + std::to_string(submap_id));
                    new_point_cloud->setRenderMode(mode);
                    new_point_cloud->setAlpha(alpha_property_->getFloat(), per_point_alpha);
                    new_point_cloud->setDimensions(size, size, size);
                    new_point_cloud->setAutoSize(auto_size_);

                    // 将新创建的 rviz::PointCloud 对象与子图 ID 关联起来
                    point_cloud_renderers_[submap_id] = new_point_cloud;
                    // 将新创建的 rviz::PointCloud 对象添加到 Ogre 场景管理器中
                    custom_scene_node_->attachObject(new_point_cloud);

                }
                const cache_point_type& cache_points = map_points_[submap_id];
                pcl::PointCloud<pcl::PointXYZRGB> pcl_points;
                pcl::fromROSMsg(cache_points,pcl_points);
                std::vector<rviz::PointCloud::Point> rviz_point_coll;
                for (const pcl::PointXYZRGB& point : pcl_points.points) {
                    rviz::PointCloud::Point rviz_point;
                    rviz_point.position.x = point.x;
                    rviz_point.position.y = point.y;
                    rviz_point.position.z = point.z;
                    rviz_point.color = Ogre::ColourValue(point.r / 255.0, point.g / 255.0, point.b / 255.0, 0.2); // 设置点的颜色（红色，不透明）
                    rviz_point_coll.push_back(rviz_point);
                }
                point_cloud_renderers_[submap_id]->clear();

                point_cloud_renderers_[submap_id]->setRenderMode(mode);
                point_cloud_renderers_[submap_id]->setAlpha(alpha_property_->getFloat(), per_point_alpha);
                point_cloud_renderers_[submap_id]->setDimensions(size, size, size);
                point_cloud_renderers_[submap_id]->setAutoSize(auto_size_);
                point_cloud_renderers_[submap_id]->addPoints(rviz_point_coll.data(),rviz_point_coll.size());
                // 设置更新标志为 false
                flag.second = false;


            }
            if(pre_modle_ != mode || pre_size_ != size)
            {
                point_cloud_renderers_[submap_id]->setRenderMode(mode);
                point_cloud_renderers_[submap_id]->setAlpha(alpha_property_->getFloat(), per_point_alpha);
                point_cloud_renderers_[submap_id]->setDimensions(size, size, size);
                point_cloud_renderers_[submap_id]->setAutoSize(auto_size_);
            }
        }

        mutex_map_points_.unlock();
    }
    updateStatus();



}

}  // namespace map_builder_3d_plugin

