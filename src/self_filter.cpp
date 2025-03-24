/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Ioan Sucan */

#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <functional>
#include <memory>
#include "robot_self_filter_oedo/self_see_filter.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/voxel_grid.h>

namespace robot_self_filter
{

class SelfFilter : public rclcpp::Node
{
public:

  SelfFilter() : rclcpp::Node("self_filter"), subscribing_(false)
  {
    this->declare_parameter("sensor_frame", "");
    this->declare_parameter("use_rgb", false);
    this->declare_parameter("max_queue_size", 10);
    this->declare_parameter("self_see_links", std::vector<std::string>());
    this->declare_parameter("robot_description", std::string());
    RCLCPP_INFO(this->get_logger(),"robot_description is %s", this->get_parameter("robot_description").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "sensor frame is set to %s", this->get_parameter("sensor_frame").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "self_filter_link_names are these:");
    try {
      // as_string_array()ではなく、get_parameter()を先に使用
      this->get_parameter("self_see_links", self_see_links_);
      
      // パラメータ取得後にログ出力
      std::cout << "Self see links: ";
      if (self_see_links_.empty()) {
        std::cout << "(empty)";
      } else {
        for (const auto& link : self_see_links_) {
          std::cout << link << " ";
        }
      }
      std::cout << std::endl;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error processing self_see_links: %s", e.what());
    }
    this->get_parameter("sensor_frame", sensor_frame_);
    this->get_parameter("use_rgb", use_rgb_);
    this->get_parameter("max_queue_size", max_queue_size_);
    
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface()));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
  
  // 初期化を別のメソッドに分離
  void initialize()
  {    
    RCLCPP_INFO(this->get_logger(), "Self filter is removing shadow points for sensor in frame '%s'.", sensor_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Self filter is using %s", use_rgb_ ? "RGB" : "XYZ");
    if (use_rgb_) 
    {
      self_filter_rgb_ = new filters::SelfFilter<pcl::PointXYZRGB>(this->shared_from_this());
    }
    else 
    {
      self_filter_ = new filters::SelfFilter<pcl::PointXYZ>(this->shared_from_this());
    }
    
    if (use_rgb_) 
    {
      self_filter_rgb_->getSelfMask()->getLinkNames(frames_);
    }
    else 
    { RCLCPP_INFO(this->get_logger(),"start getLinkNames");
      self_filter_->getSelfMask()->getLinkNames(frames_);
    }
    RCLCPP_INFO(this->get_logger(), "Self filter has %d frames", (int)frames_.size());
    auto connect_cb = std::bind(&SelfFilter::connectionCallback, this);
    RCLCPP_INFO(this->get_logger(), "Self filter connectioncallback is up");
    
    // Create a publisher with QoS profile
    rclcpp::QoS qos(1);
    pointCloudPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", qos);
    // Subscribe to topics
    subscribe();
  }
    
  ~SelfFilter()
  {
    if (self_filter_) 
    {
      delete self_filter_;
    }
    if (self_filter_rgb_)
    {
      delete self_filter_rgb_;
    }
  }
    
private:

  void connectionCallback()
  {
    // In ROS2, we don't typically check subscription count directly
    // Just make sure we're subscribed
    RCLCPP_INFO(this->get_logger(), "Self filter has a subscriber");
    if (!subscribing_) {
      subscribe();
      subscribing_ = true;
    }
    RCLCPP_INFO(this->get_logger(), "self filter subscribing %d", subscribing_);
  }
  
  void subscribe() {
    if(frames_.empty())
    {
      RCLCPP_INFO(this->get_logger(), "No valid frames have been passed into the self filter. Using a callback that will just forward scans on.");
      no_filter_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_in", 1, 
        std::bind(&SelfFilter::noFilterCallback, this, std::placeholders::_1));
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Valid frames were passed in. We'll filter them.");
      sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
        this, "/livox/lidar", rclcpp::QoS(max_queue_size_).get_rmw_qos_profile());
      
      mn_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        *sub_, *tf_buffer_, sensor_frame_, max_queue_size_, this->get_node_logging_interface(), 
        this->get_node_clock_interface(), std::chrono::duration<int>(1));
        
      mn_->setTargetFrames(frames_);
      mn_->registerCallback(std::bind(&SelfFilter::cloudCallback, this, std::placeholders::_1));
    }
  }

  void noFilterCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    pointCloudPublisher_->publish(*cloud);
    RCLCPP_INFO(this->get_logger(), "Self filter publishing unfiltered frame");
  }
    
  void cloudCallback(const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& cloud) 
  {
    RCLCPP_INFO(this->get_logger(), "Got pointcloud that is %f seconds old", 
                (this->now() - rclcpp::Time(cloud->header.stamp)).seconds());
    std::vector<int> mask;
    auto start_time = this->now();

    sensor_msgs::msg::PointCloud2 out2;
    int input_size = 0;
    int output_size = 0;
    if (use_rgb_)
    {
      // PCLの変換でエラーが出ているかもしれないので修正
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*cloud, *pcl_cloud);
      pcl::PointCloud<pcl::PointXYZRGB> out;
      self_filter_rgb_->updateWithSensorFrame(*pcl_cloud, out, sensor_frame_);
      pcl::toROSMsg(out, out2);
      out2.header.stamp = cloud->header.stamp;
      input_size = pcl_cloud->points.size();
      output_size = out.points.size();
    }
    else
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*cloud, *pcl_cloud);
      pcl::VoxelGrid<pcl::PointXYZ>::Ptr grid(new pcl::VoxelGrid<pcl::PointXYZ>);
      grid->setLeafSize(0.2, 0.2, 0.2);
      grid->setInputCloud(pcl_cloud);
      grid->filter(*pcl_cloud);
      pcl::PointCloud<pcl::PointXYZ> out;
      self_filter_->updateWithSensorFrame(*pcl_cloud, out, sensor_frame_);
      pcl::toROSMsg(out, out2);
      out2.header.stamp = cloud->header.stamp;
      input_size = pcl_cloud->points.size();
      output_size = out.points.size();
    }
      
    double sec = (this->now() - start_time).seconds();
    pointCloudPublisher_->publish(out2);
    RCLCPP_INFO(this->get_logger(), "Self filter: reduced %d points to %d points in %f seconds", 
                input_size, output_size, sec);
  }
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> mn_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_;

  filters::SelfFilter<pcl::PointXYZ> *self_filter_{nullptr};
  filters::SelfFilter<pcl::PointXYZRGB> *self_filter_rgb_{nullptr};
  std::string sensor_frame_;
  std::vector<std::string> self_see_links_;
  bool use_rgb_;
  bool subscribing_;
  std::vector<std::string> frames_;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPublisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr no_filter_sub_;
  int max_queue_size_;
};

} // namespace robot_self_filter

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_self_filter::SelfFilter>();
  
  // ノードを初期化してから処理を開始
  RCLCPP_DEBUG(node->get_logger(), "Self filter node initialized");
  std::cout << "Self filter node initialized" << std::endl;
  node->initialize();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
