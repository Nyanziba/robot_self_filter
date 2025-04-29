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
#include "robot_self_filter_oedo/self_see_filter.hpp"
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
    this->declare_parameter("min_sensor_dist", 1.0);
    this->declare_parameter("input_cloud_topic", std::string("/livox/lidar"));
    this->declare_parameter("output_cloud_topic", std::string("/livox/lidar_filtered"));
    this->get_parameter("min_sensor_dist", min_sensor_dist_);
    this->get_parameter("input_cloud_topic", input_cloud_topic_);
    this->get_parameter("output_cloud_topic", output_cloud_topic_);
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
    pointCloudPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, qos);
    
    // マーカーパブリッシャーの初期化
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "robot_self_filter/visualization_markers", 10);
    
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
        this, input_cloud_topic_, rclcpp::QoS(max_queue_size_).get_rmw_qos_profile());
      
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
      // センサーフレームが指定されており、かつ入力点群のframe_idが異なる場合は処理をスキップ
  if (!sensor_frame_.empty() && cloud->header.frame_id != sensor_frame_) {
    // フレームIDが一致しない場合は、そのまま未フィルタリングのデータを転送
    RCLCPP_INFO(this->get_logger(), "Skipping filtering for cloud with frame_id '%s', as it doesn't match sensor_frame '%s'", 
                cloud->header.frame_id.c_str(), sensor_frame_.c_str());
                return;}
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
      
      // マスクを計算 - 点群サイズに合わせて初期化
      mask.resize(pcl_cloud->points.size());
      
      // マスクを計算
      if(sensor_frame_.empty()) {
        self_filter_rgb_->getSelfMask()->maskContainment(*pcl_cloud, mask);
      } else {
        self_filter_rgb_->getSelfMask()->maskIntersection(*pcl_cloud, sensor_frame_, min_sensor_dist_, mask);
      }
      
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
      grid->setLeafSize(0.3, 0.3, 0.3);
      grid->setInputCloud(pcl_cloud);
      grid->filter(*pcl_cloud);
      
      // マスクを計算 - 点群サイズに合わせて初期化
      mask.resize(pcl_cloud->points.size());
      
      // マスクを計算
      if(sensor_frame_.empty()) {
        self_filter_->getSelfMask()->maskContainment(*pcl_cloud, mask);
      } else {
        self_filter_->getSelfMask()->maskIntersection(*pcl_cloud, sensor_frame_, min_sensor_dist_, mask);
      }
      
      pcl::PointCloud<pcl::PointXYZ> out;
      self_filter_->updateWithSensorFrame(*pcl_cloud, out, sensor_frame_);
      pcl::toROSMsg(out, out2);
      out2.header.stamp = cloud->header.stamp;
      input_size = pcl_cloud->points.size();
      output_size = out.points.size();
    }
    
    // マスク計算後にマーカーを発行 - cloudをそのまま渡す
    publishShapeFromMask(cloud, mask);
    
    // ロボット本体のジオメトリを可視化
    if (use_rgb_) {
      publishShapesFromMask(self_filter_rgb_->getSelfMask());
    } else {
      publishShapesFromMask(self_filter_->getSelfMask());
    }
    
    double sec = (this->now() - start_time).seconds();
    pointCloudPublisher_->publish(out2);
    RCLCPP_INFO(this->get_logger(), "Self filter: reduced %d points to %d points in %f seconds", 
                input_size, output_size, sec);
  }
  
  // publishShapeFromMaskの実装
  // 引数の型を修正
  void publishShapeFromMask(const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& cloud, const std::vector<int>& mask) 
  {
    if (mask.empty() || !marker_publisher_) {
      RCLCPP_WARN(this->get_logger(), "No mask or marker publisher available");
      return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    
    // マスクされた点からマーカーを作成
    visualization_msgs::msg::Marker marker;
    marker.header = cloud->header;
    marker.ns = "robot_self_filter";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    marker.lifetime = rclcpp::Duration(0, 0); // 無期限

    // PCL pointcloudに変換
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*cloud, pcl_cloud);

    // マスクの値に応じて点を追加 (INSIDE または SHADOW の点)
    for (size_t i = 0; i < mask.size() && i < pcl_cloud.points.size(); ++i) {
      if (mask[i] == robot_self_filter::INSIDE || mask[i] == robot_self_filter::SHADOW) {
        const auto& pt = pcl_cloud.points[i];
        geometry_msgs::msg::Point point;
        point.x = pt.x;
        point.y = pt.y;
        point.z = pt.z;
        marker.points.push_back(point);
      }
    }

    marker_array.markers.push_back(marker);
    
    // リンクごとの形状を追加
    RCLCPP_INFO(this->get_logger(), "Adding link markers");
    if (self_filter_) {
      RCLCPP_INFO(this->get_logger(), "Adding link markers from self_filter_");
        // getLinks()の代わりにSelfMaskから直接リンク情報を取得
        std::vector<std::string> link_names;
        self_filter_->getSelfMask()->getLinkNames(link_names);
        
        for (size_t i = 0; i < link_names.size(); i++) {
            visualization_msgs::msg::Marker link_marker;
            link_marker.header = cloud->header;
            link_marker.ns = "robot_links";
            link_marker.id = i;
            link_marker.type = visualization_msgs::msg::Marker::SPHERE;
            link_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // 姿勢を設定（リンクごとのTFフレームから取得）
            try {
                auto transform = tf_buffer_->lookupTransform(
                    cloud->header.frame_id, link_names[i], cloud->header.stamp);
                link_marker.pose.position.x = transform.transform.translation.x;
                link_marker.pose.position.y = transform.transform.translation.y;
                link_marker.pose.position.z = transform.transform.translation.z;
                link_marker.pose.orientation = transform.transform.rotation;
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", 
                          link_names[i].c_str(), cloud->header.frame_id.c_str(), ex.what());
                continue;
            }
            
            // リンク形状を表現（簡易的な表現 - デフォルト値を使用）
            link_marker.scale.x = 0.1; // パディングがわからないので適当な値
            link_marker.scale.y = 0.1;
            link_marker.scale.z = 0.1;
            
            // 別の色で各リンクを表示
            link_marker.color.r = 0.0;
            link_marker.color.g = 0.8;
            link_marker.color.b = 0.2;
            link_marker.color.a = 0.5;
            link_marker.lifetime = rclcpp::Duration(0, 0); // 無期限
            RCLCPP_INFO(this->get_logger(), "Adding link marker for %s", link_names[i].c_str());
            marker_array.markers.push_back(link_marker);
        }
    }

    // マーカー配列をパブリッシュ
    marker_publisher_->publish(marker_array);
  }

  // 新しいメソッド: ロボットのジオメトリを可視化
  template <typename PointT>
  void publishShapesFromMask(robot_self_filter::SelfMask<PointT> *mask)
  {
    if (!mask)
      return;
    const auto &bodies = mask->getBodies();
    if (bodies.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "No bodies found in SelfMask");
      return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.reserve(bodies.size());

    std::string shapes_frame = sensor_frame_.empty() ? "map" : sensor_frame_;
    for (size_t i = 0; i < bodies.size(); ++i)
    {
      const auto &see_link = bodies[i];
      const bodies::Body *body = see_link.body;
      if (!body)
        continue;

      visualization_msgs::msg::Marker mk;
      mk.header.frame_id = shapes_frame;
      mk.header.stamp = this->get_clock()->now();
      mk.ns = "self_filter_shapes";
      mk.id = static_cast<int>(i);
      mk.action = visualization_msgs::msg::Marker::ADD;
      mk.lifetime = rclcpp::Duration(0, 0);
      mk.color.a = 0.5f;
      mk.color.r = 1.0f;
      mk.color.g = 0.0f;
      mk.color.b = 0.0f;

      const tf2::Transform &tf = body->getPose();
      mk.pose.position.x = tf.getOrigin().x();
      mk.pose.position.y = tf.getOrigin().y();
      mk.pose.position.z = tf.getOrigin().z();
      tf2::Quaternion q = tf.getRotation();
      mk.pose.orientation.x = q.x();
      mk.pose.orientation.y = q.y();
      mk.pose.orientation.z = q.z();
      mk.pose.orientation.w = q.w();

      switch (body->getType())
      {
      case shapes::SPHERE:
      {
        auto sphere_body = dynamic_cast<const robot_self_filter::bodies::Sphere *>(body);
        if (sphere_body)
        {
          mk.type = visualization_msgs::msg::Marker::SPHERE;
          float d = static_cast<float>(2.0 * sphere_body->getScaledRadius());
          mk.scale.x = d;
          mk.scale.y = d;
          mk.scale.z = d;
        }
        break;
      }
      case shapes::BOX:
      {
        auto box_body = dynamic_cast<const robot_self_filter::bodies::Box *>(body);
        if (box_body)
        {
          mk.type = visualization_msgs::msg::Marker::CUBE;
          float sx = static_cast<float>(2.0 * box_body->getScaledHalfLength());
          float sy = static_cast<float>(2.0 * box_body->getScaledHalfWidth());
          float sz = static_cast<float>(2.0 * box_body->getScaledHalfHeight());
          mk.scale.x = sx;
          mk.scale.y = sy;
          mk.scale.z = sz;
        }
        break;
      }
      case shapes::CYLINDER:
      {
        auto cyl_body = dynamic_cast<const robot_self_filter::bodies::Cylinder *>(body);
        if (cyl_body)
        {
          mk.type = visualization_msgs::msg::Marker::CYLINDER;
          float radius = static_cast<float>(cyl_body->getScaledRadius());
          float length = static_cast<float>(2.0 * cyl_body->getScaledHalfLength());
          mk.scale.x = radius * 2.0f;
          mk.scale.y = radius * 2.0f;
          mk.scale.z = length;
        }
        break;
      }
      case shapes::MESH:
      {
        auto mesh_body = dynamic_cast<const robot_self_filter::bodies::ConvexMesh *>(body);
        if (mesh_body)
        {
          mk.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
          mk.scale.x = mk.scale.y = mk.scale.z = 1.0f;

          const auto &verts = mesh_body->getScaledVertices();
          const auto &tris = mesh_body->getTriangles();
          mk.points.reserve(tris.size());
          for (size_t t_i = 0; t_i < tris.size(); ++t_i)
          {
            geometry_msgs::msg::Point p;
            p.x = verts[tris[t_i]].x();
            p.y = verts[tris[t_i]].y();
            p.z = verts[tris[t_i]].z();
            mk.points.push_back(p);
          }
        }
        break;
      }
      default:
        break;
      }
      marker_array.markers.push_back(mk);
    }

    // マーカー配列をパブリッシュ
    marker_publisher_->publish(marker_array);
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> mn_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_;

  filters::SelfFilter<pcl::PointXYZ> *self_filter_{nullptr};
  filters::SelfFilter<pcl::PointXYZRGB> *self_filter_rgb_{nullptr};
  std::string sensor_frame_;
  std::string input_cloud_topic_;
  std::string output_cloud_topic_;
  std::vector<std::string> self_see_links_;
  bool use_rgb_;
  bool subscribing_;
  std::vector<std::string> frames_;
  double min_sensor_dist_{0.01}; // デフォルト値を持つメンバ変数を追加
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
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
