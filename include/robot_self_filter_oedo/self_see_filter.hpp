/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FILTERS_SELF_SEE_H_
#define FILTERS_SELF_SEE_H_

#include <filters/filter_base.hpp>
#include <robot_self_filter_oedo/self_mask.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace filters
{

/** \brief A filter to remove parts of the robot seen in a pointcloud
 *
 */

template <typename PointT>
class SelfFilter: public FilterBase<pcl::PointCloud<PointT>>
{
    
public:
  typedef pcl::PointCloud<PointT> PointCloud;
  /** \brief Construct the filter */
  SelfFilter(rclcpp::Node::SharedPtr node) : node_(node)
  {
    node_->get_parameter_or("min_sensor_dist", min_sensor_dist_, 0.01);
    double default_padding, default_scale;
    node_->get_parameter_or("self_see_default_padding", default_padding, 0.01);
    node_->get_parameter_or("self_see_default_scale", default_scale, 1.0);
    node_->get_parameter_or("keep_organized", keep_organized_, false);
    std::vector<robot_self_filter::LinkInfo> links;	
    
    if(!node_->has_parameter("self_see_links")) {
      RCLCPP_WARN(node_->get_logger(), "No links specified for self filtering.");
    } else {
      auto ssl_vals = node_->get_parameter("self_see_links").as_string_array();
      if(ssl_vals.empty()) {
        RCLCPP_WARN(node_->get_logger(), "No values in self see links array");
      } else {
        for(const auto& link_str : ssl_vals) {
          robot_self_filter::LinkInfo li;
          // Parse the link_str to get name, padding, scale
          // This is a simplified example, actual implementation may need more robust parsing
          li.name = link_str; // Basic implementation
          li.padding = default_padding;
          li.scale = default_scale;
          links.push_back(li);
        }
      }
    }
    
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    RCLCPP_INFO(node_->get_logger(),"selfmask method called");
    sm_ = new robot_self_filter::SelfMask<PointT>(tf_buffer_, links, node_);
    if (!sensor_frame_.empty())
      RCLCPP_INFO(node_->get_logger(), "Self filter is removing shadow points for sensor in frame '%s'. Minimum distance to sensor is %f.", 
                 sensor_frame_.c_str(), min_sensor_dist_);
  }
    
  /** \brief Destructor to clean up
   */
  virtual ~SelfFilter(void)
  {
    delete sm_;
  }
  
  virtual bool configure(void)
  {
    // keep only the points that are outside of the robot
    // for testing purposes this may be changed to true
    node_->get_parameter_or("invert", invert_, false);
    
    if (invert_)
      RCLCPP_INFO(node_->get_logger(), "Inverting filter output");
	
    return true;
  }

  bool updateWithSensorFrame(const PointCloud& data_in, PointCloud& data_out, const std::string& sensor_frame)
  {
    sensor_frame_ = sensor_frame;
    return update(data_in, data_out);
  }
    
  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  virtual bool update(const PointCloud& data_in, PointCloud& data_out)
  {
    std::vector<int> keep(data_in.points.size());
    if(sensor_frame_.empty()) {
      sm_->maskContainment(data_in, keep);
    } else {
      sm_->maskIntersection(data_in, sensor_frame_, min_sensor_dist_, keep);
    }	
    fillResult(data_in, keep, data_out);
    return true;
  }

  bool updateWithSensorFrame(const PointCloud& data_in, PointCloud& data_out, PointCloud& data_diff, const std::string& sensor_frame)
  {
    sensor_frame_ = sensor_frame;
    return update(data_in, data_out, data_diff);
  }

  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  virtual bool update(const PointCloud& data_in, PointCloud& data_out, PointCloud& data_diff)
  {
    std::vector<int> keep(data_in.points.size());
    if(sensor_frame_.empty()) {
      sm_->maskContainment(data_in, keep);
    } else {
      sm_->maskIntersection(data_in, sensor_frame_, min_sensor_dist_, keep);
    }
    fillResult(data_in, keep, data_out);
    fillDiff(data_in,keep,data_diff);
    return true;
  }

  void fillDiff(const PointCloud& data_in, const std::vector<int> &keep, PointCloud& data_out)
  {
    const unsigned int np = data_in.points.size();
	
    // fill in output data 
    data_out.header = data_in.header;	  
	
    data_out.points.resize(0);
    data_out.points.reserve(np);
	
    for (unsigned int i = 0 ; i < np ; ++i)
    {
      if ((keep[i] && invert_) || (!keep[i] && !invert_))
      {
        data_out.points.push_back(data_in.points[i]);
      }
    }
  }

  void fillResult(const PointCloud& data_in, const std::vector<int> &keep, PointCloud& data_out)
  {
    const unsigned int np = data_in.points.size();

    // fill in output data with points that are NOT on the robot
    data_out.header = data_in.header;	  
	
    data_out.points.resize(0);
    data_out.points.reserve(np);
    PointT nan_point;
    nan_point.x = std::numeric_limits<float>::quiet_NaN(); 
    nan_point.y = std::numeric_limits<float>::quiet_NaN();
    nan_point.z = std::numeric_limits<float>::quiet_NaN();
    for (unsigned int i = 0 ; i < np ; ++i)
    {
      if (keep[i] == robot_self_filter::OUTSIDE)
      {
        data_out.points.push_back(data_in.points[i]);
      }
      if (keep_organized_ && keep[i] != robot_self_filter::OUTSIDE)
      {
        data_out.points.push_back(nan_point);
      }
    }
    if (keep_organized_) {
      data_out.width = data_in.width;
      data_out.height = data_in.height;
    }
  }

  virtual bool updateWithSensorFrame(const std::vector<PointCloud> & data_in, std::vector<PointCloud>& data_out, const std::string& sensor_frame)
  {
    sensor_frame_ = sensor_frame;
    return update(data_in, data_out);
  }
  
  virtual bool update(const std::vector<PointCloud> & data_in, std::vector<PointCloud>& data_out)
  {
    bool result = true;
    data_out.resize(data_in.size());
    for (unsigned int i = 0 ; i < data_in.size() ; ++i)
      if (!update(data_in[i], data_out[i]))
        result = false;
    return true;
  }

  robot_self_filter::SelfMask<PointT>* getSelfMask() {
    RCLCPP_INFO(node_->get_logger(),"getselfmask method called");
    return sm_;
  }

  void setSensorFrame(const std::string& frame) {
    sensor_frame_ = frame;
  }

  // 正しいコールバック宣言
  void cloudCallback(const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& cloud);
    
protected:
    
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  robot_self_filter::SelfMask<PointT>* sm_;
  
  rclcpp::Node::SharedPtr node_;
  bool invert_;
  std::string sensor_frame_;
  double min_sensor_dist_;
  bool keep_organized_;
};

}

#endif //#ifndef FILTERS_SELF_SEE_H_
