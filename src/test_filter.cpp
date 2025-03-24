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

#include <cstdio>
#include <chrono>
#include <thread>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "robot_self_filter_oedo/self_mask.h"

class TestSelfFilter : public rclcpp::Node
{
public:

    TestSelfFilter() : Node("test_self_filter")
    {
        id_ = 1;
        vmPub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10240);
        std::vector<robot_self_filter::LinkInfo> links;
        robot_self_filter::LinkInfo li;
        li.name = "base_link";
        li.padding = .05;
        li.scale = 1.0;
        links.push_back(li);
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        sf_ = new robot_self_filter::SelfMask<pcl::PointXYZ>(tf_buffer_, links, this->shared_from_this());
    }

    ~TestSelfFilter()
    {
        delete sf_;
    }
    
    void gotIntersection(const tf2::Vector3 &pt)
    {
        sendPoint(pt.x(), pt.y(), pt.z());
    }
    
    void sendPoint(double x, double y, double z)
    {
        auto mk = std::make_shared<visualization_msgs::msg::Marker>();

        mk->header.stamp = this->now();
        mk->header.frame_id = "base_link";

        mk->ns = "test_self_filter";
        mk->id = id_++;
        mk->type = visualization_msgs::msg::Marker::SPHERE;
        mk->action = visualization_msgs::msg::Marker::ADD;
        mk->pose.position.x = x;
        mk->pose.position.y = y;
        mk->pose.position.z = z;
        mk->pose.orientation.w = 1.0;

        mk->scale.x = mk->scale.y = mk->scale.z = 0.01;

        mk->color.a = 1.0;
        mk->color.r = 1.0;
        mk->color.g = 0.04;
        mk->color.b = 0.04;
        
        mk->lifetime = rclcpp::Duration(10, 0);
        
        vmPub_->publish(*mk);
    }

    void run()
    {
        pcl::PointCloud<pcl::PointXYZ> in;
        
        uint64_t stamp = this->now().nanoseconds();
        in.header.stamp = stamp;
        in.header.frame_id = "base_link";
        
        const unsigned int N = 500000;    
        in.points.resize(N);
        for (unsigned int i = 0 ; i < N ; ++i)
        {
            in.points[i].x = uniform(1.5);
            in.points[i].y = uniform(1.5);
            in.points[i].z = uniform(1.5);
        }
        
        for (unsigned int i = 0 ; i < 1000 ; ++i)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            rclcpp::spin_some(this->shared_from_this());
        }
        
        auto start_time = this->now();
        std::vector<int> mask;
        sf_->maskIntersection(in, "laser_tilt_mount_link", 0.01, mask, 
                           std::bind(&TestSelfFilter::gotIntersection, this, std::placeholders::_1));
        //    sf_->maskContainment(in, mask);
        auto end_time = this->now();
        double duration = (end_time - start_time).seconds();
        RCLCPP_INFO(this->get_logger(), "%f points per second", (double)N / duration);

        
        int k = 0;
        for (unsigned int i = 0 ; i < mask.size() ; ++i)
        {
            //      bool v = sf_->getMaskContainment(in.points[i].x, in.points[i].y, in.points[i].z);
            //      if (v != mask[i]) 
            //          RCLCPP_ERROR("Mask does not match");       
            if (mask[i] != robot_self_filter::INSIDE) continue;
            //      sendPoint(in.points[i].x, in.points[i].y, in.points[i].z);
            k++;
        }
    }

protected:
    
    double uniform(double magnitude)
    {
        return (2.0 * drand48() - 1.0) * magnitude;
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    robot_self_filter::SelfMask<pcl::PointXYZ> *sf_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vmPub_;
    int id_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestSelfFilter>();
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    node->run();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}



