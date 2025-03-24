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

#ifndef ROBOT_SELF_FILTER_SELF_MASK_
#define ROBOT_SELF_FILTER_SELF_MASK_

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <robot_self_filter_oedo/bodies.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <functional>
#include <filesystem>
#include <string>
#include <vector>
#include <memory>

#include <urdf/model.h>
#include <resource_retriever/retriever.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robot_self_filter
{

    /** \brief The possible values of a mask computed for a point */
    enum
    {
	INSIDE = 0,
	OUTSIDE = 1,
	SHADOW = 2,
    };

struct LinkInfo
{
  std::string name;
  double padding;
  double scale;
};
    
    static inline tf2::Transform urdfPose2tf2Transform(const urdf::Pose &pose)
    {
      return tf2::Transform(tf2::Quaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w),
			   tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
    }

    static shapes::Shape* constructShape(const urdf::Geometry *geom)
    {
      if (!geom) {
        RCLCPP_ERROR(rclcpp::get_logger("robot_self_filter"), "Null geometry pointer");
        return nullptr;
      }
	
      shapes::Shape *result = NULL;
      switch (geom->type)
      {
      case urdf::Geometry::SPHERE:
	    result = new shapes::Sphere(dynamic_cast<const urdf::Sphere*>(geom)->radius);
	    break;	
      case urdf::Geometry::BOX:
	    {
		urdf::Vector3 dim = dynamic_cast<const urdf::Box*>(geom)->dim;
		result = new shapes::Box(dim.x, dim.y, dim.z);
	    }
	    break;
      case urdf::Geometry::CYLINDER:
	    result = new shapes::Cylinder(dynamic_cast<const urdf::Cylinder*>(geom)->radius,
					  dynamic_cast<const urdf::Cylinder*>(geom)->length);
	    break;
      case urdf::Geometry::MESH:
	    {
		const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*>(geom);
		if (!mesh->filename.empty())
		{
		    resource_retriever::Retriever retriever;
		    resource_retriever::MemoryResource res;
		    bool ok = true;
		    
		    try
		    {
			res = retriever.get(mesh->filename);
		    }
		    catch (resource_retriever::Exception& e)
		    {
			RCLCPP_ERROR(rclcpp::get_logger("robot_self_filter"), "%s", e.what());
			ok = false;
		    }
		    
		    if (ok)
		    {
			if (res.size == 0)
			    RCLCPP_WARN(rclcpp::get_logger("robot_self_filter"), "Retrieved empty mesh for resource '%s'", mesh->filename.c_str());
			else
			{
			    std::filesystem::path model_path(mesh->filename);
			    std::string ext = model_path.extension().string();
			    if (ext == ".dae" || ext == ".DAE") {
			      result = shapes::createMeshFromBinaryDAE(mesh->filename.c_str());
			    }
			    else {
			      result = shapes::createMeshFromBinaryStlData(reinterpret_cast<char*>(res.data.get()), res.size);
			    }
			    if (result == NULL)
				RCLCPP_ERROR(rclcpp::get_logger("robot_self_filter"), "Failed to load mesh '%s'", mesh->filename.c_str());
			}
		    }
		}
		else
		    RCLCPP_WARN(rclcpp::get_logger("robot_self_filter"), "Empty mesh filename");
	    }
	    
	    break;
      default:
	    RCLCPP_ERROR(rclcpp::get_logger("robot_self_filter"), "Unknown geometry type: %d", (int)geom->type);
	    break;
      }
	
      return result;
    }

    /** \brief Computing a mask for a pointcloud that states which points are inside the robot
     *
     */
    template <typename PointT>
    class SelfMask
    {	
    protected:
	
	struct SeeLink
	{
	    SeeLink(void)
	    {
		body = unscaledBody = NULL;
	    }
	    
	    std::string   name;
	    bodies::Body *body;
	    bodies::Body *unscaledBody;
	    tf2::Transform   constTransf;
	    double        volume;
	};
	
	struct SortBodies
	{
	    bool operator()(const SeeLink &b1, const SeeLink &b2)
	    {
		return b1.volume > b2.volume;
	    }
	};
	
    public:
	typedef pcl::PointCloud<PointT> PointCloud;

	/** \brief Construct the filter */
	SelfMask(std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::vector<LinkInfo> &links, rclcpp::Node::SharedPtr node)
	  : tf_buffer_(tf_buffer), node_(node)
	{
	    configure(links);
	}
	
	/** \brief Destructor to clean up
	 */
	~SelfMask(void)
	{
	    freeMemory();
	}
	
	/** \brief Compute the containment mask (INSIDE or OUTSIDE) for a given pointcloud. If a mask element is INSIDE, the point
	    is inside the robot. The point is outside if the mask element is OUTSIDE.
	 */
	void maskContainment(const PointCloud& data_in, std::vector<int> &mask)
        {
          mask.resize(data_in.points.size());
          if (bodies_.empty())
            std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
          else
          {
            std_msgs::msg::Header header = pcl_conversions::fromPCL(data_in.header);
            assumeFrame(header);
            maskAuxContainment(data_in, mask);
          }
        }

	/** \brief Compute the intersection mask for a given
	    pointcloud. If a mask element can have one of the values
	    INSIDE, OUTSIDE or SHADOW. If the value is SHADOW, the
	    point is on a ray behind the robot and should not have
	    been seen. If the mask element is INSIDE, the point is
	    inside the robot. The sensor frame is specified to obtain
	    the origin of the sensor. A callback can be registered for
	    the first intersection point on each body.
	 */
	void maskIntersection(const PointCloud& data_in, const std::string &sensor_frame, const double min_sensor_dist,
			      std::vector<int> &mask, const std::function<void(const tf2::Vector3&)> &intersectionCallback = nullptr)
        {
          mask.resize(data_in.points.size());
          if (bodies_.empty()) {
            std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
          }
          else
          {
            std_msgs::msg::Header header = pcl_conversions::fromPCL(data_in.header);
            assumeFrame(header, sensor_frame, min_sensor_dist);
            if (sensor_frame.empty())
              maskAuxContainment(data_in, mask);
            else
              maskAuxIntersection(data_in, mask, intersectionCallback);
          }
        }
        
        
	/** \brief Compute the intersection mask for a given pointcloud. If a mask
	    element can have one of the values INSIDE, OUTSIDE or SHADOW. If the value is SHADOW,
	    the point is on a ray behind the robot and should not have
	    been seen. If the mask element is INSIDE, the point is inside
	    the robot. The origin of the sensor is specified as well.
	 */
	void maskIntersection(const PointCloud& data_in, const tf2::Vector3 &sensor_pos, const double min_sensor_dist,
			      std::vector<int> &mask, const std::function<void(const tf2::Vector3&)> &intersectionCallback = nullptr)
        {
          mask.resize(data_in.points.size());
          if (bodies_.empty())
            std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
          else
          {
            std_msgs::msg::Header header = pcl_conversions::fromPCL(data_in.header);
            assumeFrame(header, sensor_pos, min_sensor_dist);
            maskAuxIntersection(data_in, mask, intersectionCallback);
          }
        }
	
	/** \brief Assume subsequent calls to getMaskX() will be in the frame passed to this function.
	 *   The frame in which the sensor is located is optional */
	void assumeFrame(const std_msgs::msg::Header& header)
        {
          const unsigned int bs = bodies_.size();
    
          // place the links in the assumed frame 
          for (unsigned int i = 0 ; i < bs ; ++i)
          {
            std::string error_string;
            if(!tf_buffer_->canTransform(header.frame_id, bodies_[i].name, tf2::timeFromSec(rclcpp::Time(header.stamp).seconds()), 
                                       tf2::durationFromSec(0.1), &error_string)) {
              RCLCPP_ERROR(node_->get_logger(), "Wait for transform failed from %s to %s after 100ms. Error string: %s", 
                          bodies_[i].name.c_str(), header.frame_id.c_str(), error_string.c_str());
            }
            
            // find the transform between the link's frame and the pointcloud frame
            geometry_msgs::msg::TransformStamped transform_stamped;
            try
            {
              transform_stamped = tf_buffer_->lookupTransform(header.frame_id, bodies_[i].name, 
                                                           tf2::timeFromSec(rclcpp::Time(header.stamp).seconds()));
              tf2::Transform transf;
              tf2::fromMsg(transform_stamped.transform, transf);
              
              // set it for each body; we also include the offset specified in URDF
              bodies_[i].body->setPose(transf * bodies_[i].constTransf);
              bodies_[i].unscaledBody->setPose(transf * bodies_[i].constTransf);
            }
            catch(tf2::TransformException& ex)
            {
              RCLCPP_ERROR(node_->get_logger(), "Unable to lookup transform from %s to %s. Exception: %s", 
                          bodies_[i].name.c_str(), header.frame_id.c_str(), ex.what());
            }
          }
          
          computeBoundingSpheres();
        }
	
	
        /** \brief Assume subsequent calls to getMaskX() will be in the frame passed to this function.
	 *  Also specify which possition to assume for the sensor (frame is not needed) */
	void assumeFrame(const std_msgs::msg::Header& header, const tf2::Vector3 &sensor_pos, const double min_sensor_dist)
        {
          assumeFrame(header);
          sensor_pos_ = sensor_pos;
          min_sensor_dist_ = min_sensor_dist;
        }

	/** \brief Assume subsequent calls to getMaskX() will be in the frame passed to this function.
	 *   The frame in which the sensor is located is optional */
	void assumeFrame(const std_msgs::msg::Header& header, const std::string &sensor_frame, const double min_sensor_dist)
        {
          assumeFrame(header);

          std::string error_string;
          if(!tf_buffer_->canTransform(header.frame_id, sensor_frame, tf2::timeFromSec(rclcpp::Time(header.stamp).seconds()), 
                                     tf2::durationFromSec(0.1), &error_string)) {
            RCLCPP_ERROR(node_->get_logger(), "Wait for transform failed from %s to %s after 100ms. Error string: %s", 
                        sensor_frame.c_str(), header.frame_id.c_str(), error_string.c_str());
            sensor_pos_.setValue(0, 0, 0);
          } 

          // transform should be there
          // compute the origin of the sensor in the frame of the cloud
          try
          {
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(header.frame_id, sensor_frame, 
                                                         tf2::timeFromSec(rclcpp::Time(header.stamp).seconds()));
            tf2::Transform transf;
            tf2::fromMsg(transform_stamped.transform, transf);
            sensor_pos_ = transf.getOrigin();
          }
          catch(tf2::TransformException& ex)
          {
            sensor_pos_.setValue(0, 0, 0);
            RCLCPP_ERROR(node_->get_logger(), "Unable to lookup transform from %s to %s. Exception: %s", 
                        sensor_frame.c_str(), header.frame_id.c_str(), ex.what());
          }
  
          min_sensor_dist_ = min_sensor_dist;
        }
	
        /** \brief Get the containment mask (INSIDE or OUTSIDE) value for an individual point. No
	    setup is performed, assumeFrame() should be called before use */
	int  getMaskContainment(const tf2::Vector3 &pt) const
        {
          const unsigned int bs = bodies_.size();
          int out = OUTSIDE;
          for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
            if (bodies_[j].body->containsPoint(pt))
              out = INSIDE;
          return out;
        }
	
	/** \brief Get the containment mask (INSIDE or OUTSIDE) value for an individual point. No
	    setup is performed, assumeFrame() should be called before use */
	int  getMaskContainment(double x, double y, double z) const
        {
          return getMaskContainment(tf2::Vector3(x, y, z));
        }
	
	/** \brief Get the intersection mask (INSIDE, OUTSIDE or
	    SHADOW) value for an individual point. No setup is
	    performed, assumeFrame() should be called before use */
	int  getMaskIntersection(double x, double y, double z, const std::function<void(const tf2::Vector3&)> &intersectionCallback = nullptr) const
        {
          return getMaskIntersection(tf2::Vector3(x, y, z), intersectionCallback);
        }
	
	/** \brief Get the intersection mask (INSIDE, OUTSIDE or
	    SHADOW) value for an individual point. No setup is
	    performed, assumeFrame() should be called before use */
	int  getMaskIntersection(const tf2::Vector3 &pt, const std::function<void(const tf2::Vector3&)> &intersectionCallback = nullptr) const
        {
          const unsigned int bs = bodies_.size();

          // we first check is the point is in the unscaled body. 
          // if it is, the point is definitely inside
          int out = OUTSIDE;
          for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
            if (bodies_[j].unscaledBody->containsPoint(pt))
              out = INSIDE;
    
          if (out == OUTSIDE)
          {
            // we check if the point is a shadow point 
            tf2::Vector3 dir(sensor_pos_ - pt);
            tf2Scalar  lng = dir.length();
            if (lng < min_sensor_dist_)
              out = INSIDE;
            else
            {
              dir /= lng;
	    
              std::vector<tf2::Vector3> intersections;
              for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
              {
                // get the 1st intersection of ray pt->sensor
                intersections.clear(); // intersectsRay doesn't clear the vector...
                if (bodies_[j].body->intersectsRay(pt, dir, &intersections, 1))
                {
                  // is the intersection between point and sensor?
                  if (dir.dot(sensor_pos_ - intersections[0]) >= 0.0)
                  {
                    if (intersectionCallback)
                      intersectionCallback(intersections[0]);
                    out = SHADOW;
                  }
                }
              }
	    
              // if it is not a shadow point, we check if it is inside the scaled body
              for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
                if (bodies_[j].body->containsPoint(pt))
                  out = INSIDE;
            }
          }
          return out;
        }
	
	/** \brief Get the set of link names that have been instantiated for self filtering */
	void getLinkNames(std::vector<std::string> &frames) const
        {
          for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
            frames.push_back(bodies_[i].name);
        }
	
    protected:

	/** \brief Free memory. */
	void freeMemory(void)
        {
          for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
          {
            if (bodies_[i].body)
              delete bodies_[i].body;
            if (bodies_[i].unscaledBody)
              delete bodies_[i].unscaledBody;
          }
          
          bodies_.clear();
        }


	/** \brief Configure the filter. */
	bool configure(const std::vector<LinkInfo> &links)
        {
          // in case configure was called before, we free the memory
          freeMemory();
          sensor_pos_.setValue(0, 0, 0);
    
          std::string content;
          auto urdfModel = std::make_shared<urdf::Model>();

          if (node_->get_parameter("robot_description", content))
          {
            if (!urdfModel->initString(content))
            {
              RCLCPP_ERROR(node_->get_logger(), "Unable to parse URDF description!");
              return false;
            }
          }
          else
          {
            RCLCPP_ERROR(node_->get_logger(), "Robot model not found! Did you remap 'robot_description'?");
            return false;
          }
          
          std::stringstream missing;
    
          // from the geometric model, find the shape of each link of interest
          // and create a body from it, one that knows about poses and can 
          // check for point inclusion
          for (unsigned int i = 0 ; i < links.size() ; ++i)
          {
            const urdf::Link *link = urdfModel->getLink(links[i].name).get();
            if (!link)
            {
              missing << " " << links[i].name;
              continue;
            }
            
            if (!(link->collision && link->collision->geometry))
            {
              RCLCPP_WARN(node_->get_logger(), "No collision geometry specified for link '%s'", links[i].name.c_str());
              continue;
            }
	
            shapes::Shape *shape = constructShape(link->collision->geometry.get());
	
            if (!shape)
            {
              RCLCPP_ERROR(node_->get_logger(), "Unable to construct collision shape for link '%s'", links[i].name.c_str());
              continue;
            }
	
            SeeLink sl;
            sl.body = bodies::createBodyFromShape(shape);

            if (sl.body)
            {
              sl.name = links[i].name;
              
              // collision models may have an offset, in addition to what tf gives
              // so we keep it around
              sl.constTransf = urdfPose2tf2Transform(link->collision->origin);
              
              sl.body->setScale(links[i].scale);
              sl.body->setPadding(links[i].padding);
              RCLCPP_INFO(node_->get_logger(), "Self see link name %s padding %f", links[i].name.c_str(), links[i].padding);
              sl.volume = sl.body->computeVolume();
              sl.unscaledBody = bodies::createBodyFromShape(shape);
              bodies_.push_back(sl);
            }
            else
              RCLCPP_WARN(node_->get_logger(), "Unable to create point inclusion body for link '%s'", links[i].name.c_str());
	
            delete shape;
          }
    
          if (missing.str().size() > 0)
            RCLCPP_WARN(node_->get_logger(), "Some links were included for self mask but they do not exist in the model:%s", missing.str().c_str());
    
          if (bodies_.empty())
            RCLCPP_ERROR(node_->get_logger(), "No robot links will be checked for self mask");
    
          // put larger volume bodies first -- higher chances of containing a point
          RCLCPP_INFO(node_->get_logger(), "Sorting bodies by volume");
          std::sort(bodies_.begin(), bodies_.end(), SortBodies());
          RCLCPP_INFO(node_->get_logger(), "Done sorting bodies by volume");
    
          bspheres_.resize(bodies_.size());
          bspheresRadius2_.resize(bodies_.size());

          for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
            
            RCLCPP_INFO(node_->get_logger(), "Self mask includes link %s with volume %f", bodies_[i].name.c_str(), bodies_[i].volume);
    
          return true; 
        }
	
	/** \brief Compute bounding spheres for the checked robot links. */
	void computeBoundingSpheres(void)
        {
          const unsigned int bs = bodies_.size();
          for (unsigned int i = 0 ; i < bs ; ++i)
          {
            bodies_[i].body->computeBoundingSphere(bspheres_[i]);
            bspheresRadius2_[i] = bspheres_[i].radius * bspheres_[i].radius;
          }
        }

	
	/** \brief Perform the actual mask computation. */
	void maskAuxContainment(const PointCloud& data_in, std::vector<int> &mask)
        {
          const unsigned int bs = bodies_.size();
          const unsigned int np = data_in.points.size();
    
          // compute a sphere that bounds the entire robot
          bodies::BoundingSphere bound;
          bodies::mergeBoundingSpheres(bspheres_, bound);	  
          tf2Scalar radiusSquared = bound.radius * bound.radius;
    
          // we now decide which points we keep
          //#pragma omp parallel for schedule(dynamic) 
          for (int i = 0 ; i < (int)np ; ++i)
          {
            tf2::Vector3 pt = tf2::Vector3(data_in.points[i].x, data_in.points[i].y, data_in.points[i].z);
            int out = OUTSIDE;
            if (bound.center.distance2(pt) < radiusSquared)
              for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
                if (bodies_[j].body->containsPoint(pt))
                  out = INSIDE;
	
            mask[i] = out;
          }
        }

	/** \brief Perform the actual mask computation. */
	void maskAuxIntersection(const PointCloud& data_in, std::vector<int> &mask, const std::function<void(const tf2::Vector3&)> &callback)
        {
          const unsigned int bs = bodies_.size();
          const unsigned int np = data_in.points.size();
    
          // compute a sphere that bounds the entire robot
          bodies::BoundingSphere bound;
          bodies::mergeBoundingSpheres(bspheres_, bound);	  
          tf2Scalar radiusSquared = bound.radius * bound.radius;

          // we now decide which points we keep
          //#pragma omp parallel for schedule(dynamic) 
          for (int i = 0 ; i < (int)np ; ++i)
          {
            bool print = false;
            //if(i%100 == 0) print = true;
            tf2::Vector3 pt = tf2::Vector3(data_in.points[i].x, data_in.points[i].y, data_in.points[i].z);
            int out = OUTSIDE;

            // we first check is the point is in the unscaled body. 
            // if it is, the point is definitely inside
            if (bound.center.distance2(pt) < radiusSquared)
              for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
                if (bodies_[j].unscaledBody->containsPoint(pt)) {
                  if(print)
                    std::cout << "Point " << i << " in unscaled body part " << bodies_[j].name << std::endl;
                  out = INSIDE;
                }

            // if the point is not inside the unscaled body,
            if (out == OUTSIDE)
            {
              // we check if the point is a shadow point 
              tf2::Vector3 dir(sensor_pos_ - pt);
              tf2Scalar lng = dir.length();
              if (lng < min_sensor_dist_) {
                out = INSIDE;
                //std::cout << "Point " << i << " less than min sensor distance away\n";
              }
              else
              {		
                dir /= lng;

                std::vector<tf2::Vector3> intersections;
                for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j) {
                  // get the 1st intersection of ray pt->sensor
                  intersections.clear(); // intersectsRay doesn't clear the vector...
                  if (bodies_[j].body->intersectsRay(pt, dir, &intersections, 1))
                  {
                    // is the intersection between point and sensor?
                    if (dir.dot(sensor_pos_ - intersections[0]) >= 0.0)
                    {
                      if (callback)
                        callback(intersections[0]);
                      out = SHADOW;
                      if(print) std::cout << "Point " << i << " shadowed by body part " << bodies_[j].name << std::endl;
                    }
                  }
                }
                // if it is not a shadow point, we check if it is inside the scaled body
                if (out == OUTSIDE && bound.center.distance2(pt) < radiusSquared)
                  for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
                    if (bodies_[j].body->containsPoint(pt)) {
                      if(print) std::cout << "Point " << i << " in scaled body part " << bodies_[j].name << std::endl;
                      out = INSIDE;
                    }
              }
            }
            mask[i] = out;
          }
        }
	
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        rclcpp::Node::SharedPtr node_;
	
	tf2::Vector3 sensor_pos_;
	double min_sensor_dist_;
	
	std::vector<SeeLink> bodies_;
	std::vector<double> bspheresRadius2_;
	std::vector<bodies::BoundingSphere> bspheres_;
    };
}

#endif
