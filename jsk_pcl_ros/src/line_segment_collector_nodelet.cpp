// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
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

#include "jsk_pcl_ros/line_segment_collector.h"
#include "jsk_pcl_ros/pcl_conversion_util.h"
#include "jsk_pcl_ros/pcl_util.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace jsk_pcl_ros
{
  LineSegmentCluster::LineSegmentCluster():
    delta_(Eigen::Vector3f(0, 0, 0)),
    points_(new pcl::PointCloud<pcl::PointXYZ>)
  {

  }
  
  void LineSegmentCluster::addLineSegmentEWMA(
    LineSegment::Ptr segment, const double tau)
  {
    segments_.push_back(segment);
    Eigen::Vector3f new_delta = segment->toSegment()->getDirection();
    if (new_delta.dot(delta_) < 0) {
      new_delta = - new_delta;
    }
    delta_ = (1 - tau) * delta_ + tau * new_delta;
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud = segment->getPoints();
    for (size_t i = 0; i < new_cloud->points.size(); i++) {
      points_->points.push_back(new_cloud->points[i]);
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr LineSegmentCluster::getPoints()
  {
    return points_;
  }
  
  void LineSegmentCollector::onInit()
  {
    DiagnosticNodelet::onInit();
    initialize_joint_angle_ = true;
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&LineSegmentCollector::configCallback, this, _1, _2);
    srv_->setCallback (f);
    
    if (!pnh_->getParam("fixed_frame_id", fixed_frame_id_)) {
      NODELET_ERROR("no ~fixed_frame_id is specified");
      return;
    }

    if (!pnh_->getParam("rotate_joint_name", rotate_joint_name_)) {
      NODELET_ERROR("no ~rotate_joint_name is specified");
      return;
    }

    std::string rotate_type_str;
    pnh_->param("rotate_type", rotate_type_str, std::string("tilt_two_way"));
    if (rotate_type_str == "tilt") {
      rotate_type_ = ROTATION_TILT;
    }
    else if (rotate_type_str == "tilt_two_way") {
      rotate_type_ = ROTATION_TILT_TWO_WAY;
    }
    else if (rotate_type_str == "spindle") {
      rotate_type_ = ROTATION_SPINDLE;
    }
    else {
      NODELET_ERROR("unknown ~rotate_type: %s", rotate_type_str.c_str());
      return;
    }
    
    pub_point_cloud_
      = advertise<sensor_msgs::PointCloud2>(*pnh_, "output/cloud", 1);
    pub_inliers_ = advertise<ClusterPointIndices>(*pnh_, "output/inliers", 1);
    pub_coefficients_
      = advertise<ModelCoefficientsArray>(*pnh_, "output/coefficients", 1);
    pub_polygons_
      = advertise<PolygonArray>(*pnh_, "output/polygons", 1);
    debug_pub_inliers_before_plane_
      = advertise<ClusterPointIndices>(
        *pnh_, "debug/connect_segments/inliers", 1);
  }

  void LineSegmentCollector::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    ewma_tau_ = config.ewma_tau;
    segment_connect_normal_threshold_ = config.segment_connect_normal_threshold;
    outlier_threshold_ = config.outlier_threshold;
    max_iterations_ = config.max_iterations;
    min_indices_ = config.min_indices;
  }

  void LineSegmentCollector::subscribe()
  {
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "input_indices", 1);
    sub_coefficients_.subscribe(*pnh_, "input_coefficients", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_input_, sub_indices_, sub_coefficients_);
    sync_->registerCallback(boost::bind(&LineSegmentCollector::collect,
                                        this, _1, _2, _3));
    sub_joint_state_ = pnh_->subscribe(
      "input_joint_states", 1,
      &LineSegmentCollector::jointStateCallback, this);
  }

  void LineSegmentCollector::unsubscribe()
  {
    sub_input_.unsubscribe();
    sub_indices_.unsubscribe();
    sub_coefficients_.unsubscribe();
    sub_joint_state_.shutdown();
  }
  
  void LineSegmentCollector::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {

  }

  void LineSegmentCollector::jointStateCallback(
    const sensor_msgs::JointState::ConstPtr& joint_state_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    // lookup the joint
    int target_index = -1;
    for (size_t i = 0; i < joint_state_msg->name.size(); i++) {
      if (joint_state_msg->name[i] == rotate_joint_name_) {
        target_index = i;
        break;
      }
    }
    if (target_index == -1) {
      NODELET_ERROR("failed to find the joint '%s'", rotate_joint_name_.c_str());
      return;
    }
    double current_joint_angle = joint_state_msg->position[target_index];
    if (initialize_joint_angle_) {
      initial_joint_angle_ = current_joint_angle;
      latest_joint_angle_ = current_joint_angle;
      initialize_joint_angle_ = false;
      prev_joint_velocity_ = 0.0;
      return;
    }
    if (rotate_type_ == ROTATION_TILT_TWO_WAY) {
      double velocity = latest_joint_angle_ - current_joint_angle;
      if (prev_joint_velocity_ <= 0 && velocity > 0) {
        // bottom
        cleanupBuffers(joint_state_msg->header.stamp);
      }
      prev_joint_velocity_ = velocity;
      latest_joint_angle_ = current_joint_angle;
    }
  }

  void LineSegmentCollector::cleanupBuffers(
      const ros::Time& stamp)
  {
    pointclouds_buffer_.removeBefore(stamp);
    indices_buffer_.removeBefore(stamp);
    coefficients_buffer_.removeBefore(stamp);
    segments_buffer_.removeBefore(stamp);
    segment_clusters_.resize(0);
    //segment_map_ = IntegerGraphMap();
  }

  void LineSegmentCollector::publishBeforePlaneSegmentation(
    const std_msgs::Header& header,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const std::vector<pcl::PointIndices::Ptr>& connected_indices)
  {
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    ros_cloud.header = header;
    pub_point_cloud_.publish(ros_cloud);
    ClusterPointIndices ros_indices;
    ros_indices.header = header;
    ros_indices.cluster_indices
      = pcl_conversions::convertToROSPointIndices(connected_indices, header);
    debug_pub_inliers_before_plane_.publish(ros_indices);
  }

  LineSegmentCluster::Ptr LineSegmentCollector::lookupNearestSegment(
    LineSegment::Ptr segment)
  {
    int max_index = -1;
    double max_dot = - DBL_MAX;
    for (size_t i = 0; i < segment_clusters_.size(); i++) {
      LineSegmentCluster::Ptr cluster = segment_clusters_[i];
      Eigen::Vector3f delta_cluster = cluster->getDelta();
      Eigen::Vector3f delta = segment->toSegment()->getDirection();
      if (delta_cluster.dot(delta) < 0) {
        delta = - delta;
      }
      if (delta_cluster.dot(delta) > segment_connect_normal_threshold_) {
        if (max_dot < delta_cluster.dot(delta)) {
          max_dot = delta_cluster.dot(delta);
          max_index = i;
        }
      }
    }
    if (max_index == -1) {
      return LineSegmentCluster::Ptr();
    }
    else {
      //ROS_INFO("max angle: %f", acos(max_dot) * 180.0 / M_PI);
      return segment_clusters_[max_index];
    }
  }
  
  void LineSegmentCollector::collectFromBuffers(
    const std_msgs::Header& header,
    std::vector<LineSegment::Ptr> new_segments)
  {
    for (size_t i = 0; i < new_segments.size(); i++) {
      LineSegment::Ptr segment = new_segments[i];
      LineSegmentCluster::Ptr cluster = lookupNearestSegment(segment);
      if (cluster) {
        cluster->addLineSegmentEWMA(segment, ewma_tau_);
      }
      else {
        cluster.reset(new LineSegmentCluster());
        cluster->addLineSegmentEWMA(segment, 1.0);
        segment_clusters_.push_back(cluster);
      }
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr
      connected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointIndices::Ptr> connected_indices;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds_list;
    for (size_t i = 0; i < segment_clusters_.size(); i++) {
      LineSegmentCluster::Ptr cluster = segment_clusters_[i];
      pcl::PointIndices::Ptr current_indices (new pcl::PointIndices);
      pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud
        = cluster->getPoints();
      for (size_t j = 0; j < current_cloud->points.size(); j++) {
        current_indices->indices.push_back(connected_cloud->points.size() + j);
      }
      connected_indices.push_back(current_indices);
      clouds_list.push_back(current_cloud);
      *connected_cloud = *connected_cloud + *current_cloud;
    }
    // publish debug information
    publishBeforePlaneSegmentation(
      header,
      connected_cloud,
      connected_indices);
    estimatePlanes(header, connected_cloud, connected_indices);
  }

  void LineSegmentCollector::estimateMultiPlanesFromPointCluoud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::PointIndices::Ptr initial_indices,
    std::vector<pcl::ModelCoefficients::Ptr>& coefficients_list,
    std::vector<pcl::PointIndices::Ptr>& indices_list)
  {
    pcl::PointIndices::Ptr rest_indices (new pcl::PointIndices);
    *rest_indices = *initial_indices;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (outlier_threshold_);
    seg.setMaxIterations (max_iterations_);
    seg.setInputCloud(cloud);
    while (true) {
      if (rest_indices->indices.size() > min_indices_) {
        pcl::PointIndices::Ptr
          result_indices (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr
          result_coefficients (new pcl::ModelCoefficients);
        seg.setIndices(rest_indices);
        seg.segment(*result_indices, *result_coefficients);
        if (result_indices->indices.size() > min_indices_) {
          coefficients_list.push_back(result_coefficients);
          indices_list.push_back(result_indices);
          rest_indices = subIndices(*rest_indices, *result_indices);
        }
        else {
          break;
        }
      }
      else {
        break;
      }
    }
  }

  void LineSegmentCollector::publishResult(
    const std_msgs::Header& header,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    std::vector<pcl::ModelCoefficients::Ptr> all_coefficients,
    std::vector<pcl::PointIndices::Ptr> all_indices)
  {
    ClusterPointIndices ros_indices;
    ros_indices.header = header;
    ros_indices.cluster_indices
      = pcl_conversions::convertToROSPointIndices(all_indices,
                                                  header);
    pub_inliers_.publish(ros_indices);
    ModelCoefficientsArray ros_coefficients;
    ros_coefficients.header = header;
    ros_coefficients.coefficients
      = pcl_conversions::convertToROSModelCoefficients(
        all_coefficients,
        header);
    pub_coefficients_.publish(ros_coefficients);
    PolygonArray ros_polygon;
    ros_polygon.header = header;
    for (size_t i = 0; i < all_indices.size(); i++) {
      ConvexPolygon::Ptr convex
        = convexFromCoefficientsAndInliers<pcl::PointXYZ>(
          cloud, all_indices[i], all_coefficients[i]);
      geometry_msgs::PolygonStamped polygon_stamped;
      polygon_stamped.header = header;
      polygon_stamped.polygon = convex->toROSMsg();
      ros_polygon.polygons.push_back(polygon_stamped);
    }
    pub_polygons_.publish(ros_polygon);
  }
  
  void LineSegmentCollector::estimatePlanes(
    const std_msgs::Header& header,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    std::vector<pcl::PointIndices::Ptr> initial_indices)
  {
    std::vector<pcl::ModelCoefficients::Ptr> all_coefficients;
    std::vector<pcl::PointIndices::Ptr> all_indices;
    for (size_t i = 0; i < initial_indices.size(); i++) {
      std::vector<pcl::ModelCoefficients::Ptr> coefficients_list;
      std::vector<pcl::PointIndices::Ptr> indices_list;
      estimateMultiPlanesFromPointCluoud(
        cloud, initial_indices[i], coefficients_list, indices_list);
      for (size_t i = 0; i < indices_list.size(); i++) {
        all_indices.push_back(indices_list[i]);
        all_coefficients.push_back(coefficients_list[i]);
      }
    }
    // publish the result...
    publishResult(header, cloud, all_coefficients, all_indices);
  }

  void LineSegmentCollector::collect(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
      const ClusterPointIndices::ConstPtr& indices_msg,
      const ModelCoefficientsArray::ConstPtr& coefficients_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    //NODELET_INFO("buffer length: %lu", pointclouds_buffer_.size());
    pointclouds_buffer_.push_back(cloud_msg);
    indices_buffer_.push_back(indices_msg);
    coefficients_buffer_.push_back(coefficients_msg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr
      input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *input_cloud);
    // buildup segments
    std::vector<pcl::PointIndices::Ptr> input_indices
      = pcl_conversions::convertToPCLPointIndices(indices_msg->cluster_indices);
    std::vector<pcl::ModelCoefficients::Ptr> input_coefficients
      = pcl_conversions::convertToPCLModelCoefficients(
        coefficients_msg->coefficients);
    std::vector<LineSegment::Ptr> new_segments;
    for (size_t i = 0; i < indices_msg->cluster_indices.size(); i++) {
      LineSegment::Ptr segment (new LineSegment(cloud_msg->header,
                                                input_indices[i],
                                                input_coefficients[i],
                                                input_cloud));
      segments_buffer_.push_back(segment);
      new_segments.push_back(segment);
    }
    collectFromBuffers(cloud_msg->header, new_segments);
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::LineSegmentCollector, nodelet::Nodelet);
