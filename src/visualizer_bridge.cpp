//
// Created by matteo on 18/05/22.
//

#include "visualizer_bridge.h"
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

namespace artslam::laser3d {
    VisualizerBridge::VisualizerBridge() {
        markers_pub = mt_nh.advertise<visualization_msgs::MarkerArray>("/artslam_laser_3d_wrapper/markers", 16);
        buildings_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/artslam_laser_3d_wrapper/single_cloud", 1);
        visualization_dispatcher_ = std::make_unique<core::utils::Dispatcher>("VisualizerBridgeDispatcher", 1);
    }

    VisualizerBridge::VisualizerBridge(ros::NodeHandle &nh) {
        mt_nh = nh;
        markers_pub = mt_nh.advertise<visualization_msgs::MarkerArray>("/artslam_laser_3d_wrapper/markers", 16);
        buildings_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/artslam_laser_3d_wrapper/single_cloud", 1);
        visualization_dispatcher_ = std::make_unique<core::utils::Dispatcher>("VisualizerBridgeDispatcher", 1);
    }

    // Signals that a new filtered point cloud has been received
    void VisualizerBridge::update_filtered_pointcloud_observer(pcl::PointCloud<Point3I>::ConstPtr pointcloud) {
        visualization_dispatcher_->dispatch([this, pointcloud]{send_to_rviz(pointcloud);});
    }

    void VisualizerBridge::update_slam_output_observer(pcl::PointCloud<Point3I>::Ptr map, std::vector<Eigen::Isometry3d> poses) {
        visualization_dispatcher_->dispatch([this, map, poses]{send_to_rvizz(map,poses);});
    }

    void VisualizerBridge::send_to_rviz(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud) {
        ros::Time stamp;
        stamp.fromNSec(pointcloud->header.stamp);
        EigMatrix4f relpose = EigMatrix4f::Identity();
        relpose(0,3) = 5;
        relpose(2,3) = 3;
        geometry_msgs::TransformStamped map2odom = matrix2transform(stamp, relpose, "map", "base_link");
        // broadcast the transform over tf
        broadcaster_.sendTransform(map2odom);

        sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2());
        pcl::PointCloud<Point3I>::Ptr miao(new pcl::PointCloud<Point3I>);
        *miao = *pointcloud;
        miao->header.stamp /= 1000ull;
        pcl::toROSMsg(*miao, *cloud);
        cloud->header.frame_id = "base_link";
        buildings_pub.publish(cloud);

        relpose(0,3) = -5;
        relpose(2,3) = -10;
        map2odom = matrix2transform(stamp, relpose, "map", "base_link");
        broadcaster_.sendTransform(map2odom);
    }

    void VisualizerBridge::send_to_rvizz(pcl::PointCloud<Point3I>::Ptr map, std::vector<EigIsometry3d> poses) {
        sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2());
        map->header.stamp /= 1000ull;
        pcl::toROSMsg(*map, *cloud);
        cloud->header.frame_id = "map";
        buildings_pub.publish(cloud);

        visualization_msgs::MarkerArray markers;
        markers.markers.resize(1);

        // node markers
        visualization_msgs::Marker& traj_marker = markers.markers[0];
        traj_marker.header.frame_id = "map";
        traj_marker.header.stamp = ros::Time::now();
        traj_marker.ns = "nodes";
        traj_marker.id = 0;
        traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;

        traj_marker.pose.orientation.w = 1.0;
        traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 2.5;
        traj_marker.points.resize(poses.size());
        traj_marker.colors.resize(poses.size());
        for(int i = 0; i < poses.size(); i++) {
            Eigen::Vector3d pos = poses[i].translation();
            traj_marker.points[i].x = pos.x();
            traj_marker.points[i].y = pos.y();
            traj_marker.points[i].z = pos.z();

            double p = static_cast<double>(i) / poses.size();
            traj_marker.colors[i].r = 1.0 - p;
            traj_marker.colors[i].g = p;
            traj_marker.colors[i].b = 0.0;
            traj_marker.colors[i].a = 1.0;
        }
        markers_pub.publish(markers);
    }

    geometry_msgs::TransformStamped VisualizerBridge::matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id) {
        Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
        quat.normalize();
        geometry_msgs::Quaternion odom_quat;
        odom_quat.w = quat.w();
        odom_quat.x = quat.x();
        odom_quat.y = quat.y();
        odom_quat.z = quat.z();

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = stamp;
        odom_trans.header.frame_id = frame_id;
        odom_trans.child_frame_id = child_frame_id;

        odom_trans.transform.translation.x = pose(0, 3);
        odom_trans.transform.translation.y = pose(1, 3);
        odom_trans.transform.translation.z = pose(2, 3);
        odom_trans.transform.rotation = odom_quat;

        return odom_trans;
    }
}
