#ifndef ARTSLAM_LASER_3D_WRAPPER_VISUALIZER_BRIDGE_H
#define ARTSLAM_LASER_3D_WRAPPER_VISUALIZER_BRIDGE_H

#include <observers/pointcloud_observers.h>
#include <artslam_utils/dispatcher.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <observers/output_observer.h>

namespace artslam::laser3d {
    class VisualizerBridge : public FilteredPointcloudObserver, public SlamOutputObserver {
    public:
        VisualizerBridge();
        explicit VisualizerBridge(ros::NodeHandle& nh);
        void set_handle(ros::NodeHandle& nh) {mt_nh = nh;};
        // Signals that a new filtered point cloud has been received
        void update_filtered_pointcloud_observer(pcl::PointCloud<Point3I>::ConstPtr pointcloud) override;

        void update_slam_output_observer(pcl::PointCloud<Point3I>::Ptr map, std::vector<Eigen::Isometry3d> poses) override;

    private:
        void send_to_rviz(const pcl::PointCloud<Point3I>::ConstPtr& pointcloud);

        void send_to_rvizz(pcl::PointCloud<Point3I>::Ptr map, std::vector<EigIsometry3d> poses);
        geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id);

        std::unique_ptr<artslam::core::utils::Dispatcher> visualization_dispatcher_;                // core operations handler

        ros::NodeHandle mt_nh;
        ros::Publisher markers_pub;
        ros::Publisher buildings_pub; // publish cloud with buildings being currently aligned
        tf::TransformBroadcaster broadcaster_;
    };
}


#endif //ARTSLAM_LASER_3D_WRAPPER_VISUALIZER_BRIDGE_H
