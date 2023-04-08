#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>	
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

class Camera
{
public:

    Camera(ros::NodeHandle nh_private);

    void run();

private:

    void initROSInterface();

    void draw_visualization();

    void getFrameTransformations();

    void callback_pcl(const sensor_msgs::PointCloud2::ConstPtr& msg_pcl);

    ros::NodeHandle         nh_;
    ros::NodeHandle         nh_private_;
    ros::Subscriber         subscriber_pcl_;
    ros::Publisher          publisher_pcl_;
    ros::Publisher          publisher_image_;
    ros::Publisher          publisher_marker_;
    ros::Publisher          publisher_marker_array_;
    ros::Rate               rate_;

    tf::TransformListener   tf_listener_;
    tf::StampedTransform    transform_camera_to_image_;
    tf::StampedTransform    transform_map_to_camera_;

    const std::string       frame_map_;
    const std::string       frame_camera_;
    const std::string       frame_image_;
    const std::string       topic_pcl_;
    const std::string       topic_pcl_debug_;
    const std::string       topic_image_;
    const std::string       topic_marker_viz_;
    const std::string       topic_marker_array_viz_;


    // camera intrinsic params
    const double            focal_length_;
    const double            pixel_size_;
    const int               image_width_;
    const int               image_height_;

    sensor_msgs::PointCloud2::Ptr  msg_pcl_camera_{boost::make_shared<sensor_msgs::PointCloud2>()};
    sensor_msgs::PointCloud2::Ptr  msg_pcl_image_{boost::make_shared<sensor_msgs::PointCloud2>()};
    visualization_msgs::Marker image_marker_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr    pcl_frame_map_{boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr    pcl_frame_camera_{boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr    pcl_frame_image_{boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>()};

};