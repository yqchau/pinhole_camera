#include <project.h>


Camera::Camera(ros::NodeHandle nh_private) : 
frame_map_{nh_private.param<std::string>("frame_map", "NONE")},
frame_camera_{nh_private.param<std::string>("frame_camera", "NONE")},
frame_image_{nh_private.param<std::string>("frame_image", "NONE")},
topic_pcl_{nh_private.param<std::string>("topic_pcl", "NONE")},
topic_pcl_debug_{nh_private.param<std::string>("topic_pcl_debug", "NONE")},
topic_image_{nh_private.param<std::string>("topic_image", "NONE")},
topic_marker_viz_{nh_private.param<std::string>("topic_marker_viz", "NONE")},
rate_{nh_private.param<double>("rate", 1.0)},
focal_length_{nh_private.param<double>("focal_length", 1.0)},
pixel_size_{nh_private.param<double>("pixel_size", 1000)},
image_width_{nh_private.param<int>("image_width", 480)},
image_height_{nh_private.param<int>("image_height", 360)}
{
    nh_private_ = nh_private;

    ROS_INFO("Map Frame: %s", frame_map_.c_str());
    ROS_INFO("Camera Frame: %s", frame_camera_.c_str());
    ROS_INFO("Image Frame: %s", frame_image_.c_str());
    ROS_INFO("PCL Topic: %s", topic_pcl_.c_str());
    ROS_INFO("PCL Debug Topic: %s", topic_pcl_debug_.c_str());
    ROS_INFO("Image Topic: %s", topic_image_.c_str());
    ROS_INFO("Marker Viz Topic: %s", topic_marker_viz_.c_str());
    ROS_INFO("Focal Length: %lf", focal_length_);
    ROS_INFO("Pixel Size: %f", pixel_size_);
    ROS_INFO("image width: %d", image_width_);
    ROS_INFO("image height: %d", image_height_);

}

void Camera::callback_pcl(const sensor_msgs::PointCloud2::ConstPtr& msg_pcl_map)
{
    ROS_INFO_THROTTLE(1, "callback called!");

    // using pcl_ros
    // extrinsic matrix transform: map -> camera -> image3D
    pcl_ros::transformPointCloud(
        "camera", 
        transform_map_to_camera_, 
        *msg_pcl_map, 
        *msg_pcl_camera_
    ); 
    pcl_ros::transformPointCloud(
        "image", 
        transform_camera_to_image_, 
        *msg_pcl_camera_, 
        *msg_pcl_image_);
    pcl::fromROSMsg(*msg_pcl_image_, *pcl_frame_image_);

    if (pcl_frame_image_->size() == 0)
    {
        ROS_WARN_THROTTLE(5, "empty input pointcloud detected..");
    }
    pcl::toROSMsg(*pcl_frame_image_, *msg_pcl_image_);
    msg_pcl_image_->header.frame_id = frame_image_;
    publisher_pcl_.publish(*msg_pcl_image_);

    // intrinsic matrix transform: image3D -> image2D 
    cv::Mat image(image_height_, image_width_, CV_8UC1, cv::Scalar(0));
    for (auto& point : *pcl_frame_image_)
    {
        // ROS_INFO_THROTTLE(1, "x: %f, y: %f, z: %f", point.x, point.y, point.z);
        int u, v;

        u = static_cast<int>(focal_length_ / pixel_size_ * point.x / point.z + image_width_ / 2);
        v = static_cast<int>(focal_length_ / pixel_size_ * point.y / point.z + image_height_ / 2);

        if ((0<=u<image_width_) && (0<=v<image_height_))
        {
            ROS_INFO("u: %d, v: %d", u, v);
            image.at<uint8_t>(v,u) = 255;
        }
    }

    auto msg_image = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
    publisher_image_.publish(msg_image);
    publisher_marker_.publish(image_marker_);
    

}

void Camera::initROSInterface()
{
    subscriber_pcl_ = nh_.subscribe(topic_pcl_, 1, &Camera::callback_pcl, this);
    publisher_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_pcl_debug_, 1);
    publisher_image_ = nh_.advertise<sensor_msgs::Image>(topic_image_, 1);
    publisher_marker_ = nh_.advertise<visualization_msgs::Marker>(topic_marker_viz_, 1);
}

void Camera::getFrameTransformations()
{
    if (tf_listener_.waitForTransform(frame_image_, frame_camera_, ros::Time(0), ros::Duration(1.0)))
    {
        try
        {
            tf_listener_.lookupTransform(frame_camera_, frame_map_, ros::Time(0), transform_map_to_camera_);
            std::cout << "Translation: " << transform_map_to_camera_.getOrigin().getX() << ", " << transform_map_to_camera_.getOrigin().getY() << ", " << transform_map_to_camera_.getOrigin().getZ() << std::endl;
            std::cout << "Rotation: " << transform_map_to_camera_.getRotation().getX() << ", " << transform_map_to_camera_.getRotation().getY() << ", " << transform_map_to_camera_.getRotation().getZ() << ", " << transform_map_to_camera_.getRotation().getW() << std::endl;
            

            std::cout << "\n";

            tf_listener_.lookupTransform(frame_image_, frame_camera_, ros::Time(0), transform_camera_to_image_);
            std::cout << "Translation: " << transform_camera_to_image_.getOrigin().getX() << ", " << transform_camera_to_image_.getOrigin().getY() << ", " << transform_camera_to_image_.getOrigin().getZ() << std::endl;
            std::cout << "Rotation: " << transform_camera_to_image_.getRotation().getX() << ", " << transform_camera_to_image_.getRotation().getY() << ", " << transform_camera_to_image_.getRotation().getZ() << ", " << transform_camera_to_image_.getRotation().getW() << std::endl;

        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
    }
}

void Camera::draw_visualization(){
    // visualization_msgs::Marker image_square;

    image_marker_.id = 0;
    image_marker_.header.frame_id = frame_image_;
    image_marker_.type = visualization_msgs::Marker::CUBE;
    image_marker_.scale.x = image_width_ * pixel_size_;
    image_marker_.scale.y = image_height_ * pixel_size_;
    image_marker_.scale.z = 0.01;
    image_marker_.color.a = 0.3;
    image_marker_.color.r = 1.0;
    image_marker_.color.g = 1.0;
    image_marker_.color.b = 1.0;
    image_marker_.pose.position.x = 0.0;
    image_marker_.pose.position.y = 0.0;
    image_marker_.pose.position.z = 0.0;
    image_marker_.pose.orientation.w = 1;
    image_marker_.pose.orientation.x = 0;
    image_marker_.pose.orientation.y = 0;
    image_marker_.pose.orientation.z = 0;

}

void Camera::run()
{
    getFrameTransformations();

    draw_visualization();

    initROSInterface();

    ros::spin();
}

int main(int argc, char** argv){

    ros::init(argc, argv, "camera");

    ros::NodeHandle nh_private("~");

    Camera camera(nh_private);

    camera.run();

    return 0;
}