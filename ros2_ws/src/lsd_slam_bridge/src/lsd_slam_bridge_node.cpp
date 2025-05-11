#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <rmw/qos_profiles.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <future>
#include <chrono>
#include <fcntl.h>
#include <functional>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Transform.h>

#include <mutex>
#include <queue>
#include <thread>
#include <condition_variable>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>


#define MB 1024*1024
struct CloudPoint3D{
    float x;
    float y;
    float z;
};

struct CloudPoints3D
{
    uint32_t id;
    int32_t cloud_points_num;
    float time;
    bool isKeyframe;
    CloudPoint3D cloud_points[300000];
    float scale;
    float camToWorld[7];
};

bool recv_exact(int sock, uint8_t* buffer, size_t size) {
        size_t total_received = 0;
        while (total_received < size) {
            std::cout << "trying to receive" << std::endl;
            ssize_t bytes = recv(sock, buffer + total_received, size - total_received, 0);
            
            if (bytes <= 0) {
                return false;  // Connection closed or error
            }
            total_received += bytes;
        }
        std::cout << "received " << total_received << std::endl;
        return true;
    }
class LSDSlamBridgeNode : public rclcpp::Node
{
public:
    LSDSlamBridgeNode()
        : Node("lsd_slam_bridge_node")
    {
        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // rclcpp::QoS qos(rmw_qos_profile_sensor_data);
        // cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_in", qos);
        // rclcpp::QoS qos_profile = rclcpp::QoS(rmw_qos_profile_sensor_data);
        cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar",10);

        in_sock_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (in_sock_fd_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create socket.");
            rclcpp::shutdown();
            return;
        }
        

        in_server_addr_.sin_family = AF_INET;
        in_server_addr_.sin_port = htons(9002);  
        in_server_addr_.sin_addr.s_addr = inet_addr("127.0.0.1");

        if (connect(in_sock_fd_, (struct sockaddr*)&in_server_addr_, sizeof(in_server_addr_)) < 0) {
            std::cerr << "Connection failed input socket\n";
            return;
        }
        int flags = fcntl(in_sock_fd_, F_GETFL, 0);
        if (flags == -1) {
            perror("fcntl get failed");
        }

        if (fcntl(in_sock_fd_, F_SETFL, flags | O_NONBLOCK) == -1) {
            perror("fcntl set failed");
        }
        sleep(3);
        // Socket setup
        out_sock_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (out_sock_fd_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create socket.");
            rclcpp::shutdown();
            return;
        }

        out_server_addr_.sin_family = AF_INET;
        out_server_addr_.sin_port = htons(9000);  
        out_server_addr_.sin_addr.s_addr = inet_addr("127.0.0.1");

        if (connect(out_sock_fd_, (struct sockaddr*)&out_server_addr_, sizeof(out_server_addr_)) < 0) {
            std::cerr << "Connection failed output socket\n";
            return;
        }

        sleep(3);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), // 1 Hz
            std::bind(&LSDSlamBridgeNode::receive_cloud_points3d, this)
        );
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30), 
            std::bind(&LSDSlamBridgeNode::publish_tf, this)
        );
        sender_thread_ = std::thread(&LSDSlamBridgeNode::send_image,this);
        
    }
    
    void update_tf(float camToWorld[7],rclcpp::Time time_stamp)
    {
        static const tf2::Quaternion q_cv_to_ros = []{
        tf2::Quaternion q;
            // roll = -90°, pitch = 0, yaw = -90°
            q.setRPY(-M_PI/2, 0.0, -M_PI/2);
            return q;
        }();
        static const tf2::Transform tf_cv_to_ros = []{
            tf2::Transform t; t.setIdentity();
            t.setRotation(q_cv_to_ros);
            return t;
        }();

        tf2::Transform tf_cam_cv;
        tf_cam_cv.setOrigin({camToWorld[4], camToWorld[5], camToWorld[6]});
        tf2::Quaternion q_cv{camToWorld[0], camToWorld[1], camToWorld[2], camToWorld[3]};
        q_cv.normalize();
        tf_cam_cv.setRotation(q_cv);

        tf2::Transform tf_world_cv = tf_cam_cv.inverse();
        tf2::Transform tf_out = tf_cv_to_ros * tf_world_cv;

        t.header.stamp = time_stamp;
        t.header.frame_id = "map";
        t.child_frame_id = "camera";

        t.transform.translation.x = tf_out.getOrigin().x();
        t.transform.translation.y = tf_out.getOrigin().y();
        t.transform.translation.z = tf_out.getOrigin().z();

        tf2::Quaternion q_inv = tf_out.getRotation();
        t.transform.rotation.x = q_inv.x();
        t.transform.rotation.y = q_inv.y();
        t.transform.rotation.z = q_inv.z();
        t.transform.rotation.w = q_inv.w();

        
    }
    void publish_tf()
    {
        if(t.header.stamp.sec == 0 && t.header.stamp.nanosec == 0)
            return;
        tf_broadcaster_->sendTransform(t);
    }
    
    void receive_cloud_points3d()
    {
        CloudPoints3D cloud_points;
        bool success = read_cloud_points3d(cloud_points);
        if(!success)
            return;
        std::cout << "received cloud points" << std::endl;
        rclcpp::Time time_stamp = this->get_clock()->now();
        update_tf(cloud_points.camToWorld,time_stamp);
        publish_pointcloud(cloud_points,time_stamp);

    }
    bool read_cloud_points3d(CloudPoints3D& cloud_points)
    {
        
        uint8_t size_buf[4];
        bool success;
        // success = execute_with_timeout(300,recv_exact,in_sock_fd_, size_buf, 4);
        success = recv_exact(in_sock_fd_, size_buf, 4);
        if (!success) {
            // std::cout << "Connection closed or error receiving size.\n";
            return false;
            // break;
        }

        uint32_t buffer_size = (size_buf[0] << 24) | (size_buf[1] << 16) |
                                (size_buf[2] << 8) | size_buf[3];

        std::cout << "Expecting data of " << buffer_size << " bytes\n";
        std::vector<uint8_t> cloud_points_data(buffer_size);
        // success = execute_with_timeout(300,recv_exact,cloud_points_data.data(), buffer_size);
        success = recv_exact(in_sock_fd_,cloud_points_data.data(), buffer_size);
        if (!success) {
            std::cout << "Connection closed during cloud points reception.\n";
            return false;
            // break;
        }
        // Ensure we received enough data to fill the struct
        if (buffer_size != sizeof(CloudPoints3D)) {
            std::cerr << "Mismatch: expected " << sizeof(CloudPoints3D)
                    << " bytes but got " << buffer_size << "\n";
            return false;
        }

        // Deserialize raw bytes into struct
        std::memcpy(&cloud_points, cloud_points_data.data(), sizeof(CloudPoints3D));
        return true;
    }
    

    void setup_subscription()
    {
        subscription_ = image_transport::create_subscription(
            this,  
            "camera/image",
            std::bind(&LSDSlamBridgeNode::image_callback, this, std::placeholders::_1),
            "raw"
        );
    }
    ~LSDSlamBridgeNode()
    {
        running_ = false;
        cv_.notify_all();
        if (sender_thread_.joinable())
            sender_thread_.join();

        if (out_sock_fd_ >= 0)
            close(out_sock_fd_);
        if (in_sock_fd_ >= 0)
            close(in_sock_fd_);
    }


private:

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try {
            cv::Mat image = cv_bridge::toCvShare(msg, msg->encoding)->image.clone();
            std::vector<uchar> buffer;
            cv::imencode(".jpg", image, buffer);
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                image_queue_.push(std::move(buffer));
            }
            cv_.notify_one();
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }



    void send_image()
    {
        while (running_) {
            std::vector<uchar> image_data;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                cv_.wait(lock, [this]() { return !image_queue_.empty() || !running_; });
                if (!running_) break;
                image_data = std::move(image_queue_.front());
                image_queue_.pop();
            }

            uint32_t size = image_data.size();
            uint32_t size_net = htonl(size);
            if (send(out_sock_fd_, &size_net, sizeof(size_net), 0) < 0) {
                std::cerr << "Failed to send image size\n";
                continue;
            }

            if (send(out_sock_fd_, image_data.data(), size, 0) < 0) {
                std::cerr << "Failed to send image data\n";
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "Sent image of size: %u bytes", size);
        }
    }

    void publish_pointcloud(const CloudPoints3D& cloud_data,rclcpp::Time time_stamp)
    {
        sensor_msgs::msg::PointCloud2 msg;
        float scale = cloud_data.scale;
        msg.header.stamp = time_stamp;
        msg.header.frame_id = "camera"; 

        msg.height = 1;
        msg.width = cloud_data.cloud_points_num;
        msg.is_dense = false;
        msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(cloud_data.cloud_points_num);

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

        for (int i = 0; i < cloud_data.cloud_points_num; ++i, ++iter_x, ++iter_y, ++iter_z) {
            float x = cloud_data.cloud_points[i].x * scale;
            float y = cloud_data.cloud_points[i].y * scale;
            float z = cloud_data.cloud_points[i].z * scale;

            *iter_x = x;
            *iter_y = y;
            *iter_z = z;
        }

        cloud_publisher_->publish(msg);
    }

    

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    image_transport::Subscriber subscription_;
    int out_sock_fd_;
    struct sockaddr_in out_server_addr_;

    int in_sock_fd_;
    struct sockaddr_in in_server_addr_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped t;

    std::queue<std::vector<uchar>> image_queue_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    std::thread sender_thread_;
    std::atomic<bool> running_{true};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;

};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LSDSlamBridgeNode>();
    node->setup_subscription();  // safe call after full construction
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
