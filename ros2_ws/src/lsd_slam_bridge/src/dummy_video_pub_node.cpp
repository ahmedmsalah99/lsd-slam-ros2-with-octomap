#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dirent.h>
#include <opencv2/opencv.hpp>

class DummyVideoPubNode : public rclcpp::Node
{
public:
    DummyVideoPubNode()
        : Node("dummy_video_pub_node")
    {
        std::string image_folder_;
        this->declare_parameter<std::string>("image_folder", "./images");
        this->get_parameter("image_folder", image_folder_);
        std::cout << image_folder_ <<std::endl;
        std::cout << "HI" <<std::endl;
        const std::string dataset_root = append_slash_to_dirname(image_folder_);
        const std::string source = dataset_root + "images/";
        std::cout << source << std::endl;
        if(getdir(source) >= 0) {
            printf("found %d image files in folder %s!\n", (int)files.size(),
                    source.c_str());
        } else {
            std::cerr << "Could not read the image directory" << std::endl;
            exit(-1);
        }

        publisher_ = image_transport::create_publisher(this, "camera/image");

        std::cout << "advertiser created" << std::endl;
        // Timer to publish periodically
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&DummyVideoPubNode::publish_image, this)
        );
    }

private:

    std::string append_slash_to_dirname(std::string dirname) {
        if(dirname[dirname.length()-1] == '/') {
            return dirname;
        }
        return dirname + "/";
    }

    cv::Mat load_image()
    {
        if (img_idx >= files.size()) {
            RCLCPP_INFO(this->get_logger(), "All images published.");
            return cv::Mat(); // or loop: img_idx = 0;
        }
        std::cout << "img_idx " << img_idx <<std::endl;
        cv::Mat imageDist = cv::imread(files[img_idx], cv::IMREAD_GRAYSCALE);
        assert(imageDist.type() == CV_8U);
        std::vector<uchar> buffer;
        cv::imencode(".jpg", imageDist, buffer);
        img_idx ++;
        return imageDist;
    }
    void publish_image()
    {
        // Create or load an OpenCV image
        if (img_idx >= files.size()) {
            img_idx = 0;
        }
        cv::Mat image = load_image();
        
        // Convert to ROS image message
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();

        publisher_.publish(msg);
    }

    int getdir (std::string dir)
    {
        files.clear();
        DIR *dp;
        struct dirent *dirp;
        if((dp  = opendir(dir.c_str())) == NULL)
        {
            return -1;
        }

        while ((dirp = readdir(dp)) != NULL) {
            std::string name = std::string(dirp->d_name);

            if(name != "." && name != "..")
                files.push_back(name);
        }
        closedir(dp);


        std::sort(files.begin(), files.end());
        dir = append_slash_to_dirname(dir);
        for(unsigned int i=0; i<files.size(); i++)
        {
            if(files[i].at(0) != '/')
                files[i] = dir + files[i];
        }

        return files.size();
    }
    std::vector<std::string> files;
    image_transport::Publisher publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint32_t img_idx = 0;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyVideoPubNode>());
    rclcpp::shutdown();
    return 0;
}

