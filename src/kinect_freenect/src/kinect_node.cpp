#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <libfreenect.h>

class KinectNode : public rclcpp::Node {
public:
    KinectNode() : Node("kinect_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("kinect_image", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&KinectNode::publishImage, this));
    }

private:
void publishImage() {
    static freenect_context *f_ctx = nullptr;
    static freenect_device *f_dev = nullptr;

    if (!f_ctx) {
        if (freenect_init(&f_ctx, nullptr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize freenect");
            return;
        }
        freenect_set_log_level(f_ctx, FREENECT_LOG_INFO);
        if (freenect_open_device(f_ctx, &f_dev, 0) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open Kinect device");
            freenect_shutdown(f_ctx);
            return;
        }
    }

    uint8_t *rgb_data = nullptr;
    freenect_frame_mode mode = freenect_get_current_video_mode(f_dev);
    rgb_data = new uint8_t[mode.bytes];
    freenect_set_video_buffer(f_dev, rgb_data);
    freenect_update_tilt_state(f_dev);

    auto msg = sensor_msgs::msg::Image();
    msg.height = mode.height;
    msg.width = mode.width;
    msg.encoding = "rgb8";
    msg.step = mode.width * 3;
    msg.data.assign(rgb_data, rgb_data + mode.bytes);

    publisher_->publish(msg);
    delete[] rgb_data;

    RCLCPP_INFO(this->get_logger(), "Published Kinect image");
}

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinectNode>());
    rclcpp::shutdown();
    return 0;
}