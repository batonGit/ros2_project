#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <libfreenect.h>
#include <memory>
#include <functional>
#include <vector>
#include <mutex>

class KinectNode : public rclcpp::Node {
public:
    KinectNode() : Node("kinect_node") {
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("kinect_image", 10);
        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("kinect_pointcloud", 10);
        
        // Инициализация устройства при запуске
        initializeKinect();
        
        if (f_dev_) {
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100), std::bind(&KinectNode::publishData, this));
            RCLCPP_INFO(this->get_logger(), "Kinect initialized successfully");
        }
    }
    
    ~KinectNode() {
        // Очистка ресурсов при завершении
        if (f_dev_) {
            freenect_stop_video(f_dev_);
            freenect_stop_depth(f_dev_);
            freenect_close_device(f_dev_);
        }
        
        if (f_ctx_) {
            freenect_shutdown(f_ctx_);
        }
        
        RCLCPP_INFO(this->get_logger(), "Kinect node shutting down");
    }

private:
    // Буферы для хранения данных
    std::vector<uint8_t> rgb_buffer_;
    std::vector<uint16_t> depth_buffer_;
    std::mutex rgb_mutex_;
    std::mutex depth_mutex_;
    
    void initializeKinect() {
        if (freenect_init(&f_ctx_, nullptr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize freenect");
            return;
        }
        
        freenect_set_log_level(f_ctx_, FREENECT_LOG_INFO);
        
        int num_devices = freenect_num_devices(f_ctx_);
        if (num_devices <= 0) {
            RCLCPP_ERROR(this->get_logger(), "No Kinect devices found");
            freenect_shutdown(f_ctx_);
            f_ctx_ = nullptr;
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Found %d Kinect devices", num_devices);
        
        if (freenect_open_device(f_ctx_, &f_dev_, 0) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open Kinect device");
            freenect_shutdown(f_ctx_);
            f_ctx_ = nullptr;
            return;
        }
        
        // Настройка параметров для RGB и глубины
        freenect_frame_mode rgb_mode = freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB);
        freenect_frame_mode depth_mode = freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT);
        
        // Выделяем память для буферов данных
        rgb_buffer_.resize(rgb_mode.bytes);
        depth_buffer_.resize(depth_mode.width * depth_mode.height);
        
        // Настройка колбэков для получения данных
        freenect_set_video_callback(f_dev_, [](freenect_device* dev, void* rgb, uint32_t timestamp) {
            // Этот колбэк вызывается при получении нового кадра RGB
            KinectNode* node = static_cast<KinectNode*>(freenect_get_user(dev));
            if (node && rgb) {
                std::lock_guard<std::mutex> lock(node->rgb_mutex_);
                freenect_frame_mode mode = freenect_get_current_video_mode(dev);
                memcpy(node->rgb_buffer_.data(), rgb, mode.bytes);
            }
        });
        
        freenect_set_depth_callback(f_dev_, [](freenect_device* dev, void* depth, uint32_t timestamp) {
            // Этот колбэк вызывается при получении нового кадра глубины
            KinectNode* node = static_cast<KinectNode*>(freenect_get_user(dev));
            if (node && depth) {
                std::lock_guard<std::mutex> lock(node->depth_mutex_);
                freenect_frame_mode mode = freenect_get_current_depth_mode(dev);
                memcpy(node->depth_buffer_.data(), depth, mode.bytes);
            }
        });
        
        // Устанавливаем указатель на текущий объект для колбэков
        freenect_set_user(f_dev_, this);
        
        // Выделяем буферы для данных
        freenect_set_video_buffer(f_dev_, rgb_buffer_.data());
        freenect_set_depth_buffer(f_dev_, depth_buffer_.data());
        
        // Запуск стримов
        freenect_start_video(f_dev_);
        freenect_start_depth(f_dev_);
    }
    
    void publishData() {
        if (!f_dev_) {
            RCLCPP_WARN(this->get_logger(), "Kinect device not available");
            return;
        }
        
        // Получение и публикация RGB изображения
        publishRgbImage();
        
        // Получение и публикация облака точек
        publishPointCloud();
        
        // Обработка событий libfreenect
        freenect_process_events(f_ctx_);
    }
    
    void publishRgbImage() {
        freenect_frame_mode rgb_mode = freenect_get_current_video_mode(f_dev_);
        
        std::vector<uint8_t> current_rgb_data;
        {
            std::lock_guard<std::mutex> lock(rgb_mutex_);
            current_rgb_data = rgb_buffer_; // Копируем данные, чтобы освободить мьютекс
        }
        
        auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
        image_msg->header.stamp = this->now();
        image_msg->header.frame_id = "kinect_rgb_frame";
        image_msg->height = rgb_mode.height;
        image_msg->width = rgb_mode.width;
        image_msg->encoding = "rgb8";
        image_msg->step = rgb_mode.width * 3;
        image_msg->data = current_rgb_data;
        
        image_publisher_->publish(std::move(image_msg));
    }
    
    void publishPointCloud() {
        freenect_frame_mode depth_mode = freenect_get_current_depth_mode(f_dev_);
        
        std::vector<uint16_t> current_depth_data;
        {
            std::lock_guard<std::mutex> lock(depth_mutex_);
            current_depth_data = depth_buffer_; // Копируем данные, чтобы освободить мьютекс
        }
        
        auto pointcloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pointcloud_msg->header.stamp = this->now();
        pointcloud_msg->header.frame_id = "kinect_depth_frame";
        pointcloud_msg->height = depth_mode.height;
        pointcloud_msg->width = depth_mode.width;
        pointcloud_msg->is_dense = false;
        pointcloud_msg->is_bigendian = false;
        
        sensor_msgs::PointCloud2Modifier modifier(*pointcloud_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(*pointcloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*pointcloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*pointcloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*pointcloud_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*pointcloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*pointcloud_msg, "b");
        
        // Константы для преобразования глубины в метры (настроить для конкретной модели)
        const float fx = 594.21f;  // фокусное расстояние по X
        const float fy = 591.04f;  // фокусное расстояние по Y
        const float cx = 339.5f;   // центр изображения по X
        const float cy = 242.7f;   // центр изображения по Y
        const float depth_scale = 0.001f; // масштаб для преобразования сырых значений глубины в метры
        
        for (int v = 0; v < depth_mode.height; ++v) {
            for (int u = 0; u < depth_mode.width; ++u, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
                uint16_t depth_value = current_depth_data[v * depth_mode.width + u];
                
                // Преобразование глубины в метры
                float z = depth_value * depth_scale;
                
                // Если глубина равна 0 или 2047 (максимум для 11-bit), пропустить точку
                if (depth_value == 0 || depth_value == 2047) {
                    *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
                    *iter_r = *iter_g = *iter_b = 0;
                    continue;
                }
                
                // Преобразование 2D координат в 3D с использованием параметров камеры
                *iter_x = (u - cx) * z / fx;
                *iter_y = (v - cy) * z / fy;
                *iter_z = z;
                
                // Цвет в зависимости от глубины (можно объединить с RGB изображением)
                *iter_r = static_cast<uint8_t>(255 * (1.0f - z / 5.0f));
                *iter_g = static_cast<uint8_t>(255 * (z / 5.0f));
                *iter_b = 128;
            }
        }
        
        pointcloud_publisher_->publish(std::move(pointcloud_msg));
    }
    
    // Издатели
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Переменные для работы с libfreenect
    freenect_context* f_ctx_ = nullptr;
    freenect_device* f_dev_ = nullptr;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinectNode>());
    rclcpp::shutdown();
    return 0;
}