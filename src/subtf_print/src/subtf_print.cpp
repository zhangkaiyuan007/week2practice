#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

class TFPrinter : public rclcpp::Node
{
public:
    TFPrinter() : Node("tf_printer")
    {
        this->declare_parameter<std::string>("print_frame", "odom");
        print_frame_ = this->get_parameter("print_frame").as_string();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TFPrinter::printTransform, this));
    }

private:
    void printTransform()
    {
        try
        {
            
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "map",  
                print_frame_,  
                tf2::TimePointZero);  

            
            RCLCPP_INFO(this->get_logger(),
                        "Transform from 'map' to '%s':\n"
                        "  Translation: [x=%.2f, y=%.2f, z=%.2f]\n"
                        "  Rotation: [x=%.2f, y=%.2f, z=%.2f, w=%.2f]",
                        print_frame_.c_str(),
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z,
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string print_frame_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TFPrinter>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}