#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include "tf2/LinearMath/Vector3.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <deque>

pcl::PointCloud<pcl::PointXYZ>::Ptr mapCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

//debug 
pcl::PointCloud<pcl::PointXYZ>::Ptr debugCloud(new pcl::PointCloud<pcl::PointXYZ>());

pcl::VoxelGrid<pcl::PointXYZ> laserDwzFilter;
// 创建滤波器对象sor
pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;


/* 雷达的坐标系 */
std::string target_link="lidar_link";

/*地图边长，一般设置为路面的宽度*2 */
double costmapSize=9.0;

/*体素滤波参数*/
double voxelGridResolution = 0.1;

/*离群点滤波参数*/
double searchR = 0.1;
double searchNum=4;

// 标志位
bool updateMap = false;

// 地图缓存
double origin_x, origin_y, resolution;
unsigned int map_width, map_height;
std::vector<int8_t> occupancy_grid;


class LocalCostmap : public rclcpp::Node
{
    public:
        LocalCostmap() : Node("local_costmap")
        {
            publisher_local_costmap = this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_costmap", 10); //用于实际导航
            publisher_raw_costmap = this->create_publisher<nav_msgs::msg::OccupancyGrid>("raw_costmap", 10); //用于调试
            map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                    "/map", 
                    10, 
                    std::bind(&LocalCostmap::topic_callback_map, 
                    this, 
                    std::placeholders::_1));

            rclcpp::QoS qos_profile(10); //Quality of Service 用于控制通信机制，消息队列长度为10

            qos_profile.keep_last(10); //控制消息记录，保留最新10条消息

            laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                    "/top_laser",
                    10,
                    std::bind(&LocalCostmap::topic_callback_laser,
                    this,
                    std::placeholders::_1));
            
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            laserDwzFilter.setLeafSize(voxelGridResolution, voxelGridResolution, 1.0);
            sor.setRadiusSearch(searchR);
            sor.setMinNeighborsInRadius (searchNum);
        }

        void topic_callback_map(const nav_msgs::msg::OccupancyGrid & map)
        {
            if(!updateMap)
            {
                mapCloud->clear();
                origin_x = map.info.origin.position.x;
                origin_y = map.info.origin.position.y;
                map_width = map.info.width;
                map_height = map.info.height;
                resolution = map.info.resolution; //分辨率
                occupancy_grid = map.data;
                updateMap = true;
            }
        }

        void topic_callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
        {
            if(!updateMap)
                return;

            // 地图消息初始化
            nav_msgs::msg::OccupancyGrid costmap;
            costmap.header.frame_id = "base_link";
            costmap.header.stamp = this->now();
            costmap.info.resolution = resolution;
            costmap.info.width = costmapSize/resolution;
            costmap.info.height = costmapSize/resolution;
            costmap.data.resize(costmap.info.width * costmap.info.height, -1);

            laserCloud->clear();
            laserCloudCrop->clear();
            laserCloudDwz->clear();

            float angle = scan_msg->angle_min;
            for (size_t i = 0; i < scan_msg->ranges.size(); i++)
            {
                float range = scan_msg->ranges[i];
                if (std::isinf(range) || std::isnan(range))
                {
                    angle += scan_msg->angle_increment;
                    continue;
                }

                // 极坐标转笛卡尔坐标
                pcl::PointXYZ point;
                point.x = range * cos(angle);
                point.y = range * sin(angle);
                point.z = 0.0;  // 2D雷达的z坐标为固定值
                laserCloud->push_back(point);
                angle += scan_msg->angle_increment;
            }


            // 将点云转换为ROS消息格式
            sensor_msgs::msg::PointCloud2 map_cloud_msg;
            pcl::toROSMsg(*laserCloud, map_cloud_msg);


            // 坐标变换：从雷达坐标系到base_link
            try
            {
                geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                    "base_link", target_link, tf2::TimePointZero);

                costmap.info.origin.position.x = transform.transform.translation.x-costmapSize/2;
                costmap.info.origin.position.y = transform.transform.translation.y-costmapSize/2;

                tf2::doTransform(map_cloud_msg, map_cloud_msg, transform);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "TF Error: %s", ex.what());
                return;
            }

            // 转换回PCL格式
            pcl::fromROSMsg(map_cloud_msg, *laserCloudCrop);

            // 体素滤波
            laserDwzFilter.setInputCloud(laserCloudCrop);
            laserDwzFilter.filter(*laserCloudDwz);

            // 离群点去除
            sor.setInputCloud(laserCloudDwz);
            sor.filter(*cloud_filtered);

            for (const auto& point : cloud_filtered->points)
            {
                if (!pcl::isFinite<pcl::PointXYZ>(point)) 
                {
                    continue;
                }

                // 计算点在costmap中的坐标
                int mx = static_cast<int>((point.x - costmap.info.origin.position.x) / resolution);
                int my = static_cast<int>((point.y - costmap.info.origin.position.y) / resolution);

                // 检查边界
                if (mx >= 0 && mx < static_cast<int>(costmap.info.width) && my >= 0 && my < static_cast<int>(costmap.info.height))
                {
                    unsigned int index = my * costmap.info.width + mx;
                    costmap.data[index] = 100; // Mark as occupied
                }
            }
            
            publisher_raw_costmap->publish(costmap);

            // 将局部地图起始点坐标转换为代价地图索引开始值
            int global_map_x;
            int global_map_y;
            try 
            {
                geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
                global_map_x = static_cast<int>((costmap.info.origin.position.x - origin_x +transform.transform.translation.x) / resolution);
                global_map_y = static_cast<int>((costmap.info.origin.position.y - origin_y +transform.transform.translation.y) / resolution);

                tf2::doTransform(map_cloud_msg, map_cloud_msg, transform);

                for(int x=0;x<costmapSize/resolution;x++)
                {
                    for(int y=0;y<costmapSize/resolution;y++)
                    {
                        int gx = global_map_x + x;
                        int gy = global_map_y + y;
                        if(gx >= 0 && gx < static_cast<int>(map_width) && gy >= 0 && static_cast<int>(map_height))
                        {
                            unsigned int global_index = gy * map_width + gx;
                            unsigned int local_index = y * costmap.info.width + x;
                    
                            if(costmap.data[local_index] != 100 && occupancy_grid[global_index] == 100)
                            {
                                costmap.data[local_index] = occupancy_grid[global_index];
                            }
                        }
                    }
                }
            } 
            catch (tf2::TransformException& ex)
            {
                RCLCPP_ERROR(this->get_logger(), "TF Error: %s", ex.what());
                return;
            }

            // 发布局部代价地图
            publisher_local_costmap->publish(costmap);
        }

    private:
        // 订阅器和发布器
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_local_costmap;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_raw_costmap;

        // TF相关
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalCostmap>();
    node->declare_parameter<std::string>("target_link", target_link);
    node->declare_parameter<double>("costmapSize", costmapSize);
    node->declare_parameter<double>("voxelGridResolution", voxelGridResolution);
    node->declare_parameter<double>("searchR", searchR);
    node->declare_parameter<double>("searchNum", searchNum);

    node->get_parameter("target_link", target_link);
    node->get_parameter("costmapSize", costmapSize);
    node->get_parameter("voxelGridResolution", voxelGridResolution);
    node->get_parameter("searchR", searchR);
    node->get_parameter("searchNum", searchNum);


    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
