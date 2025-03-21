import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import yaml
import cv2
import numpy as np

class MyMap(Node):
    def __init__(self):
        # 初始化节点
        super().__init__('my_map_node')
        
        # 声明参数
        self.declare_parameter('map_path', 'my_map.yaml')
        
        # 创建地图发布者
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # 加载地图
        self.load_map()
        
        # 创建定时器，定期发布地图
        self.map_timer = self.create_timer(0.1, self.publish_map)

    def load_map(self):
        # 获取地图路径参数
        map_path = self.get_parameter('map_path').get_parameter_value().string_value
        self.get_logger().info(f"Loading map from: {map_path}")
        
        try:
            # 读取 YAML 文件
            with open(map_path, 'r') as file:
                data = yaml.safe_load(file)
            
            # 加载 PGM 图像
            image_path = data['image']
            self.get_logger().info(f"Loading map image from: {image_path}")
            map_image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
            if map_image is None:
                self.get_logger().error(f"Failed to load map image: {image_path}")
                return
            
            # 反转图像（如果需要）
            map_image = np.invert(map_image)
            
            # 垂直翻转图像（因为图像坐标系的原点在左上，地图坐标系的原点在左下）
            map_image = cv2.flip(map_image, 0)
            
            # 创建 OccupancyGrid 消息
            self.occupancy_grid = OccupancyGrid()
            self.occupancy_grid.header.frame_id = 'map'
            self.occupancy_grid.info.resolution = float(data['resolution'])
            self.occupancy_grid.info.width = map_image.shape[1]
            self.occupancy_grid.info.height = map_image.shape[0]
            self.occupancy_grid.info.origin.position.x = float(data['origin'][0])
            self.occupancy_grid.info.origin.position.y = float(data['origin'][1])
            self.occupancy_grid.info.origin.position.z = 0.0
            
            # 转换图像数据为 OccupancyGrid 的 data 字段
            map_data = map_image.flatten()
            map_data[map_data == 255] = 100  # 占用区域
            map_data[map_data == 1] = 0       # 自由区域
            map_data[map_data == 50] = -1     # 未知区域
            self.occupancy_grid.data = map_data.astype(np.int8).tolist()
            
            self.get_logger().info("Map loaded successfully!")
            self.get_logger().info(f"Map resolution: {self.occupancy_grid.info.resolution}")
            self.get_logger().info(f"Map size: {self.occupancy_grid.info.width}x{self.occupancy_grid.info.height}")
        
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {str(e)}")

    def publish_map(self):
        if hasattr(self, 'occupancy_grid'):
            # 将时间戳设置为 0
            self.occupancy_grid.header.stamp.sec = 0
            self.occupancy_grid.header.stamp.nanosec = 0
            self.map_publisher.publish(self.occupancy_grid)
            self.get_logger().info("Map published!")
        else:
            self.get_logger().warn("No map data to publish.")

def main(args=None):
    rclpy.init(args=args)
    my_map_node = MyMap()
    rclpy.spin(my_map_node)
    my_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()