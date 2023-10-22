# 发布测试栅格地图
from nav_msgs.msg import OccupancyGrid

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

# tf2坐标变换
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
# 静态和动态坐标系广播器
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
# 这里提供了ros里面四元数和欧拉角的转换
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.map_publisher_ = self.create_publisher(
            OccupancyGrid, '/planning/scenario_planning/parking/costmap_generator/occupancy_grid', 10)
        self.reset_map_origin_subscriber = self.create_subscription(
            String, 'resetOccGridOrigin', self.resetCmdCallback, 10) # 重置栅格地图原点
        timer_period = 3  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.timer_callback()
        # self.pub_test_grid_map()
        # self.i = 0
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)

    def timer_callback(self):
        # all_slots = [
        #     [(321, 105), (321, 231), (561, 105), (561, 231)],
        #     [(321, 231), (320, 318), (423, 232), (422, 319)],
        #     [(104, 318), (106, 226), (2, 316), (4, 224)],
        #     [(320, 318), (316, 409), (422, 323), (418, 414)], # 指定的停车位，起车位中间点：(420，368) -> (420, 144),错的重新计算
        #     [(106, 405), (106, 226), (4, 405), (4, 226)],
        #     [(106, 405), (104, 318), (4, 407), (2, 320)],
        # ]
        all_slots = [
            # 停车位，优选过的点，车位goal pose点为从image坐标系到girdmap坐标系 (426,395) -> (117, 86)，地图扩大一倍：->(373,342)
            [(327, 343), (321, 434), (429, 350), (423, 441)],
            # [(321, 434), (144, 429), (318, 536), (141, 531)]
        ]
        height = 1024
        width = 1024
        occ_map = OccupancyGrid()
        occ_map.header.frame_id = "fake_OccGrid_origin"
        # occ_map.header.frame_id = "map"
        occ_map.info.width = width
        occ_map.info.height = height
        occ_map.info.resolution = 0.02292  # [m/pixel]
        for i in range(height):
            for j in range(width):
                occ_map.data.append(0)
        lines_points = []
        for slot in all_slots:
            line1_points = LineDDA(slot[0], slot[1])
            line2_points = LineDDA(slot[0], slot[2])
            line3_points = LineDDA(slot[1], slot[3])
            lines_points.extend(line1_points)
            lines_points.extend(line2_points)
            lines_points.extend(line3_points)

        lines_points_indice = cv_img_points2occ_index(
            lines_points, height, width)
        for i in lines_points_indice:
            occ_map.data[i] = 100
        self.map_publisher_.publish(occ_map)
        self.get_logger().info('Published map onece!')
        
    def resetCmdCallback(self, msg: String):
        # 发布静态坐标变换，carla_world -> fake_ego_vehilce -> OccGrid_origin.
        if msg.data == 'to_pub_map':
            # 发布地图
            self.pub_test_grid_map()
            
    def pub_test_grid_map(self):
        # tf求出occ_grid的坐标原点
        # 查询坐标变换关系
        try:
            parent_frame_rel = "map"
            child_frame_rel = "fake_OccGrid_origin"
            t = self.tf_buffer.lookup_transform(
                parent_frame_rel,
                child_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            return
        
        self.factor = 4
        # self.factor = 8
        # all_slots = [
        #     [(321, 105), (321, 231), (561, 105), (561, 231)],
        #     [(321, 231), (320, 318), (423, 232), (422, 319)],
        #     [(104, 318), (106, 226), (2, 316), (4, 224)],
        #     [(320, 318), (316, 409), (422, 323), (418, 414)], # 指定的停车位，起车位中间点：(420，368) -> (420, 144),错的重新计算
        #     [(106, 405), (106, 226), (4, 405), (4, 226)],
        #     [(106, 405), (104, 318), (4, 407), (2, 320)],
        # ]
        all_slots = [
            # 停车位，优选过的点，车位goal pose点为从image坐标系到girdmap坐标系 (426,395) -> (117, 86)，地图扩大一倍：->(373,342)
            [(327, 343), (321, 434), (429, 350), (423, 441)],
            # [(321, 434), (144, 429), (318, 536), (141, 531)]
        ]
        height = 1024//self.factor
        width = 1024//self.factor
        occ_map = OccupancyGrid()
        # occ_map.header.frame_id = "fake_OccGrid_origin"
        occ_map.header.frame_id = "map"
        trans = t.transform.translation
        print(f"x:{trans.x},y:{trans.y},z:{trans.z}")
        occ_map.info.origin.position.x = t.transform.translation.x # 设置栅格地图原点
        occ_map.info.origin.position.y = t.transform.translation.y
        occ_map.info.origin.position.z = t.transform.translation.z
        # 四元数转欧拉角
        quat = t.transform.rotation
        tmp_quat = [quat.x, quat.y, quat.z, quat.w]
        euler_angles = euler_from_quaternion(tmp_quat)
        target_quat = quaternion_from_euler(0, 0, euler_angles[2])
        occ_map.info.origin.orientation.x = target_quat[0]
        occ_map.info.origin.orientation.y = target_quat[1]
        occ_map.info.origin.orientation.z = target_quat[2]
        occ_map.info.origin.orientation.w = target_quat[3]
        occ_map.info.width = width
        occ_map.info.height = height
        occ_map.info.resolution = 0.02292*self.factor  # [m/pixel]
        for i in range(height):
            for j in range(width):
                occ_map.data.append(0)
            
        new_line_points = []
        # autoware测试车位尺寸：2.75*5.5
        # 中国垂直车位：2.5*5.0
        # 这里的栅格地图分辨率大小为：0.02292
        # 因此车位尺寸为 (120, 240)
        # lines = [
        #     # [(327,0),(327,300)],
        #     [[327,300],[574,300]],
        #     [[574,300],[574,510]],
        #     [[574,510],[327,510]],
        #     # [(327,470),(327,510)]
        # ]
        lines = [
            # [(327,0),(327,300)],
            [[370,250],[574,250]],
            [[574,250],[574,490]],
            [[574,490],[370,490]],
            # [(327,470),(327,510)]
        ]
        # lines = []
        for line in lines:
            for point in line:
                point[0] = point[0]//self.factor
                point[1] = point[1]//self.factor
                
        for line in lines:
            new_line_points.extend(LineDDA(line[0], line[1]))

        lines_points_indice = cv_img_points2occ_index(
            new_line_points, height, width)
        
        
        for i in lines_points_indice:
            occ_map.data[i] = 100
        self.map_publisher_.publish(occ_map)
        self.get_logger().info('Published map once!')


def LineDDA(start, end):
    """DDA算法, 绘制直线

    Args:
        start (_type_): _description_
        end (_type_): _description_

    Returns:
        _type_: _description_
    """
    points = []
    start_x = start[0]
    start_y = start[1]
    end_x = end[0]
    end_y = end[1]
    delta_x = end_x - start_x
    delta_y = end_y - start_y

    if abs(delta_x) > abs(delta_y):
        steps = abs(delta_x)
    else:
        steps = abs(delta_y)

    x_step = delta_x / steps
    y_step = delta_y / steps

    x = start_x
    y = start_y
    while steps >= 0:
        points.append([round(x), round(y)])
        x += x_step
        y += y_step
        steps -= 1
    return points


def cv_img2occ_coord(point, height, width):
    """
    将图像坐标转换为占据坐标
    """
    return [height - (height//4 + point[1]), width - (width//4 + point[0])]


def coord2index(coord, height, width):
    """
    将占据坐标转换为占据索引
    """
    if coord[0] < 0 or coord[0] >= width or coord[1] < 0 or coord[1] >= height:
        return -1
    return coord[0] + coord[1]*height


def cv_img_points2occ_index(points, height, width):
    """
    将图像坐标转换为占据索引
    """
    indice = []
    for p in points:
        coord = cv_img2occ_coord(p, height, width)
        occ_index = coord2index(coord, height, width)
        indice.append(occ_index)
    return indice


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()