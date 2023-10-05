#! /usr/bin/env python3

import rclpy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped

def main(args=None):

    # ----------------------------- Node setup ----------------------------- # 

    rclpy.init(args=args)

    # Node setup
    map_pub_node = rclpy.create_node('map_pub_topic')
    map_pub = map_pub_node.create_publisher(OccupancyGrid, 'map_viz', 10)
    initial_point_pub = map_pub_node.create_publisher(PointStamped, 'initial_point', 10)
    goal_point_pub = map_pub_node.create_publisher(PointStamped, 'goal_point', 10)

    map_data = OccupancyGrid()
    map_data.header.frame_id = 'map'
    map_data.info.resolution = 1.0
    map_data.info.width = 10
    map_data.info.height = 10
    map_data.info.origin.position.x = 0.0
    map_data.info.origin.position.y = 0.0
    map_data.info.origin.position.z = 0.0
    map_data.info.origin.orientation.x = 0.0
    map_data.info.origin.orientation.y = 0.0
    map_data.info.origin.orientation.z = 0.0
    map_data.info.origin.orientation.w = 1.0

    data_array = [
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1,100, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1,  0,  0,  0,  0, -1, -1, -1, -1,
        -1, -1, -1, -1, -1,  0, -1, -1, -1, -1,
        -1, -1, -1, -1, -1,  0, -1, -1, -1, -1,
        -1, 99, -1, -1, -1,  0, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 
    ]

    # data_array = [i for i in range(-128,127)]

    map_data.data = data_array

    initial_point = PointStamped()
    initial_point.header.frame_id = 'map'
    initial_point.point.x = (data_array.index(99) % 10) + 0.5
    initial_point.point.y = (data_array.index(99) // 10) + 0.5

    goal_point = PointStamped()
    goal_point.header.frame_id = 'map'
    goal_point.point.x = (data_array.index(100) % 10) + 0.5
    goal_point.point.y = (data_array.index(100) // 10) + 0.5

    # ------------------------------ Main loop ------------------------------ #

    def master_callback():
        
        nonlocal map_data
        nonlocal initial_point
        nonlocal goal_point

        # publish data
        map_pub.publish(map_data)
        initial_point_pub.publish(initial_point)
        goal_point_pub.publish(goal_point)

    # ----------------------------------------------------------------------- #

    # timer to call master_callback repetitively
    timer_period = 0.02  # seconds
    timer = map_pub_node.create_timer(timer_period, master_callback)

    try:
        rclpy.spin(map_pub_node)
    except KeyboardInterrupt:
        pass
    finally:
        # destroy node
        map_pub_node.destroy_timer(timer)
        map_pub_node.destroy_node()
        rclpy.shutdown()

# ------------------------------------------------------------------------------- #
if __name__ == "__main__":
    main()