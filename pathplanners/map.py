#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

class MetaData(Node):

    def __init__(self):
        # Node setup
        super().__init__('map_pub')
        self.map_pub = self.create_publisher(OccupancyGrid, 'map_viz', 10)
        self.srv = self.create_service(GetMap, 'get_map', self.get_map_callback)

        # timer to call master_callback repetitively
        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.master_callback)
        
        # initial start pose
        self.initial_point_x = 8
        self.initial_point_y = 41

        # goal pose
        self.goal_point_x = 41
        self.goal_point_y = 8

        # 2D array of the map
        rows = 50
        cols = 50

        data_array = [[0 for _ in range(rows)] for _ in range(cols)]
        
        for i in range(1, rows - 1):
            data_array[i][1:-1] = [-1] * (cols - 2)
        

        data_array[int(rows/3)][-34:-1] = [0] * 33
        data_array[int(2 * rows/3)][1:34] = [0] * 33

        """
        data_array = [
            [0,0],[1,0] . . . 
            [1,0]           .
              .             .
              .             .
              . . . . .  [49,49]
        ]
        """
     
        # playground map
        self.map_data = OccupancyGrid()
        self.map_data.header.frame_id = 'map'
        self.map_data.info.resolution = 1.0
        self.map_data.info.width = rows
        self.map_data.info.height = cols
        self.map_data.info.origin.position.x = 0.0
        self.map_data.info.origin.position.y = 0.0
        self.map_data.info.origin.position.z = 0.0
        self.map_data.info.origin.orientation.x = 0.00
        self.map_data.info.origin.orientation.y = 0.00
        self.map_data.info.origin.orientation.z = 0.00
        self.map_data.info.origin.orientation.w = 1.00

        # Assign a value to a specific cell based on init  anf goal pose
        data_array[self.initial_point_y][self.initial_point_x] = -2
        data_array[self.goal_point_y][self.goal_point_x] = 1
        
        # convert 2D array to single list
        for list in data_array:
            self.map_data.data.extend(list)


    def get_map_callback(self, request, response):
        # generated ref map
        self.get_logger().info(f"{request}")
        response.map = self.map_data
        self.get_logger().info("Now this looks like a job for me!")

        return response

    def master_callback(self):
        # publish data
        self.map_pub.publish(self.map_data)

def main(args=None):
    rclpy.init(args=args)

    metadata_node = MetaData()
    rclpy.spin(metadata_node)

    metadata_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()