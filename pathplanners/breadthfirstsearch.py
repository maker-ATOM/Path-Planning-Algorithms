#! /usr/bin/env python3

from collections import deque

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from custom_interfaces.msg import AdjacencyList, Neighbors, Point
from std_msgs.msg import Empty

from nav_msgs.srv import GetMap


class BFS(Node):

    def __init__(self):
        # Node setup
        super().__init__('bfs_traversal')
        self.traversal_pub = self.create_publisher(OccupancyGrid, 'algo_viz', 10)

        # timer to call master_callback repetitively
        self.timer_period = 0.05  # seconds
        self.timer = self.create_timer(self.timer_period, self.master_callback)
        self.get_map_service = self.create_client(GetMap, 'get_map')  

        # Wait until service is available 
        while not self.get_map_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting again...')

        # Request for initial goal pose 
        request = GetMap.Request()
        self.get_map_future = self.get_map_service.call_async(request)
        self.get_map_future.add_done_callback(self.get_map_callback)

        # Flag instead of sync service call
        self.received_data = False

        self.map_data = OccupancyGrid()

        # ref map
        self.map = []

        # start and end points
        self.initial_point = Point()
        self.goal_point = Point()

        self.queue = []

        self.color = -2

        self.goal_reached = False

    def get_map_callback(self, future):
        self.received_data = True

        self.map_data = future.result()
        self.map_data.map.header.frame_id = 'algo_map'

        height = self.map_data.map.info.height
        width = self.map_data.map.info.width

        map_list = list(self.map_data.map.data)

        # print(map_list)

        for j in range(0, width):
            row = []
            for i in range(0, height):
                row.append(map_list[width*j + i])
                if row[-1] == 1:
                    self.goal_point.x = i
                    self.goal_point.y = j
                if row[-1] == -2:
                    self.initial_point.x = i
                    self.initial_point.y = j
            self.map.append(row)
        
        self.queue = deque([self.initial_point])

        self.get_logger().info("Received map data")
        self.get_logger().info(f"Initial pose => x: {self.initial_point.x}, y: {self.initial_point.y}")
        self.get_logger().info(f"Goal pose => x: {self.goal_point.x}, y: {self.goal_point.y}")
    
    def master_callback(self):

        dx = [0, -1, 0, 1]
        dy = [1, 0, -1, 0]

        if self.received_data:

            if len(self.queue) > 0:
                node = self.queue.popleft()
                self.map[node.y][node.x] = int(self.color)

                for i in range(0, 4):
                    neighbor = Point()
                    neighbor.x = node.x + dx[i]
                    neighbor.y = node.y + dy[i]

                    if self.map[neighbor.y][neighbor.x] == -1:
                        self.queue.append(neighbor)
                        self.map[neighbor.y][neighbor.x] = int(self.color)
                    elif self.map[neighbor.y][neighbor.x] == 1:
                        self.received_data = False
                        self.goal_reached = True
            
            self.color -= 0.78
            if self.color < -127:
                self.color = 98

            self.map_data.map.data = []
            
            # convert 2D array to single list
            for list in self.map:
                self.map_data.map.data.extend(list)

            # publish data
            self.traversal_pub.publish(self.map_data.map)

        elif self.goal_reached:
            self.get_logger().info("Goal Reached",once=True)

def main(args=None):
    rclpy.init(args=args)

    bfs_node = BFS()

    rclpy.spin(bfs_node)

    bfs_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()