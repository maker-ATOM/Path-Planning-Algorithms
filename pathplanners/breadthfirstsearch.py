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
        
        # constants
        self.FREE_SPACE = -1
        self.START = -2
        self.GOAL = 1
        self.TRAVERSAL_COLOR_INCREMENT = 0.78
        self.FINAL_COLOR = -127
        self.TRAVERSAL_COLOR_UPDATE = 98

        # Flag instead of sync service call
        self.received_data = False

        # transmitted on topic
        self.map_data = GetMap.Response()
        
        # start and end points
        self.initial_point = Point()
        self.goal_point = Point()

        # map
        self.map = []
        self.parent_map = []

        self.queue = []

        self.color = self.START

        self.goal_reached = False

        # Node setup
        super().__init__('bfs_traversal')
        self.traversal_pub = self.create_publisher(OccupancyGrid, 'algo_viz', 10)
        self.get_map_service = self.create_client(GetMap, 'get_map') 

        # timer to call master_callback repetitively
        self.timer_period = 0.05  # seconds
        self.timer = self.create_timer(self.timer_period, self.master_callback)
         
        # Wait until service is available 
        while not self.get_map_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting again...')

        # Request for initial goal pose 
        request = GetMap.Request()
        self.get_map_future = self.get_map_service.call_async(request)
        self.get_map_future.add_done_callback(self.get_map_callback)

    def get_map_callback(self, future):
        self.received_data = True

        self.map_data = future.result()
        self.map_data.map.header.frame_id = 'algo_map'

        # extract height and width of map
        height = self.map_data.map.info.height
        width = self.map_data.map.info.width

        map_list = list(self.map_data.map.data)

        # generate the 2D array

        for j in range(0, width):
            row = []
            for i in range(0, height):
                row.append(map_list[width*j + i])
                if row[-1] == self.GOAL:
                    self.goal_point.x = i
                    self.goal_point.y = j
                if row[-1] == self.START:
                    self.initial_point.x = i
                    self.initial_point.y = j
            self.map.append(row)
        
        # initial_point to queue
        self.queue = deque([self.initial_point])

        # set parent map
        self.parent_map = [[-1] * height for _ in range(width)]
        
        # required to generate the map
        self.path_node = self.goal_point

        self.get_logger().info("Received map data")
        self.get_logger().info(f"Initial pose => x: {self.initial_point.x}, y: {self.initial_point.y}")
        self.get_logger().info(f"Goal pose => x: {self.goal_point.x}, y: {self.goal_point.y}")
    
    def master_callback(self):

        # direction vector to generate neighbors
        dx = [0, -1, 0, 1]
        dy = [1, 0, -1, 0]

        if self.received_data:
            # do traversal 

            if len(self.queue) > 0:
                # get recent node and mark it as visited
                node = self.queue.popleft()
                # self.map[node.y][node.x] = int(self.color)

                for i in range(0, 4):
                    # generate new neighbos
                    neighbor = Point()
                    neighbor.x = node.x + dx[i]
                    neighbor.y = node.y + dy[i]

                    # neighbor is traversable free space
                    if self.map[neighbor.y][neighbor.x] == self.FREE_SPACE:
                        
                        # add to queue
                        self.queue.append(neighbor)
                        # update the map indicating neighbor visited
                        self.map[neighbor.y][neighbor.x] = int(self.color)

                        # parent of neighbor is node
                        self.parent_map[neighbor.y][neighbor.x] = node

                    # if any of the neighbor is goal pose
                    elif self.map[neighbor.y][neighbor.x] == self.GOAL:
                        self.received_data = False
                        self.goal_reached = True

                        # parent of neighbor is node
                        self.parent_map[neighbor.y][neighbor.x] = node
            

            self.color -= self.TRAVERSAL_COLOR_INCREMENT
            # uneven transition
            if self.color < self.FINAL_COLOR:
                self.color = self.TRAVERSAL_COLOR_UPDATE

        elif self.goal_reached:
            self.get_logger().info("Goal Reached",once=True)

            try:
                # get parent node to generate path
                path_element = self.parent_map[self.path_node.y][self.path_node.x]
            except:
                    self.goal_reached = False
                    self.get_logger().info("Path Generated")

                    # program should not exit but else does it has to do?
                    exit()
           
           # update map with path
            self.map[self.path_node.y][self.path_node.x] = 101
            
            # node is path-element
            self.path_node = path_element        


        self.map_data.map.data = []

        # convert 2D array to single list
        for list in self.map:
            self.map_data.map.data.extend(list)

        # publish data
        self.traversal_pub.publish(self.map_data.map)

def main(args=None):
    rclpy.init(args=args)

    bfs_node = BFS()

    rclpy.spin(bfs_node)

    bfs_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()