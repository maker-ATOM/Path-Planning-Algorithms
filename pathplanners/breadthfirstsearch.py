#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from custom_interfaces.msg import AdjacencyList, Neighbors, Point
from collections import deque

class BFS(Node):

    def __init__(self):
        # Node setup
        super().__init__('bfs_traversal')
        self.traversal_pub = self.create_publisher(OccupancyGrid, 'algo_viz', 10)

        # replace with service
        self.adjacency_list_sub = self.create_subscription(AdjacencyList, 'adjacency_list', self.adjacency_list_callback, 10)
        # self.initial_point_sub = self.create_subscription(PointStamped, 'initial_point', self.initial_point_callback, 10)
       
        self.adjacency_list_sub
        self.adjacency_list_sub

        # timer to call master_callback repetitively
        self.timer_period = 0.2  # seconds
        self.timer = self.create_timer(self.timer_period, self.master_callback)
        

        self.map_data = OccupancyGrid()
        self.map_data.header.frame_id = 'algo_map'
        self.map_data.info.resolution = 1.0
        self.map_data.info.width = 10
        self.map_data.info.height = 10
        self.map_data.info.origin.position.x = 0.0
        self.map_data.info.origin.position.y = 0.0
        self.map_data.info.origin.position.z = 0.0
        self.map_data.info.origin.orientation.x = 0.00
        self.map_data.info.origin.orientation.y = 0.00
        self.map_data.info.origin.orientation.z = 0.00
        self.map_data.info.origin.orientation.w = 1.00

        self.data_array = [
            [ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1], 
        ]

        self.adj_list = []
        self.perform = True


        self.bfs = []  # List to store BFS traversal
        # self.visited = [[-1] * 10 for _ in range(10)]  # Initialize visited array
        
        # should happen with service
        initpoint = Point()
        initpoint.x = 1
        initpoint.y = 7
        self.queue = deque([initpoint])  # Start BFS from node 0


        # self.color = 89
        self.color = -127

        self.once = False
    

    # replace with service
    def adjacency_list_callback(self, msg):
        self.once =  True
        # self.perform = True
        self.adj_list = msg.adjacencylist



    # replace with service
    # def initial_point_callback(self, msg):
    #     self.initial_point = msg.point

    
    def master_callback(self):

            
        if self.perform and self.once:
        
            if len(self.queue) > 0:
                node = self.queue.popleft()
                
                self.data_array[node.y][node.x] = self.color


                if node.x == 8 and node.y == 1:
                    self.perform = False 

                for neighbor in self.adj_list[node.y * 10 + node.x].neighbors:
                    
                    if self.data_array[neighbor.y][neighbor.x] == -1:
                        
                        self.queue.append(neighbor)
                        self.data_array[neighbor.y][neighbor.x] = self.color
            

            self.color += 1 

            # self.color -= 1     
            self.map_data.data = []

            # convert 2D array to single list
            for list in self.data_array:
                self.map_data.data.extend(list)

            # publish data
            self.traversal_pub.publish(self.map_data)

        else:
            self.get_logger().info("Goal Reached",once=True)

def main(args=None):
    rclpy.init(args=args)

    bfs_node = BFS()

    rclpy.spin(bfs_node)

    bfs_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()