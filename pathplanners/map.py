#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from custom_interfaces.msg import AdjacencyList, Neighbors, Point

class MetaData(Node):

    def __init__(self):
        # Node setup
        super().__init__('map_pub')
        self.map_pub = self.create_publisher(OccupancyGrid, 'map_viz', 10)
        self.initial_point_pub = self.create_publisher(PointStamped, 'initial_point', 10)
        self.goal_point_pub = self.create_publisher(PointStamped, 'goal_point', 10)
        self.adj_list_pub = self.create_publisher(AdjacencyList, 'adjacency_list', 10)

        # timer to call master_callback repetitively
        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.master_callback)
        
        # initial start pose
        self.initial_point = PointStamped()
        self.initial_point.header.frame_id = 'map'
        self.initial_point.point.x = 1.5
        self.initial_point.point.y = 7.5

        # goal pose
        self.goal_point = PointStamped()
        self.goal_point.header.frame_id = 'map'
        self.goal_point.point.x = 8.5
        self.goal_point.point.y = 1.5

        # 2D array of the map
        data_array = [
            [ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [ -1, -1,  0,  0,  0,  0, -1, -1, -1, -1],
            [ -1, -1, -1, -1, -1,  0, -1, -1, -1, -1],
            [ -1, -1, -1, -1, -1,  0, -1, -1, -1, -1],
            [ -1, -1, -1, -1, -1,  0, -1, -1, -1, -1],
            [ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
            [ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1], 
        ]

        """
        data_array = [
            [0,0],[1,0] . . . 
            [1,0]           .
              .             .
              .             .
              . . . . . . [9,9]
        ]
        """

        # Adjacency list of the matrix generated

        self.adj_list = AdjacencyList()

        # delta
        dx = [-1, 1, 0, 0]
        dy = [ 0, 0, 1,-1]

        # max
        mx = len(data_array[0])
        my = len(data_array)

        for y in range(my):
            for x in range(mx):
                neighbors = Neighbors()
                for i in range(4):
                    # new
                    point = Point()
                    point.x = x + dx[i]
                    point.y = y + dy[i]
                    if point.x < 0 or point.y < 0: continue
                    if point.x >= mx or point.y >= my: continue
                    if data_array[point.y][point.x] == -1:
                        neighbors.neighbors.append(point)

                self.adj_list.adjacencylist.append(neighbors)

        
        # playground map
        self.map_data = OccupancyGrid()
        self.map_data.header.frame_id = 'map'
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

        # Assign a value to a specific cell based on init  anf goal pose
        data_array[int(self.initial_point.point.y - 0.5)][int(self.initial_point.point.x - 0.5)] = 99
        data_array[int(self.goal_point.point.y - 0.5)][int(self.goal_point.point.x - 0.5)] = 100
        
        # convert 2D array to single list
        for list in data_array:
            self.map_data.data.extend(list)

    def master_callback(self):
        # publish data
        self.map_pub.publish(self.map_data)
        self.initial_point_pub.publish(self.initial_point)
        self.goal_point_pub.publish(self.goal_point)
        self.adj_list_pub.publish(self.adj_list)

def main(args=None):
    rclpy.init(args=args)

    metadata_node = MetaData()

    rclpy.spin(metadata_node)

    metadata_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()