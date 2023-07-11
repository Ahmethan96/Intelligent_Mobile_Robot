#!/usr/bin/env python
""" Simple occupancy-grid-based mapping without localization.

Subscribed topics:
/scan
/odom (new)

Published topics:
/map
/map_metadata

"""
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np


class Map(object):
    """
    The Map class stores an occupancy grid as a two dimensional
    numpy array.

    Public instance variables:

        width      --  Number of columns in the occupancy grid.
        height     --  Number of rows in the occupancy grid.
        resolution --  Width of each grid square in meters.
        origin_x   --  Position of the grid cell (0,0) in
        origin_y   --    in the map coordinate system.
        grid       --  numpy array with height rows and width columns.


    Note that x increases with increasing column number and y increases
    with increasing row number.
    """

    def __init__(self, origin_x=-5.0, origin_y=-5.0, resolution=0.1,
                 width=600, height=600):
        """ Construct an empty occupancy grid.

        Arguments: origin_x,
                   origin_y  -- The position of grid cell (0,0) in the
                                map coordinate frame.
                   resolution-- width and height of the grid cells
                                in meters.
                   width,
                   height    -- The grid will have height rows and width
                                columns cells.  width is the size of
                                the x-dimension and height is the size
                                of the y-dimension.

         The default arguments put (0,0) in the center of the grid.

        """
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width))

    def to_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """

        grid_msg = OccupancyGrid() ## so it is not ouccupied 

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        # .info is a nav_msgs/MapMetaData message.
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height

        # Rotated maps are not supported... quaternion represents no
        # rotation.
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                                    Quaternion(0, 0, 0, 1))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).

        flat_grid = self.grid.reshape((self.grid.size,)) * 100
        # print('-----------')

        # print (flat_grid)
        flat_grid = flat_grid.astype('int8')

        ##########

        grid_msg.data = list(np.round(flat_grid))

        return grid_msg

    def set_cell(self, x, y, val):
        """ Set the value of a cell in the grid.

        Arguments:
            x, y  - This is a point in the map coordinate frame.
            val   - This is the value that should be assigned to the
                    grid cell that contains (x,y).

        This would probably be a helpful method!  Feel free to throw out
        point that land outside of the grid.
        """
        pass


class Mapper(object):
    """
    The Mapper class creates a map from laser scan data.
    """

    def __init__(self):
        """ Start the mapper. """

        rospy.init_node('mapper')
        self._map = Map()

        # Setting the queue_size to 1 will prevent the subscriber from
        # buffering scan messages.  This is important because the
        # callback is likely to be too slow to keep up with the scan
        # messages. If we buffer those messages we will fall behind
        # and end up processing really old scans.  Better to just drop
        # old scans and always work with the most recent available.
        rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber('odom',Odometry, self.odom_callback, queue_size=1)

        # Latched publishers are used for slow changing topics like
        # maps.  Data will sit on the topic until someone reads it.
        self._map_pub = rospy.Publisher('map', OccupancyGrid, latch=True)
        self._map_data_pub = rospy.Publisher('map_metadata',
                                             MapMetaData, latch=True)

        rospy.spin()


    def odom_callback(self,msg):
        global roll, pitch, yaw
        global pos
        pos = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        print('yaw: ', yaw)
        print('pos: ', pos)
        print('pos.x: ', pos.x)
    def bayes(self, pos_x, pos_y, sensor_reading):
        # R.Bayes theorem
        if pos_y >= 0 and pos_y < self._map.height: # if the position inside the boundaries
            if pos_x >= 0 and pos_x < self._map.width:
                previous_occupied_probability = self._map.grid[pos_x, pos_y] # to save the prior probability
                denominator_of_bayas = previous_occupied_probability * sensor_reading + (1 - sensor_reading) * (previous_occupied_probability)
                numerator_of_bayas = previous_occupied_probability * sensor_reading
                bayes_theorem = numerator_of_bayas/denominator_of_bayas


                return bayes_theorem

    def get_indix(self, x, y):
        x = x - self._map.origin_x
        y = y - self._map.origin_y
        i = int(round(x / self._map.resolution))
        j = int(round(y / self._map.resolution))

        return i, j

    def scan_callback(self, scan):
        """ Update the map on every scan callback. """
        print('--------------------------------')
        print('the length of the range array is: ')
        print(len(scan.ranges))
        print('angle_min=', scan.angle_min)
        print('angle_max=', scan.angle_max)

        print('yaw_Scan: ', yaw)
        print('pos.x_scan: ', pos.x)
        print('pos.y_scan: ', pos.y)
        ###############updat the mape based on scan reading##########
        # you need to writr your code heer to update your map based on
        # sensor data

        # Fill some cells in the map just so we can see that something is
        # being published.
        #self._map.grid[0, 1] = .9
        #self._map.grid[0, 2] = .7
        #self._map.grid[1, 0] = .5
        #self._map.grid[2, 0] = 0.3

        # adjust the value of other grid and see what is happened on RVIZ:
        """
        for i  in range(self._map.width):
            self._map.grid[25, i] = 1
        for j in range(50):
            self._map.grid[j,j]=0.5
        """
        radius_of_robot = 0.0000143#radius of turtlebot3
        # building a scattergram map
        for i in range(len(scan.ranges)):
            #angle_in_rad converting from degree to radians
            scan_range = scan.ranges[i]
            angle_in_rad = math.radians(i)

            if (scan.ranges[i]) != np.inf:
                if scan.ranges[i] > scan.range_min:
                    if scan.ranges[i] < scan.range_max:
                        # x and y obstacle positions
                        x_pos = math.cos(angle_in_rad) * (radius_of_robot + scan_range)
                        y_pos = math.sin(angle_in_rad) * (radius_of_robot + scan_range)
                        # rotating x and y
                        x_prime = (x_pos * math.cos(yaw)) + (y_pos* -math.sin(yaw))
                        y_prime = (x_pos * math.sin(yaw)) + (y_pos* math.cos(yaw))
                        # translating x and y
                        x_double_prime = x_prime + pos.x
                        y_double_prime = y_prime + pos.y
                        (global_cor_x, global_cor_y ) = self.get_indix(x_double_prime,y_double_prime)
                        calculated_probabilty = self.bayes(global_cor_x,global_cor_y, 1)
                        self._map.grid[global_cor_x, global_cor_y] = calculated_probabilty
                        print(calculated_probabilty)
            else:
                (global_cor_x, global_cor_y ) = self.get_indix(x_double_prime,y_double_prime)
                self._map.grid[global_cor_x, global_cor_y] = 0.1



        ############################################
        # Now that the map wass updated, so publish it!
        rospy.loginfo("Scan is processed, publishing updated map.")
        self.publish_map()

    def publish_map(self):
        """ Publish the map. """
        grid_msg = self._map.to_message()
        self._map_data_pub.publish(grid_msg.info)
        self._map_pub.publish(grid_msg)


if __name__ == '__main__':
    try:
        m = Mapper()
    except rospy.ROSInterruptException:
        pass

"""
https://w3.cs.jmu.edu/spragunr/CS354_S14/labs/mapping/mapper.py
https://www.youtube.com/watch?v=K1ZFkR4YsRQ
"""
