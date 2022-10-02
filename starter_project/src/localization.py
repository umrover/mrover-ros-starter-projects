#!/usr/bin/env python3

# python linear algebra library
import numpy as np

# library for interacting with ROS and TF tree
import rospy
import tf2_ros

# ROS message types we need to use
from sensor_msgs.msg import NavSatFix, Imu

# SE3 library for handling poses and TF tree
from util.SE3 import SE3


class Localization:
    pose: SE3

    def __init__(self):
        # create subscribers for GPS and IMU data, linking them to our callback functions
        # TODO
        gps = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        imu = rospy.Subscriber('/imu', Imu, self.imu_callback)

        # create a transform broadcaster for publishing to the TF tree
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # initialize pose to all zeros
        self.pose = SE3()

    def gps_callback(self, msg: NavSatFix):
        """
        This function will be called every time this node receives a NavSatFix message
        on the /gps topic. It should read the GPS location from the given NavSatFix message,
        convert it to cartesian coordiantes, store that value in `self.pose`, then publish
        that pose to the TF tree.
        """
        # TODO
        # spherical, reference coords passed in 
        coords = self.spherical_to_cartesian(np.array([msg.latitude, msg.longitude]),
                                             np.array([42.2, -83.7]))
        rospy.logerr(f"coordinates = {coords[0]}, {coords[1]}, {coords[2]}")
        self.pose = SE3(position=coords.copy(), rotation=self.pose.rotation)
        self.pose.publish_to_tf_tree(self.tf_broadcaster, "map", "base_link")


    def imu_callback(self, msg: Imu):
        """
        This function will be called every time this node receives an Imu message
        on the /imu topic. It should read the orientation data from the given Imu message,
        store that value in `self.pose`, then publish that pose to the TF tree.
        """
        # TODO
        self.pose = SE3.from_pos_quat(position=self.pose.position, quaternion=np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))
        self.pose.publish_to_tf_tree(self.tf_broadcaster, "map", "base_link")


    @staticmethod
    def spherical_to_cartesian(spherical_coord: np.ndarray, reference_coord: np.ndarray) -> np.ndarray:
        """
        This is a utility function that should convert spherical (latitude, longitude)
        coordinates into cartesian (x, y, z) coordinates using the specified reference point
        as the center of the tangent plane used for approximation.
        :param spherical_coord: the spherical coordinate to convert,
                                given as a numpy array [latiude, longitude]
        :param reference_coord: the reference coordinate to use for conversion,
                                given as a numpy array [latiude, longitude]
        :returns: the approximated cartesian coordinates in meters, given as a numpy array [x, y, z]
        """
        # TODO
        R = 6371000
        y = -(R*(np.radians(spherical_coord[1])-np.radians(reference_coord[1]))*np.cos(np.radians(reference_coord[0])))
        x = R*(np.radians(spherical_coord[0])-np.radians(reference_coord[0]))
        z = 0
        rospy.logerr(f"degrees = {spherical_coord[0]}, {spherical_coord[1]}, {reference_coord[0]}, {reference_coord[1]}")
        return np.array([x,y,z])


def main():
    # initialize the node
    rospy.init_node("localization")

    # create and start our localization system
    localization = Localization()

    # let the callback functions run asynchronously in the background
    rospy.spin()


if __name__ == "__main__":
    main()