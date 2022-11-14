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
        rospy.Subscriber("gps/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("imu", Imu, self.imu_callback)

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
        # get latitude and longitude from message
        lat = msg.latitude
        lon = msg.longitude

        # put our coord and ref coord in np arrays
        spherical_coord = np.array([lat, lon])
        reference_coord = np.array([42.2, -83.7])

        # convert spherical to cartesian
        cartesian_coord = Localization.spherical_to_cartesian(spherical_coord, reference_coord)

        # update our global pose with the new position and publish it to TF tree
        current_rot = self.pose.rotation.quaternion
        self.pose = SE3.from_pos_quat(position=cartesian_coord, quaternion=current_rot)
        self.pose.publish_to_tf_tree(self.tf_broadcaster, parent_frame="map", child_frame="base_link")

    def imu_callback(self, msg: Imu):
        """
        This function will be called every time this node receives an Imu message
        on the /imu topic. It should read the orientation data from the given Imu message,
        store that value in `self.pose`, then publish that pose to the TF tree.
        """
        # get orientation quaternion from message into a numpy array
        quaternion = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

        # update global pose with the new orientation and publish it to TF tree
        current_postion = self.pose.position
        self.pose = SE3.from_pos_quat(position=current_postion, quaternion=quaternion)
        self.pose.publish_to_tf_tree(self.tf_broadcaster, parent_frame="map", child_frame="base_link")

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
        EARTH_RADIUS = 6371000

        lat, lon = spherical_coord
        ref_lat, ref_lon = reference_coord

        scaled_lon = (lon - ref_lon) * np.cos(np.radians(lat))
        y = np.radians(scaled_lon) * EARTH_RADIUS
        x = np.radians(lat - ref_lat) * EARTH_RADIUS

        return np.array([x, y, 0])


def main():
    # initialize the node
    rospy.init_node("localization")

    # create and start our localization system
    localization = Localization()

    # let the callback functions run asynchronously in the background
    rospy.spin()


if __name__ == "__main__":
    main()
