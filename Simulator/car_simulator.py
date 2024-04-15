#!/usr/bin/python
import tf
import math
import pymap3d
import sys
import signal
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Int8
from libs.rviz_utils import *
from libs.vehicle import Vehicle

def signal_handler(sig, frame):
    sys.exit(0)

def Sphere(ns, id_, data, scale, color):
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.header.frame_id = 'world'
    marker.ns = ns
    marker.id = id_
    marker.lifetime = rospy.Duration(0)
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.r = color[0]/255
    marker.color.g = color[1]/255
    marker.color.b = color[2]/255
    marker.color.a = 0.9
    marker.pose.position.x = data[0]
    marker.pose.position.y = data[1]
    marker.pose.position.z = 0
    return marker

class HLVSimulator:
    def __init__(self):
        wheelbase = 2.97
        self.base_lla = [35.64750540757964, 128.40264207604886, 7]
        self.ego = Vehicle(0, 0, 1.664, 0, wheelbase)

        self.roll = 0.0
        self.pitch = 0.0

        self.ego_car = CarViz('ego_car', 'ego_car_info', [0, 0, 0], [241, 76, 152, 1])
        self.ego_car_info = CarInfoViz('ego_car', 'ego_car', '',[0,0,0])
        self.br = tf.TransformBroadcaster()

        self.mode = 0
        self._steer = 0
        self._accel = 0
        self._brake = 0

        self.pub_ego_car = rospy.Publisher('/car/ego_car', Marker, queue_size=1)
        self.pub_ego_car_info = rospy.Publisher('/car/ego_car_info', Marker, queue_size=1)
        self.pub_pose = rospy.Publisher('/car/pose', Pose, queue_size=1)
        self.pub_mode = rospy.Publisher('/car/mode', Int8, queue_size=1)
        self.pub_fake_obstacles = rospy.Publisher('/simulator/obstacle', MarkerArray, queue_size=1)
        rospy.Subscriber('/selfdrive/actuator', Vector3, self.actuator_cb)
        rospy.Subscriber('/mode', Int8, self.mode_cb)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_pose_cb)

    def init_pose_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y,
                      orientation.z, orientation.w)
        self.roll, self.pitch, yaw = tf.transformations.euler_from_quaternion(
            quaternion)
        self.ego.set(x, y, yaw)

    def actuator_cb(self, msg):
        self._steer = math.radians(msg.x)
        self._accel = msg.y
        self._brake = msg.z
   
    
    def mode_cb(self, msg):
        self.mode = msg.data

     
    def pub_obstacles(self):
        obj_list = [(-146.977, 220.238), (-293.271, 380.288), (-539.258, 659.329), (-359.330, 463.394), (-247.289, 340.765)]
        marker_array = MarkerArray()
        for i, obj in enumerate(obj_list):
            marker_array.markers.append(Sphere(f'obj{i}', i, obj, 5.0, (0,0,255)))
        self.pub_fake_obstacles.publish(marker_array)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            dt = 0.05
            if self.mode == 1:
                x, y, yaw, v = self.ego.next_state(
                dt, self._steer,self._accel, self._brake)
            else:
                x, y, yaw, v = self.ego.x, self.ego.y, self.ego.yaw, self.ego.v
            lat, lon, alt = pymap3d.enu2geodetic(x, y, 0, self.base_lla[0], self.base_lla[1], self.base_lla[2])

            pose = Pose()
            pose.position.x = lat
            pose.position.y = lon
            self.yaw = math.degrees(yaw)
            pose.position.z = self.yaw
            pose.orientation.x = v
            self.pub_pose.publish(pose)

            info = f"{(v*3.6):.2f}km/h {self.yaw:.2f}deg"
            self.ego_car_info.text = info

            quaternion = tf.transformations.quaternion_from_euler(math.radians(self.roll), math.radians(self.pitch), math.radians(self.yaw))  # RPY
            self.br.sendTransform(
                (x, y, 0),
                (quaternion[0], quaternion[1],
                    quaternion[2], quaternion[3]),
                rospy.Time.now(),
                'ego_car',
                'world'
            )
            self.pub_ego_car.publish(self.ego_car)
            self.pub_ego_car_info.publish(self.ego_car_info)
            self.pub_mode.publish(Int8(self.mode))
            self.pub_obstacles()
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('CarSimulator', anonymous=False)
    st = HLVSimulator()
    st.run()

if __name__ == "__main__":
    main()