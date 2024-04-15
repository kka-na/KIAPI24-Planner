import pymap3d as pm
import rospy
from geometry_msgs.msg import Pose,Vector3, PoseArray
from geometry_msgs.msg import Point as GPoint
from morai_msgs.msg import GPSMessage, EgoVehicleStatus
from std_msgs.msg import Float32, Int8, Float32MultiArray
from visualization_msgs.msg import Marker
from jsk_recognition_msgs.msg import BoundingBoxArray

from math import radians

from config.config import Config
from vehicle_state import VehicleState
from localization.point import Point

class RosManager:
    def __init__(self, self_drive):
        config = Config()
        rospy.init_node("SelfDrive", anonymous=True)

        self.sampling_rate = config["common"]["sampling_rate"]
        self.ros_rate = rospy.Rate(self.sampling_rate)
        self.base_lla = config['map']['base_lla']

        self.self_drive = self_drive
        self.vehicle_state = VehicleState()
        self.local_path = None
        self.local_vel = None
        self.local_kappa = None
        self.lidar_object = [] 
        self.mode = 0
        self.d = [0,0]
        
    def execute(self):
        print("Start Simulation")
        self.set_protocol()
        
        while not rospy.is_shutdown():
            if self.local_path != None:
                actuators, target_velocity = self.self_drive.execute(self.mode, self.vehicle_state, self.local_path, self.local_vel, self.local_kappa, self.lidar_object)
                self.send_data(actuators, target_velocity)
            self.ros_rate.sleep()

    def set_protocol(self):
        rospy.Subscriber('/car/pose', Pose, self.pose_cb)
        rospy.Subscriber('/ltpl/local_action_set', PoseArray, self.local_action_set_cb)
        rospy.Subscriber('/car/mode', Int8,self.mode_cb)
        rospy.Subscriber('/mobinha/perception/lidar/track_box', BoundingBoxArray, self.lidar_cluster_cb)
        rospy.Subscriber('/pid_params', Vector3, self.pid_params_cb)
        
        self.actuator_pub = rospy.Publisher('/selfdrive/actuator', Vector3, queue_size=1)
        self.local_path_pub = rospy.Publisher('/selfdrive/local_path', Marker, queue_size=1)
        self.target_velocity_pub = rospy.Publisher('/selfdrive/target_velocity', Float32, queue_size=1)

    def conver_to_enu(self, lat, lng):
        x, y, _ = pm.geodetic2enu(lat, lng, 20, self.base_lla['latitude'], self.base_lla['longitude'], self.base_lla['altitude'])
        return x, y

    def mode_cb(self, msg):
        self.mode = msg.data 

    def pose_cb(self, msg):
        x, y = self.conver_to_enu(msg.position.x, msg.position.y)
        self.vehicle_state = VehicleState(x, y, radians(msg.position.z), msg.orientation.x*3.6) # enu x, enu y, radian heading, km/h velocity

    def local_action_set_cb(self, msg):
        local_path = []
        local_vel = []
        local_kappa = []

        for pose in msg.poses:
            local_path.append(Point(pose.position.x, pose.position.y))
            local_kappa.append(pose.orientation.y)
            local_vel.append(pose.orientation.z)

        self.local_path = local_path
        self.local_vel = local_vel
        self.local_kappa = local_kappa
        self.repub_local_path(local_path)

    def repub_local_path(self, local_path):
        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.header.frame_id = 'world'
        marker.ns = 'local_path'
        marker.id = 0
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = 0.7
        marker.color.r = 241/255
        marker.color.g = 76/255
        marker.color.b = 152/255
        marker.color.a = 1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        for pt in local_path:
            point = GPoint()
            point.x = pt[0]
            point.y = pt[1]
            point.z = 0.5  # Assuming all points are at height 0.5
            marker.points.append(point)
        self.local_path_pub.publish(marker)

    def pid_params_cb(self, msg):
        pid_gains = [msg.x, msg.y, msg.z]
        self.self_drive.pid_test(pid_gains)

    def lidar_cluster_cb(self, msg):
        objects = []
        for obj in msg.boxes:
            x, y = obj.pose.position.x, obj.pose.position.y
            v_rel = obj.value #velocity
            track_id = obj.label # 1~: tracking
            w = obj.pose.orientation.z #heading
            objects.append([x, y, w, v_rel, track_id])
        self.lidar_object = objects

    def send_data(self, actuators, target_velocity):
        vector3 = Vector3()
        vector3.x = actuators.steering
        vector3.y = actuators.accel
        vector3.z = actuators.brake
        self.actuator_pub.publish(vector3)
        
        self.target_velocity_pub.publish(Float32(target_velocity/3.6))