import rospy
from visualization_msgs.msg import Marker

from lanelet import LaneletMap, TileMap
from micro_lanelet_graph import MicroLaneletGraph
import gp_utils as gput

rospy.init_node('PreRound1', anonymous=False)
pub_pr1_path = rospy.Publisher('/pr1_path', Marker, queue_size=1)

tile_size = 5
cut_dist = 15
precision = 0.5
lmap = LaneletMap('KCity-Racing.json')
tmap = TileMap(lmap.lanelets, tile_size)
#graph = MicroLaneletGraph(lmap, cut_dist).graph

gput.lanelets = lmap.lanelets
gput.tiles = tmap.tiles
gput.tile_size = tile_size

# Path Generation
start_pose = [0,0]
start_ll = gput.lanelet_matching(start_pose)
r1, idnidx1 = gput.get_straight_path(start_ll, 4000, '85')
_, idnidx2 = gput.get_change_path(idnidx1, 15, 1)
r2, idnidx3 = gput.get_straight_path(idnidx2, 3000, '')

pr1_path = gput.smooth_interpolate(r1+r2, precision)

pr1_path_viz = gput.PreRound1Viz(pr1_path)

rate = rospy.Rate(0.05)
while not rospy.is_shutdown():
    pub_pr1_path.publish(pr1_path_viz)
    rate.sleep()
