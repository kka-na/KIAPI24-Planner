import rospy
from visualization_msgs.msg import Marker

from lanelet import LaneletMap, TileMap
import gp_utils as gput
import save_ 

rospy.init_node('PreRound1', anonymous=False)
pub_pr1_path = rospy.Publisher('/pr1_path', Marker, queue_size=1)

tile_size = 5
cut_dist = 15
precision = 0.5
diag_len = 70
lmap = LaneletMap('KCity-Racing.json')
tmap = TileMap(lmap.lanelets, tile_size)

gput.lanelets = lmap.lanelets
gput.tiles = tmap.tiles
gput.tile_size = tile_size
gput.cut_dist = cut_dist-1
# Path Generation
start_pose = [0,0]
final_path = []
final_ids = []

for i in range(1,6):
    start_ll = gput.lanelet_matching(start_pose)
    r1, idnidx1, ids1 = gput.get_straight_path(start_ll, 200, '84', 'Right')
    r2, idnidx2, ids2 = gput.get_straight_path(idnidx1, 3500, '21', 'Right') #'Right' means choose 104 node before joker lap
    lap_path = r1+r2
    lap_ids = ids1+ids2
    start_pose = r2[-1] 
    if i == 5:
        idnidx3 = gput.get_merged_point(idnidx2, diag_len-50, 2)
        r3, _, ids3 = gput.get_straight_path(idnidx3, 30, '27')
        lap_path = lap_path+r3
        lap_ids = lap_ids+ids3

    final_path.extend(lap_path)
    final_ids.extend(lap_ids)

save_.to_csv('./PreRound1.csv', final_path)
save_.to_txt('./PreRound1_id.txt', final_ids)
pr1_path = gput.smooth_interpolate(final_path, precision)
pr1_path_viz = gput.PreRound1Viz(pr1_path)

rate = rospy.Rate(5)
while not rospy.is_shutdown():
    pub_pr1_path.publish(pr1_path_viz)
    rate.sleep()
