import rospy
from visualization_msgs.msg import Marker

from lanelet import LaneletMap, TileMap
import gp_utils as gput
import save_

rospy.init_node('PreRound2', anonymous=False)
pub_pr2_path = rospy.Publisher('/pr2_path', Marker, queue_size=1)

start_index = 1 # 0 : lane 2, 1 : lane 4
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
start_poses = [(-7.514, -0.297),(0,0)]
start_pose = start_poses[start_index]
final_path = []
final_ids = []

if start_index == 0:
    start_ll = gput.lanelet_matching(start_pose)
    r1, idnidx1, ids1 = gput.get_straight_path(start_ll, 3500, '23')
    idnidx2 = gput.get_merged_point(idnidx1, diag_len, 2)
    r2, idnidx3, ids2 = gput.get_straight_path(idnidx2, 3500, '103', 'Left')
    idnidx4 = gput.get_merged_point(idnidx3, diag_len, 2)
    r3, _, ids3 = gput.get_straight_path(idnidx4, 3000, '21')
    start_pose = r3[-1]
    final_path = r1+r2+r3
    final_ids = ids1+ids2+ids3
else:
    start_ll = gput.lanelet_matching(start_pose)
    r1, idnidx1, ids1 = gput.get_straight_path(start_ll, 3500, '22', 'Right')
    idnidx2 = gput.get_merged_point(idnidx1, diag_len, 1)
    r2, idnidx3, ids2 = gput.get_straight_path(idnidx2, 3000, '97')
    idnidx4 = gput.get_merged_point(idnidx3, diag_len, 2)
    r3, _, ids3 = gput.get_straight_path(idnidx4, 3000, '21')
    start_pose = r3[-1]
    final_path = r1+r2+r3
    final_ids = ids1+ids2+ids3

for i in range(3,6):
    start_ll = gput.lanelet_matching(start_pose)
    r1, idnidx1, ids1 = gput.get_straight_path(start_ll, 200, '84', 'Right')
    r2, idnidx2, ids2 = gput.get_straight_path(idnidx1, 3500, '21', 'Right')
    lap_path = r1+r2
    lap_id = ids1+ids2
    start_pose = r2[-1] 
    if i == 5:
        idnidx3 = gput.get_merged_point(idnidx2, diag_len-50, 2)
        r3, idnidx4, ids3 = gput.get_straight_path(idnidx3, 30, '27')
        lap_path = lap_path+r3
        lap_id = lap_id+ids3
    final_path = final_path+lap_path
    final_ids = final_ids+lap_id

save_.to_csv('./PreRound2b.csv', final_path)
save_.to_txt('./PreRound2b_id.txt', final_ids)

pr2_path = gput.smooth_interpolate(final_path, precision)
pr2_path_viz = gput.PreRound2Viz(pr2_path)

rate = rospy.Rate(5)
while not rospy.is_shutdown():
    pub_pr2_path.publish(pr2_path_viz)
    rate.sleep()
