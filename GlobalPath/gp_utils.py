import numpy as np
import copy
import rospy
from scipy.ndimage import gaussian_filter1d

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from quadratic_spline_interpolate import QuadraticSplineInterpolate

lanelets = None
tiles = None
tile_size = None
cut_dist = None 
M_TO_IDX = 1/0.5
IDX_TO_M = 0.5

def euc_distance(pt1, pt2):
    return np.sqrt((pt2[0]-pt1[0])**2+(pt2[1]-pt1[1])**2)

def find_nearest_idx(pts, pt):
    min_dist = float('inf')
    min_idx = 0

    for idx, pt1 in enumerate(pts):
        dist = euc_distance(pt1, pt)
        if dist < min_dist:
            min_dist = dist
            min_idx = idx

    return min_idx

def lanelet_matching(t_pt):
    row = int(t_pt[0] // tile_size)
    col = int(t_pt[1] // tile_size)

    min_dist = float('inf')
    l_id, l_idx = None, None

    for i in range(-1, 2):
        for j in range(-1, 2):
            selected_tile = tiles.get((row+i, col+j))
            if selected_tile is not None:
                for id_, data in selected_tile.items():
                    for idx, pt in enumerate(data['waypoints']):
                        dist = euc_distance(t_pt, pt)
                        if dist < min_dist:
                            min_dist = dist
                            l_id = id_
                            l_idx = data['idx'][idx]
    if l_id is not None:
        return (l_id, l_idx)
    else:
        return None

def get_straight_path(idnidx, path_len, stop_id, prior='Left'):
    s_n = idnidx[0]
    s_i = idnidx[1]
    wps = copy.deepcopy(lanelets[s_n]['waypoints'])
    lnum = lanelets[s_n]['laneNo']
    lls_len = len(wps)
    lnums = [lnum]*lls_len
    
    u_n = s_n
    u_i = s_i+int(path_len*M_TO_IDX)
    e_i = u_i
    
    path_ids = []

    while u_i >= lls_len:
        _u_n = get_possible_successor(u_n, prior)
        if _u_n == stop_id:
            u_i = lls_len-1
            break
        if _u_n == None:
            e_i = len(wps)
            break
        u_n = _u_n
        u_i -= lls_len
        e_i += u_i
        u_wp = lanelets[u_n]['waypoints']
        u_lnum = lanelets[u_n]['laneNo']
        lls_len = len(u_wp)
        lnums.extend([u_lnum]*lls_len)

        for i in range(lls_len):
            if i % int(cut_dist*M_TO_IDX) == 0:
                micro_id = i //int(cut_dist*M_TO_IDX)
                path_ids.append(f"{u_n}_{micro_id}")
        wps += u_wp
    r = wps[s_i:e_i]
    return r, [u_n, u_i], path_ids, lnums

def get_merged_point(idnidx, path_len, to=1):
        wps, [u_n, u_i],_, _ = get_straight_path(idnidx, path_len, '')
        c_pt = wps[-1]
        l_id, r_id = get_neighbor( u_n)
        n_id = l_id if to == 1 else r_id
        if n_id != None:
            r = lanelets[n_id]['waypoints']
            u_n = n_id
            u_i = find_nearest_idx(r, c_pt)

        return [u_n, u_i]
    

def get_possible_successor(node, prior='Left'):
    successor = None
    left_lanes, right_lanes, me = get_whole_neighbor(node)
    if len(lanelets[node]['successor']) <= 0:
        if prior == 'Left':
            check_a = left_lanes
            check_b = right_lanes
        else:   
            check_a = right_lanes
            check_b = left_lanes
        
        most_successor = find_most_successor(check_a)
        if most_successor == None:
            most_successor = find_most_successor(check_b)

        successor = most_successor
    else:
        if prior == 'Left':
            i = 0
        else:
            i = -1
        successor = lanelets[node]['successor'][i]

    return successor

def get_whole_neighbor(node):
    num = 1
    find_node = node
    left_most = True
    right_most = True
    left_lanes = []
    right_lanes = []

    while left_most:
        if lanelets[find_node]['adjacentLeft'] != None:
            find_node = lanelets[find_node]['adjacentLeft']
            left_lanes.append(find_node)
            num += 1
        else:
            left_most = False
            find_node = node
    
    while right_most:
        if lanelets[find_node]['adjacentRight'] != None:
            find_node = lanelets[find_node]['adjacentRight']
            right_lanes.append(find_node)
            num += 1
            
        else:
            right_most = False
            find_node = node

    me_idx = len(left_lanes)

    return left_lanes, right_lanes, me_idx

def find_most_successor(check_l):
    most_successor = None
    for c in check_l:
        if len(lanelets[c]['successor']) <= 0:
            continue
        else:
            most_successor = lanelets[c]['successor'][0]
            break
    return most_successor


def get_neighbor(node):
    l_id = lanelets[node]['adjacentLeft']
    r_id = lanelets[node]['adjacentRight']
    return l_id, r_id

def gaussian_smoothing_2d(points, sigma=1):
    wx, wy = zip(*points)
    smoothed_wx = gaussian_filter1d(wx, sigma=sigma)
    smoothed_wy = gaussian_filter1d(wy, sigma=sigma)
    return list(zip(smoothed_wx, smoothed_wy))

def smooth_interpolate(points, precision):
    points = filter_same_points(points)
    smoothed_path = gaussian_smoothing_2d(points)
    wx, wy = zip(*smoothed_path)
    itp = QuadraticSplineInterpolate(list(wx), list(wy))
    itp_points = []
    for ds in np.arange(0.0, itp.s[-1], precision):
        x, y = itp.calc_position(ds)
        itp_points.append((float(x), float(y)))

    return itp_points

def filter_same_points(points):
    filtered_points = []
    pre_pt = None

    for pt in points:
        if pre_pt is None or pt != pre_pt:
            filtered_points.append(pt)

        pre_pt = pt

    return filtered_points


def PreRound1Viz(waypoints):
    return Path(waypoints, 999, 0.2, 1.5, (255/255,79/255, 66/255, 0.5))

def PreRound2Viz(waypoints):
    return Path(waypoints, 999, 0.2, 1.5, (150/255,59/255, 255/255, 0.5))


def Path(waypoints, id_, z, scale, color):
    marker = Line('path', int(id_), scale, color, len(waypoints))
    for pt in waypoints:
        marker.points.append(Point(x=pt[0], y=pt[1], z=z))
    return marker

def Line(ns, id_, scale, color, len):
    marker = Marker()
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.header.frame_id = 'world'
    marker.ns = ns
    marker.id = id_
    marker.lifetime = rospy.Duration(0)
    marker.scale.x = scale
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    return marker
    
