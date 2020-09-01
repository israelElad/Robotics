#!/usr/bin/python

import rospy
import sys
import math
import tf
import time
from nav_msgs.srv import GetMap

import stc_path_plan
import load_map
import globals
import move_robot


# A function to get the robot location
def tf_get_loc():
    x = None
    y = None
    rate = rospy.Rate(2.0)
    listener = tf.TransformListener()
    listener.waitForTransform("/map", "/base_footprint", rospy.Time(0), rospy.Duration(10))
    try:
        (trans, _) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        x = trans[0]
        y = trans[1]
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.logerr("location error!")
    return x, y


# A function to get the robot index
def calc_loc_idx(robot_diameter, old_loc_in_world, origin_loc_in_map):
    return int((old_loc_in_world - origin_loc_in_map) / robot_diameter)


if __name__ == "__main__":
    rospy.init_node("coverage_node", argv=sys.argv)

    globals.direction = globals.direction_up

    robot_diameter = 0.35
    if rospy.has_param("robot_diameter"):
        robot_diameter = rospy.get_param("robot_diameter")

    rospy.wait_for_service('static_map')

    response = None
    try:
        get_static_map = rospy.ServiceProxy('static_map', GetMap)
        response = get_static_map()
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)

    static_map = response.map

    grid = load_map.create_occupancy_grid(static_map)
    robot_diameter_pixels = int(math.ceil(robot_diameter / static_map.info.resolution))
    d_grid = load_map.create_d_size_grid(grid, robot_diameter_pixels, static_map.info.height, static_map.info.width)
    load_map.print_map_to_file(d_grid, "new_grid.txt")
    d4_grid = load_map.create_4D_grid(d_grid)
    load_map.print_map_to_file(d4_grid, "d4_grid.txt")

    map_x = static_map.info.origin.position.x
    map_y = static_map.info.origin.position.y
    world_x, world_y = tf_get_loc()
    four_grid_x = calc_loc_idx(robot_diameter * 2, world_x, map_x)
    four_grid_y = calc_loc_idx(robot_diameter * 2, world_y, map_y)
    grid_x = calc_loc_idx(robot_diameter, world_x, map_x)
    grid_y = calc_loc_idx(robot_diameter, world_y, map_y)

    d4_grid = load_map.load_grid_from_file("d4_grid.txt")
    d_grid = load_map.load_grid_from_file("new_grid.txt")

    spanning_tree = stc_path_plan.DFS(d4_grid, four_grid_x, four_grid_y, robot_diameter * 2)
    circle_path = stc_path_plan.hamiltonian_path(spanning_tree, grid_x, grid_y, world_x, world_y, grid_x,
                                                 grid_y, robot_diameter)

    for point in circle_path[:-1]:
        d_grid[point[0]][point[1]] = 2

    file = open("Coverage_path.txt", "w")
    for point in circle_path:
        file.write("row:" + str(point[0]) + "\t" + "col:" + str(point[1]) + "\n")
    file.close()

    time.sleep(10)
    move_robot.move_along_path(circle_path, robot_diameter)
    rospy.spin()
