#!/usr/bin/python

import math


def print_map_to_file(d_grid, filename):
    with open(filename, "w+") as grid_file:
        for row in reversed(d_grid):
            for cell in row:
                grid_file.write("1") if cell else grid_file.write("0")
            grid_file.write("\n")


def create_occupancy_grid(my_map):
    # creating the occupancy grid
    grid = [[None] * my_map.info.width for i in xrange(my_map.info.height)]
    for i in xrange(my_map.info.height):
        for j in xrange(my_map.info.width):
            if my_map.data[i * my_map.info.width + j] == 0:
                grid[i][j] = False
            else:
                grid[i][j] = True
    return grid


def create_d_size_grid(grid, robot_diameter_pixels, regular_grid_height, regular_grid_width):
    # height = num of rows
    d_grid_rows = int(math.floor(regular_grid_height / robot_diameter_pixels))
    # width = num of columns
    d_grid_columns = int(math.floor(regular_grid_width / robot_diameter_pixels))
    d_grid = [[None] * d_grid_columns for i in xrange(d_grid_rows)]

    # we're going through the regular grid with pixels according to the robot diameter in pixels
    row_start = 0
    column_start = 0
    for i in range(d_grid_rows):
        for j in range(d_grid_columns):
            r_end = row_start + robot_diameter_pixels
            c_end = column_start + robot_diameter_pixels
            d_grid[i][j] = int(is_square_occupied(grid, row_start, column_start, r_end, c_end, regular_grid_height,
                                                  regular_grid_width))
            column_start += robot_diameter_pixels

        column_start = 0
        row_start += robot_diameter_pixels

    return d_grid


def is_square_occupied(grid, row_start, column_start, row_end, column_end, regular_grid_height, regular_grid_width):
    if row_end >= regular_grid_height or column_end >= regular_grid_width:
        return False
    row_traverser = row_start
    while row_traverser < row_end:
        column_traverser = column_start
        while column_traverser < column_end:
            if grid[row_traverser][column_traverser] == True:
                return True
            column_traverser = column_traverser + 1
        row_traverser = row_traverser + 1
    return False


def create_4D_grid(d_grid):
    d4_grid = [[None] * (len(d_grid[0]) / 2) for i in range(len(d_grid) / 2)]
    d_i, d_j = 0, 0
    for i in range(0, len(d_grid), 2):
        for j in range(0, len(d_grid[i]), 2):
            # occupied
            if d_grid[i][j] == 1 or d_grid[i + 1][j] == 1 or d_grid[i][j + 1] == 1 or d_grid[i + 1][j + 1] == 1:
                d4_grid[d_i][d_j] = True
            else:  # not occupied
                d4_grid[d_i][d_j] = False
            if d_j < len(d4_grid[0]) - 1:
                d_j += 1
            else:
                break
        d_j = 0
        if d_i < len(d4_grid) - 1:
            d_i += 1
        else:
            break
    return d4_grid


def load_grid_from_file(path):
    file = open(path, "r")
    grid = []

    for line in file:
        line_list = []
        for char in line:
            if char != '\n':
                line_list.append(char)
        grid.append(line_list)
    file.close()
    return grid
