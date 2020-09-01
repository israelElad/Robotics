#!/usr/bin/python
import globals


# Returns a list of childs for given index in grid
def get_neighbors(grid, x, y):
    possible_neighbors_list = list()
    if x + 1 < len(grid) and grid[x + 1][y] == '0':
        possible_neighbors_list.append((x + 1, y))
    if x - 1 >= 0 and grid[x - 1][y] == '0':
        possible_neighbors_list.append((x - 1, y))
    if y + 1 < len(grid[0]) and grid[x][y + 1] == '0':
        possible_neighbors_list.append((x, y + 1))
    if y - 1 >= 0 and grid[x][y - 1] == '0':
        possible_neighbors_list.append((x, y - 1))
    return possible_neighbors_list


# A DFS implementation
def DFS(grid, start_x, start_y, robot_diameter):
    open_set = set()
    closed_set = set()
    spanning_tree_4d = []

    open_set.add((start_x, start_y))
    while open_set:
        curr_cell = open_set.pop()

        if curr_cell in closed_set:
            continue

        # add cell to closed list
        closed_set.add(curr_cell)

        neighbors = get_neighbors(grid, curr_cell[0], curr_cell[1])
        for neighbor in neighbors:
            if neighbor not in closed_set and neighbor not in open_set:
                current_world_x = (robot_diameter / 2) + ((curr_cell[0] - 0) * robot_diameter)
                current_world_y = (robot_diameter / 2) + ((curr_cell[1] - 0) * robot_diameter)
                neighbor_world_x = (robot_diameter / 2) + ((neighbor[0] - 0) * robot_diameter)
                neighbor_world_y = (robot_diameter / 2) + ((neighbor[1] - 0) * robot_diameter)
                spanning_tree_4d.append((current_world_x, current_world_y, neighbor_world_x, neighbor_world_y))
                open_set.add(neighbor)

    return spanning_tree_4d


# Returns the actual direction of robot at current state
def get_correct_direction(direction):
    if globals.direction == globals.direction_up:
        return direction

    elif globals.direction == globals.direction_right:
        if direction == globals.direction_right:
            return globals.direction_down
        if direction == globals.direction_up:
            return globals.direction_right
        if direction == globals.direction_left:
            return globals.direction_up
        return globals.direction_left

    elif globals.direction == globals.direction_down:
        if direction == globals.direction_right:
            return globals.direction_left
        if direction == globals.direction_up:
            return globals.direction_down
        if direction == globals.direction_left:
            return globals.direction_right
        return globals.direction_up

    elif globals.direction == globals.direction_left:
        if direction == globals.direction_right:
            return globals.direction_up
        if direction == globals.direction_up:
            return globals.direction_left
        if direction == globals.direction_left:
            return globals.direction_down
        return globals.direction_right


def is_crossing_line(line, curr_cell, next_cell, robot_diameter, world_x, world_y, start_index_grid_x, start_index_grid_y):
    current_world_x = world_x + ((curr_cell[0] - start_index_grid_x) * robot_diameter)
    current_world_y = world_y + ((curr_cell[1] - start_index_grid_y) * robot_diameter)
    neighbor_world_x = world_x + ((next_cell[0] - start_index_grid_x) * robot_diameter)
    neighbor_world_y = world_y + ((next_cell[1] - start_index_grid_y) * robot_diameter)

    step_is_vertical = current_world_y == neighbor_world_y
    step_is_horizontal = current_world_x == neighbor_world_x
    wall_is_vertical = line[1] == line[3]
    wall_is_horizontal = line[0] == line[2]

    if step_is_vertical == wall_is_vertical:
        return False
    if step_is_horizontal == wall_is_horizontal:
        return False

    if step_is_vertical:
        #   Step is vertical and wall is horizontal
        if min(current_world_x, neighbor_world_x) < line[0] < max(current_world_x, neighbor_world_x):
            if min(line[1], line[3]) < current_world_y < max(line[1], line[3]):
                return True

    if wall_is_vertical:
        #   Wall is vertical and step is horizontal
        if min(line[0], line[2]) < current_world_x < max(line[0], line[2]):
            if min(current_world_y, neighbor_world_y) < line[1] < max(current_world_y, neighbor_world_y):
                return True

    return False


def get_next_cell(direction, curr_cell, spanning_tree_for_dgrid,
                  world_x, world_y, start_index_grid_x, start_index_grid_y, robot_diameter):
    real_direction = get_correct_direction(direction)
    next_cell = None
    if real_direction == globals.direction_up:
        next_cell = (curr_cell[0] + 1, curr_cell[1])
    elif real_direction == globals.direction_down:
        next_cell = (curr_cell[0] - 1, curr_cell[1])
    elif real_direction == globals.direction_right:
        next_cell = (curr_cell[0], curr_cell[1] + 1)
    elif real_direction == globals.direction_left:
        next_cell = (curr_cell[0], curr_cell[1] - 1)

    for line in spanning_tree_for_dgrid:
        if is_crossing_line(line, curr_cell, next_cell, robot_diameter, world_x, world_y,
                            start_index_grid_x, start_index_grid_y):
            return None

    return next_cell


def hamiltonian_path(spanning_tree, start_d_x, start_d_y, world_x, world_y, start_index_grid_x,
                     start_index_grid_y, robot_diameter):
    destination = (start_d_x, start_d_y)
    curr_cell = (start_d_x, start_d_y)
    path = []
    path.append(curr_cell)

    is_first_round = True

    while curr_cell != destination or is_first_round:
        is_first_round = False
        next_cell = get_next_cell(globals.direction_right, curr_cell, spanning_tree, world_x, world_y,
                                  start_index_grid_x, start_index_grid_y, robot_diameter)
        if next_cell:
            curr_cell = next_cell
            path.append(curr_cell)
            globals.direction = get_correct_direction(globals.direction_right)
            continue
        next_cell = get_next_cell(globals.direction_up, curr_cell, spanning_tree, world_x, world_y,
                                  start_index_grid_x, start_index_grid_y, robot_diameter)
        if next_cell:
            curr_cell = next_cell
            path.append(curr_cell)
            globals.direction = get_correct_direction(globals.direction_up)
            continue
        next_cell = get_next_cell(globals.direction_left, curr_cell, spanning_tree, world_x, world_y,
                                  start_index_grid_x, start_index_grid_y, robot_diameter)
        if next_cell:
            curr_cell = next_cell
            path.append(curr_cell)
            globals.direction = get_correct_direction(globals.direction_left)
            continue

    return path
