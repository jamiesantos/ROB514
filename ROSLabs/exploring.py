#!/usr/bin/env python3

# This assignment lets you both define a strategy for picking the next point to explore and determine how you
#  want to chop up a full path into way points. You'll need path_planning.py as well (for calculating the paths)
#
# Note that there isn't a "right" answer for either of these. This is (mostly) a light-weight way to check
#  your code for obvious problems before trying it in ROS. It's set up to make it easy to download a map and
#  try some robot starting/ending points
#
# Given to you:
#   Image handling
#   plotting
#   Some structure for keeping/changing waypoints and converting to/from the map to the robot's coordinate space
#
# Slides

# The ever-present numpy
import numpy as np

# Your path planning code
import path_planning as path_planning


# -------------- Showing start and end and path ---------------
def plot_with_explore_points(im_threshhold, zoom=1.0, robot_loc=None, explore_points=None, best_pt=None):
    """Show the map plus, optionally, the robot location and points marked as ones to explore/use as end-points
    @param im - the image of the SLAM map
    @param im_threshhold - the image of the SLAM map
    @param robot_loc - the location of the robot in pixel coordinates
    @param best_pt - The best explore point (tuple, i,j)
    @param explore_points - the proposed places to explore, as a list"""

    # Putting this in here to avoid messing up ROS
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(1, 2)
    axs[0].imshow(im_threshhold, origin='lower', cmap="gist_gray")
    axs[0].set_title("original image")
    axs[1].imshow(im_threshhold, origin='lower', cmap="gist_gray")
    axs[1].set_title("threshold image")
    """
    # Used to double check that the is_xxx routines work correctly
    for i in range(0, im_threshhold.shape[1]-1, 10):
        for j in range(0, im_threshhold.shape[0]-1, 2):
            if is_reachable(im_thresh, (i, j)):
                axs[1].plot(i, j, '.b')
    """

    # Show original and thresholded image
    if explore_points is not None:
        for p in explore_points:
            axs[1].plot(p[0], p[1], '.b', markersize=2)

    for i in range(0, 2):
        if robot_loc is not None:
            axs[i].plot(robot_loc[0], robot_loc[1], '+r', markersize=10)
        if best_pt is not None:
            axs[i].plot(best_pt[0], best_pt[1], '*y', markersize=10)
        axs[i].axis('equal')

    for i in range(0, 2):
        # Implements a zoom - set zoom to 1.0 if no zoom
        width = im_threshhold.shape[1]
        height = im_threshhold.shape[0]

        axs[i].set_xlim(width / 2 - zoom * width / 2, width / 2 + zoom * width / 2)
        axs[i].set_ylim(height / 2 - zoom * height / 2, height / 2 + zoom * height / 2)


# -------------- For converting to the map and back ---------------
def convert_pix_to_x_y(im_size, pix, size_pix):
    """Convert a pixel location [0..W-1, 0..H-1] to a map location (see slides)
    Note: Checks if pix is valid (in map)
    @param im_size - width, height of image
    @param pix - tuple with i, j in [0..W-1, 0..H-1]
    @param size_pix - size of pixel in meters
    @return x,y """
    if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
        raise ValueError(f"Pixel {pix} not in image, image size {im_size}")

    return [size_pix * pix[i] / im_size[1-i] for i in range(0, 2)]


def convert_x_y_to_pix(im_size, x_y, size_pix):
    """Convert a map location to a pixel location [0..W-1, 0..H-1] in the image/map
    Note: Checks if x_y is valid (in map)
    @param im_size - width, height of image
    @param x_y - tuple with x,y in meters
    @param size_pix - size of pixel in meters
    @return i, j (integers) """
    pix = [int(x_y[i] * im_size[1-i] / size_pix) for i in range(0, 2)]

    if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
        raise ValueError(f"Loc {x_y} not in image, image size {im_size}")
    return pix


def is_reachable(im, pix):
    """ Is the pixel reachable, i.e., has a neighbor that is free?
    Used for
    @param im - the image
    @param pix - the pixel i,j"""

    # GUIDE: Returns True (the pixel is adjacent to a pixel that is free)
    #  False otherwise
    # You can use four or eight connected - eight will return more points
    # YOUR CODE HERE

    if not path_planning.is_unseen(im, pix):
        return False
    
    width = im.shape[1]
    height = im.shape[0]

    for i, j in path_planning.four_connected(pix):
        out_of_bounds = (i < 0 or i >= width or j < 0 or j >= height)
        if not out_of_bounds and path_planning.is_free(im, (i, j)):
            return True
    return False

def find_all_possible_goals(im):
    """ Find all of the places where you have a pixel that is unseen next to a pixel that is free
    It is probably easier to do this, THEN cull it down to some reasonable places to try
    This is because of noise in the map - there may be some isolated pixels
    @param im - thresholded image
    @return dictionary or list or binary image of possible pixels"""

    # YOUR CODE HERE
    possible_pixels = []
    for j in range(im.shape[0]):
        for i in range(im.shape[1]):
            if path_planning.is_unseen(im, (i,j)) and   is_reachable(im, (i,j)):
                possible_pixels.append((i,j))
    return possible_pixels

def find_reachable_free_neighbor(im, pix, robot_loc):
    """
    Given a frontier point (unseen next to free), return a reachable free pixel
    that is close to it.
    """
    candidates = []

    # Gather 8-connected neighbors so we get more choices
    for nbr in path_planning.eight_connected(pix):
        x, y = nbr
        if 0 <= x < im.shape[1] and 0 <= y < im.shape[0]:
            if path_planning.is_free(im, nbr):
                candidates.append(nbr)

    best = None
    best_dist = float('inf')

    # Among all free neighbors, choose the one with the shortest A*/Dijkstra path
    for nbr in candidates:
        path = path_planning.dijkstra(im, robot_loc, nbr)
        if path is not None:
            if len(path) < best_dist:
                best = nbr
                best_dist = len(path)

    return best   # can be None if nothing reachable
'''    
def find_best_point(im, possible_points, robot_loc):
    """ Pick one of the unseen points to go to
    @param im - thresholded image
    @param possible_points - possible points to chose from
    @param robot_loc - location of the robot (in case you want to factor that in)
    """
    # YOUR CODE HERE
    #if not possible_points:
    #    return None

    # Filter out points outside of map
    mask = im < 250
    ys, xs = np.where(mask)
    y_min, y_max = ys.min(), ys.max()
    x_min, x_max = xs.min(), xs.max()
    best_point = None

    # Travel to the farthest point
    distance_max = 0

    for point in possible_points:
        x,y = point
        if not (x_min <= x <= x_max and y_min <= y <= y_max):
            continue

        free = []
        for point_adj in path_planning.four_connected(point):
            i, j = point_adj
            if 0 <= i < im.shape[1] and 0 <= j < im.shape[0]:
                if path_planning.is_free(im, point_adj):
                    free.append(point_adj)
        if not free:
            continue
        for point_adj in free:
            try:
                path = path_planning.dijkstra(im, robot_loc, point_adj)
                if path is not None:
                    distance = len(path)
                    if distance > distance_max:
                        best_point = point_adj
                        distance_max = distance
            except:
                continue

    return best_point
'''
def find_best_point(im, possible_points, robot_loc):

    best_point = None
    longest_dist = 0

    for frontier in possible_points:

        # IMPORTANT CHANGE:
        # Instead of trying to go to the frontier point directly,
        # we find a reachable free pixel near it.
        reachable_neighbor = find_reachable_free_neighbor(im, frontier, robot_loc)

        if reachable_neighbor is None:
            continue   # Skip unreachable frontier pixels

        # Now compute path to the reachable free pixel
        path = path_planning.dijkstra(im, robot_loc, reachable_neighbor)
        if path is None:
            continue

        dist = len(path)
        if dist > longest_dist:
            longest_dist = dist
            best_point = reachable_neighbor   # NOT frontier; use reachable pixel

    return best_point

def find_waypoints(im, path):
    """ Place waypoints along the path
    @param im - the thresholded image
    @param path - the initial path
    @ return - a new path"""

    # Again, no right answer here
    # YOUR CODE HERE
    if not path:
        return path

    smoothing_window = 30
    waypoints = path[::smoothing_window]
    if waypoints[-1] != path[-1]:
        waypoints.append(path[-1])      # Include final goal

    return waypoints

if __name__ == '__main__':
    im, im_thresh = path_planning.open_image("map.pgm")

    robot_start_loc = (1940, 1953)

    all_unseen = find_all_possible_goals(im_thresh)
    best_unseen = find_best_point(im_thresh, all_unseen, robot_loc=robot_start_loc)

    plot_with_explore_points(im_thresh, zoom=0.1, robot_loc=robot_start_loc, explore_points=all_unseen, best_pt=best_unseen)

    path = path_planning.dijkstra(im_thresh, robot_start_loc, best_unseen)
    waypoints = find_waypoints(im_thresh, path)
    path_planning.plot_with_path(im, im_thresh, zoom=0.1, robot_loc=robot_start_loc, goal_loc=best_unseen, path=waypoints)

    # Depending on if your mac, windows, linux, and if interactive is true, you may need to call this to get the plt
    # windows to show
    # Putting this in here to avoid messing up ROS
    import matplotlib.pyplot as plt
    plt.show()

    print("Done")
