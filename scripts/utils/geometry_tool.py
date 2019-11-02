#!/usr/bin/env python

import numpy as np

def dist_point_to_line(line_start, line_end, point):
    '''
    Function to calculate perpendicular distance from a point to line, the
    line is given by two points.
    :param line_start: the start point of a line, list [latitude, longitude]
    :param line_end: the end point of a line, list [latitude, longitude]
    :param point: the point outside the line, list [latitude, longitude]
    :return: distance, float
    '''

    dist = 0.
    start_ary = np.asarray(line_start)
    end_ary = np.asarray(line_end)
    point_ary = np.asarray(point)

    dist = np.linalg.norm(np.cross(end_ary - start_ary, start_ary - point_ary)) / np.linalg.norm(end_ary - start_ary)

    return dist