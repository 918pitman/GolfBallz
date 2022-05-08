import argparse
from cv2 import split
import geopy.distance
import math
import numpy as np
from collections import OrderedDict

parser = argparse.ArgumentParser(description='Generate a mission with inclusion and exclusion zones')
parser.add_argument('-i', dest='include', help='Path to the waypoint file containing the included geometry', required=True)
#parser.add_argument('-e', dest='exclude', help='Path to the waypoint file containing the excluded geometry', required=True)
#parser.add_argument('-o', dest='output', help='Path to write the waypoint file containing generated mission', required=True)
args = parser.parse_args()

stride = 0.5
bearing = 0

def get_relative_coord(start, distance, bearing):
    return geopy.distance.distance(meters=distance).destination(start, bearing=bearing)

def get_poly(path):
    poly = []
    with open(path, 'r') as file:
        Lines = list(OrderedDict.fromkeys(file.read().splitlines()))
        for line in Lines:
            if line[0] != '#':
                latlon = line.split(' ')
                poly.append( ( float(latlon[0]), float(latlon[1]) ) )
    return poly

def parse_fence_file(path):
    inclusions = []
    exclusions = []
    with open(path, 'r') as file:
        Lines = file.read().splitlines()
        for line in Lines:
            l = line.split('\t')
            if len(l) == 12:
                if l[3] == "5001":
                    inclusions.append(l)
                elif l[3] == "5002":
                    exclusions.append(l)
    return inclusions, exclusions

def get_polygons():
    print("TODO")

def get_bound_box(poly):
    N, E = poly[0]
    S, W = poly[0]
    for p in poly:
        if p[0] > N:
            N = p[0]
        if p[0] < S:
            S = p[0]
        if p[1] > E:
            E = p[1]
        if p[1] < W:
            W = p[1]

    return [(N,W),(N,E),(S,E),(S,W)]

include, exclude = parse_fence_file(args.include)

in_polys = get_polygons(include)
_polys = get_polygons(exclude)

# in_poly = get_poly(args.include)

# bounds = get_bound_box(in_poly)

# with open("../test/bounds.poly", 'w') as file:
#     for p in bounds:
#         file.write( str(p[0]) + ' ' + str(p[1]) + '\n' )

# ex_poly = get_poly(args.exclude)
 

# p1 = (36.0845, -95.77, 208)
# p2 = (36.0850, -95.77, 208)

# flat_distance = geopy.distance.geodesic(p1, p2, ellipsoid='WGS-84').meters
# flat_distance = geopy.distance.geodesic((36.0845, -95.77, 208), (36.0850, -95.77, 208), ellipsoid='WGS-84').meters
# print(flat_distance)

# print(get_relative_coord(55.480286253047204, 0))