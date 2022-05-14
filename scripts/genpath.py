import os
import argparse
import math
from shapely.geometry import LineString, Polygon
from lib2to3.pgen2.parse import ParseError
import geopy.distance
from collections import OrderedDict

parser = argparse.ArgumentParser(description='Generate a mission with inclusion and exclusion zones')
parser.add_argument('-i', dest='include', help='Path to the waypoint file input', required=True)
parser.add_argument('-o', dest='output', help='Path to the waypoint file output', required=True)
args = parser.parse_args()

def get_bound_box(poly):
    x = [p[0] for p in poly]
    y = [p[1] for p in poly]
    x_min = min(x)
    x_max = max(x)
    y_min = min(y)
    y_max = max(y)
    return [(x_max, y_max),
            (x_max, y_min),
            (x_min, y_min),
            (x_min, y_max)]

def get_center_point(poly):
    p_num = len(poly)
    x_sum = math.fsum([p[0] for p in poly])
    y_sum = math.fsum([p[1] for p in poly])
    return ((x_sum / p_num), (y_sum / p_num))

def rotatePolygon(poly, theta):
    theta = math.radians(theta)
    rotatedPolygon = []
    for p in poly :
        rotatedPolygon.append((p[0] * math.cos(theta) - p[1] * math.sin(theta),
                               p[0] * math.sin(theta) + p[1] * math.cos(theta)))
    return rotatedPolygon

def get_poly(path):
    poly = []
    with open(path, 'r') as file:
        Lines = list(OrderedDict.fromkeys(file.read().splitlines()))
        for line in Lines:
            if line[0] != '#':
                latlon = line.split(' ')
                poly.append((float(latlon[0]), float(latlon[1])))
    return poly

def parse_fence_file(path):

    last_type = ""
    inclusion_poly = []
    exclusion_poly = []
    poly = []

    with open(path, 'r') as file:
        Lines = file.read().splitlines()
        for i, line in enumerate(Lines):
            l = line.split('\t')
            
            if len(l) == 12:
                linetype = l[3]
                latitude = float(l[8])
                longitude = float(l[9])
                polyindex = float(l[10])
                
                # Beginning of new poly
                if polyindex == 0:
                    if last_type == "5001":
                        inclusion_poly.append(poly.copy())
                    if last_type == "5002":
                        exclusion_poly.append(poly.copy())
                    poly.clear()

                poly.append((latitude, longitude))

                # Check for end of file since polys are appended on every 0 index
                if i+1 == len(Lines):
                    if last_type == "5001":
                        inclusion_poly.append(poly.copy())
                    if last_type == "5002":
                        exclusion_poly.append(poly.copy())

                last_type = linetype

    return inclusion_poly, exclusion_poly
                    
def get_poly_segments(poly):
    segments = []
    for i, p in enumerate(poly):
        one_i = i + 1
        if one_i == len(poly):
            segments.append((poly[i], poly[0]))
        else:
            segments.append((poly[i], poly[one_i]))
    return segments

def get_origin(poly):
    return ((min([lat[0] for lat in poly]), min([lon[1] for lon in poly])))

def to_cartesian(origin, poly):
    cart_poly = []
    for p in poly:
        x = geopy.distance.geodesic((p[0], origin[1]), p, ellipsoid='WGS-84').meters
        y = geopy.distance.geodesic((origin[0], p[1]), p, ellipsoid='WGS-84').meters
        cart_poly.append((x, y))
    return cart_poly

def to_ellipsoid(origin, poly):
    elli_poly = []
    for p in poly:
        lat = geopy.distance.distance(meters=p[1]).destination(origin, bearing=0)[0]
        lon = geopy.distance.distance(meters=p[0]).destination(origin, bearing=90)[1]
        elli_poly.append((lat, lon))
    return elli_poly

def get_divided_line(seg, stride):
    line = LineString(seg)
    print(f"Length of line is: {str(line.length)}")
    remainder = math.fmod(line.length, stride)
    print(f"Remainder after divided by {str(stride)} is {str(remainder)}")
    numstrides = int((line.length-remainder)/stride)
    print(f"There is {str(numstrides)} strides that can fit inside the line")

    result = [seg[0]]
    intdivA = line.interpolate(remainder/2).coords[0]
    intdivB = LineString((seg[1], seg[0])).interpolate(remainder/2).coords[0]
    intdivLine = LineString([intdivA, intdivB])
    for i in range(numstrides):
        result.append(intdivLine.interpolate(i*stride).coords[0])
    result.append(seg[1])

    print("Result")
    for p in result:
        print(p)

    return result



    
def get_stride_lines(box):
    boxlines = get_poly_segments(box)
    startseg = boxlines[0]
    finishseg = boxlines[2]
    return get_divided_line(startseg, 5)

def get_rotated_bbox(poly, angle):
    rotated = rotatePolygon(poly, angle)
    rotated_box = get_bound_box(rotated)
    return rotatePolygon(rotated_box, (angle*-1))

def get_file_end(path):
    with open(path, "rb") as file:
        try:
            file.seek(-2, os.SEEK_END)
            while file.read(1) != b'\n':
                file.seek(-2, os.SEEK_CUR)
        except OSError:
            file.seek(0)
        return file.readline().decode()

def form_writeline(index, home, lat, lon):
    return f"{index}\t{home}\t0\t16\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t{lat}\t{lon}\t100.000000\t1\n"

def add_poly_to_mission(path, poly):
    mission_idx = int(get_file_end(path).split('\t')[0]) + 1
    with open(path , 'a') as file:
        for i, p in enumerate(poly):
            file.write(form_writeline(str(mission_idx+i), "0", f"{p[0]:.8f}", f"{p[1]:.8f}"))



included, excluded = parse_fence_file(args.include)

num_in = len(included)
num_ex = len(excluded)
print("Found "+str(num_in)+" inclusion polygons")
print("Found "+str(num_ex)+" exclusion polygons")
if num_in != 1:
    raise ParseError("This script requires the waypoint file to contain exactly one inclusion polygon!")

included_seg = get_poly_segments(included[0])

line_out = []

with open(args.output, 'w') as file:
    file.write("QGC WPL 110"+"\n")
    file.write(form_writeline("0", "1", "36.0844996", "-95.7699999"))

origin = get_origin(included[0])


cartesian = to_cartesian(origin, included[0])

box = get_rotated_bbox(cartesian, 20)
for p in box:
    print(p)

_mission = get_stride_lines(box)
mission = to_ellipsoid(origin, _mission)
add_poly_to_mission(args.output, mission)



# new_points = to_ellipsoid(origin, cartesian)
# center = get_center_point(included[0])


# elli_box = to_ellipsoid(origin, box)
# add_poly_to_mission(args.output, elli_box)
