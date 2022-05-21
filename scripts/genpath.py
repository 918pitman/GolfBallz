import os
import math
import pathlib
import argparse
from fastkml import kml
from shapely.geometry import *
from shapely.ops import *
from lib2to3.pgen2.parse import ParseError
import geopy.distance

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

def rotatePolygon(poly, theta):
    theta = math.radians(theta)
    rotatedPolygon = []
    for p in poly :
        rotatedPolygon.append((p[0] * math.cos(theta) - p[1] * math.sin(theta),
                               p[0] * math.sin(theta) + p[1] * math.cos(theta)))
    return rotatedPolygon

def parse_kml_file(path):
    with open(path, 'rb') as file:
        doc = file.read()
    k = kml.KML()
    k.from_string(doc)
    outer = list(k.features())
    inner = list(outer[0].features())
    inclusion_poly = []
    exclusion_poly = []
    for geo in inner:
        poly = [(point[1], point[0]) for point in list(geo.geometry.exterior.coords)]
        if geo.name == "Include":
            inclusion_poly.append(poly)
        if geo.name == "Exclude":
            exclusion_poly.append(poly)
    return inclusion_poly, exclusion_poly

def parse_waypoints_file(path):
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
                if polyindex == 0:
                    if last_type == "5001":
                        inclusion_poly.append(poly.copy())
                    if last_type == "5002":
                        exclusion_poly.append(poly.copy())
                    poly.clear()
                poly.append((latitude, longitude))
                if i+1 == len(Lines):
                    if last_type == "5001":
                        inclusion_poly.append(poly.copy())
                    if last_type == "5002":
                        exclusion_poly.append(poly.copy())
                last_type = linetype
    return inclusion_poly, exclusion_poly

def parse_file(path):
    file_ext = pathlib.Path(path).suffix
    if file_ext == ".kml":
        return parse_kml_file(path)
    elif file_ext == ".waypoints":
        return parse_waypoints_file(path)
    else:
        raise ParseError(f"Input file format unsupported: {file_ext}\n\rMust be .waypoint or .kml file")

def get_poly_segments(poly):
    segments = []
    for i, p in enumerate(poly):
        one_i = i + 1
        if one_i == len(poly):
            segments.append([poly[i], poly[0]])
        else:
            segments.append([poly[i], poly[one_i]])
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

def get_divided_line(seg, space):
    line = LineString(seg)
    remainder = math.fmod(line.length, space)
    count = int((line.length-remainder)/space)+1
    result = [seg[0]]
    intdivA = line.interpolate(remainder/2).coords[0]
    intdivB = LineString((seg[1], seg[0])).interpolate(remainder/2).coords[0]
    intdivLine = LineString([intdivA, intdivB])
    for i in range(count):
        result.append(intdivLine.interpolate(i*space).coords[0])
    result.append(seg[1])
    return result
    
def get_stride_lines(box, stride):
    boxlines = get_poly_segments(box)
    startpts = get_divided_line(boxlines[0], stride)
    finishpts = get_divided_line(boxlines[2], stride)
    finishpts.reverse()
    result = []
    for i in range(len(startpts)):
        result.append([startpts[i], finishpts[i]])
    return result

def trim_lines(lines, poly):
    result = []
    fence = Polygon(poly)
    for l in lines:
        trimmed = fence.intersection(LineString(l))
        if trimmed.geom_type == "LineString" and len(trimmed.coords) == 2:
            result.append([trimmed.coords[0], trimmed.coords[1]])
    return result

def get_rotated_bbox(poly, angle):
    rotated = rotatePolygon(poly, angle)
    rotated_box = get_bound_box(rotated)
    return rotatePolygon(rotated_box, (angle*-1))

def order_line(line, start):
    i = line.index(start)
    return line[i:] + line[:i]

def distance_sort(route, start):
    return LineString([start, route[0]]).length

def get_shortest_route(line, poly):
    route = []
    tool = LineString(line)
    exclusion = Polygon(poly)
    intersections = tool.intersection(exclusion)
    if intersections.geom_type == "LineString" and len(intersections.coords) == 2:
        start = intersections.coords[0]
        cutpoly = split(exclusion, tool)
        if cutpoly.geoms[0].length <= cutpoly.geoms[1].length:
            shortest = cutpoly.geoms[0].exterior.coords[1:]
        else:
            shortest = cutpoly.geoms[1].exterior.coords[1:]
        ordered = order_line(shortest, start)        
        if ordered[1] == intersections.coords[1]:
            # Route needs reversed and reordered
            ordered.reverse()
            route = order_line(ordered, start)
        else:
            route = ordered
    return route

def get_reroute(line, polys, origin):
    routes = []
    for p in polys:
        exc = to_cartesian(origin, p)
        shortest_rt = get_shortest_route(line, exc)
        if len(shortest_rt) != 0:
            routes.append(shortest_rt)
    routes.sort(key=lambda route: distance_sort(route, line[0]))
    pathpts = [line[0]]
    for r in routes:
        pathpts += r
    pathpts.append(line[1])
    return pathpts

def export_kml_mission(path, stem, poly):
    k = kml.KML()
    d = kml.Document(ns="{http://www.opengis.net/kml/2.2}", name=stem)
    p = kml.Placemark(ns="{http://www.opengis.net/kml/2.2}", name="Mission")
    p.geometry = LineString([(point[1], point[0], 0) for point in poly])
    d.append(p)
    k.append(d)
    with open(path, 'w') as file:
        file.write(k.to_string(prettyprint=True))

def form_writeline(index, home, lat, lon):
    return f"{index}\t{home}\t0\t16\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t{lat}\t{lon}\t100.000000\t1\n"

def export_waypoint_mission(path, poly):
    with open(path, 'w') as file:
        file.write("QGC WPL 110"+"\n")
        file.write(form_writeline("0", "1", "36.0844996", "-95.7699999"))
        for i, p in enumerate(poly):
            file.write(form_writeline(str(i+1), "0", f"{p[0]:.8f}", f"{p[1]:.8f}"))

def export_mission(path, mission):
    file_ext = pathlib.Path(path).suffix
    if file_ext == ".kml":
        export_kml_mission(path, pathlib.PurePath(path).stem, mission)
    elif file_ext == ".waypoints":
        export_waypoint_mission(path, mission)
    else:
        raise ParseError(f"Output file format unsupported: {file_ext}\n\rMust be .waypoint or .kml file")

parser = argparse.ArgumentParser(description='Generate a mission with inclusion and exclusion zones')
parser.add_argument('-i', dest='include', help='Path to the waypoint file input', required=True)
parser.add_argument('-o', dest='output', help='Path to the waypoint file output', required=True)
args = parser.parse_args()

included, excluded = parse_file(args.include)

num_in = len(included)
num_ex = len(excluded)
print("Found "+str(num_in)+" inclusion polygons")
print("Found "+str(num_ex)+" exclusion polygons")

if num_in != 1:
    raise ParseError("This script requires the waypoint file to contain exactly one inclusion polygon!")
print("Creating Mission")
origin = get_origin(included[0])
cartesian = to_cartesian(origin, included[0])
box = get_rotated_bbox(cartesian, 0)
strides = get_stride_lines(box, 1)
trimmed = trim_lines(strides, cartesian)

linepath = []
flip = 0
for i,t in enumerate(trimmed):
    r = get_reroute(t, excluded, origin)
    if (flip+i) % 2 == 0:
        r.reverse()
    linepath += r

mission = to_ellipsoid(origin, linepath)
print(f"Writing Mission to {args.output}")
export_mission(args.output, mission)
print("Done")
print(f"Total Mission Distance is {str((LineString(linepath).length)/1609.34)} miles")