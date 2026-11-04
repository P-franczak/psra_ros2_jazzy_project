#!/usr/bin/env python3
"""
Generate a smooth closed rally track for Gazebo Sim.

Usage:
    python3 generate_track.py --output models/rally_track/meshes

Only Python's standard library is required.
"""
import argparse
import math
from pathlib import Path

# ------------------------- Track parameters -------------------------
ROAD_WIDTH = 4.0
ROAD_THICKNESS = 0.10
BARRIER_HEIGHT = 0.50
BARRIER_THICKNESS = 0.15
BARRIER_BASE_Z = 0.05
SAMPLES_PER_METER = 20.0  # 0.1 m nominal spacing on straight sections

START = (0.0, 0.0)
START_HEADING = 0.0  # radians; +X direction

STRAIGHT_1 = 2.0
RIGHT_RADIUS = 5.0
STRAIGHT_2 = 3.0
LEFT_RADIUS = 2.5
STRAIGHT_3 = 15.0

# Increase this to make the return curve wider/smoother.
RETURN_HANDLE = 30.0


def add_line(points, p, heading, length):
    n = max(2, math.ceil(length * SAMPLES_PER_METER))
    for i in range(1, n + 1):
        t = i / n
        points.append((p[0] + length * t * math.cos(heading),
                       p[1] + length * t * math.sin(heading)))
    return points[-1], heading


def add_arc(points, p, heading, radius, angle_deg):
    """Append a circular arc. Positive angle = left, negative = right."""
    delta = math.radians(angle_deg)
    sign = 1.0 if delta > 0 else -1.0
    # Center lies one radius to the left/right of the current heading.
    cx = p[0] - sign * radius * math.sin(heading)
    cy = p[1] + sign * radius * math.cos(heading)

    rx, ry = p[0] - cx, p[1] - cy
    arc_length = abs(delta) * radius
    n = max(8, math.ceil(arc_length * SAMPLES_PER_METER))
    for i in range(1, n + 1):
        a = delta * i / n
        x = cx + rx * math.cos(a) - ry * math.sin(a)
        y = cy + rx * math.sin(a) + ry * math.cos(a)
        points.append((x, y))
    return points[-1], heading + delta


def bezier(p0, p1, p2, p3, t):
    u = 1.0 - t
    x = (u**3 * p0[0] + 3*u*u*t*p1[0] +
         3*u*t*t*p2[0] + t**3*p3[0])
    y = (u**3 * p0[1] + 3*u*u*t*p1[1] +
         3*u*t*t*p2[1] + t**3*p3[1])
    return (x, y)


def generate_centerline():
    points = [START]
    p, heading = START, START_HEADING

    p, heading = add_line(points, p, heading, STRAIGHT_1)
    p, heading = add_arc(points, p, heading, RIGHT_RADIUS, -90.0)
    p, heading = add_line(points, p, heading, STRAIGHT_2)
    p, heading = add_arc(points, p, heading, LEFT_RADIUS, 150.0)
    p, heading = add_line(points, p, heading, STRAIGHT_3)

    # Return curve: starts tangent to the last straight and ends tangent
    # to the initial +X direction. Its endpoints are exactly connected.
    p0 = p
    p3 = START
    p1 = (p0[0] + RETURN_HANDLE * math.cos(heading),
          p0[1] + RETURN_HANDLE * math.sin(heading))
    p2 = (p3[0] - RETURN_HANDLE, p3[1])

    # Dense sampling gives a smooth road mesh.
    n = 300
    for i in range(1, n + 1):
        points.append(bezier(p0, p1, p2, p3, i / n))

    # Remove the duplicate final point: the road is closed, and the first
    # point is already present. Keep the final segment between last and first.
    points.pop()
    return points


def unit(v):
    length = math.hypot(v[0], v[1])
    if length < 1e-10:
        raise ValueError("Zero-length segment encountered in centerline")
    return (v[0] / length, v[1] / length)


def offsets(points, distance):
    result = []
    count = len(points)
    for i, p in enumerate(points):
        prev_p = points[(i - 1) % count]
        next_p = points[(i + 1) % count]
        tangent = unit((next_p[0] - prev_p[0], next_p[1] - prev_p[1]))
        # Left-hand normal.
        normal = (-tangent[1], tangent[0])
        result.append((p[0] + distance * normal[0],
                       p[1] + distance * normal[1]))
    return result


def normal(a, b, c):
    ux, uy, uz = b[0]-a[0], b[1]-a[1], b[2]-a[2]
    vx, vy, vz = c[0]-a[0], c[1]-a[1], c[2]-a[2]
    nx, ny, nz = (uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx)
    length = math.sqrt(nx*nx + ny*ny + nz*nz)
    if length < 1e-12:
        return (0.0, 0.0, 1.0)
    return (nx/length, ny/length, nz/length)


def write_stl(path, triangles, solid_name):
    with open(path, "w", encoding="ascii") as f:
        f.write(f"solid {solid_name}\n")
        for a, b, c in triangles:
            nx, ny, nz = normal(a, b, c)
            f.write(f" facet normal {nx:.7g} {ny:.7g} {nz:.7g}\n")
            f.write("  outer loop\n")
            for v in (a, b, c):
                f.write(f"   vertex {v[0]:.7g} {v[1]:.7g} {v[2]:.7g}\n")
            f.write("  endloop\n endfacet\n")
        f.write(f"endsolid {solid_name}\n")


def quad(tris, a, b, c, d):
    tris.append((a, b, c))
    tris.append((a, c, d))


def build_road(points):
    half = ROAD_WIDTH / 2.0
    left = offsets(points, half)
    right = offsets(points, -half)
    z_top = BARRIER_BASE_Z
    z_bottom = z_top - ROAD_THICKNESS
    tris = []
    n = len(points)

    for i in range(n):
        j = (i + 1) % n
        # Top surface, counter-clockwise as viewed from above.
        a = (left[i][0], left[i][1], z_top)
        b = (right[i][0], right[i][1], z_top)
        c = (right[j][0], right[j][1], z_top)
        d = (left[j][0], left[j][1], z_top)
        quad(tris, a, b, c, d)

        # Bottom surface.
        ab = (left[i][0], left[i][1], z_bottom)
        bb = (right[i][0], right[i][1], z_bottom)
        cb = (right[j][0], right[j][1], z_bottom)
        db = (left[j][0], left[j][1], z_bottom)
        quad(tris, db, cb, bb, ab)

        # Outer and inner side faces.
        quad(tris, a, d, db, ab)
        quad(tris, c, b, bb, cb)

    return tris, left, right


def build_barriers(left, right):
    tris = []
    half_t = BARRIER_THICKNESS / 2.0
    z0 = BARRIER_BASE_Z
    z1 = z0 + BARRIER_HEIGHT
    n = len(left)

    for edge in (left, right):
        # Build each wall as a sequence of closed rectangular prisms.
        for i in range(n):
            j = (i + 1) % n
            direction = unit((edge[j][0]-edge[i][0],
                              edge[j][1]-edge[i][1]))
            normal_xy = (-direction[1], direction[0])
            def vertex(point, side, z):
                return (point[0] + side*half_t*normal_xy[0],
                        point[1] + side*half_t*normal_xy[1], z)

            a0 = vertex(edge[i], -1, z0)
            b0 = vertex(edge[i],  1, z0)
            c0 = vertex(edge[j],  1, z0)
            d0 = vertex(edge[j], -1, z0)
            a1 = vertex(edge[i], -1, z1)
            b1 = vertex(edge[i],  1, z1)
            c1 = vertex(edge[j],  1, z1)
            d1 = vertex(edge[j], -1, z1)

            quad(tris, a1, b1, c1, d1)  # top
            quad(tris, d0, c0, b0, a0)  # bottom
            quad(tris, a0, a1, d1, d0)
            quad(tris, b0, c0, c1, b1)
            quad(tris, a0, b0, b1, a1)
            quad(tris, d0, d1, c1, c0)

    return tris


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="models/rally_track/meshes",
                        help="Directory where STL meshes will be written")
    args = parser.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)

    centerline = generate_centerline()
    road_tris, left, right = build_road(centerline)
    barrier_tris = build_barriers(left, right)

    write_stl(out / "road.stl", road_tris, "road")
    write_stl(out / "barriers.stl", barrier_tris, "barriers")

    print(f"Generated {len(centerline)} centerline points")
    print(f"Road triangles: {len(road_tris)}")
    print(f"Barrier triangles: {len(barrier_tris)}")
    print(f"Output directory: {out.resolve()}")
    print("Track start/end position is closed; start tangent is +X.")


if __name__ == "__main__":
    main()
