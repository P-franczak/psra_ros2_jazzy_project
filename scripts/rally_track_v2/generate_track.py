#!/usr/bin/env python3
"""Generate a closed smooth rally track, continuous barriers and visual markings."""
import argparse, math
from pathlib import Path

# Geometry parameters (metres)
ROAD_WIDTH = 4.0
ROAD_THICKNESS = 0.10
ROAD_TOP_Z = 0.05
BARRIER_HEIGHT = 0.50
BARRIER_THICKNESS = 0.15
BARRIER_BASE_Z = ROAD_TOP_Z
LINE_WIDTH = 0.12
LINE_Z = ROAD_TOP_Z + 0.003
START_LINE_WIDTH = 0.18
SAMPLES_PER_METER = 20.0

STRAIGHT_1 = 2.0
RIGHT_RADIUS = 3.0
STRAIGHT_2 = 5.0
LEFT_RADIUS = 2.5
STRAIGHT_3 = 12.0
RETURN_HANDLE = 35.0
MITER_LIMIT = 1.0  # max miter length in multiples of offset distance


def generate_edge_lines(centerline, road_width):
    offset = road_width / 2 - 0.16

    left_edge = offset_polyline(
        centerline,
        offset,
        closed=True
    )

    right_edge = offset_polyline(
        centerline,
        -offset,
        closed=True
    )

    left_mesh = ribbon_mesh(
        left_edge,
        width=0.10,
        z=ROAD_TOP_Z + 0.004,
        closed=True
    )

    right_mesh = ribbon_mesh(
        right_edge,
        width=0.10,
        z=ROAD_TOP_Z + 0.004,
        closed=True
    )

    return left_mesh + right_mesh


def generate_start_line(centerline, road_width):

    x, y = centerline[0]

    rows = 2
    cols = 8

    cell_width = 0.22
    cell_height = road_width / cols

    white_mesh = []
    black_mesh = []

    z = ROAD_TOP_Z + 0.008

    for row in range(rows):
        for col in range(cols):

            x0 = x - rows * cell_width / 2 + row * cell_width
            x1 = x0 + cell_width

            y0 = y - road_width / 2 + col * cell_height
            y1 = y0 + cell_height

            a = (x0, y0, z)
            b = (x0, y1, z)
            c = (x1, y1, z)
            d = (x1, y0, z)

            if (row + col) % 2 == 0:
                quad(white_mesh, a, d, c, b)
            else:
                quad(black_mesh, a, d, c, b)

    return white_mesh, black_mesh


def generate_curbs(centerline):

    red_mesh = []
    white_mesh = []

    curb_width = 0.32
    curb_distance = ROAD_WIDTH / 2 - curb_width / 2 - 0.10

    z = ROAD_TOP_Z + 0.006

    distance = 0.0

    for i in range(1, len(centerline) - 1):

        p0 = centerline[i - 1]
        p1 = centerline[i]
        p2 = centerline[i + 1]

        v1 = (
            p1[0] - p0[0],
            p1[1] - p0[1]
        )

        v2 = (
            p2[0] - p1[0],
            p2[1] - p1[1]
        )

        cross = v1[0] * v2[1] - v1[1] * v2[0]

        curvature = abs(cross)

        # Pomijamy odcinki proste.
        if curvature < 0.0001:
            distance += math.dist(p1, p2)
            continue

        length = math.hypot(*v2)

        nx = -v2[1] / length
        ny = v2[0] / length

        # Wewnętrzna strona zakrętu.
        direction = 1 if cross > 0 else -1

        cx = p1[0] + direction * nx * curb_distance
        cy = p1[1] + direction * ny * curb_distance

        # Pozycja następnego punktu.
        nx2 = -v2[1] / length
        ny2 = v2[0] / length

        ex = p2[0] + direction * nx2 * curb_distance
        ey = p2[1] + direction * ny2 * curb_distance

        half = curb_width / 2

        a = (cx - half * nx, cy - half * ny, z)
        b = (cx + half * nx, cy + half * ny, z)
        c = (ex + half * nx, ey + half * ny, z)
        d = (ex - half * nx, ey - half * ny, z)

        target = red_mesh if int(distance / 0.65) % 2 else white_mesh

        quad(target, a, b, c, d)

        distance += math.dist(p1, p2)

    return red_mesh, white_mesh

def generate_barrier_colors(edge):

    red_mesh = []
    white_mesh = []

    inner = offset_polyline(
        edge,
        -BARRIER_THICKNESS / 2
    )

    outer = offset_polyline(
        edge,
        BARRIER_THICKNESS / 2
    )

    distance = 0.0

    for i in range(len(edge)):

        j = (i + 1) % len(edge)

        segment_length = math.dist(edge[i], edge[j])

        middle = distance + segment_length / 2

        distance += segment_length

        target = (
            red_mesh
            if int(middle / 1.5) % 2
            else white_mesh
        )

        z0 = BARRIER_BASE_Z
        z1 = z0 + BARRIER_HEIGHT

        a = (inner[i][0], inner[i][1], z0)
        b = (outer[i][0], outer[i][1], z0)
        c = (outer[j][0], outer[j][1], z0)
        d = (inner[j][0], inner[j][1], z0)

        A = (a[0], a[1], z1)
        B = (b[0], b[1], z1)
        C = (c[0], c[1], z1)
        D = (d[0], d[1], z1)

        quad(target, A, B, C, D)
        quad(target, b, c, C, B)
        quad(target, a, A, D, d)

    return red_mesh, white_mesh


def add_line(points, p, heading, length):
    n = max(2, math.ceil(length * SAMPLES_PER_METER))
    for i in range(1, n + 1):
        t = i / n
        points.append((p[0] + length*t*math.cos(heading),
                       p[1] + length*t*math.sin(heading)))
    return points[-1], heading


def add_arc(points, p, heading, radius, angle_deg):
    delta = math.radians(angle_deg)
    sign = 1.0 if delta > 0 else -1.0
    cx = p[0] - sign*radius*math.sin(heading)
    cy = p[1] + sign*radius*math.cos(heading)
    rx, ry = p[0]-cx, p[1]-cy
    n = max(12, math.ceil(abs(delta)*radius*SAMPLES_PER_METER))
    for i in range(1, n+1):
        a = delta*i/n
        points.append((cx + rx*math.cos(a)-ry*math.sin(a),
                       cy + rx*math.sin(a)+ry*math.cos(a)))
    return points[-1], heading+delta


def bezier(p0,p1,p2,p3,t):
    u=1-t
    return (u**3*p0[0]+3*u*u*t*p1[0]+3*u*t*t*p2[0]+t**3*p3[0],
            u**3*p0[1]+3*u*u*t*p1[1]+3*u*t*t*p2[1]+t**3*p3[1])


def centerline():
    pts=[(0.0,0.0)]
    p,h=(0.0,0.0),0.0
    p,h=add_line(pts,p,h,STRAIGHT_1)
    p,h=add_arc(pts,p,h,RIGHT_RADIUS,-90.0)
    p,h=add_line(pts,p,h,STRAIGHT_2)
    p,h=add_arc(pts,p,h,LEFT_RADIUS,150.0)
    p,h=add_line(pts,p,h,STRAIGHT_3)
    p0,p3=p,(0.0,0.0)
    p1=(p0[0]+RETURN_HANDLE*math.cos(h),p0[1]+RETURN_HANDLE*math.sin(h))
    p2=(p3[0]-RETURN_HANDLE,p3[1])
    # Dense Bezier sampling; final point is omitted because it equals start.
    for i in range(1,1201):
        pts.append(bezier(p0,p1,p2,p3,i/1201))
    return pts


def unit(v):
    l=math.hypot(v[0],v[1])
    if l < 1e-10: raise ValueError("Zero-length centerline segment")
    return v[0]/l,v[1]/l


def offset_polyline(points, distance, closed=True):
    """Offset with miter joins, preventing gaps between independent wall blocks."""
    n=len(points); out=[]
    for i,p in enumerate(points):
        if closed:
            prev=points[(i-1)%n]; nxt=points[(i+1)%n]
        else:
            prev=points[max(0,i-1)]; nxt=points[min(n-1,i+1)]
        t0=unit((p[0]-prev[0],p[1]-prev[1]))
        t1=unit((nxt[0]-p[0],nxt[1]-p[1]))
        n0=(-t0[1],t0[0]); n1=(-t1[1],t1[0])
        m=unit((n0[0]+n1[0],n0[1]+n1[1]))
        denom=m[0]*n1[0]+m[1]*n1[1]
        if abs(denom)<1e-5: scale=distance
        else: scale=distance/denom
        scale=max(-MITER_LIMIT*abs(distance),min(MITER_LIMIT*abs(distance),scale))
        out.append((p[0]+m[0]*scale,p[1]+m[1]*scale))
    return out


def norm(a,b,c):
    u=(b[0]-a[0],b[1]-a[1],b[2]-a[2]); v=(c[0]-a[0],c[1]-a[1],c[2]-a[2])
    q=(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0])
    l=math.sqrt(sum(x*x for x in q))
    return (0,0,1) if l<1e-12 else tuple(x/l for x in q)


def write_stl(path,tris,name):
    with open(path,"w",encoding="ascii") as f:
        f.write(f"solid {name}\n")
        for a,b,c in tris:
            nx,ny,nz=norm(a,b,c)
            f.write(f" facet normal {nx:.7g} {ny:.7g} {nz:.7g}\n  outer loop\n")
            for v in (a,b,c): f.write(f"   vertex {v[0]:.7g} {v[1]:.7g} {v[2]:.7g}\n")
            f.write("  endloop\n endfacet\n")
        f.write(f"endsolid {name}\n")


def quad(t,a,b,c,d):
    t.extend(((a,b,c),(a,c,d)))


def road_mesh(left,right):
    t=[]; zb=ROAD_TOP_Z-ROAD_THICKNESS; zt=ROAD_TOP_Z; n=len(left)
    for i in range(n):
        j=(i+1)%n
        a=(left[i][0],left[i][1],zt); b=(right[i][0],right[i][1],zt)
        c=(right[j][0],right[j][1],zt); d=(left[j][0],left[j][1],zt)
        quad(t,a,b,c,d)
        ab=(a[0],a[1],zb); bb=(b[0],b[1],zb); cb=(c[0],c[1],zb); db=(d[0],d[1],zb)
        quad(t,db,cb,bb,ab); quad(t,a,d,db,ab); quad(t,c,b,bb,cb)
    return t


def wall_mesh(edge):
    """One continuous extruded wall ribbon, with shared vertices at every join."""
    inner=offset_polyline(edge,-BARRIER_THICKNESS/2,True)
    outer=offset_polyline(edge, BARRIER_THICKNESS/2,True)
    z0=BARRIER_BASE_Z; z1=z0+BARRIER_HEIGHT; n=len(edge); t=[]
    for i in range(n):
        j=(i+1)%n
        a=(inner[i][0],inner[i][1],z0); b=(outer[i][0],outer[i][1],z0)
        c=(outer[j][0],outer[j][1],z0); d=(inner[j][0],inner[j][1],z0)
        A=(a[0],a[1],z1); B=(b[0],b[1],z1)
        C=(c[0],c[1],z1); D=(d[0],d[1],z1)
        quad(t,A,B,C,D)       # top
        quad(t,d,c,b,a)       # bottom
        quad(t,a,A,D,d)       # inner side
        quad(t,b,c,C,B)       # outer side
    return t


def ribbon_mesh(points,width,z,closed=True):
    left=offset_polyline(points,width/2,closed)
    right=offset_polyline(points,-width/2,closed)
    n=len(points); t=[]
    count=n if closed else n-1
    for i in range(count):
        j=(i+1)%n
        a=(left[i][0],left[i][1],z); b=(right[i][0],right[i][1],z)
        c=(right[j][0],right[j][1],z); d=(left[j][0],left[j][1],z)
        quad(t,a,b,c,d)
    return t


def start_line_mesh(pts):
    # Stripe perpendicular to initial direction (+X), spanning road width.
    x,y=pts[0]
    # Initial tangent is +X, therefore normal is +Y.
    half=ROAD_WIDTH/2
    x0,x1=x-START_LINE_WIDTH/2,x+START_LINE_WIDTH/2
    y0,y1=y-half,y+half
    z=LINE_Z
    a=(x0,y0,z); b=(x0,y1,z); c=(x1,y1,z); d=(x1,y0,z)
    return [(a,b,c),(a,c,d)]


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--output",default="models/rally_track/meshes")
    args=ap.parse_args(); out=Path(args.output); out.mkdir(parents=True,exist_ok=True)
    pts=centerline()
    left=offset_polyline(pts,ROAD_WIDTH/2,True)
    right=offset_polyline(pts,-ROAD_WIDTH/2,True)

    edge_mesh = generate_edge_lines(
        pts,
        ROAD_WIDTH
    )

    write_stl(
        out / "edge_lines.stl",
        edge_mesh,
        "edge_lines"
    )

    start_white, start_black = generate_start_line(
        pts,
        ROAD_WIDTH
    )

    red_left, white_left = generate_barrier_colors(left)
    red_right, white_right = generate_barrier_colors(right)

    write_stl(
        out / "barriers_red.stl",
        red_left + red_right,
        "barriers_red"
    )

    write_stl(
        out / "barriers_white.stl",
        white_left + white_right,
        "barriers_white"
    )

    write_stl(out / "start_white.stl", start_white, "start_white")
    write_stl(out / "start_black.stl", start_black, "start_black")

    write_stl(out/"road.stl",road_mesh(left,right),"road")
    walls=wall_mesh(left)+wall_mesh(right)
    write_stl(out/"barriers.stl",walls,"barriers")
    write_stl(out/"centerline.stl",ribbon_mesh(pts,LINE_WIDTH,LINE_Z,True),"centerline")
    write_stl(out/"start_line.stl",start_line_mesh(pts),"start_line")
    print(f"OK: {len(pts)} centerline samples")
    print("Generated road.stl, barriers.stl, centerline.stl, start_line.stl")
    print(f"Output: {out.resolve()}")

if __name__=="__main__": main()
