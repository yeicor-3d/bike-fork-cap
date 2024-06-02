import os
from math import *

from build123d import *
from build123d import export_stl

# %%

# General parameters
tol = 0.2 * MM  # Tolerance for operations
eps = 0.0001 * MM  # Epsilon for operations
min_wall = 0.4 * MM  # Minimum wall thickness on XY (common for FDM 3D printing)
wall = 3 * min_wall  # perimeters on XY

# Bike fork cap parameters
cap_radius = 40.5 / 2 * MM
cap_height = 42 * MM  # Including the head
cap_head_radius = 45 / 2 * MM
cap_head_height = 15 * MM
cap_to_support_taper = -5  # degrees
cap_support_top_offset = -4 * MM
cap_support_profile = import_svg('profile.svg', is_inkscape_label=True).wire()
cap_support_length = 2 * CM
cap_support_angle = 30  # degrees
cap_cut_hole_radius = 0  # 4 * MM
cap_cut_hole_offset_xz = (-13 * MM, -(cap_cut_hole_radius + wall) * MM)


def core_filled(offset: float = 0):
    with BuildPart() as _core_filled:  # Useful to offset a wrapper and to cut the support!
        with BuildSketch():
            Circle(cap_radius + offset)
        shank_height = cap_height - cap_head_height
        shank_height_flat = shank_height_tapered = shank_height / 2
        extrude(amount=shank_height_flat)
        shank_taper_angle = degrees(-atan2(cap_head_radius - cap_radius, shank_height_tapered))
        extrude(faces().group_by(Axis.Z)[-1], amount=shank_height_tapered, taper=shank_taper_angle)
        extrude(faces().group_by(Axis.Z)[-1], amount=cap_head_height + offset)
        split(bisect_by=Plane.YZ, keep=Keep.BOTTOM)  # Rectangular support half
        extrude(faces().group_by(Axis.X)[-1], amount=cap_head_radius + offset, taper=cap_to_support_taper)
        cap_hole_sketch_to_cut = None
        if cap_cut_hole_radius > 0:
            bb = _core_filled.part.bounding_box()
            with BuildSketch(Plane.XZ.shift_origin(
                    (bb.max.X + cap_cut_hole_offset_xz[0], 0, bb.max.Z + cap_cut_hole_offset_xz[1])).offset(
                -(bb.min.Y - wall))) as cap_hole_sketch_to_cut:
                Circle(cap_cut_hole_radius)
                Rectangle(2 * cap_cut_hole_radius, bb.size.Z, align=(Align.CENTER, Align.MAX))
    return _core_filled, cap_hole_sketch_to_cut


with BuildLine() as support_line:
    bb = cap_support_profile.bounding_box()
    add(cap_support_profile.translate((-bb.min.X, -(bb.max.Y + wall))))
    mirror(about=Plane.YZ)
del cap_support_profile

core_filled_base, cap_hole_sketch_to_cut = core_filled()
core_filled_extra = core_filled(wall)[0]
m = core_filled_base.part.bounding_box().max
support_loc = Pos(m.X, 0, m.Z + cap_support_top_offset) * Plane.YZ.location
extrude_dir = Vector(cos(radians(cap_support_angle)), 0, sin(radians(cap_support_angle))).normalized()
extrude_in_length = wall / cos(radians(cap_support_angle))
with BuildPart() as support_cut:
    with BuildSketch(support_loc) as sk:
        with BuildLine():
            add(support_line)
            min_x_point = support_line.line.vertices().group_by(Axis.X)[0].group_by(Axis.Z)[0].vertex().center()
            max_x_point = support_line.line.vertices().group_by(Axis.X)[-1].group_by(Axis.Z)[0].vertex().center()
            min_conn = support_line.line @ 0
            max_conn = support_line.line @ 1
            Polyline(min_conn,
                     Vector(min_x_point.X, min_conn.Y, min_x_point.Z),
                     min_x_point - Vector(0, cap_height, 0),
                     max_x_point - Vector(0, cap_height, 0),
                     Vector(max_x_point.X, max_conn.Y, max_x_point.Z),
                     max_conn)
        make_face()
    extrude(sk.sketch, amount=extrude_in_length, dir=-extrude_dir, both=True)
    del sk

with BuildPart() as support:
    with BuildSketch(support_loc) as sk:
        with BuildLine():
            add(support_line)
            inside_wire = Wire(edges())
            offset(amount=wall, side=Side.RIGHT)  # Make sure this side is the outside...
            outside_wire = Wire([e for e in edges() if min([e.distance(e2) for e2 in inside_wire.edges()]) > eps])
        make_face()
        with BuildLine():  # Round the corners
            p1 = outside_wire @ 0
            p2 = inside_wire @ 1
            t1 = -(outside_wire % 0)
            a = JernArc(p1, t1, (p2 - p1).length / 2, 180)
            if (a @ 1 - p2).length > eps:
                Line(a @ 1, p2)
            Line(p2, p1)
        make_face()
        with BuildLine():  # Round the corners
            p1 = inside_wire @ 0
            p2 = outside_wire @ 1
            t1 = -(inside_wire % 0)
            a = JernArc(p1, t1, (p2 - p1).length / 2, 180)
            if (a @ 1 - p2).length > eps:
                Line(a @ 1, p2)
            Line(p2, p1)
        make_face()
        del inside_wire, outside_wire
    extrude(sk.sketch, amount=cap_support_length, dir=extrude_dir)
    del sk
    loft_base = faces().group_by(Axis.X)[-1].face()
    bb = loft_base.bounding_box()
    rotate_from = Vector(bb.center().X, bb.center().Y, bb.max.Z)
    assert fabs(rotate_from.Y) <= eps, rotate_from
    loft_top = loft_base.rotate(Axis(rotate_from, Vector(0, 1, 0)), -cap_support_angle)
    loft_top = loft_top.translate(  # HACK!
        (0, 0, (bb.size.Z - loft_top.bounding_box().size.Z) / 2))
    loft([loft_base, loft_top])
    del loft_base, loft_top
    add(core_filled_base, mode=Mode.SUBTRACT)
del support_line, support_loc

with (BuildPart() as bike_fork_cap):
    add(core_filled_extra)
    add(core_filled_base, mode=Mode.SUBTRACT)  # HACK: alternative to offset that does not crash
    del core_filled_extra, core_filled_base
    add(support_cut, mode=Mode.SUBTRACT)
    del support_cut
    bb = bike_fork_cap.part.bounding_box()
    cut_from_support = Box(bb.size.X, bb.size.Y, bb.size.Z, align=Align.MAX, mode=Mode.PRIVATE
                           ).translate((bb.max.X, bb.max.Y, bb.min.Z))
    add(support.part - cut_from_support)
    del support, cut_from_support
    if cap_hole_sketch_to_cut is not None:
        add(extrude(to_extrude=cap_hole_sketch_to_cut.sketch, amount=-cap_head_radius), mode=Mode.SUBTRACT)
        del cap_hole_sketch_to_cut

# -- export/show boilerplate --

if os.getenv('export_stl'):
    print('Exporting STL file...')
    export_stl(bike_fork_cap.part, 'bike_fork_cap.stl')

if os.getenv('export_step'):
    print('Exporting STEP file...')
    export_step(bike_fork_cap.part, 'bike_fork_cap.step')

try:
    from yacv_server import *

    show_all()

    with open('reference.glb', 'rb') as f:
        show(f.read(), names=['reference'], auto_clear=False)

    if os.getenv('export_yacv'):
        print('Exporting YACV file...')
        export_all('.', lambda name, obj: name == 'bike_fork_cap')
except BaseException as e:
    print(f'yacv_server not found or another error happened, skipping visualization: {e}')
