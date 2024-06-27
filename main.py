import os

from build123d import *
from build123d import export_stl

# %%

reference: ShapeList[Shell] = Mesher().read('reference.stl')[0].shells()
print('Loaded reference mesh with shells:', [shell.area for shell in reference.shells()])
reference: Shell = reference.shells().sort_by(SortBy.AREA)[-1].shell()
print('Loaded reference shell areas:', reference.area)

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
# cap_support_profile = import_svg('profile.svg', is_inkscape_label=True).wire()
cap_support_length = 2 * CM
cap_support_angle = 30  # degrees
cap_cut_hole_radius = 0  # 4 * MM
cap_cut_hole_offset_xz = (-13 * MM, -(cap_cut_hole_radius + wall) * MM)

# Sample the bottom of the reference to generate a curve
NUM_SAMPLES = 10
reference_bb = reference.bounding_box()
all_points = []
for i in range(NUM_SAMPLES):
    y_pct = 0.54 + 0.185 * i / NUM_SAMPLES
    y_abs = reference_bb.min.Y + y_pct * reference_bb.size.Y
    bottom_point = reference.find_intersection(Axis((0, y_abs, reference_bb.max.Z), (0, 0, -1)))[-1][0]
    print(f'Sample {i}: {bottom_point}')
    all_points.append(bottom_point)

with BuildLine() as bottom_curve:
    Spline(*all_points)

# Now create planes from each bottom curve point to figure out contours
NUM_SAMPLES = 10
all_contours = []
for i in range(NUM_SAMPLES):
    pt_param = (i / (NUM_SAMPLES - 1))
    sample_plane = Plane(bottom_curve.line ^ pt_param, x_dir=Vector(1, 0, 0))
    sample_plane.origin = sample_plane.origin + sample_plane.y_dir * 3
    with BuildSketch(sample_plane) as sample_circle:
        Circle(radius=reference_bb.size.X / 2)
    sample_circle = sample_circle.sketch.edge()
    print(f'Sample {i}: {sample_plane.origin}')
    # Collapse the perimeter of the sample circle to get the contour
    NUM_SUB_SAMPLES = 50
    points = []
    for j in range(NUM_SUB_SAMPLES):
        pt_param = (j / (NUM_SUB_SAMPLES - 1))
        sample_from = sample_circle @ pt_param
        sample_from_test = Location(sample_from)
        sample_dir = (sample_plane.origin - sample_from).normalized()
        print(f'Subsample {j}: from {sample_from} to {sample_plane.origin}')
        intersections = reference.find_intersection(Axis(sample_from, sample_dir))
        print(f'Subsample {j}: with {len(intersections)} intersections')
        if len(intersections) == 0:
            continue
        expected = intersections[0][0]
        points.append(expected)
    with BuildLine() as contour:
        try:
            Spline(*points) # TODO: Match tangents
        except Exception as e:
            print(f'Failed to create contour {i}: {e}')
            continue
    all_contours.append(contour.line)
del sample_circle, contour

# %%

# TODO: crazy sweep with ALL the contours and the bottom curve
bike_fork_cap = sweep(sections=[Wire(c) for c in all_contours], path=bottom_curve.line)
del bottom_curve, all_contours

# %%
# -- export/show boilerplate --

if os.getenv('export_stl'):
    print('Exporting STL file...')
    export_stl(bike_fork_cap, 'bike_fork_cap.stl')

if os.getenv('export_step'):
    print('Exporting STEP file...')
    export_step(bike_fork_cap, 'bike_fork_cap.step')

try:
    from yacv_server import *

    show_all()

    if os.getenv('export_yacv'):
        print('Exporting YACV file...')
        export_all('.', lambda name, obj: name == 'bike_fork_cap')
except BaseException as e:
    print(f'yacv_server not found or another error happened, skipping visualization: {e}')
