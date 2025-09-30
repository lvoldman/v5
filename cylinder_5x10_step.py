import cadquery as cq

# --- Parameters (mm) ---
R = 5.0     # radius in mm
H = 10.0    # height in mm

# --- Model ---
# CadQuery uses millimeters by default
cyl = cq.Workplane("XY").circle(R).extrude(H)

# --- Export as STEP ---
cq.exporters.export(cyl, "cylinder_5x10.step")

# (Optional) also export STL for 3D printing
# cq.exporters.export(cyl, "cylinder_5x10.stl")
