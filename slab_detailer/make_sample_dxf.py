#!/usr/bin/env python3
"""
Creates a test slab DXF similar to the example project:
27.4 x 27.4 m flat slab, 5.40 m grid, 40/40 and Ø45 columns,
edge walls, a closed core with a lift/stair void, and a shaft opening.

Layers:  SLAB (outline), OPENING, COLUMN, WALL, AXIS
Units:   cm
"""
import ezdxf

doc = ezdxf.new("R2010", setup=True)
doc.header["$INSUNITS"] = 4  # mm
msp = doc.modelspace()
for name, color in [("SLAB", 7), ("OPENING", 1), ("COLUMN", 3), ("WALL", 4), ("AXIS", 8)]:
    doc.layers.add(name, color=color)


def rect(x0, y0, x1, y1, layer):
    msp.add_lwpolyline([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], close=True,
                       dxfattribs={"layer": layer})


AX = [200 + i * 5400 for i in range(6)]          # 200 ... 27200
L = 27400                                        # slab size

# slab outline
rect(0, 0, L, L, "SLAB")

# axes
for i, a in enumerate(AX):
    msp.add_line((a, -1500), (a, L + 1500), dxfattribs={"layer": "AXIS"})
    msp.add_line((-1500, a), (L + 1500, a), dxfattribs={"layer": "AXIS"})
    msp.add_text(str(i + 1), height=400, dxfattribs={"layer": "AXIS"}).set_placement((a, L + 1700))
    msp.add_text("ABCDEF"[i], height=400, dxfattribs={"layer": "AXIS"}).set_placement((-2200, a))

# core: closed box of 250 mm walls between axes 3-4 / C-D (drawn as outer + inner loop)
c0, c1 = AX[2] - 125, AX[3] + 125
rect(c0, c0, c1, c1, "WALL")
rect(c0 + 250, c0 + 250, c1 - 250, c1 - 250, "WALL")
# stair/lift void inside the core
rect(AX[2] + 300, AX[2] + 300, AX[2] + 3000, AX[3] - 300, "OPENING")

# edge walls (350 thick)
rect(0, AX[2], 350, AX[3], "WALL")                 # W1 on left edge, along y
rect(AX[2], 0, AX[3], 350, "WALL")                 # W2 on bottom edge, along x
rect(AX[4] - 175, AX[1] + 1200, AX[4] + 175, AX[1] + 3700, "WALL")  # short interior wall 35/250

# shaft opening next to a column
rect(AX[4] + 500, AX[1] + 500, AX[4] + 1500, AX[1] + 1700, "OPENING")

core_nodes = {(2, 2), (3, 2), (2, 3), (3, 3)}
round_cols = {(1, 3), (4, 2)}
for i, x in enumerate(AX):
    for j, y in enumerate(AX):
        if (i, j) in core_nodes:
            continue
        # keep edge columns flush with the slab edge
        x0 = 0 if i == 0 else (L - 400 if i == 5 else x - 200)
        y0 = 0 if j == 0 else (L - 400 if j == 5 else y - 200)
        if (i, j) in round_cols:
            msp.add_circle((x, y), 225, dxfattribs={"layer": "COLUMN"})
        else:
            rect(x0, y0, x0 + 400, y0 + 400, "COLUMN")

# the example project is drawn in cm -> scale the drawing to cm
from ezdxf.math import Matrix44
for e in msp:
    e.transform(Matrix44.scale(0.1))
doc.header["$INSUNITS"] = 5  # cm
doc.saveas("sample_slab.dxf")
print("sample_slab.dxf written")
