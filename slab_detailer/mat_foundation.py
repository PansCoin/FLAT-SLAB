#!/usr/bin/env python3
"""
mat_foundation.py - MAT FOUNDATION reinforcement detailing to Eurocode 2 (EN 1992-1-1)

Does for the mat foundation what slab_detailer.py does for the flat slab, from the same building
drawing (slab outline, openings, columns, walls, axes) and with the same outputs: formwork plan,
four reinforcement plans, bar schedule, report.

A mat is an upside-down flat slab: the soil pushes it up, the columns and walls hold it down.
So the tension is at the BOTTOM over the supports and at the TOP in the spans, and compared with
the flat slab:
  * top bars are lapped OVER THE SUPPORTS (column / wall lines)
  * bottom bars are lapped AT MID-SPAN, between the supports
  * extra bars at the columns / walls (if switched on) are bottom bars, under the columns
  * the sections through the walls on the formwork plan show the wall above the mat only:
    the mat is the lowest slab, it ends at the bottom
Top and bottom meshes are Φ20 (config_mat.yaml). Top bars of a 70 cm mat are in poor bond
(EC2 8.4.2(2)), so their laps are longer than the bottom ones.

Usage:
    python mat_foundation.py my_building.dxf                   (uses config_mat.yaml next to it)
    python mat_foundation.py my_building.dxf -c my_mat.yaml -o mat_output
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import slab_detailer  # noqa: E402  (the shared engine, next to this file)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description="Mat foundation reinforcement detailer (EC2)")
    ap.add_argument("dxf", help="DXF of the building: outline, openings, columns and walls")
    ap.add_argument("-c", "--config", default=os.path.join(here, "config_mat.yaml"),
                    help="config YAML (default: config_mat.yaml next to this script)")
    ap.add_argument("-o", "--out", default="mat_output", help="output folder")
    a = ap.parse_args()
    slab_detailer.run(a.dxf, a.config, a.out, structure="mat_foundation")


if __name__ == "__main__":
    main()
