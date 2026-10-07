# Flat slab detailer (Eurocode 2)

The program details flat slab reinforcement. You choose the bars in `config.yaml`, and it lays them out along the geometry of your AutoCAD slab. It does **not** design the slab: bar sizes and spacings are your input.

## 1. Install (once)

You need Python 3.10 or newer. Then run:

```
pip install ezdxf shapely openpyxl matplotlib pyyaml
```

## 2. Prepare your DXF

In AutoCAD, save the drawing as **DXF** (SAVEAS → DXF, 2010 or 2013). Only four things are needed, each on its own layer:

| What | How it can be drawn |
|---|---|
| Slab outline | closed polyline, or lines/arcs that close. It may also stop at the wall and column faces, as usually drawn: the wall and column outlines then close it |
| Openings | closed polylines (holes drawn on the slab layer also work) |
| Columns | rectangles, circles, blocks or hatches |
| Walls / core | closed outlines or hatches; L, U and box shapes are fine |

Axes, dimensions, text and other layers are ignored, and they are copied unchanged into the output drawings.

To see which layers your file has, run:

```
python slab_detailer.py my_slab.dxf --list-layers
```

## 3. Fill in `config.yaml`

1. **`units`** is the unit your drawing is really drawn in (`mm`, `cm` or `m`). The default is `cm`. Check this even if AutoCAD's INSUNITS says something else.
2. **`layers`** are your layer names (default: SLAB, OPENING, COLUMN, WALL).
3. **`slab`** and **`materials`** are the thickness, covers and concrete/steel classes.
4. **`top_layout`** chooses how the basic top reinforcement is laid out:
   - `mesh`: a full top mesh over the whole slab, lapped at mid-span.
   - `bands_and_distribution` (default): bands over the grid lines, plus Φ10 distribution bars filling the gaps between them. Each distribution bar laps into the bands, and every lap is dimensioned.
   - `bands`: bands only.
5. **The reinforcement**:
   - bottom mesh
   - top mesh / grid bands / distribution bars
   - extra bars over internal, edge and corner columns
   - wall bars
   - integrity bars
   - trimmers
   - U-bars

All values in `config.yaml` are in **mm**, whatever the drawing units are.

## 4. Run

```
python slab_detailer.py my_slab.dxf -o output
```

## 5. What you get (in `output/`)

The formwork plan and all four reinforcement plans are in **one DXF**, laid out 2 × 2:

| | left | right |
|---|---|---|
| upper row | Bottom reinforcement - direction X | Bottom reinforcement - direction Y |
| lower row | Top reinforcement - direction X | Top reinforcement - direction Y |

Each plan is a copy of your drawing plus the bars of that layer.

| File | Content |
|---|---|
| `*_REINFORCEMENT_PLANS.dxf` | the four plans |
| `*_bar_schedule.xlsx` | bar marks, number, Ø, length, weight; summary per Ø |
| `*_report.txt` | lap/anchorage lengths, column classification, EC2 checks, warnings |
| `*_REINFORCEMENT_PLANS.png` | quick preview |

What each plan contains:
- **Bottom X / Y**: mesh, integrity bars, trimmers and free-edge U-bars.
- **Top X / Y**: mesh or bands, bars over columns and walls, trimmers and the u1 perimeters.

Notes on the drawing:
- **Rebars** are drawn **white (colour 7) with a 20 mm polyline width**. Set `bar_color` and `bar_width` to change this.
- **Bar labels and plan titles** are white too (`text_color`).
- **Main bars only by default.** Integrity bars, extra bars over columns and walls, trimmers, U-bars and punching perimeters are all switched off (`enabled: false`). Turn any of them back on in the config.
- **No dimension lines by default.** Lap dimensions, distribution lines and the notes block are off. They can be turned back on with `lap_dimensions`, `distribution_lines` and `notes`.
- **Separate files**: set `separate_plan_files: true` to also get each plan as its own DXF.
- **Variable-length bars**: bars that end on a sloping edge are grouped into one variable-length mark, for example `N41 9ф10x(503÷598)/20`. They are not split into many single bars.

Each plan has a title and a notes block with the lap lengths for that face.

### Formwork plan

A fifth drawing, the **formwork plan**, sits above the four reinforcement plans (switch it off with `formwork: enabled: false`). It shows:

- **Shear walls:** cross-hatched, with name and size in X and Y, e.g. `SW6 200X25` or `SW1 25X220` (cm). The name sits beside the wall, clear of its section.
- **Cores:** a closed box of walls, or an opening walled on most of its sides, is named `CORE`. Its walls get no SW names.
- **Columns:** cross-hatched, with name and size, e.g. `C3-50/50` or `C9-Ø50`.
- **Walls and columns** use the same hatch (ANSI37, about 3 cm spacing at 1:50).
- **Openings:** crossed out.
- **Wall sections:** a section through every shear wall, laid flat on the wall. The slab (grey, as thick as the slab) runs across the wall, and the wall (grey, as thick as the wall) shows 15 cm above and below the slab. All ends are broken off with wavy lines.
  - Slab on both sides of the wall gives a cross; slab on one side only (edge walls, core walls) gives a T.
  - Each section has the slab thickness (`24`) and a level mark on the top face of the slab (`+ 7.20`). The level is `formwork: section_level`, or `formwork: level` when that is empty.
  - For walls along X the section is turned 90°, so the top of the slab faces left.
  - The section sits a third of the way along the wall; the wall hatch is left out under it.
  - Settings: `formwork: wall_sections` (`enabled`, `slab_extension` 450 mm, `wall_extension` 150 mm).
- **R.C. slab sections:** grey strips as wide as the slab thickness, with a thickness dimension (e.g. `24`). There is one at mid-span on every grid line.
- **Axes:** bubbles 1, 2, 3 … and A, B, C …, read from the axis layer (`layers: axes`).
- **Dimensions:** chains slab edge – axes – slab edge, plus the overall size, on all four sides.
- **Internal dimensions** along every grid line, drawn on the line itself so they pass through the columns and shear walls: slab edge – element face – element width – clear span – … – slab edge (`internal_dimensions`).
- **Element dimensions** (`element_dimensions`):
  - every wall's thickness at one end, split at the axis through it (e.g. `12.5 | 12.5`, or `10 | 30` for an off-axis wall)
  - every column in X and Y, split at the axes (e.g. `30 | 20` and `15 | 35`)
  - wall lengths where no grid line runs along the wall
  - opening sizes
  - a text that does not fit between its extension lines is moved outside the chain
- **Title, level, scale, legend and notes.** The notes include "Slab thickness h = 24 cm", concrete, steel and covers.

The slab thickness comes from `slab: thickness` (240 = 24 cm). The level and title come from the `formwork:` section of the config.

### Bars drawn as dimensions

By default (`bar_style: dimension`) every drawn bar is **one DIMENSION entity**:
- the dimension line is the bar: magenta, 0.53 mm, no arrowheads, both extension lines suppressed
- dimension style `REBAR M-200`
- the text is only the bar's **live length** (`label_mode: length`). Set `label_mode: full` to get the whole label, e.g. `N23 39ф12x<>/20`

To correct a bar, stretch the dimension's end grips in AutoCAD and its length in the label updates by itself.

Notes:
- The number of bars and the spacing are plain text, so edit those with the text editor.
- Bars of a variable-length group keep a fixed label such as `(377÷413)`.
- Set `bar_style: polyline` to go back to polylines.

### Axes, hatch and dimensions on the reinforcement plans

The four reinforcement plans show the same axes, hatching and dimensions as the formwork plan. Each can be switched off under `drawing:`:

- `axes_on_plans`: axis lines and bubbles (1, 2, 3 … / A, B, C …)
- `axis_dimensions_on_plans`: the outer dimension chains
- `internal_dimensions_on_plans`: the chains along the grid lines
- `element_dimensions_on_plans`: the wall, column and opening dimensions
- `hatch_on_plans`: walls and columns cross-hatched

### Lap dimensions

Wherever two main bars overlap, a dimension is drawn just over the two bars (above X bars, left of Y bars). It shows the lap length, e.g. `75` for Φ12 and `85` for Φ14.

- **Ends:** its extension lines start exactly at the end of one bar and the start of the next.
- **Style and layer:** dimension style `REBAR LAP`, on layer `<bar layer>_LAP`.
- **Live value:** the text is the measured length.
- **Switching off:** set `lap_dimensions: false`.

At every lap a second dimension runs from the **closest axis line to the end of the bar** (e.g. `122`), so the lap can be set out from the grid on site.

- It is drawn on the other side of the bars: below X bars, right of Y bars.
- It is on layer `<bar layer>_AXDIM`.
- Switch it off with `axis_to_bar_end_dimensions: false`.

### Bar notes

Under every bar group (right of a vertical bar) a white text gives the bar description, e.g. `7Φ12x446/20`:

- **Number of bars** = length of the group's distribution dimension / spacing (20 cm).
- **Φ12 / Φ14** = bottom / top bars.
- **Length** = length of the bar in cm.
- **Spacing** in cm.

The notes are on layer `<bar layer>_TEXT`. Their bar counts add up exactly to the bar schedule. Switch them off with `bar_notes: false`.

### Distribution dimensions

Every bar group gets a second, thin magenta dimension across the bars, with oblique ticks. It shows the width of the area reinforced by that bar (e.g. `560`).

- **Extent:** from half a spacing before the first bar to half a spacing after the last bar, cut at the slab edge or opening edge. Neighbouring areas therefore meet at exactly the same point, with no gap.
- **Chaining:** neighbouring groups are placed on the same line, so their dimensions form one continuous chain from edge to edge.

- It is placed at 1/4 of the bar length.
- It is drawn on layer `<bar layer>_RANGE`, so it can be switched off separately.
- Set `distribution_dimensions: false` to leave these dimensions out.

### Laps on the plans

Wherever bars are lapped:
- the two bars of the lap are drawn side by side, slightly offset, so the overlap is visible
- every overlap gets a **dimension** with the lap length

The dimensions are in cm by default (`dimension_unit` in the config). The bars are drawn mid-panel, away from the columns.

### Layers created

| Layer | Contents |
|---|---|
| `REB_BOT_MESH_X/Y` | bottom mesh |
| `REB_BOT_INTEG_X/Y` | integrity bars |
| `REB_TOP_MESH_X/Y` | top mesh |
| `REB_TOP_BAND_X/Y` | top grid bands |
| `REB_TOP_COL_X/Y` | extra top bars over columns |
| `REB_TOP_WALL_X/Y` | top bars over walls |
| `REB_BOT_TRIM`, `REB_TOP_TRIM` | opening trimmers |
| `REB_UBAR` | free-edge U-bars |
| `REB_PUNCH` | punching perimeters u1 |
| `REB_NOTES` | plan title and generated notes |
| `FW_WALL_HATCH`, `FW_COLUMN_HATCH` | wall and column hatching (formwork and reinforcement plans) |
| `FW_WALL_SECTION` | sections through the shear walls, with their level marks |
| `FW_DIM` | formwork dimensions (also on the reinforcement plans) |

Turn layers on and off to make your sheets.

### How each bar group is drawn

By default (`mode: representative`) each group is drawn the usual way:
- one bold representative bar, with bends shown as stubs
- a distribution line with end ticks
- a label such as `N7 136ф10x440/20` (mark, number, Ø, length in cm, spacing in cm)

Set `mode: all` to draw every bar.

## 6. EC2 detailing rules the program applies

**Laps (8.4, 8.7)**
- Lap length is l0 = α6·lb,rqd ≥ l0,min.
- Poor bond is taken for top bars when h > 250 mm.
- Bottom laps are placed **over the support lines**; top laps at **mid-span**.
- Each whole lap stays inside a zone of ±0.2 × span around that line (`lap_zone`).
- Laps can be staggered (`stagger_laps`).

**Bar lengths**
- Bars are at most **6 m** long (`stock_length`).
- Bar ends stay **50 mm** inside the slab edges and openings (`cover_side`).

**Over columns (9.4.1(2))**
- Extra top bars are placed **between** the band/mesh bars.
- They extend to the given fraction of each adjacent span.
- `count: auto` uses 0.25 × the panel width.

**Edge and corner columns (9.4.2)**
- These are recognised automatically from the slab outline.
- Each type uses its own bars.

**Walls (9.3.1.2(2))**
- Top bars cross each wall.
- They extend at least 0.2 × the span (factor and minimum are set in the config).

**Integrity (9.4.1(3))**
- Bottom bars pass through the columns.
- They extend 2d + l_bd beyond the column face.

**Free edges (9.3.1.4)**
- U-bars are placed along unsupported edges, with legs of 2h.
- Top bars ending at an edge get a bend down.

**Punching (6.4)**
- The u1 perimeter is drawn at 2d around each column.
- A warning is given when an opening is closer than 6d (6.4.2(3)).

**Checks (9.2.1.1, 9.3.1.1)**
- As,min and maximum bar spacing.

## 7. Current limits

- Bars run in X and Y only, so the slab should be orthogonal. Walls at an angle get no wall bars, and a warning is given.
- Spans are taken from the column and wall lines across the whole slab. On an irregular grid, check the bar lengths around the irregular areas.
- Punching shear reinforcement (links/studs) is not generated yet; only u1 is drawn.
- Labels are placed to avoid each other, but busy areas (openings near columns) usually need some manual tidying in AutoCAD.

## Test file

`make_sample_dxf.py` creates `sample_slab.dxf`, a 27.4 × 27.4 m slab with a 5.4 m grid, a core, edge walls and a shaft. You can use it to try the program.
