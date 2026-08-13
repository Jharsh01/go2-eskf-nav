#!/usr/bin/env python3
"""Generate the uneven-terrain Gazebo world for go2_eskf slip-model work.

Writes into --out-dir (default src/go2_eskf/worlds/):

    terrain.sdf            the world -- flat.sdf's twin, ground plane swapped for
                           a heightmap, plus low-friction patches
    terrain_params.txt     provenance: the exact command and the resulting slope /
                           step / patch numbers. Separate from the SDF because XML
                           forbids a double hyphen inside a comment, so option flags
                           cannot be recorded there.
    terrain_height.png     the heightmap (greyscale, (2^n)+1 square)
    terrain_diffuse.png    a ground texture for the heightmap visual
    terrain_normal.png     flat normal map (gz wants one alongside the diffuse)

Why generated rather than hand-written: gz resolves a heightmap <uri> only as an
absolute `file://` path -- MEASURED, neither a path relative to the world file nor
GZ_SIM_RESOURCE_PATH works -- so the world is not relocatable. Re-run this script
after moving or re-cloning the workspace and the paths are correct again. It also
makes the terrain a set of knobs rather than 40 000 hand-typed numbers.

    python3 src/go2_eskf/scripts/make_terrain_world.py            # defaults
    python3 src/go2_eskf/scripts/make_terrain_world.py --relief 2.5 --patch-mu 0.05
    python3 src/go2_eskf/scripts/make_terrain_world.py --no-patches --seed 7

Then, with no rebuild needed (worlds/ is symlink-installed):

    ./run_go2_teleop.sh --terrain --square

Reverting to flat ground needs no file edits and no regeneration: flat.sdf is
untouched and remains the DEFAULT world. Just drop the --terrain flag.

Heightmap facts established by measurement against gz sim 8.11 (see skills.md):
  * image column index -> +X, row index 0 -> +Y (max Y). Standard DEM convention.
  * elevation = (pixel / MAX PIXEL IN THE IMAGE) * size_z -- gz normalises by the
    image's own maximum, NOT by 255. This script always emits a 255 maximum so the
    scaling is simply pixel/255 * relief.
  * heightmap collision works under bullet-featherstone (the physics engine this
    world inherits from flat.sdf).
"""
import argparse
import os
import xml.dom.minidom

import numpy as np
from PIL import Image

# The 10 m square this world is laid out for: (0,0) -> (10,0) -> (10,-10) ->
# (0,-10) -> (0,0). Patches default to the midpoint of each leg.
#
# NOTE: square_test.py's --side now defaults to 5 m, so the driven route is HALF
# this box and crosses only the (5,0) and (0,-5) patches — at its corners, not
# mid-leg. Halve both tuples here (and regenerate) to put the friction patches
# back under the middle of each leg; the slope report the script prints is
# computed along SQUARE, so it must match the route to mean anything.
SQUARE = ((0.0, 0.0), (10.0, 0.0), (10.0, -10.0), (0.0, -10.0))
DEFAULT_PATCHES = ((5.0, 0.0), (10.0, -5.0), (5.0, -10.0), (0.0, -5.0))

# Fractal octaves: (wavelength [m], relative amplitude). The last two are at foot
# scale -- they are what actually perturbs contact timing and leg odometry, which
# is the point of the exercise; the long ones just make it look like terrain.
OCTAVES = ((18.0, 1.00), (9.0, 0.50), (4.5, 0.25), (2.2, 0.12), (0.9, 0.05))
WAVES_PER_OCTAVE = 4


def smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def height_field(X, Y, seed, pad_r0, pad_r1):
    """Normalised terrain height in [0, 1], flat (0.0) on the start pad."""
    rng = np.random.default_rng(seed)
    z = np.zeros_like(X)
    for wavelength, amp in OCTAVES:
        k = 2.0 * np.pi / wavelength
        for _ in range(WAVES_PER_OCTAVE):
            th = rng.uniform(0.0, 2.0 * np.pi)
            ph = rng.uniform(0.0, 2.0 * np.pi)
            z += amp * np.sin(k * (np.cos(th) * X + np.sin(th) * Y) + ph)

    z -= z.min()
    z /= z.max()

    # Flat start pad at elevation 0 so the robot spawns exactly as it does over
    # flat.sdf (world_init_z 0.375, ~0.3 m of clearance) and so flat-vs-terrain
    # runs begin from an identical pose. Multiplying by the ramp keeps the field
    # in [0, 1] and blends smoothly instead of leaving a rim.
    r = np.hypot(X, Y)
    z *= smoothstep((r - pad_r0) / max(pad_r1 - pad_r0, 1e-6))

    return z / z.max()


def bilinear(grid, i, j):
    """Sample grid[row, col] at fractional (i=col, j=row), as gz interpolates."""
    n = grid.shape[0]
    i0, j0 = int(np.floor(i)), int(np.floor(j))
    i1, j1 = min(i0 + 1, n - 1), min(j0 + 1, n - 1)
    i0, j0 = max(i0, 0), max(j0, 0)
    fi, fj = i - i0, j - j0
    return float(
        grid[j0, i0] * (1 - fi) * (1 - fj) + grid[j0, i1] * fi * (1 - fj) +
        grid[j1, i0] * (1 - fi) * fj + grid[j1, i1] * fi * fj)


class Terrain:
    """Elevation grid plus the world<->pixel mapping gz uses."""

    def __init__(self, elev, extent):
        self.elev = elev                      # metres, [row, col]
        self.n = elev.shape[0]
        self.extent = extent
        self.cell = extent / (self.n - 1)
        # np.gradient returns d/drow, d/dcol. Row increases toward -Y.
        dz_drow, dz_dcol = np.gradient(elev, self.cell)
        self.gx = dz_dcol
        self.gy = -dz_drow

    def ij(self, x, y):
        i = (x + self.extent / 2.0) / self.extent * (self.n - 1)
        j = (self.extent / 2.0 - y) / self.extent * (self.n - 1)
        return i, j

    def height(self, x, y):
        i, j = self.ij(x, y)
        return bilinear(self.elev, i, j)

    def gradient(self, x, y):
        i, j = self.ij(x, y)
        return bilinear(self.gx, i, j), bilinear(self.gy, i, j)

    def slope_deg(self):
        return np.degrees(np.arctan(np.hypot(self.gx, self.gy)))


def diffuse_texture(path, seed, size=512):
    """A plain rocky/earthy texture so the heightmap renders as ground."""
    rng = np.random.default_rng(seed + 1)
    base = np.zeros((size, size))
    for wl, amp in ((128.0, 1.0), (64.0, 0.5), (16.0, 0.25), (4.0, 0.12)):
        noise = rng.normal(size=(int(size / wl) + 2, int(size / wl) + 2))
        img = Image.fromarray(noise).resize((size, size), Image.BICUBIC)
        base += amp * np.asarray(img)
    base = (base - base.min()) / (base.ptp() + 1e-9)

    lo = np.array([0.32, 0.30, 0.22])   # dry earth
    hi = np.array([0.55, 0.52, 0.44])   # pale rock
    rgb = lo + (hi - lo) * base[..., None]
    Image.fromarray((rgb * 255).astype(np.uint8)).save(path)


def normal_texture(path, size=32):
    flat = np.zeros((size, size, 3), dtype=np.uint8)
    flat[..., 0] = 128
    flat[..., 1] = 128
    flat[..., 2] = 255
    Image.fromarray(flat).save(path)


PATCH_THICK = 0.04   # slip-tile thickness [m]; most of it sits inside the terrain


def patch_model(name, px, py, terrain, side, tile, lift, mu):
    """One patch as a grid of small tiles, each conforming to the local surface.

    A patch must NOT be a single flat plate. Laid on curved terrain a 3 m plate
    only touches near its centre and its edges float: MEASURED up to 27 cm above
    the ground, a curb taller than the Go2 is tall, made of ice. That is what
    wedged the robot on the first terrain run. Tiling drops the worst step onto a
    patch to a couple of cm -- below the terrain's own fine roughness.
    """
    n = max(1, int(round(side / tile)))
    t = side / n
    links = []
    for a in range(n):
        for b in range(n):
            cx = px - side / 2.0 + (a + 0.5) * t
            cy = py - side / 2.0 + (b + 0.5) * t
            z0 = terrain.height(cx, cy)
            gx, gy = terrain.gradient(cx, cy)
            pitch = -np.arctan(gx)
            roll = np.arctan(gy * np.cos(pitch))
            zc = z0 + lift - PATCH_THICK / 2.0
            links.append(f"""        <link name="t_{a}_{b}">
          <pose>{cx:.4f} {cy:.4f} {zc:.4f} {roll:.5f} {pitch:.5f} 0</pose>
          <collision name="collision">
            <geometry><box><size>{t:.4f} {t:.4f} {PATCH_THICK}</size></box></geometry>
            <surface>
              <friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction>
            </surface>
          </collision>
          <visual name="visual">
            <geometry><box><size>{t:.4f} {t:.4f} {PATCH_THICK}</size></box></geometry>
            <material>
              <ambient>0.62 0.76 0.85 1</ambient>
              <diffuse>0.70 0.85 0.95 1</diffuse>
              <specular>0.9 0.9 0.9 1</specular>
            </material>
          </visual>
        </link>""")
    return f"""
    <!-- Low-friction patch: mu={mu} over {side} x {side} m centred on
         ({px:+.2f}, {py:+.2f}), as {n}x{n} tiles of {t:.2f} m. Each tile is placed at
         its OWN local terrain height and gradient and stands {lift * 100:.1f} cm
         proud, so the feet contact the patch rather than the ground without
         having to climb a step. -->
    <model name="{name}">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
{chr(10).join(links)}
    </model>
"""


def patch_metrics(px, py, terrain, side, tile, lift):
    """Worst step up onto a tiled patch, and worst terrain poking above it [m]."""
    n = max(1, int(round(side / tile)))
    t = side / n
    step = poke = 0.0
    for a in range(n):
        for b in range(n):
            cx = px - side / 2.0 + (a + 0.5) * t
            cy = py - side / 2.0 + (b + 0.5) * t
            z0 = terrain.height(cx, cy)
            gx, gy = terrain.gradient(cx, cy)
            edge = a in (0, n - 1) or b in (0, n - 1)
            for dx in np.linspace(-t / 2, t / 2, 7):
                for dy in np.linspace(-t / 2, t / 2, 7):
                    top = z0 + gx * dx + gy * dy + lift
                    ground = terrain.height(cx + dx, cy + dy)
                    poke = max(poke, ground - top)
                    if edge:
                        step = max(step, top - ground)
    return step, poke

HEADER = """<?xml version="1.0" ?>
<!--
  terrain.sdf — GENERATED by src/go2_eskf/scripts/make_terrain_world.py.
  Do not hand-edit; re-run the generator instead (it prints the command it used).

  Uneven "mountainous" twin of flat.sdf, for exercising the go2_eskf slip model.
  Identical GPS datum, physics engine, plugins and sun; the flat ground plane is
  replaced by a fractal heightmap, and low-friction patches are laid on the 10 m
  square that square_test.py drives.

  Reverting to flat ground needs NO file edits and NO regeneration: flat.sdf is
  untouched and is still the DEFAULT world, so dropping the launcher's terrain flag
  is all it takes.

  The exact command that produced this world, and the full knob list, are in
  terrain_params.txt next to this file. XML forbids a double hyphen inside a
  comment, so option flags cannot be written here at all — that is why the command
  lives in the sidecar. The generator validates its own output with a strict XML
  parser; gz uses TinyXML2 and tolerates the illegal form, so this only ever breaks
  on stricter tooling.

  The heightmap uri MUST be an absolute file:// path (measured: gz resolves neither
  a world-relative path nor GZ_SIM_RESOURCE_PATH for heightmaps), so this file is
  machine-specific. Re-run the generator after moving the workspace.

{stats}-->
<sdf version="1.8">
  <world name="default">
    <!-- Geographic origin for GPS simulation — keep in sync with flat.sdf and
         default.sdf; the ESKF's lat/lon->ENU conversion anchors on this datum. -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>30.0444</latitude_deg>
      <longitude_deg>31.2357</longitude_deg>
      <elevation>74.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <physics name="1ms" type="bullet-featherstone">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>

    <!-- Lights -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", default=None,
                    help="default: src/go2_eskf/worlds next to this script")
    ap.add_argument("--extent", type=float, default=40.0,
                    help="terrain side length [m] (default 40, centred on origin)")
    ap.add_argument("--samples", type=int, default=257,
                    help="heightmap resolution, must be (2^n)+1 (default 257)")
    ap.add_argument("--relief", type=float, default=0.7,
                    help="peak elevation above the start pad [m] (default 0.7). "
                         "THE difficulty knob — slope scales with it. Measured on "
                         "the square path (median / max slope): 0.6 -> 3.2/14.4 "
                         "deg, 0.7 -> 3.7/16.6, 1.0 -> 5.2/23.1, 1.6 -> 8.4/34.3. "
                         "CHAMP's blind gait already falls occasionally on FLAT "
                         "ground, so start low and work up.")
    ap.add_argument("--max-slope", type=float, default=None,
                    help="cap the STEEPEST cell anywhere on the terrain at this "
                         "many degrees, overriding --relief. Slope is exactly "
                         "linear in relief (the quantised heightmap shape is fixed "
                         "and elevation is pixel/255*relief), so this is solved in "
                         "closed form, not searched. Prefer this over --relief when "
                         "what you care about is 'nothing steeper than X'.")
    ap.add_argument("--seed", type=int, default=3, help="terrain RNG seed")
    ap.add_argument("--pad-radius", type=float, default=1.5,
                    help="radius of the flat spawn pad [m] (default 1.5)")
    ap.add_argument("--pad-blend", type=float, default=2.5,
                    help="extra radius over which the pad blends into terrain [m]")
    ap.add_argument("--patch-mu", type=float, default=0.30,
                    help="friction of the slip patches (terrain default is ~1.0). "
                         "A foot only holds if mu > tan(local slope), and walking "
                         "needs roughly twice that, so mu must clear the slope the "
                         "patch sits on — the generator warns if it does not. 0.30 "
                         "is gravel/wet grass: real slip, still traversable. 0.08 is "
                         "ice and the robot cannot stand on any slope above 4.6 deg.")
    ap.add_argument("--patch-size", type=float, default=3.0,
                    help="side length of each square slip patch [m]")
    ap.add_argument("--patch-tile", type=float, default=0.4,
                    help="patches are built from tiles this size [m] (default 0.4) "
                         "so they conform to the terrain instead of forming a curb. "
                         "A single flat plate floats up to 27 cm above curved ground.")
    ap.add_argument("--patch-lift", type=float, default=0.012,
                    help="how far a patch tile stands above the local terrain [m]")
    ap.add_argument("--no-patches", action="store_true",
                    help="uneven terrain only, no low-friction patches")
    args = ap.parse_args()

    if bin(args.samples - 1).count("1") != 1:
        ap.error(f"--samples must be (2^n)+1; {args.samples} is not")

    here = os.path.dirname(os.path.abspath(__file__))
    out_dir = args.out_dir or os.path.abspath(os.path.join(here, os.pardir, "worlds"))
    os.makedirs(out_dir, exist_ok=True)

    n, ext = args.samples, args.extent
    axis = np.linspace(-ext / 2.0, ext / 2.0, n)
    X, Y = np.meshgrid(axis, axis[::-1])        # row 0 -> +Y (max Y)

    z01 = height_field(X, Y, args.seed, args.pad_radius,
                       args.pad_radius + args.pad_blend)

    # gz normalises by the image's own max pixel, so force an exact 255 maximum
    # and the elevation is then simply (pixel / 255) * relief.
    pix = np.rint(z01 * 255.0).astype(np.uint8)
    pix[np.unravel_index(np.argmax(z01), z01.shape)] = 255

    # --max-slope resolves to a relief. The quantised shape `pix` does not depend on
    # relief (elevation is pixel/255 * relief), so the steepest grade in the world is
    # exactly relief * max|grad(pix/255)| — invert that instead of searching. Doing it
    # on the QUANTISED image means the number reported below is the one physics sees.
    if args.max_slope is not None:
        cell = ext / (n - 1)
        d_row, d_col = np.gradient(pix.astype(np.float64) / 255.0, cell)
        grade_per_m = float(np.hypot(d_col, -d_row).max())
        if grade_per_m <= 0.0:
            ap.error("terrain is perfectly flat — --max-slope has nothing to scale")
        args.relief = float(np.tan(np.radians(args.max_slope)) / grade_per_m)

    height_png = os.path.join(out_dir, "terrain_height.png")
    Image.fromarray(pix).save(height_png)

    diffuse_png = os.path.join(out_dir, "terrain_diffuse.png")
    normal_png = os.path.join(out_dir, "terrain_normal.png")
    diffuse_texture(diffuse_png, args.seed)
    normal_texture(normal_png)

    # Model the terrain exactly as the physics will see it: the QUANTISED image,
    # scaled by pixel/255 * relief. Patch placement then matches to the millimetre.
    terrain = Terrain(pix.astype(np.float64) / 255.0 * args.relief, ext)

    patches, warnings = [], []
    if not args.no_patches:
        for k, (px, py) in enumerate(DEFAULT_PATCHES):
            zt = terrain.height(px, py)
            # Worst slope anywhere on the footprint, not just at the centre — that
            # is what decides whether a foot can hold.
            half = args.patch_size / 2.0
            slopes = [np.hypot(*terrain.gradient(px + dx, py + dy))
                      for dx in np.linspace(-half, half, 13)
                      for dy in np.linspace(-half, half, 13)]
            slope_max = float(np.max(slopes))
            step, poke = patch_metrics(px, py, terrain, args.patch_size,
                                       args.patch_tile, args.patch_lift)
            patches.append((f"slip_patch_{k}", px, py, zt,
                            np.degrees(np.arctan(slope_max)), step, poke))
            # mu must exceed tan(slope) for a foot to hold at all, and about twice
            # that to push off and walk.
            if args.patch_mu < 1.5 * slope_max:
                warnings.append(
                    f"slip_patch_{k} at ({px:+.1f},{py:+.1f}) reaches "
                    f"{np.degrees(np.arctan(slope_max)):.1f} deg slope, needing "
                    f"mu > {slope_max:.3f} to stand and ~{1.5 * slope_max:.3f} to "
                    f"walk, but --patch-mu is {args.patch_mu}. The robot will slide "
                    f"and probably fall here.")

    # ---- stats, echoed into the file header and to stdout
    slope = terrain.slope_deg()
    path_pts = []
    for a, b in zip(SQUARE, SQUARE[1:] + SQUARE[:1]):
        for t in np.linspace(0, 1, 120):
            path_pts.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    path_h = np.array([terrain.height(x, y) for x, y in path_pts])
    path_s = np.array([np.degrees(np.arctan(np.hypot(*terrain.gradient(x, y))))
                       for x, y in path_pts])

    stats = (
        f"    terrain      {ext:.0f} x {ext:.0f} m, {n}x{n} samples "
        f"({terrain.cell * 100:.1f} cm/cell), relief {args.relief:.2f} m\n"
        f"    slope        median {np.median(slope):.1f} deg, "
        f"95th pct {np.percentile(slope, 95):.1f} deg, max {slope.max():.1f} deg\n"
        f"    square path  elevation {path_h.min():.2f}..{path_h.max():.2f} m, "
        f"slope median {np.median(path_s):.1f} deg / max {path_s.max():.1f} deg\n"
        f"    start pad    flat (elevation 0.000 m) out to r={args.pad_radius:.1f} m, "
        f"blended to r={args.pad_radius + args.pad_blend:.1f} m\n"
        f"    slip patches {len(patches)}"
        + (f" of {args.patch_size:.1f} x {args.patch_size:.1f} m at mu={args.patch_mu}, "
           f"tiled at {args.patch_tile:.2f} m, {args.patch_lift * 1000:.0f} mm proud; "
           f"worst step onto a patch "
           f"{max(p[5] for p in patches) * 100:.1f} cm"
           if patches else " (disabled)") + "\n")

    cmdline = ("python3 src/go2_eskf/scripts/make_terrain_world.py"
               f" --extent {ext:g} --samples {n} --relief {args.relief:g}"
               f" --seed {args.seed} --pad-radius {args.pad_radius:g}"
               f" --pad-blend {args.pad_blend:g}"
               + ("" if not patches else
                  f" --patch-mu {args.patch_mu:g} --patch-size {args.patch_size:g}"
                  f" --patch-tile {args.patch_tile:g}"
                  f" --patch-lift {args.patch_lift:g}")
               + (" --no-patches" if args.no_patches else ""))

    body = [HEADER.format(stats=stats)]
    body.append(f"""
    <!-- Terrain. Replaces flat.sdf's infinite ground plane; there is no plane
         underneath, so the {ext:.0f} x {ext:.0f} m heightmap is the only floor.
         The 10 m square lives well inside it. -->
    <model name="terrain">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>file://{height_png}</uri>
              <size>{ext} {ext} {args.relief}</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <uri>file://{height_png}</uri>
              <size>{ext} {ext} {args.relief}</size>
              <pos>0 0 0</pos>
              <texture>
                <diffuse>file://{diffuse_png}</diffuse>
                <normal>file://{normal_png}</normal>
                <size>4</size>
              </texture>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>
""")

    for name, px, py, _zt, _sl, _step, _poke in patches:
        body.append(patch_model(name, px, py, terrain, args.patch_size,
                                args.patch_tile, args.patch_lift, args.patch_mu))

    body.append("  </world>\n</sdf>\n")

    sdf_text = "".join(body)

    # Fail loudly rather than shipping malformed XML. gz's TinyXML2 is lenient (it
    # accepted a double hyphen inside a comment, which XML forbids), so without this
    # check the world only breaks later, on stricter tooling.
    try:
        xml.dom.minidom.parseString(sdf_text)
    except Exception as exc:  # noqa: BLE001 - report and abort whatever it is
        raise SystemExit(f"generated SDF is not well-formed XML: {exc}")

    world_path = os.path.join(out_dir, "terrain.sdf")
    with open(world_path, "w") as fh:
        fh.write(sdf_text)

    # Provenance sidecar: plain text, so it can hold the flags the SDF comment cannot.
    params_path = os.path.join(out_dir, "terrain_params.txt")
    with open(params_path, "w") as fh:
        fh.write("terrain.sdf provenance — regenerate with exactly this command:\n\n")
        fh.write(f"    {cmdline}\n\n")
        fh.write(stats)
        fh.write("\nPer-patch detail (step = worst climb onto a patch; a patch is\n"
                 "tiled so it conforms to the terrain instead of forming a curb):\n")
        for name, px, py, zt, sl, step, poke in patches:
            fh.write(f"    {name}: ({px:+.1f}, {py:+.1f}) terrain z={zt:.3f} m, "
                     f"max slope {sl:.1f} deg, worst step up {step * 100:.1f} cm, "
                     f"terrain pokes above by {poke * 100:.1f} cm\n")
        if warnings:
            fh.write("\nWARNINGS:\n")
            for w in warnings:
                fh.write(f"    * {w}\n")
        fh.write("\nFull knob list: run the generator with -h.\n")

    print(f"wrote {world_path}")
    print(f"      {params_path}")
    print(f"      {height_png}")
    print(f"      {diffuse_png}")
    print(f"      {normal_png}")
    print()
    print(stats.rstrip())
    for name, px, py, zt, sl, step, poke in patches:
        print(f"    {name}: ({px:+.1f}, {py:+.1f}) terrain z={zt:.3f} m, "
              f"max slope {sl:.1f} deg, worst step up {step * 100:.1f} cm, "
              f"terrain pokes above by {poke * 100:.1f} cm")
    if warnings:
        print()
        print("  WARNING — the robot will not be able to walk on these patches:")
        for w in warnings:
            print(f"    * {w}")
        print("    Raise --patch-mu, or move/shrink the patches onto flatter ground.")
    print()
    print("Run it (no rebuild needed — worlds/ is symlink-installed):")
    print("    ./run_go2_teleop.sh --terrain --square")


if __name__ == "__main__":
    main()
