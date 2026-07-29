#!/usr/bin/env python3
"""Generate the uneven-terrain Gazebo world for go2_eskf slip-model work.

Writes three files into --out-dir (default src/go2_eskf/worlds/):

    terrain.sdf            the world -- flat.sdf's twin, ground plane swapped for
                           a heightmap, plus low-friction patches
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

import numpy as np
from PIL import Image

# The 10 m square that square_test.py drives: (0,0) -> (10,0) -> (10,-10) ->
# (0,-10) -> (0,0). Patches default to the midpoint of each leg.
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


def patch_model(name, x, y, z, roll, pitch, side, mu):
    half_t = 0.05
    return f"""
    <!-- Low-friction patch: mu={mu} over {side} x {side} m at ({x:+.2f}, {y:+.2f}).
         Sits {PATCH_LIFT * 100:.1f} cm proud of the terrain and is tilted to the
         local gradient, so it is the surface the feet actually contact. -->
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} {roll:.5f} {pitch:.5f} 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{side} {side} {2 * half_t}</size></box>
          </geometry>
          <surface>
            <friction>
              <ode><mu>{mu}</mu><mu2>{mu}</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{side} {side} {2 * half_t}</size></box>
          </geometry>
          <material>
            <ambient>0.62 0.76 0.85 1</ambient>
            <diffuse>0.70 0.85 0.95 1</diffuse>
            <specular>0.9 0.9 0.9 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


PATCH_LIFT = 0.015   # how far a patch's top face stands above the terrain [m]

HEADER = """<?xml version="1.0" ?>
<!--
  terrain.sdf — GENERATED by src/go2_eskf/scripts/make_terrain_world.py.
  Do not hand-edit; re-run the generator instead (it prints the command it used).

  Uneven "mountainous" twin of flat.sdf, for exercising the go2_eskf slip model.
  Identical GPS datum, physics engine, plugins and sun; the flat ground plane is
  replaced by a fractal heightmap, and low-friction patches are laid on the 10 m
  square that square_test.py drives.

  Reverting to flat ground needs NO file edits and NO regeneration — flat.sdf is
  untouched and is still the DEFAULT world:

      ./run_go2_teleop.sh --square              # flat.sdf   (default, as before)
      ./run_go2_teleop.sh --terrain --square    # this world
      ./run_go2_teleop.sh --obstacles           # vendored default.sdf

  The heightmap <uri> MUST be an absolute file:// path (measured: gz resolves
  neither a world-relative path nor GZ_SIM_RESOURCE_PATH for heightmaps), so this
  file is machine-specific. Re-run the generator after moving the workspace.

  GENERATED WITH: {cmdline}
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
    ap.add_argument("--seed", type=int, default=3, help="terrain RNG seed")
    ap.add_argument("--pad-radius", type=float, default=1.5,
                    help="radius of the flat spawn pad [m] (default 1.5)")
    ap.add_argument("--pad-blend", type=float, default=2.5,
                    help="extra radius over which the pad blends into terrain [m]")
    ap.add_argument("--patch-mu", type=float, default=0.08,
                    help="friction of the slip patches (terrain default is ~1.0)")
    ap.add_argument("--patch-size", type=float, default=3.0,
                    help="side length of each square slip patch [m]")
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
    height_png = os.path.join(out_dir, "terrain_height.png")
    Image.fromarray(pix).save(height_png)

    diffuse_png = os.path.join(out_dir, "terrain_diffuse.png")
    normal_png = os.path.join(out_dir, "terrain_normal.png")
    diffuse_texture(diffuse_png, args.seed)
    normal_texture(normal_png)

    # Model the terrain exactly as the physics will see it: the QUANTISED image,
    # scaled by pixel/255 * relief. Patch placement then matches to the millimetre.
    terrain = Terrain(pix.astype(np.float64) / 255.0 * args.relief, ext)

    patches = []
    if not args.no_patches:
        for k, (px, py) in enumerate(DEFAULT_PATCHES):
            zt = terrain.height(px, py)
            gx, gy = terrain.gradient(px, py)
            pitch = -np.arctan(gx)
            roll = np.arctan(gy * np.cos(pitch))
            zc = zt + PATCH_LIFT - 0.05
            patches.append((f"slip_patch_{k}", px, py, zc, roll, pitch, zt,
                            np.degrees(np.arctan(np.hypot(gx, gy)))))

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
        + (f" of {args.patch_size:.1f} x {args.patch_size:.1f} m at mu={args.patch_mu}"
           if patches else " (disabled)") + "\n")

    cmdline = ("python3 src/go2_eskf/scripts/make_terrain_world.py"
               f" --extent {ext:g} --samples {n} --relief {args.relief:g}"
               f" --seed {args.seed} --pad-radius {args.pad_radius:g}"
               f" --pad-blend {args.pad_blend:g}"
               + ("" if not patches else
                  f" --patch-mu {args.patch_mu:g} --patch-size {args.patch_size:g}")
               + (" --no-patches" if args.no_patches else ""))

    body = [HEADER.format(cmdline=cmdline, stats=stats)]
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

    for name, px, py, zc, roll, pitch, _zt, _sl in patches:
        body.append(patch_model(name, px, py, zc, roll, pitch,
                                args.patch_size, args.patch_mu))

    body.append("  </world>\n</sdf>\n")

    world_path = os.path.join(out_dir, "terrain.sdf")
    with open(world_path, "w") as fh:
        fh.write("".join(body))

    print(f"wrote {world_path}")
    print(f"      {height_png}")
    print(f"      {diffuse_png}")
    print(f"      {normal_png}")
    print()
    print(stats.rstrip())
    for name, px, py, _zc, roll, pitch, zt, sl in patches:
        print(f"    {name}: ({px:+.1f}, {py:+.1f}) terrain z={zt:.3f} m, "
              f"slope {sl:.1f} deg, rpy=({roll:+.3f}, {pitch:+.3f}, 0)")
    print()
    print("Run it (no rebuild needed — worlds/ is symlink-installed):")
    print("    ./run_go2_teleop.sh --terrain --square")


if __name__ == "__main__":
    main()
