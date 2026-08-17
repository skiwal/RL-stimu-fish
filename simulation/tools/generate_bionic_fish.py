#!/usr/bin/env python3
"""
generate_bionic_fish.py
=======================

Generate the BionicFish V1 visual and physics OBJ meshes from
BionicFishV1Config (YAML).

Design goals
------------
1. The YAML config is the single source of truth. No fish dimensions are
   silently hard-coded in the generator.
2. The body is a closed streamlined superelliptic loft.
3. Tail0..Tail4 are separate closed link-local meshes. Each tail mesh origin
   is its proximal joint so it can articulate naturally in Stonefish.
4. Left/right pectoral fins are real tapered swept hydrofoil-like meshes,
   not rectangular plates.
5. fishsim finTail.obj / finTop.obj are reused when available.
6. Visual and physics meshes are generated separately.
7. A neutral-pose assembly preview and a machine-readable mesh manifest are
   generated for inspection before writing the Stonefish .scn robot.

Coordinate convention
---------------------
BionicFish body frame (NED-compatible):
    +X : forward / head
    +Y : fish right
    +Z : down

Thus:
    tail extends toward -X
    dorsal fin extends toward -Z
    forward sonar points +X

Usage
-----
From the repository root:

    python3 simulation/tools/generate_bionic_fish.py \
        --config simulation/config/bionic_fish_v1_config.yaml

If fishsim is cloned locally:

    python3 simulation/tools/generate_bionic_fish.py \
        --config simulation/config/bionic_fish_v1_config.yaml \
        --fishsim-root ../fishsim

Or let the script fetch only the two MIT-licensed reference OBJ files:

    python3 simulation/tools/generate_bionic_fish.py \
        --config simulation/config/bionic_fish_v1_config.yaml \
        --fetch-reference-meshes

For a completely offline geometry test without fishsim meshes:

    python3 simulation/tools/generate_bionic_fish.py \
        --config simulation/config/bionic_fish_v1_config.yaml \
        --allow-procedural-fin-fallback

Dependency:
    PyYAML

Install if needed:
    python3 -m pip install pyyaml
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import shutil
import sys
import urllib.request
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Sequence, Tuple

try:
    import yaml
except ImportError as exc:
    raise SystemExit(
        "PyYAML is required. Install it with:\n"
        "  python3 -m pip install pyyaml"
    ) from exc


Vec3 = Tuple[float, float, float]
Tri = Tuple[int, int, int]


FISHSIM_RAW_BASE = (
    "https://raw.githubusercontent.com/srl-ethz/fishsim/main/Geometry/Meshes"
)

FISHSIM_REFERENCE_FILES = {
    "caudal_fin": "finTail.obj",
    "dorsal_fin": "finTop.obj",
}


# =============================================================================
# Basic vector helpers
# =============================================================================

def v_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_mul(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def rotate_y_pi(v: Vec3) -> Vec3:
    """180 deg rotation around +Y."""
    return (-v[0], v[1], -v[2])


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# =============================================================================
# Mesh container
# =============================================================================

@dataclass
class Mesh:
    name: str
    vertices: List[Vec3] = field(default_factory=list)
    faces: List[Tri] = field(default_factory=list)

    def copy(self, name: str | None = None) -> "Mesh":
        return Mesh(
            name=name or self.name,
            vertices=list(self.vertices),
            faces=list(self.faces),
        )

    def add_vertex(self, p: Vec3) -> int:
        self.vertices.append((float(p[0]), float(p[1]), float(p[2])))
        return len(self.vertices) - 1

    def add_tri(self, a: int, b: int, c: int) -> None:
        if a == b or b == c or c == a:
            return
        self.faces.append((a, b, c))

    def transformed(
        self,
        *,
        translate: Vec3 = (0.0, 0.0, 0.0),
        rotate_y_180: bool = False,
        name: str | None = None,
    ) -> "Mesh":
        verts: List[Vec3] = []
        for p in self.vertices:
            q = rotate_y_pi(p) if rotate_y_180 else p
            verts.append(v_add(q, translate))
        return Mesh(name or self.name, verts, list(self.faces))

    def translate_in_place(self, t: Vec3) -> None:
        self.vertices = [v_add(p, t) for p in self.vertices]

    def reverse_winding(self) -> None:
        self.faces = [(a, c, b) for a, b, c in self.faces]

    def bbox(self) -> Tuple[Vec3, Vec3]:
        if not self.vertices:
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        xs = [p[0] for p in self.vertices]
        ys = [p[1] for p in self.vertices]
        zs = [p[2] for p in self.vertices]
        return (
            (min(xs), min(ys), min(zs)),
            (max(xs), max(ys), max(zs)),
        )

    def bbox_size(self) -> Vec3:
        lo, hi = self.bbox()
        return (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])

    def signed_volume(self) -> float:
        """
        Signed volume of a closed triangular mesh.
        Positive/negative sign depends on face winding.
        """
        total = 0.0
        for ia, ib, ic in self.faces:
            a = self.vertices[ia]
            b = self.vertices[ib]
            c = self.vertices[ic]
            total += dot(a, cross(b, c))
        return total / 6.0

    def volume(self) -> float:
        return abs(self.signed_volume())

    def ensure_positive_volume(self) -> None:
        if self.signed_volume() < 0.0:
            self.reverse_winding()

    def orient_consistently_outward(self) -> None:
        """
        Make neighboring triangles traverse every shared edge in opposite
        directions, then orient the resulting closed surface outward.

        This is deliberately implemented here instead of depending on trimesh,
        because the generator itself should remain lightweight and reproducible.
        """
        if not self.faces:
            return

        # edge -> [(face index, directed edge u->v), ...]
        edge_map: Dict[Tuple[int, int], List[Tuple[int, int, int]]] = {}

        for fi, (a, b, c) in enumerate(self.faces):
            for u, v in ((a, b), (b, c), (c, a)):
                key = (u, v) if u < v else (v, u)
                edge_map.setdefault(key, []).append((fi, u, v))

        # For a consistently oriented manifold surface, two neighboring faces
        # must traverse the shared edge in opposite directions.
        adjacency: Dict[int, List[Tuple[int, bool]]] = {
            i: [] for i in range(len(self.faces))
        }

        for entries in edge_map.values():
            if len(entries) != 2:
                # Boundary/non-manifold edges are handled by later validation.
                continue

            f0, u0, v0 = entries[0]
            f1, u1, v1 = entries[1]

            same_direction = (u0 == u1 and v0 == v1)

            adjacency[f0].append((f1, same_direction))
            adjacency[f1].append((f0, same_direction))

        flip: List[bool | None] = [None] * len(self.faces)

        for seed in range(len(self.faces)):
            if flip[seed] is not None:
                continue

            flip[seed] = False
            stack = [seed]

            while stack:
                current = stack.pop()
                current_flip = bool(flip[current])

                for neighbor, same_direction in adjacency[current]:
                    # If the original directed shared edges are the same,
                    # exactly one of the two faces must be flipped.
                    required = current_flip ^ same_direction

                    if flip[neighbor] is None:
                        flip[neighbor] = required
                        stack.append(neighbor)
                    elif bool(flip[neighbor]) != required:
                        raise ValueError(
                            f"Mesh {self.name!r} has inconsistent/non-orientable "
                            "triangle topology."
                        )

        fixed: List[Tri] = []
        for should_flip, (a, b, c) in zip(flip, self.faces):
            fixed.append((a, c, b) if should_flip else (a, b, c))

        self.faces = fixed

        # Once winding is locally consistent, signed volume can determine
        # whether the entire closed surface faces inward or outward.
        self.ensure_positive_volume()

    def boundary_edge_count(self) -> int:
        """
        Count edges that appear exactly once. Zero means topologically closed
        for a normal manifold triangulated surface.
        """
        counts: Dict[Tuple[int, int], int] = {}
        for a, b, c in self.faces:
            for u, v in ((a, b), (b, c), (c, a)):
                key = (u, v) if u < v else (v, u)
                counts[key] = counts.get(key, 0) + 1
        return sum(1 for count in counts.values() if count == 1)

    def write_obj(self, path: Path, comment: str = "") -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as f:
            f.write("# Generated by generate_bionic_fish.py\n")
            f.write("# Units: meters\n")
            if comment:
                for line in comment.splitlines():
                    f.write(f"# {line}\n")
            f.write(f"o {self.name}\n")
            for x, y, z in self.vertices:
                f.write(f"v {x:.9f} {y:.9f} {z:.9f}\n")
            for a, b, c in self.faces:
                # OBJ is 1-based.
                f.write(f"f {a + 1} {b + 1} {c + 1}\n")


def combine_meshes(name: str, meshes: Sequence[Mesh]) -> Mesh:
    out = Mesh(name)
    offset = 0
    for mesh in meshes:
        out.vertices.extend(mesh.vertices)
        out.faces.extend(
            (a + offset, b + offset, c + offset)
            for a, b, c in mesh.faces
        )
        offset += len(mesh.vertices)
    return out


# =============================================================================
# OBJ import
# =============================================================================

def load_obj(path: Path, name: str) -> Mesh:
    mesh = Mesh(name)
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue

            fields = s.split()

            if fields[0] == "v" and len(fields) >= 4:
                mesh.vertices.append(
                    (float(fields[1]), float(fields[2]), float(fields[3]))
                )

            elif fields[0] == "f" and len(fields) >= 4:
                indices: List[int] = []
                for token in fields[1:]:
                    raw = token.split("/")[0]
                    if not raw:
                        continue
                    idx = int(raw)
                    if idx < 0:
                        idx = len(mesh.vertices) + idx
                    else:
                        idx -= 1
                    indices.append(idx)

                # Fan triangulation.
                for i in range(1, len(indices) - 1):
                    mesh.add_tri(indices[0], indices[i], indices[i + 1])

    if not mesh.vertices or not mesh.faces:
        raise ValueError(f"OBJ contains no usable triangles: {path}")

    mesh.orient_consistently_outward()
    return mesh


# =============================================================================
# Config helpers
# =============================================================================

def require(mapping: dict, key: str, context: str):
    if key not in mapping:
        raise KeyError(f"Missing config key: {context}.{key}")
    return mapping[key]


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError("Top-level YAML value must be a mapping.")

    name = data.get("config_name")
    if name != "BionicFishV1Config":
        raise ValueError(
            f"Expected config_name=BionicFishV1Config, got {name!r}"
        )

    return data


def discover_repo_root(config_path: Path, explicit: Path | None) -> Path:
    if explicit is not None:
        return explicit.resolve()

    candidates = [Path.cwd().resolve()]
    candidates.extend(config_path.resolve().parents)

    for candidate in candidates:
        if (candidate / "simulation").is_dir():
            return candidate

    # Safe fallback: current working directory.
    return Path.cwd().resolve()


# =============================================================================
# Superellipse loft
# =============================================================================

def superellipse_ring(
    x: float,
    width: float,
    height: float,
    exponent: float,
    points: int,
) -> List[Vec3]:
    if points < 8:
        raise ValueError("Cross-section needs at least 8 points.")
    if width <= 0.0 or height <= 0.0:
        raise ValueError("Cross-section width/height must be positive.")
    if exponent <= 0.0:
        raise ValueError("Superellipse exponent must be positive.")

    a = 0.5 * width
    b = 0.5 * height
    power = 2.0 / exponent

    ring: List[Vec3] = []

    for i in range(points):
        theta = 2.0 * math.pi * i / points
        c = math.cos(theta)
        s = math.sin(theta)

        y = a * math.copysign(abs(c) ** power, c)
        z = b * math.copysign(abs(s) ** power, s)

        ring.append((x, y, z))

    # ------------------------------------------------------------------
    # Cross-section area preservation
    # ------------------------------------------------------------------
    # A low-poly physics ring otherwise underestimates the continuous
    # superellipse area noticeably. Scale Y/Z uniformly so the polygonal
    # section preserves the analytic area while retaining its low vertex
    # count. This is important for Stonefish displaced-volume/buoyancy.
    polygon_area_twice = 0.0
    for p0, p1 in zip(ring, ring[1:] + ring[:1]):
        polygon_area_twice += p0[1] * p1[2] - p1[1] * p0[2]

    polygon_area = 0.5 * abs(polygon_area_twice)

    area_factor = (
        4.0
        * math.gamma(1.0 + 1.0 / exponent) ** 2
        / math.gamma(1.0 + 2.0 / exponent)
    )
    analytic_area = area_factor * a * b

    if polygon_area > 0.0 and analytic_area > 0.0:
        scale = math.sqrt(analytic_area / polygon_area)
        ring = [(px, py * scale, pz * scale) for px, py, pz in ring]

    return ring


def smoothstep(t: float) -> float:
    t = clamp(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def interpolate_profile(
    controls: Sequence[dict],
    x: float,
) -> Tuple[float, float]:
    """
    Smooth piecewise interpolation of width and height.

    Control station positions are preserved exactly. Between stations we use a
    cubic smoothstep blend to avoid sharp visible corners in the loft.
    """
    stations = sorted(controls, key=lambda s: float(s["x_m"]))

    if x <= float(stations[0]["x_m"]):
        return (
            float(stations[0]["width_y_m"]),
            float(stations[0]["height_z_m"]),
        )

    if x >= float(stations[-1]["x_m"]):
        return (
            float(stations[-1]["width_y_m"]),
            float(stations[-1]["height_z_m"]),
        )

    for a, b in zip(stations[:-1], stations[1:]):
        xa = float(a["x_m"])
        xb = float(b["x_m"])

        if xa <= x <= xb:
            t = (x - xa) / (xb - xa)
            t = smoothstep(t)

            wa = float(a["width_y_m"])
            wb = float(b["width_y_m"])
            ha = float(a["height_z_m"])
            hb = float(b["height_z_m"])

            return (
                wa + (wb - wa) * t,
                ha + (hb - ha) * t,
            )

    raise RuntimeError("Profile interpolation failed unexpectedly.")


def profile_sample_positions(
    controls: Sequence[dict],
    total_sections: int,
) -> List[float]:
    """
    Generate longitudinal sampling positions while guaranteeing that all
    original control stations remain present.
    """
    if total_sections < len(controls):
        total_sections = len(controls)

    xs_control = sorted({float(s["x_m"]) for s in controls})
    xmin = xs_control[0]
    xmax = xs_control[-1]

    uniform = [
        xmin + (xmax - xmin) * i / (total_sections - 1)
        for i in range(total_sections)
    ]

    xs = sorted(set(round(x, 12) for x in (uniform + xs_control)))
    return xs


def generate_closed_loft(
    *,
    name: str,
    control_sections: Sequence[dict],
    longitudinal_sections: int,
    circumferential_points: int,
    exponent: float,
) -> Mesh:
    xs = profile_sample_positions(control_sections, longitudinal_sections)

    mesh = Mesh(name)
    rings: List[List[int]] = []

    for x in xs:
        width, height = interpolate_profile(control_sections, x)
        ring_points = superellipse_ring(
            x=x,
            width=width,
            height=height,
            exponent=exponent,
            points=circumferential_points,
        )
        ring_ids = [mesh.add_vertex(p) for p in ring_points]
        rings.append(ring_ids)

    # Side surface.
    n = circumferential_points
    for r0, r1 in zip(rings[:-1], rings[1:]):
        for j in range(n):
            jn = (j + 1) % n
            mesh.add_tri(r0[j], r1[j], r1[jn])
            mesh.add_tri(r0[j], r1[jn], r0[jn])

    # End caps.
    x0 = xs[0]
    x1 = xs[-1]
    c0 = mesh.add_vertex((x0, 0.0, 0.0))
    c1 = mesh.add_vertex((x1, 0.0, 0.0))

    r0 = rings[0]
    r1 = rings[-1]

    for j in range(n):
        jn = (j + 1) % n
        mesh.add_tri(c0, r0[jn], r0[j])
        mesh.add_tri(c1, r1[j], r1[jn])

    mesh.orient_consistently_outward()
    return mesh


# =============================================================================
# Tail links
# =============================================================================

def generate_tail_link(
    *,
    name: str,
    proximal_section: dict,
    distal_section: dict,
    longitudinal_sections: int,
    circumferential_points: int,
    exponent: float,
) -> Mesh:
    """
    Generate one articulated tail link in LINK-LOCAL coordinates.

    Local origin:
        proximal joint

    Local axis:
        tail goes toward negative X

    Therefore the .scn joint can be placed at the link origin.
    """
    xp = float(proximal_section["x_m"])
    xd = float(distal_section["x_m"])

    length = abs(xd - xp)

    controls = [
        {
            "x_m": 0.0,
            "width_y_m": float(proximal_section["width_y_m"]),
            "height_z_m": float(proximal_section["height_z_m"]),
        },
        {
            "x_m": -length,
            "width_y_m": float(distal_section["width_y_m"]),
            "height_z_m": float(distal_section["height_z_m"]),
        },
    ]

    return generate_closed_loft(
        name=name,
        control_sections=controls,
        longitudinal_sections=longitudinal_sections,
        circumferential_points=circumferential_points,
        exponent=exponent,
    )


# =============================================================================
# NACA-like pectoral hydrofoil
# =============================================================================

def naca_symmetric_thickness(xi: float, t_ratio: float) -> float:
    """
    Symmetric NACA 00xx thickness distribution.
    xi: 0 leading edge -> 1 trailing edge
    returns half-thickness / chord
    """
    xi = clamp(xi, 0.0, 1.0)
    return 5.0 * t_ratio * (
        0.2969 * math.sqrt(max(xi, 1e-12))
        - 0.1260 * xi
        - 0.3516 * xi**2
        + 0.2843 * xi**3
        - 0.1036 * xi**4
    )


def airfoil_ring(
    *,
    y: float,
    chord: float,
    center_x: float,
    thickness_abs: float,
    half_samples: int,
) -> List[Vec3]:
    if half_samples < 5:
        raise ValueError("half_samples must be >= 5")

    t_ratio = thickness_abs / chord

    # Cosine spacing gives more resolution near leading/trailing edges.
    xis = [
        0.5 * (1.0 - math.cos(math.pi * i / (half_samples - 1)))
        for i in range(half_samples)
    ]

    upper: List[Vec3] = []

    for xi in xis:
        x = center_x + 0.5 * chord - xi * chord
        z = chord * naca_symmetric_thickness(xi, t_ratio)
        upper.append((x, y, z))

    lower: List[Vec3] = []

    # Exclude TE and LE duplicates.
    for xi in reversed(xis[1:-1]):
        x = center_x + 0.5 * chord - xi * chord
        z = -chord * naca_symmetric_thickness(xi, t_ratio)
        lower.append((x, y, z))

    return upper + lower


def generate_pectoral_fin(
    *,
    name: str,
    side_sign: float,
    span: float,
    root_chord: float,
    tip_chord: float,
    thickness: float,
    sweep_angle_deg: float,
    span_stations: int,
    chord_half_samples: int,
) -> Mesh:
    """
    Link-local hydrofoil. Local origin is the fin root joint center.

    side_sign:
        +1 right fin (+Y)
        -1 left fin  (-Y)
    """
    if side_sign not in (-1.0, 1.0):
        raise ValueError("side_sign must be -1 or +1")

    mesh = Mesh(name)
    rings: List[List[int]] = []

    sweep = math.radians(sweep_angle_deg)

    for i in range(span_stations):
        frac = i / (span_stations - 1)
        s = frac * span
        y = side_sign * s

        # Linear taper.
        chord = root_chord + (tip_chord - root_chord) * frac

        # Positive sweep angle moves the tip rearward (-X).
        center_x = -math.tan(sweep) * s

        # Slight tip thinning gives a more realistic hydrodynamic shape.
        local_thickness = thickness * (1.0 - 0.30 * frac)

        ring = airfoil_ring(
            y=y,
            chord=chord,
            center_x=center_x,
            thickness_abs=local_thickness,
            half_samples=chord_half_samples,
        )

        rings.append([mesh.add_vertex(p) for p in ring])

    n = len(rings[0])

    for r0, r1 in zip(rings[:-1], rings[1:]):
        for j in range(n):
            jn = (j + 1) % n
            mesh.add_tri(r0[j], r1[j], r1[jn])
            mesh.add_tri(r0[j], r1[jn], r0[jn])

    # Root and tip caps.
    for ring, reverse in ((rings[0], True), (rings[-1], False)):
        center = mesh.add_vertex(
            tuple(
                sum(mesh.vertices[idx][k] for idx in ring) / len(ring)
                for k in range(3)
            )
        )
        for j in range(n):
            jn = (j + 1) % n
            if reverse:
                mesh.add_tri(center, ring[jn], ring[j])
            else:
                mesh.add_tri(center, ring[j], ring[jn])

    mesh.orient_consistently_outward()
    return mesh


# =============================================================================
# Simple procedural fin fallbacks
# =============================================================================

def extrude_xz_polygon_y(
    *,
    name: str,
    polygon_xz: Sequence[Tuple[float, float]],
    thickness_y: float,
) -> Mesh:
    """
    Closed prism from an X-Z polygon, extruded along Y.
    """
    if len(polygon_xz) < 3:
        raise ValueError("Polygon must contain at least 3 points.")

    mesh = Mesh(name)
    hy = 0.5 * thickness_y

    front = [mesh.add_vertex((x, -hy, z)) for x, z in polygon_xz]
    back = [mesh.add_vertex((x, hy, z)) for x, z in polygon_xz]

    n = len(polygon_xz)

    # Front / back fan triangulation. Polygon is expected convex-ish.
    for i in range(1, n - 1):
        mesh.add_tri(front[0], front[i + 1], front[i])
        mesh.add_tri(back[0], back[i], back[i + 1])

    # Sides.
    for i in range(n):
        j = (i + 1) % n
        mesh.add_tri(front[i], back[i], back[j])
        mesh.add_tri(front[i], back[j], front[j])

    mesh.orient_consistently_outward()
    return mesh


def procedural_caudal_fin(cfg: dict, name: str) -> Mesh:
    dims = [float(v) for v in cfg["initial_dimensions_xyz_m"]]
    length_x, thickness_y, height_z = dims

    # Fish-like forked/swept vertical fin. Root at x=0, tail points -X.
    hz = 0.5 * height_z

    polygon = [
        (0.0, 0.030 * height_z),
        (-0.25 * length_x, 0.42 * height_z),
        (-0.85 * length_x, hz),
        (-1.00 * length_x, 0.20 * height_z),
        (-0.70 * length_x, 0.0),
        (-1.00 * length_x, -0.20 * height_z),
        (-0.85 * length_x, -hz),
        (-0.25 * length_x, -0.42 * height_z),
        (0.0, -0.030 * height_z),
    ]

    return extrude_xz_polygon_y(
        name=name,
        polygon_xz=polygon,
        thickness_y=thickness_y,
    )


def procedural_dorsal_fin(cfg: dict, name: str) -> Mesh:
    dims = [float(v) for v in cfg["source_dimensions_xyz_m"]]
    length_x, thickness_y, height_z = dims

    polygon = [
        (0.40 * length_x, 0.0),
        (0.20 * length_x, -0.50 * height_z),
        (-0.15 * length_x, -height_z),
        (-0.45 * length_x, -0.75 * height_z),
        (-0.60 * length_x, 0.0),
    ]

    return extrude_xz_polygon_y(
        name=name,
        polygon_xz=polygon,
        thickness_y=thickness_y,
    )


# =============================================================================
# fishsim reference mesh resolution
# =============================================================================

def find_fishsim_mesh(
    *,
    fishsim_root: Path | None,
    file_name: str,
) -> Path | None:
    candidates: List[Path] = []

    if fishsim_root is not None:
        candidates.append(
            fishsim_root / "Geometry" / "Meshes" / file_name
        )

    env_root = os.environ.get("FISHSIM_ROOT")
    if env_root:
        candidates.append(
            Path(env_root) / "Geometry" / "Meshes" / file_name
        )

    cwd = Path.cwd()
    for base in (
        cwd / "fishsim",
        cwd.parent / "fishsim",
        cwd / "third_party" / "fishsim",
        cwd / "external" / "fishsim",
    ):
        candidates.append(base / "Geometry" / "Meshes" / file_name)

    for path in candidates:
        if path.is_file():
            return path.resolve()

    return None


def fetch_reference_mesh(
    *,
    output_reference_dir: Path,
    file_name: str,
) -> Path:
    output_reference_dir.mkdir(parents=True, exist_ok=True)
    dst = output_reference_dir / file_name

    if dst.is_file():
        return dst

    url = f"{FISHSIM_RAW_BASE}/{file_name}"

    print(f"[fetch] {url}")
    try:
        with urllib.request.urlopen(url, timeout=30) as response:
            data = response.read()
    except Exception as exc:
        raise RuntimeError(
            f"Failed to download {file_name} from fishsim: {exc}"
        ) from exc

    dst.write_bytes(data)
    return dst


def load_or_generate_reference_fins(
    *,
    cfg: dict,
    fishsim_root: Path | None,
    reference_dir: Path,
    fetch: bool,
    fallback: bool,
) -> Tuple[Mesh, Mesh, dict]:
    caudal_cfg = cfg["caudal_fin"]
    dorsal_cfg = cfg["dorsal_fin"]

    source_status = {}

    # ---------------------------------------------------------------------
    # Caudal
    # ---------------------------------------------------------------------
    tail_file = FISHSIM_REFERENCE_FILES["caudal_fin"]
    tail_path = find_fishsim_mesh(
        fishsim_root=fishsim_root,
        file_name=tail_file,
    )

    if tail_path is None and fetch:
        tail_path = fetch_reference_mesh(
            output_reference_dir=reference_dir,
            file_name=tail_file,
        )

    if tail_path is not None:
        raw = load_obj(tail_path, "CaudalFinReference")

        # fishsim -> BionicFish orientation.
        caudal = raw.transformed(
            rotate_y_180=True,
            name="CaudalFin",
        )

        # Make the fin root x=0 in link-local coordinates.
        _, hi = caudal.bbox()
        caudal.translate_in_place((-hi[0], 0.0, 0.0))

        # Center Y and Z numerically.
        lo, hi = caudal.bbox()
        caudal.translate_in_place(
            (
                0.0,
                -0.5 * (lo[1] + hi[1]),
                -0.5 * (lo[2] + hi[2]),
            )
        )

        caudal.orient_consistently_outward()
        source_status["caudal_fin"] = {
            "mode": "fishsim_reference_mesh",
            "source": str(tail_path),
        }

    elif fallback:
        caudal = procedural_caudal_fin(
            caudal_cfg,
            "CaudalFin",
        )
        source_status["caudal_fin"] = {
            "mode": "procedural_fallback",
            "source": "BionicFishV1Config dimensions",
        }

    else:
        raise FileNotFoundError(
            "fishsim finTail.obj not found. Use one of:\n"
            "  --fishsim-root /path/to/fishsim\n"
            "  --fetch-reference-meshes\n"
            "  --allow-procedural-fin-fallback"
        )

    # ---------------------------------------------------------------------
    # Dorsal
    # ---------------------------------------------------------------------
    top_file = FISHSIM_REFERENCE_FILES["dorsal_fin"]
    top_path = find_fishsim_mesh(
        fishsim_root=fishsim_root,
        file_name=top_file,
    )

    if top_path is None and fetch:
        top_path = fetch_reference_mesh(
            output_reference_dir=reference_dir,
            file_name=top_file,
        )

    if top_path is not None:
        raw = load_obj(top_path, "DorsalFinReference")
        dorsal = raw.transformed(
            rotate_y_180=True,
            name="DorsalFin",
        )

        # Center only Y. Keep fishsim local X/Z relation intact.
        lo, hi = dorsal.bbox()
        dorsal.translate_in_place(
            (0.0, -0.5 * (lo[1] + hi[1]), 0.0)
        )

        dorsal.orient_consistently_outward()
        source_status["dorsal_fin"] = {
            "mode": "fishsim_reference_mesh",
            "source": str(top_path),
        }

    elif fallback:
        dorsal = procedural_dorsal_fin(
            dorsal_cfg,
            "DorsalFin",
        )
        source_status["dorsal_fin"] = {
            "mode": "procedural_fallback",
            "source": "BionicFishV1Config dimensions",
        }

    else:
        raise FileNotFoundError(
            "fishsim finTop.obj not found. Use one of:\n"
            "  --fishsim-root /path/to/fishsim\n"
            "  --fetch-reference-meshes\n"
            "  --allow-procedural-fin-fallback"
        )

    return caudal, dorsal, source_status


# =============================================================================
# Validation and manifest
# =============================================================================

def mesh_stats(mesh: Mesh) -> dict:
    lo, hi = mesh.bbox()
    return {
        "vertices": len(mesh.vertices),
        "faces": len(mesh.faces),
        "boundary_edges": mesh.boundary_edge_count(),
        "closed": mesh.boundary_edge_count() == 0,
        "volume_m3": mesh.volume(),
        "bbox_min_xyz_m": list(lo),
        "bbox_max_xyz_m": list(hi),
        "bbox_dimensions_xyz_m": list(mesh.bbox_size()),
    }


def assert_closed(mesh: Mesh, label: str) -> None:
    count = mesh.boundary_edge_count()
    if count != 0:
        raise ValueError(
            f"{label} is not closed/watertight: {count} boundary edges"
        )


def midpoint_int(values: Sequence[int], fallback: int) -> int:
    if not values:
        return fallback
    return int(round(0.5 * (int(values[0]) + int(values[-1]))))


# =============================================================================
# Main generation
# =============================================================================

def generate(
    *,
    config: dict,
    repo_root: Path,
    fishsim_root: Path | None,
    fetch_reference_meshes: bool,
    allow_fallback: bool,
    output_override: Path | None,
    no_preview: bool,
) -> Path:
    bf = config["bionic_fish_v1"]

    output_layout = bf["output_layout"]

    if output_override is not None:
        output_root = output_override.resolve()
        visual_dir = output_root / "visual"
        physics_dir = output_root / "physics"
    else:
        output_root = repo_root / output_layout["root"]
        visual_dir = repo_root / output_layout["visual_dir"]
        physics_dir = repo_root / output_layout["physics_dir"]

    reference_dir = output_root / "reference" / "fishsim"

    visual_dir.mkdir(parents=True, exist_ok=True)
    physics_dir.mkdir(parents=True, exist_ok=True)
    reference_dir.mkdir(parents=True, exist_ok=True)

    pipeline = bf["mesh_pipeline"]

    visual_long = midpoint_int(
        pipeline["visual_mesh"]["resolution_guideline"][
            "longitudinal_sections_range"
        ],
        64,
    )
    visual_circ = midpoint_int(
        pipeline["visual_mesh"]["resolution_guideline"][
            "points_per_section_range"
        ],
        28,
    )

    physics_long = midpoint_int(
        pipeline["physics_mesh"]["resolution_guideline"][
            "longitudinal_sections_range"
        ],
        14,
    )
    physics_circ = midpoint_int(
        pipeline["physics_mesh"]["resolution_guideline"][
            "points_per_section_range"
        ],
        10,
    )

    body_cfg = bf["body"]
    body_controls = body_cfg["cross_sections"]
    body_exp = float(body_cfg["cross_section_superellipse_exponent"])

    # ---------------------------------------------------------------------
    # Body
    # ---------------------------------------------------------------------
    body_visual = generate_closed_loft(
        name="Body",
        control_sections=body_controls,
        longitudinal_sections=visual_long,
        circumferential_points=visual_circ,
        exponent=body_exp,
    )

    body_physics = generate_closed_loft(
        name="Body",
        control_sections=body_controls,
        longitudinal_sections=physics_long,
        circumferential_points=physics_circ,
        exponent=body_exp,
    )

    # ---------------------------------------------------------------------
    # Tail links
    # ---------------------------------------------------------------------
    envelope = bf["tail"]["smooth_mesh_envelope"]["sections"]

    if len(envelope) != 6:
        raise ValueError(
            "Expected six tail envelope stations: Joint0..Joint4 + FinRoot."
        )

    tail_visual: List[Mesh] = []
    tail_physics: List[Mesh] = []

    for i in range(5):
        proximal = envelope[i]
        distal = envelope[i + 1]

        tail_visual.append(
            generate_tail_link(
                name=f"Tail{i}",
                proximal_section=proximal,
                distal_section=distal,
                longitudinal_sections=7,
                circumferential_points=max(20, visual_circ),
                exponent=2.0,
            )
        )

        tail_physics.append(
            generate_tail_link(
                name=f"Tail{i}",
                proximal_section=proximal,
                distal_section=distal,
                longitudinal_sections=3,
                circumferential_points=max(8, physics_circ),
                exponent=2.0,
            )
        )

    # ---------------------------------------------------------------------
    # Pectoral fins
    # ---------------------------------------------------------------------
    pcfg = bf["pectoral_fins"]

    common_fin_args = dict(
        span=float(pcfg["span_m"]),
        root_chord=float(pcfg["root_chord_m"]),
        tip_chord=float(pcfg["tip_chord_m"]),
        thickness=float(pcfg["thickness_m"]),
        sweep_angle_deg=float(pcfg["sweep_angle_deg"]),
    )

    left_visual = generate_pectoral_fin(
        name="LeftPectoralFin",
        side_sign=-1.0,
        span_stations=15,
        chord_half_samples=18,
        **common_fin_args,
    )

    right_visual = generate_pectoral_fin(
        name="RightPectoralFin",
        side_sign=1.0,
        span_stations=15,
        chord_half_samples=18,
        **common_fin_args,
    )

    left_physics = generate_pectoral_fin(
        name="LeftPectoralFin",
        side_sign=-1.0,
        span_stations=6,
        chord_half_samples=8,
        **common_fin_args,
    )

    right_physics = generate_pectoral_fin(
        name="RightPectoralFin",
        side_sign=1.0,
        span_stations=6,
        chord_half_samples=8,
        **common_fin_args,
    )

    # ---------------------------------------------------------------------
    # fishsim caudal + dorsal fins
    # ---------------------------------------------------------------------
    caudal, dorsal, reference_status = load_or_generate_reference_fins(
        cfg=bf,
        fishsim_root=fishsim_root,
        reference_dir=reference_dir,
        fetch=fetch_reference_meshes,
        fallback=allow_fallback,
    )

    # Source meshes are already modest (< 1k faces), so V1 uses the same
    # shape for visual and physics. Later we can introduce decimation if
    # profiling shows this matters.
    caudal_visual = caudal.copy("CaudalFin")
    caudal_physics = caudal.copy("CaudalFin")
    dorsal_visual = dorsal.copy("DorsalFin")
    dorsal_physics = dorsal.copy("DorsalFin")

    # ---------------------------------------------------------------------
    # Validate watertightness.
    # ---------------------------------------------------------------------
    all_generated = [
        ("body_visual", body_visual),
        ("body_physics", body_physics),
        *[(f"tail{i}_visual", m) for i, m in enumerate(tail_visual)],
        *[(f"tail{i}_physics", m) for i, m in enumerate(tail_physics)],
        ("left_pectoral_visual", left_visual),
        ("right_pectoral_visual", right_visual),
        ("left_pectoral_physics", left_physics),
        ("right_pectoral_physics", right_physics),
        ("caudal_visual", caudal_visual),
        ("caudal_physics", caudal_physics),
        ("dorsal_visual", dorsal_visual),
        ("dorsal_physics", dorsal_physics),
    ]

    for label, mesh in all_generated:
        assert_closed(mesh, label)

    # ---------------------------------------------------------------------
    # Write part OBJ files.
    # ---------------------------------------------------------------------
    body_visual.write_obj(
        visual_dir / "body.obj",
        "Body-local coordinates. BionicFish body frame origin.",
    )
    body_physics.write_obj(
        physics_dir / "body.obj",
        "Low-poly closed body used for Stonefish physics.",
    )

    for i, mesh in enumerate(tail_visual):
        mesh.write_obj(
            visual_dir / f"tail_{i}.obj",
            f"Tail{i} link-local mesh. Origin is Joint{i}. Tail extends -X.",
        )

    for i, mesh in enumerate(tail_physics):
        mesh.write_obj(
            physics_dir / f"tail_{i}.obj",
            f"Low-poly Tail{i} link-local physics mesh. Origin is Joint{i}.",
        )

    left_visual.write_obj(
        visual_dir / "left_pectoral.obj",
        "Left pectoral link-local hydrofoil. Origin is left pectoral joint.",
    )
    right_visual.write_obj(
        visual_dir / "right_pectoral.obj",
        "Right pectoral link-local hydrofoil. Origin is right pectoral joint.",
    )

    left_physics.write_obj(
        physics_dir / "left_pectoral.obj",
        "Low-poly left pectoral physics hydrofoil.",
    )
    right_physics.write_obj(
        physics_dir / "right_pectoral.obj",
        "Low-poly right pectoral physics hydrofoil.",
    )

    caudal_visual.write_obj(
        visual_dir / "caudal_fin.obj",
        "Caudal-fin link-local mesh. Root is x=0; fin extends toward -X.",
    )
    caudal_physics.write_obj(
        physics_dir / "caudal_fin.obj",
        "Caudal-fin physics mesh.",
    )

    dorsal_visual.write_obj(
        visual_dir / "dorsal_fin.obj",
        "Dorsal-fin link-local mesh. Apply the configured Body mount transform.",
    )
    dorsal_physics.write_obj(
        physics_dir / "dorsal_fin.obj",
        "Dorsal-fin physics mesh.",
    )

    # ---------------------------------------------------------------------
    # Neutral assembly preview in BODY coordinates.
    # ---------------------------------------------------------------------
    preview_parts: List[Mesh] = [body_visual.copy("Body")]

    tail_joint_positions = bf["tail"]["joint_positions_body_xyz_m"]

    for i, mesh in enumerate(tail_visual):
        t = tuple(float(v) for v in tail_joint_positions[f"Joint{i}"])
        preview_parts.append(
            mesh.transformed(
                translate=t,
                name=f"Tail{i}",
            )
        )

    fin_root_x = float(envelope[-1]["x_m"])
    preview_parts.append(
        caudal_visual.transformed(
            translate=(fin_root_x, 0.0, 0.0),
            name="CaudalFin",
        )
    )

    dorsal_mount = tuple(
        float(v)
        for v in bf["dorsal_fin"]["bionic_mount_position_body_xyz_m"]
    )
    preview_parts.append(
        dorsal_visual.transformed(
            translate=dorsal_mount,
            name="DorsalFin",
        )
    )

    left_mount = tuple(
        float(v)
        for v in pcfg["left"]["root_position_body_xyz_m"]
    )
    right_mount = tuple(
        float(v)
        for v in pcfg["right"]["root_position_body_xyz_m"]
    )

    preview_parts.append(
        left_visual.transformed(
            translate=left_mount,
            name="LeftPectoralFin",
        )
    )
    preview_parts.append(
        right_visual.transformed(
            translate=right_mount,
            name="RightPectoralFin",
        )
    )

    preview = combine_meshes(
        "BionicFishV1_NeutralAssembly",
        preview_parts,
    )

    if not no_preview:
        preview.write_obj(
            output_root / "assembly_preview.obj",
            (
                "Neutral-pose body-frame assembly for visual inspection only.\n"
                "Do NOT use this combined mesh as the articulated Stonefish robot."
            ),
        )

    # ---------------------------------------------------------------------
    # Manifest.
    # ---------------------------------------------------------------------
    manifest = {
        "generator": "generate_bionic_fish.py",
        "config_name": config.get("config_name"),
        "config_revision": config.get("config_revision"),
        "units": "m",
        "coordinate_frame": {
            "x": "forward/head",
            "y": "right",
            "z": "down",
        },
        "reference_meshes": reference_status,
        "resolution": {
            "visual": {
                "body_longitudinal_sections_requested": visual_long,
                "body_circumferential_points": visual_circ,
            },
            "physics": {
                "body_longitudinal_sections_requested": physics_long,
                "body_circumferential_points": physics_circ,
            },
        },
        "link_origins_body_xyz_m": {
            "Body": [0.0, 0.0, 0.0],
            **{
                f"Tail{i}": list(
                    map(
                        float,
                        tail_joint_positions[f"Joint{i}"],
                    )
                )
                for i in range(5)
            },
            "CaudalFin": [fin_root_x, 0.0, 0.0],
            "DorsalFin": list(dorsal_mount),
            "LeftPectoralFin": list(left_mount),
            "RightPectoralFin": list(right_mount),
        },
        "visual_meshes": {
            "body.obj": mesh_stats(body_visual),
            **{
                f"tail_{i}.obj": mesh_stats(mesh)
                for i, mesh in enumerate(tail_visual)
            },
            "caudal_fin.obj": mesh_stats(caudal_visual),
            "dorsal_fin.obj": mesh_stats(dorsal_visual),
            "left_pectoral.obj": mesh_stats(left_visual),
            "right_pectoral.obj": mesh_stats(right_visual),
        },
        "physics_meshes": {
            "body.obj": mesh_stats(body_physics),
            **{
                f"tail_{i}.obj": mesh_stats(mesh)
                for i, mesh in enumerate(tail_physics)
            },
            "caudal_fin.obj": mesh_stats(caudal_physics),
            "dorsal_fin.obj": mesh_stats(dorsal_physics),
            "left_pectoral.obj": mesh_stats(left_physics),
            "right_pectoral.obj": mesh_stats(right_physics),
        },
        "assembly_preview": (
            mesh_stats(preview)
            if not no_preview
            else {"status": "disabled"}
        ),
        "important": [
            (
                "assembly_preview.obj is only a neutral-pose visualization. "
                "The real Stonefish robot must use the separate link meshes."
            ),
            (
                "Tail link origins are their proximal joints, so the generated "
                "meshes are ready for an articulated kinematic chain."
            ),
            (
                "Do not freeze vehicle mass/inertia until physics-mesh volumes "
                "and Stonefish link mass distribution are calculated."
            ),
        ],
    }

    manifest_path = output_root / "mesh_manifest.yaml"
    with manifest_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(
            manifest,
            f,
            sort_keys=False,
            allow_unicode=True,
            width=110,
        )

    # ---------------------------------------------------------------------
    # Reference attribution.
    # ---------------------------------------------------------------------
    attribution = output_root / "REFERENCE_SOURCES.md"
    attribution.write_text(
        """# BionicFish V1 reference geometry

The BionicFish V1 mechanical reference is based in part on:

- ETH Zürich SRL `srl-ethz/fishsim`
- Repository: https://github.com/srl-ethz/fishsim
- Reference meshes:
  - `Geometry/Meshes/finTail.obj`
  - `Geometry/Meshes/finTop.obj`

The fishsim repository is distributed under the license stated in that
repository. Keep the upstream license/attribution when vendoring its files.

The BionicFish body, articulated tail envelopes, pectoral hydrofoils,
coordinate conversion and Stonefish-oriented layout are generated by the
RL-stimu-fish project and are not direct copies of the fishsim MuJoCo body
boxes.
""",
        encoding="utf-8",
    )

    print()
    print("BionicFish V1 mesh generation complete")
    print("======================================")
    print(f"Config revision : {config.get('config_revision')}")
    print(f"Output root     : {output_root}")
    print(f"Visual dir      : {visual_dir}")
    print(f"Physics dir     : {physics_dir}")
    print(f"Manifest        : {manifest_path}")
    if not no_preview:
        print(f"Assembly preview: {output_root / 'assembly_preview.obj'}")
    print()
    print(
        f"Body physics volume: {body_physics.volume():.6f} m^3 "
        f"({body_physics.volume() * 1000.0:.3f} L)"
    )
    print(
        "All generated part meshes passed the watertight boundary-edge check."
    )
    print()

    return output_root


# =============================================================================
# CLI
# =============================================================================

def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Generate BionicFish V1 OBJ meshes from YAML config."
    )

    p.add_argument(
        "--config",
        type=Path,
        required=True,
        help="Path to bionic_fish_v1_config YAML.",
    )

    p.add_argument(
        "--repo-root",
        type=Path,
        default=None,
        help=(
            "RL-stimu-fish repository root. Auto-detected if omitted."
        ),
    )

    p.add_argument(
        "--output-root",
        type=Path,
        default=None,
        help=(
            "Override output directory. Useful for testing; otherwise "
            "output_layout from the YAML is used."
        ),
    )

    p.add_argument(
        "--fishsim-root",
        type=Path,
        default=None,
        help="Optional local clone of srl-ethz/fishsim.",
    )

    p.add_argument(
        "--fetch-reference-meshes",
        action="store_true",
        help=(
            "Download finTail.obj and finTop.obj from the fishsim GitHub "
            "repository if no local copy is found."
        ),
    )

    p.add_argument(
        "--allow-procedural-fin-fallback",
        action="store_true",
        help=(
            "If fishsim fin meshes are unavailable, generate approximate "
            "closed fins from config dimensions. Manifest records fallback."
        ),
    )

    p.add_argument(
        "--no-preview",
        action="store_true",
        help="Do not generate assembly_preview.obj.",
    )

    p.add_argument(
        "--validate-only",
        action="store_true",
        help="Validate config and print key values without generating meshes.",
    )

    return p


def validate_config(config: dict) -> None:
    bf = require(config, "bionic_fish_v1", "root")

    body = require(bf, "body", "bionic_fish_v1")
    sections = require(body, "cross_sections", "bionic_fish_v1.body")

    if len(sections) < 4:
        raise ValueError("Body needs at least four cross-section stations.")

    tail = require(bf, "tail", "bionic_fish_v1")
    if int(tail["segment_count"]) != 5:
        raise ValueError("BionicFish V1 expects exactly five tail segments.")

    envelope = tail["smooth_mesh_envelope"]["sections"]
    if len(envelope) != 6:
        raise ValueError(
            "Tail smooth_mesh_envelope must contain 6 stations."
        )

    pectoral = require(bf, "pectoral_fins", "bionic_fish_v1")
    if int(pectoral["count"]) != 2:
        raise ValueError("BionicFish V1 expects two pectoral fins.")

    for value_name in (
        "span_m",
        "root_chord_m",
        "tip_chord_m",
        "thickness_m",
    ):
        if float(pectoral[value_name]) <= 0.0:
            raise ValueError(
                f"pectoral_fins.{value_name} must be positive."
            )

    print("Config validation: OK")
    print(f"  revision      : {config.get('config_revision')}")
    print(f"  body sections : {len(sections)}")
    print(f"  tail segments : {tail['segment_count']}")
    print(
        f"  pectoral span : {float(pectoral['span_m']):.3f} m"
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)

    config_path = args.config.resolve()

    if not config_path.is_file():
        raise FileNotFoundError(f"Config not found: {config_path}")

    config = load_config(config_path)
    validate_config(config)

    if args.validate_only:
        return 0

    repo_root = discover_repo_root(
        config_path=config_path,
        explicit=args.repo_root,
    )

    fishsim_root = (
        args.fishsim_root.resolve()
        if args.fishsim_root is not None
        else None
    )

    generate(
        config=config,
        repo_root=repo_root,
        fishsim_root=fishsim_root,
        fetch_reference_meshes=args.fetch_reference_meshes,
        allow_fallback=args.allow_procedural_fin_fallback,
        output_override=args.output_root,
        no_preview=args.no_preview,
    )

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
