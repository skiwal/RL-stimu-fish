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
   generated for inspection.
8. OBJ files explicitly contain smooth vertex normals (`vn`) so Stonefish
   can correctly light the generated surfaces.

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
        --config simulation/config/bionic_fish_v1_config.yaml \
        --fishsim-root ../fishsim

Dependency:

    python3 -m pip install pyyaml
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
import urllib.request
from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

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
    "https://raw.githubusercontent.com/"
    "srl-ethz/fishsim/main/Geometry/Meshes"
)

FISHSIM_REFERENCE_FILES = {
    "caudal_fin": "finTail.obj",
    "dorsal_fin": "finTop.obj",
}


# ================================================================
# Vector helpers
# ================================================================

def v_add(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[0] + b[0],
        a[1] + b[1],
        a[2] + b[2],
    )


def v_sub(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[0] - b[0],
        a[1] - b[1],
        a[2] - b[2],
    )


def dot(a: Vec3, b: Vec3) -> float:
    return (
        a[0] * b[0]
        + a[1] * b[1]
        + a[2] * b[2]
    )


def cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def length(v: Vec3) -> float:
    return math.sqrt(
        dot(v, v)
    )


def normalize(
    v: Vec3,
    eps: float = 1e-15,
) -> Vec3:

    n = length(v)

    if n <= eps:
        return (
            0.0,
            0.0,
            0.0,
        )

    inv = 1.0 / n

    return (
        v[0] * inv,
        v[1] * inv,
        v[2] * inv,
    )


def rotate_y_pi(v: Vec3) -> Vec3:
    """
    Rotate by 180 degrees around +Y.

    fishsim:
        tail approximately +X

    BionicFish:
        tail -X
    """

    return (
        -v[0],
        v[1],
        -v[2],
    )


def clamp(
    value: float,
    lo: float,
    hi: float,
) -> float:

    return max(
        lo,
        min(
            hi,
            value,
        ),
    )


# ================================================================
# Mesh
# ================================================================

@dataclass
class Mesh:

    name: str

    vertices: List[Vec3] = field(
        default_factory=list
    )

    faces: List[Tri] = field(
        default_factory=list
    )


    def copy(
        self,
        name: str | None = None,
    ) -> "Mesh":

        return Mesh(
            name=name or self.name,
            vertices=list(
                self.vertices
            ),
            faces=list(
                self.faces
            ),
        )


    def add_vertex(
        self,
        point: Vec3,
    ) -> int:

        self.vertices.append(
            (
                float(point[0]),
                float(point[1]),
                float(point[2]),
            )
        )

        return (
            len(self.vertices)
            - 1
        )


    def add_tri(
        self,
        a: int,
        b: int,
        c: int,
    ) -> None:

        if (
            a == b
            or b == c
            or c == a
        ):
            return

        self.faces.append(
            (
                a,
                b,
                c,
            )
        )


    def transformed(
        self,
        *,
        translate: Vec3 = (
            0.0,
            0.0,
            0.0,
        ),
        rotate_y_180: bool = False,
        name: str | None = None,
    ) -> "Mesh":

        vertices: List[Vec3] = []

        for point in self.vertices:

            q = (
                rotate_y_pi(point)
                if rotate_y_180
                else point
            )

            vertices.append(
                v_add(
                    q,
                    translate,
                )
            )

        return Mesh(
            name or self.name,
            vertices,
            list(self.faces),
        )


    def translate_in_place(
        self,
        translation: Vec3,
    ) -> None:

        self.vertices = [
            v_add(
                point,
                translation,
            )
            for point
            in self.vertices
        ]


    def reverse_winding(
        self,
    ) -> None:

        self.faces = [
            (
                a,
                c,
                b,
            )
            for a, b, c
            in self.faces
        ]


    def bbox(
        self,
    ) -> Tuple[Vec3, Vec3]:

        if not self.vertices:

            return (
                (
                    0.0,
                    0.0,
                    0.0,
                ),
                (
                    0.0,
                    0.0,
                    0.0,
                ),
            )

        xs = [
            p[0]
            for p
            in self.vertices
        ]

        ys = [
            p[1]
            for p
            in self.vertices
        ]

        zs = [
            p[2]
            for p
            in self.vertices
        ]

        return (
            (
                min(xs),
                min(ys),
                min(zs),
            ),
            (
                max(xs),
                max(ys),
                max(zs),
            ),
        )


    def bbox_size(
        self,
    ) -> Vec3:

        lo, hi = self.bbox()

        return (
            hi[0] - lo[0],
            hi[1] - lo[1],
            hi[2] - lo[2],
        )


    def signed_volume(
        self,
    ) -> float:

        total = 0.0

        for ia, ib, ic in self.faces:

            a = self.vertices[ia]
            b = self.vertices[ib]
            c = self.vertices[ic]

            total += dot(
                a,
                cross(
                    b,
                    c,
                ),
            )

        return (
            total
            / 6.0
        )


    def volume(
        self,
    ) -> float:

        return abs(
            self.signed_volume()
        )


    def ensure_positive_volume(
        self,
    ) -> None:

        if (
            self.signed_volume()
            < 0.0
        ):
            self.reverse_winding()


    def orient_consistently_outward(
        self,
    ) -> None:
        """
        Ensure all neighboring triangles have compatible winding,
        then orient the entire closed surface outward.
        """

        if not self.faces:
            return

        edge_map: Dict[
            Tuple[int, int],
            List[Tuple[int, int, int]]
        ] = {}


        for face_index, (
            a,
            b,
            c,
        ) in enumerate(
            self.faces
        ):

            for u, v in (
                (a, b),
                (b, c),
                (c, a),
            ):

                key = (
                    (u, v)
                    if u < v
                    else (v, u)
                )

                edge_map.setdefault(
                    key,
                    [],
                ).append(
                    (
                        face_index,
                        u,
                        v,
                    )
                )


        adjacency: Dict[
            int,
            List[Tuple[int, bool]]
        ] = {

            i: []

            for i in range(
                len(self.faces)
            )
        }


        for entries in edge_map.values():

            if len(entries) != 2:
                continue

            f0, u0, v0 = entries[0]
            f1, u1, v1 = entries[1]

            same_direction = (
                u0 == u1
                and v0 == v1
            )

            adjacency[f0].append(
                (
                    f1,
                    same_direction,
                )
            )

            adjacency[f1].append(
                (
                    f0,
                    same_direction,
                )
            )


        flip: List[
            bool | None
        ] = [

            None

            for _ in self.faces
        ]


        for seed in range(
            len(self.faces)
        ):

            if (
                flip[seed]
                is not None
            ):
                continue

            flip[seed] = False

            stack = [
                seed
            ]


            while stack:

                current = (
                    stack.pop()
                )

                current_flip = bool(
                    flip[current]
                )


                for (
                    neighbor,
                    same_direction,
                ) in adjacency[
                    current
                ]:

                    required = (
                        current_flip
                        ^ same_direction
                    )

                    if (
                        flip[neighbor]
                        is None
                    ):

                        flip[neighbor] = (
                            required
                        )

                        stack.append(
                            neighbor
                        )

                    elif (
                        bool(
                            flip[neighbor]
                        )
                        != required
                    ):

                        raise ValueError(
                            f"Mesh {self.name!r} "
                            "has inconsistent "
                            "or non-orientable "
                            "triangle topology."
                        )


        fixed_faces: List[Tri] = []


        for (
            should_flip,
            (
                a,
                b,
                c,
            ),
        ) in zip(
            flip,
            self.faces,
        ):

            if should_flip:

                fixed_faces.append(
                    (
                        a,
                        c,
                        b,
                    )
                )

            else:

                fixed_faces.append(
                    (
                        a,
                        b,
                        c,
                    )
                )


        self.faces = (
            fixed_faces
        )

        self.ensure_positive_volume()


    def boundary_edge_count(
        self,
    ) -> int:

        counts: Dict[
            Tuple[int, int],
            int
        ] = {}


        for a, b, c in self.faces:

            for u, v in (
                (a, b),
                (b, c),
                (c, a),
            ):

                key = (
                    (u, v)
                    if u < v
                    else (v, u)
                )

                counts[key] = (
                    counts.get(
                        key,
                        0,
                    )
                    + 1
                )


        return sum(
            1
            for count
            in counts.values()
            if count == 1
        )


    # ============================================================
    # NEW:
    # Explicit smooth normals for Stonefish
    # ============================================================

    def compute_vertex_normals(
        self,
    ) -> List[Vec3]:
        """
        Compute one area-weighted smooth unit normal per vertex.

        For triangle:

            a, b, c

        raw face normal:

            n = (b-a) x (c-a)

        The magnitude of this cross product is 2 * triangle area.

        Therefore adding the raw vector to every incident vertex
        automatically produces area-weighted smoothing.

        This method assumes the triangle winding has already been
        oriented consistently outward.
        """

        accum: List[
            List[float]
        ] = [

            [
                0.0,
                0.0,
                0.0,
            ]

            for _
            in self.vertices
        ]


        for ia, ib, ic in self.faces:

            a = self.vertices[ia]
            b = self.vertices[ib]
            c = self.vertices[ic]


            ab = v_sub(
                b,
                a,
            )

            ac = v_sub(
                c,
                a,
            )


            face_normal = cross(
                ab,
                ac,
            )


            # Skip degenerate triangles.
            if (
                dot(
                    face_normal,
                    face_normal,
                )
                <= 1e-30
            ):
                continue


            for index in (
                ia,
                ib,
                ic,
            ):

                accum[index][0] += (
                    face_normal[0]
                )

                accum[index][1] += (
                    face_normal[1]
                )

                accum[index][2] += (
                    face_normal[2]
                )


        normals: List[Vec3] = []

        invalid_vertices: List[
            int
        ] = []


        for index, value in enumerate(
            accum
        ):

            normal = normalize(
                (
                    value[0],
                    value[1],
                    value[2],
                )
            )


            if (
                dot(
                    normal,
                    normal,
                )
                <= 0.0
            ):

                invalid_vertices.append(
                    index
                )

                normals.append(
                    (
                        0.0,
                        0.0,
                        1.0,
                    )
                )

            else:

                normals.append(
                    normal
                )


        if invalid_vertices:

            raise ValueError(
                f"Mesh {self.name!r} "
                f"contains "
                f"{len(invalid_vertices)} "
                "vertices without a valid "
                "surface normal."
            )


        return normals


    def write_obj(
        self,
        path: Path,
        comment: str = "",
    ) -> None:
        """
        Write Stonefish-friendly Wavefront OBJ.

        Output form:

            v  x y z
            vn nx ny nz

            f  1//1 2//2 3//3

        One explicit normal is written per vertex.

        The position index and normal index are therefore identical.
        """

        path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )


        normals = (
            self.compute_vertex_normals()
        )


        if (
            len(normals)
            != len(self.vertices)
        ):

            raise RuntimeError(
                f"Mesh {self.name!r}: "
                "vertex/normal count mismatch."
            )


        with path.open(
            "w",
            encoding="utf-8",
        ) as file:

            file.write(
                "# Generated by "
                "generate_bionic_fish.py\n"
            )

            file.write(
                "# Units: meters\n"
            )

            file.write(
                "# Explicit smooth normals "
                "included for Stonefish.\n"
            )


            if comment:

                for line in (
                    comment.splitlines()
                ):

                    file.write(
                        f"# {line}\n"
                    )


            file.write(
                f"o {self.name}\n"
            )


            # Smooth shading hint for OBJ-compatible tools.
            file.write(
                "s 1\n"
            )


            # Positions
            for x, y, z in self.vertices:

                file.write(
                    f"v "
                    f"{x:.9f} "
                    f"{y:.9f} "
                    f"{z:.9f}\n"
                )


            # Normals
            for (
                nx,
                ny,
                nz,
            ) in normals:

                file.write(
                    f"vn "
                    f"{nx:.9f} "
                    f"{ny:.9f} "
                    f"{nz:.9f}\n"
                )


            # Faces:
            #
            # vertex_index // normal_index
            #
            # OBJ indexing is 1-based.
            for (
                a,
                b,
                c,
            ) in self.faces:

                ai = a + 1
                bi = b + 1
                ci = c + 1


                file.write(
                    f"f "
                    f"{ai}//{ai} "
                    f"{bi}//{bi} "
                    f"{ci}//{ci}\n"
                )


# ================================================================
# Mesh utility
# ================================================================

def combine_meshes(
    name: str,
    meshes: Sequence[Mesh],
) -> Mesh:

    output = Mesh(
        name
    )

    offset = 0


    for mesh in meshes:

        output.vertices.extend(
            mesh.vertices
        )


        output.faces.extend(

            (
                a + offset,
                b + offset,
                c + offset,
            )

            for a, b, c
            in mesh.faces
        )


        offset += (
            len(mesh.vertices)
        )


    return output


# ================================================================
# OBJ loading
# ================================================================

def load_obj(
    path: Path,
    name: str,
) -> Mesh:

    mesh = Mesh(
        name
    )


    with path.open(
        "r",
        encoding="utf-8",
        errors="replace",
    ) as file:

        for line in file:

            stripped = (
                line.strip()
            )


            if (
                not stripped
                or stripped.startswith(
                    "#"
                )
            ):
                continue


            fields = (
                stripped.split()
            )


            if (
                fields[0] == "v"
                and len(fields) >= 4
            ):

                mesh.vertices.append(
                    (
                        float(fields[1]),
                        float(fields[2]),
                        float(fields[3]),
                    )
                )


            elif (
                fields[0] == "f"
                and len(fields) >= 4
            ):

                indices: List[int] = []


                for token in fields[1:]:

                    # Supports:
                    #
                    #   f 1
                    #   f 1/2
                    #   f 1//3
                    #   f 1/2/3
                    #
                    raw = (
                        token.split(
                            "/"
                        )[0]
                    )


                    if not raw:
                        continue


                    index = int(
                        raw
                    )


                    if index < 0:

                        index = (
                            len(
                                mesh.vertices
                            )
                            + index
                        )

                    else:

                        index -= 1


                    indices.append(
                        index
                    )


                # Fan triangulation.
                for i in range(
                    1,
                    len(indices) - 1,
                ):

                    mesh.add_tri(
                        indices[0],
                        indices[i],
                        indices[i + 1],
                    )


    if (
        not mesh.vertices
        or not mesh.faces
    ):

        raise ValueError(
            f"OBJ contains no usable "
            f"triangles: {path}"
        )


    mesh.orient_consistently_outward()


    return mesh


# ================================================================
# Config helpers
# ================================================================

def require(
    mapping: dict,
    key: str,
    context: str,
):

    if key not in mapping:

        raise KeyError(
            f"Missing config key: "
            f"{context}.{key}"
        )


    return mapping[key]


def load_config(
    path: Path,
) -> dict:

    with path.open(
        "r",
        encoding="utf-8",
    ) as file:

        data = (
            yaml.safe_load(
                file
            )
        )


    if not isinstance(
        data,
        dict,
    ):

        raise ValueError(
            "Top-level YAML value "
            "must be a mapping."
        )


    name = (
        data.get(
            "config_name"
        )
    )


    if (
        name
        != "BionicFishV1Config"
    ):

        raise ValueError(
            "Expected "
            "config_name="
            "BionicFishV1Config, "
            f"got {name!r}"
        )


    return data


def discover_repo_root(
    config_path: Path,
    explicit: Path | None,
) -> Path:

    if explicit is not None:

        return (
            explicit.resolve()
        )


    candidates = [
        Path.cwd().resolve()
    ]


    candidates.extend(
        config_path.resolve().parents
    )


    for candidate in candidates:

        if (
            candidate
            / "simulation"
        ).is_dir():

            return candidate


    return (
        Path.cwd().resolve()
    )


# ================================================================
# Superellipse body geometry
# ================================================================

def superellipse_ring(
    x: float,
    width: float,
    height: float,
    exponent: float,
    points: int,
) -> List[Vec3]:

    if points < 8:

        raise ValueError(
            "Cross-section needs "
            "at least 8 points."
        )


    if (
        width <= 0.0
        or height <= 0.0
    ):

        raise ValueError(
            "Cross-section "
            "width/height "
            "must be positive."
        )


    if exponent <= 0.0:

        raise ValueError(
            "Superellipse exponent "
            "must be positive."
        )


    a = (
        0.5
        * width
    )

    b = (
        0.5
        * height
    )


    power = (
        2.0
        / exponent
    )


    ring: List[Vec3] = []


    for i in range(
        points
    ):

        theta = (
            2.0
            * math.pi
            * i
            / points
        )


        cosine = (
            math.cos(
                theta
            )
        )

        sine = (
            math.sin(
                theta
            )
        )


        y = (
            a
            * math.copysign(
                abs(cosine) ** power,
                cosine,
            )
        )


        z = (
            b
            * math.copysign(
                abs(sine) ** power,
                sine,
            )
        )


        ring.append(
            (
                x,
                y,
                z,
            )
        )


    # ============================================================
    # Area preservation
    #
    # Low-resolution polygon sections would otherwise underestimate
    # the intended continuous superellipse area.
    # ============================================================

    polygon_area_twice = 0.0


    for p0, p1 in zip(
        ring,
        ring[1:] + ring[:1],
    ):

        polygon_area_twice += (
            p0[1] * p1[2]
            - p1[1] * p0[2]
        )


    polygon_area = (
        0.5
        * abs(
            polygon_area_twice
        )
    )


    area_factor = (

        4.0

        * math.gamma(
            1.0
            + 1.0 / exponent
        ) ** 2

        / math.gamma(
            1.0
            + 2.0 / exponent
        )
    )


    analytic_area = (
        area_factor
        * a
        * b
    )


    if (
        polygon_area > 0.0
        and analytic_area > 0.0
    ):

        scale = math.sqrt(
            analytic_area
            / polygon_area
        )


        ring = [

            (
                px,
                py * scale,
                pz * scale,
            )

            for (
                px,
                py,
                pz,
            )
            in ring
        ]


    return ring


def smoothstep(
    value: float,
) -> float:

    value = clamp(
        value,
        0.0,
        1.0,
    )


    return (
        value
        * value
        * (
            3.0
            - 2.0 * value
        )
    )


def interpolate_profile(
    controls: Sequence[dict],
    x: float,
) -> Tuple[
    float,
    float,
]:

    stations = sorted(
        controls,
        key=lambda station:
            float(
                station["x_m"]
            ),
    )


    if (
        x
        <= float(
            stations[0]["x_m"]
        )
    ):

        return (
            float(
                stations[0][
                    "width_y_m"
                ]
            ),
            float(
                stations[0][
                    "height_z_m"
                ]
            ),
        )


    if (
        x
        >= float(
            stations[-1]["x_m"]
        )
    ):

        return (
            float(
                stations[-1][
                    "width_y_m"
                ]
            ),
            float(
                stations[-1][
                    "height_z_m"
                ]
            ),
        )


    for a, b in zip(
        stations[:-1],
        stations[1:],
    ):

        xa = float(
            a["x_m"]
        )

        xb = float(
            b["x_m"]
        )


        if (
            xa
            <= x
            <= xb
        ):

            t = (
                (x - xa)
                / (xb - xa)
            )


            t = smoothstep(
                t
            )


            wa = float(
                a["width_y_m"]
            )

            wb = float(
                b["width_y_m"]
            )

            ha = float(
                a["height_z_m"]
            )

            hb = float(
                b["height_z_m"]
            )


            return (
                wa
                + (wb - wa)
                * t,

                ha
                + (hb - ha)
                * t,
            )


    raise RuntimeError(
        "Profile interpolation "
        "failed unexpectedly."
    )


def profile_sample_positions(
    controls: Sequence[dict],
    total_sections: int,
) -> List[float]:

    if (
        total_sections
        < len(controls)
    ):

        total_sections = (
            len(controls)
        )


    control_x = sorted(
        {
            float(
                station["x_m"]
            )
            for station
            in controls
        }
    )


    xmin = (
        control_x[0]
    )

    xmax = (
        control_x[-1]
    )


    uniform = [

        xmin
        + (xmax - xmin)
        * i
        / (
            total_sections
            - 1
        )

        for i in range(
            total_sections
        )
    ]


    return sorted(
        set(
            round(
                x,
                12,
            )

            for x in (
                uniform
                + control_x
            )
        )
    )


def generate_closed_loft(
    *,
    name: str,
    control_sections: Sequence[dict],
    longitudinal_sections: int,
    circumferential_points: int,
    exponent: float,
) -> Mesh:

    xs = (
        profile_sample_positions(
            control_sections,
            longitudinal_sections,
        )
    )


    mesh = Mesh(
        name
    )


    rings: List[
        List[int]
    ] = []


    for x in xs:

        width, height = (
            interpolate_profile(
                control_sections,
                x,
            )
        )


        ring_points = (
            superellipse_ring(
                x=x,
                width=width,
                height=height,
                exponent=exponent,
                points=(
                    circumferential_points
                ),
            )
        )


        ring_ids = [

            mesh.add_vertex(
                point
            )

            for point
            in ring_points
        ]


        rings.append(
            ring_ids
        )


    count = (
        circumferential_points
    )


    # Side surface
    for r0, r1 in zip(
        rings[:-1],
        rings[1:],
    ):

        for j in range(
            count
        ):

            next_j = (
                (j + 1)
                % count
            )


            mesh.add_tri(
                r0[j],
                r1[j],
                r1[next_j],
            )


            mesh.add_tri(
                r0[j],
                r1[next_j],
                r0[next_j],
            )


    # End caps
    x0 = xs[0]
    x1 = xs[-1]


    center0 = mesh.add_vertex(
        (
            x0,
            0.0,
            0.0,
        )
    )


    center1 = mesh.add_vertex(
        (
            x1,
            0.0,
            0.0,
        )
    )


    ring0 = rings[0]
    ring1 = rings[-1]


    for j in range(
        count
    ):

        next_j = (
            (j + 1)
            % count
        )


        mesh.add_tri(
            center0,
            ring0[next_j],
            ring0[j],
        )


        mesh.add_tri(
            center1,
            ring1[j],
            ring1[next_j],
        )


    mesh.orient_consistently_outward()


    return mesh


# ================================================================
# Tail
# ================================================================

def generate_tail_link(
    *,
    name: str,
    proximal_section: dict,
    distal_section: dict,
    longitudinal_sections: int,
    circumferential_points: int,
    exponent: float,
) -> Mesh:

    proximal_x = float(
        proximal_section[
            "x_m"
        ]
    )

    distal_x = float(
        distal_section[
            "x_m"
        ]
    )


    segment_length = abs(
        distal_x
        - proximal_x
    )


    controls = [
        {
            "x_m": 0.0,

            "width_y_m":
                float(
                    proximal_section[
                        "width_y_m"
                    ]
                ),

            "height_z_m":
                float(
                    proximal_section[
                        "height_z_m"
                    ]
                ),
        },

        {
            "x_m":
                -segment_length,

            "width_y_m":
                float(
                    distal_section[
                        "width_y_m"
                    ]
                ),

            "height_z_m":
                float(
                    distal_section[
                        "height_z_m"
                    ]
                ),
        },
    ]


    return generate_closed_loft(
        name=name,
        control_sections=controls,
        longitudinal_sections=(
            longitudinal_sections
        ),
        circumferential_points=(
            circumferential_points
        ),
        exponent=exponent,
    )


# ================================================================
# Pectoral hydrofoils
# ================================================================

def naca_symmetric_thickness(
    xi: float,
    thickness_ratio: float,
) -> float:

    xi = clamp(
        xi,
        0.0,
        1.0,
    )


    return (

        5.0
        * thickness_ratio
        * (

            0.2969
            * math.sqrt(
                max(
                    xi,
                    1e-12,
                )
            )

            - 0.1260
            * xi

            - 0.3516
            * xi**2

            + 0.2843
            * xi**3

            - 0.1036
            * xi**4
        )
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

        raise ValueError(
            "half_samples "
            "must be >= 5"
        )


    thickness_ratio = (
        thickness_abs
        / chord
    )


    xis = [

        0.5
        * (
            1.0
            - math.cos(
                math.pi
                * i
                / (
                    half_samples
                    - 1
                )
            )
        )

        for i in range(
            half_samples
        )
    ]


    upper: List[Vec3] = []


    for xi in xis:

        x = (
            center_x
            + 0.5 * chord
            - xi * chord
        )


        z = (
            chord
            * naca_symmetric_thickness(
                xi,
                thickness_ratio,
            )
        )


        upper.append(
            (
                x,
                y,
                z,
            )
        )


    lower: List[Vec3] = []


    for xi in reversed(
        xis[1:-1]
    ):

        x = (
            center_x
            + 0.5 * chord
            - xi * chord
        )


        z = (
            -chord
            * naca_symmetric_thickness(
                xi,
                thickness_ratio,
            )
        )


        lower.append(
            (
                x,
                y,
                z,
            )
        )


    return (
        upper
        + lower
    )


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

    if (
        side_sign
        not in (
            -1.0,
            1.0,
        )
    ):

        raise ValueError(
            "side_sign must "
            "be -1 or +1"
        )


    mesh = Mesh(
        name
    )


    rings: List[
        List[int]
    ] = []


    sweep = math.radians(
        sweep_angle_deg
    )


    for i in range(
        span_stations
    ):

        fraction = (
            i
            / (
                span_stations
                - 1
            )
        )


        span_position = (
            fraction
            * span
        )


        y = (
            side_sign
            * span_position
        )


        chord = (

            root_chord

            + (
                tip_chord
                - root_chord
            )

            * fraction
        )


        center_x = (

            -math.tan(
                sweep
            )

            * span_position
        )


        local_thickness = (

            thickness

            * (
                1.0
                - 0.30
                * fraction
            )
        )


        ring = airfoil_ring(
            y=y,
            chord=chord,
            center_x=center_x,
            thickness_abs=(
                local_thickness
            ),
            half_samples=(
                chord_half_samples
            ),
        )


        rings.append(

            [
                mesh.add_vertex(
                    point
                )

                for point
                in ring
            ]
        )


    count = (
        len(
            rings[0]
        )
    )


    for r0, r1 in zip(
        rings[:-1],
        rings[1:],
    ):

        for j in range(
            count
        ):

            next_j = (
                (j + 1)
                % count
            )


            mesh.add_tri(
                r0[j],
                r1[j],
                r1[next_j],
            )


            mesh.add_tri(
                r0[j],
                r1[next_j],
                r0[next_j],
            )


    # Root and tip caps
    for (
        ring,
        reverse,
    ) in (
        (
            rings[0],
            True,
        ),
        (
            rings[-1],
            False,
        ),
    ):

        center = mesh.add_vertex(

            tuple(

                sum(
                    mesh.vertices[
                        index
                    ][dimension]

                    for index
                    in ring
                )

                / len(ring)

                for dimension
                in range(3)
            )
        )


        for j in range(
            count
        ):

            next_j = (
                (j + 1)
                % count
            )


            if reverse:

                mesh.add_tri(
                    center,
                    ring[next_j],
                    ring[j],
                )

            else:

                mesh.add_tri(
                    center,
                    ring[j],
                    ring[next_j],
                )


    mesh.orient_consistently_outward()


    return mesh


# ================================================================
# Procedural fin fallback
# ================================================================

def extrude_xz_polygon_y(
    *,
    name: str,
    polygon_xz: Sequence[
        Tuple[
            float,
            float,
        ]
    ],
    thickness_y: float,
) -> Mesh:

    if len(
        polygon_xz
    ) < 3:

        raise ValueError(
            "Polygon must contain "
            "at least 3 points."
        )


    mesh = Mesh(
        name
    )


    half_y = (
        0.5
        * thickness_y
    )


    front = [

        mesh.add_vertex(
            (
                x,
                -half_y,
                z,
            )
        )

        for x, z
        in polygon_xz
    ]


    back = [

        mesh.add_vertex(
            (
                x,
                half_y,
                z,
            )
        )

        for x, z
        in polygon_xz
    ]


    count = (
        len(
            polygon_xz
        )
    )


    for i in range(
        1,
        count - 1,
    ):

        mesh.add_tri(
            front[0],
            front[i + 1],
            front[i],
        )


        mesh.add_tri(
            back[0],
            back[i],
            back[i + 1],
        )


    for i in range(
        count
    ):

        next_i = (
            (i + 1)
            % count
        )


        mesh.add_tri(
            front[i],
            back[i],
            back[next_i],
        )


        mesh.add_tri(
            front[i],
            back[next_i],
            front[next_i],
        )


    mesh.orient_consistently_outward()


    return mesh


def procedural_caudal_fin(
    config: dict,
    name: str,
) -> Mesh:

    dimensions = [

        float(value)

        for value
        in config[
            "initial_dimensions_xyz_m"
        ]
    ]


    (
        length_x,
        thickness_y,
        height_z,
    ) = dimensions


    half_height = (
        0.5
        * height_z
    )


    polygon = [
        (
            0.0,
            0.030 * height_z,
        ),
        (
            -0.25 * length_x,
            0.42 * height_z,
        ),
        (
            -0.85 * length_x,
            half_height,
        ),
        (
            -length_x,
            0.20 * height_z,
        ),
        (
            -0.70 * length_x,
            0.0,
        ),
        (
            -length_x,
            -0.20 * height_z,
        ),
        (
            -0.85 * length_x,
            -half_height,
        ),
        (
            -0.25 * length_x,
            -0.42 * height_z,
        ),
        (
            0.0,
            -0.030 * height_z,
        ),
    ]


    return extrude_xz_polygon_y(
        name=name,
        polygon_xz=polygon,
        thickness_y=(
            thickness_y
        ),
    )


def procedural_dorsal_fin(
    config: dict,
    name: str,
) -> Mesh:

    dimensions = [

        float(value)

        for value
        in config[
            "source_dimensions_xyz_m"
        ]
    ]


    (
        length_x,
        thickness_y,
        height_z,
    ) = dimensions


    polygon = [
        (
            0.40 * length_x,
            0.0,
        ),
        (
            0.20 * length_x,
            -0.50 * height_z,
        ),
        (
            -0.15 * length_x,
            -height_z,
        ),
        (
            -0.45 * length_x,
            -0.75 * height_z,
        ),
        (
            -0.60 * length_x,
            0.0,
        ),
    ]


    return extrude_xz_polygon_y(
        name=name,
        polygon_xz=polygon,
        thickness_y=(
            thickness_y
        ),
    )


# ================================================================
# fishsim reference meshes
# ================================================================

def find_fishsim_mesh(
    *,
    fishsim_root: Path | None,
    file_name: str,
) -> Path | None:

    candidates: List[Path] = []


    if fishsim_root is not None:

        candidates.append(
            fishsim_root
            / "Geometry"
            / "Meshes"
            / file_name
        )


    environment_root = (
        os.environ.get(
            "FISHSIM_ROOT"
        )
    )


    if environment_root:

        candidates.append(
            Path(
                environment_root
            )
            / "Geometry"
            / "Meshes"
            / file_name
        )


    cwd = (
        Path.cwd()
    )


    for base in (
        cwd / "fishsim",
        cwd.parent / "fishsim",
        cwd / "third_party" / "fishsim",
        cwd / "external" / "fishsim",
    ):

        candidates.append(
            base
            / "Geometry"
            / "Meshes"
            / file_name
        )


    for path in candidates:

        if path.is_file():

            return (
                path.resolve()
            )


    return None


def fetch_reference_mesh(
    *,
    output_reference_dir: Path,
    file_name: str,
) -> Path:

    output_reference_dir.mkdir(
        parents=True,
        exist_ok=True,
    )


    destination = (
        output_reference_dir
        / file_name
    )


    if destination.is_file():

        return destination


    url = (
        f"{FISHSIM_RAW_BASE}/"
        f"{file_name}"
    )


    print(
        f"[fetch] {url}"
    )


    try:

        with urllib.request.urlopen(
            url,
            timeout=30,
        ) as response:

            data = (
                response.read()
            )

    except Exception as exc:

        raise RuntimeError(
            f"Failed to download "
            f"{file_name} "
            f"from fishsim: "
            f"{exc}"
        ) from exc


    destination.write_bytes(
        data
    )


    return destination


def load_or_generate_reference_fins(
    *,
    config: dict,
    fishsim_root: Path | None,
    reference_dir: Path,
    fetch: bool,
    fallback: bool,
) -> Tuple[
    Mesh,
    Mesh,
    dict,
]:

    caudal_config = (
        config[
            "caudal_fin"
        ]
    )

    dorsal_config = (
        config[
            "dorsal_fin"
        ]
    )


    source_status = {}


    # ============================================================
    # Caudal fin
    # ============================================================

    tail_filename = (
        FISHSIM_REFERENCE_FILES[
            "caudal_fin"
        ]
    )


    tail_path = find_fishsim_mesh(
        fishsim_root=(
            fishsim_root
        ),
        file_name=(
            tail_filename
        ),
    )


    if (
        tail_path is None
        and fetch
    ):

        tail_path = (
            fetch_reference_mesh(
                output_reference_dir=(
                    reference_dir
                ),
                file_name=(
                    tail_filename
                ),
            )
        )


    if tail_path is not None:

        raw = load_obj(
            tail_path,
            "CaudalFinReference",
        )


        caudal = raw.transformed(
            rotate_y_180=True,
            name="CaudalFin",
        )


        # Put fin root at local X=0.
        _, high = (
            caudal.bbox()
        )


        caudal.translate_in_place(
            (
                -high[0],
                0.0,
                0.0,
            )
        )


        low, high = (
            caudal.bbox()
        )


        caudal.translate_in_place(
            (
                0.0,
                -0.5
                * (
                    low[1]
                    + high[1]
                ),
                -0.5
                * (
                    low[2]
                    + high[2]
                ),
            )
        )


        caudal.orient_consistently_outward()


        source_status[
            "caudal_fin"
        ] = {
            "mode":
                "fishsim_reference_mesh",

            "source":
                str(
                    tail_path
                ),
        }


    elif fallback:

        caudal = (
            procedural_caudal_fin(
                caudal_config,
                "CaudalFin",
            )
        )


        source_status[
            "caudal_fin"
        ] = {
            "mode":
                "procedural_fallback",

            "source":
                "BionicFishV1Config "
                "dimensions",
        }


    else:

        raise FileNotFoundError(
            "fishsim finTail.obj "
            "not found. Use:\n"
            "  --fishsim-root "
            "/path/to/fishsim\n"
            "or\n"
            "  --fetch-reference-meshes\n"
            "or\n"
            "  --allow-procedural-fin-fallback"
        )


    # ============================================================
    # Dorsal fin
    # ============================================================

    dorsal_filename = (
        FISHSIM_REFERENCE_FILES[
            "dorsal_fin"
        ]
    )


    dorsal_path = find_fishsim_mesh(
        fishsim_root=(
            fishsim_root
        ),
        file_name=(
            dorsal_filename
        ),
    )


    if (
        dorsal_path is None
        and fetch
    ):

        dorsal_path = (
            fetch_reference_mesh(
                output_reference_dir=(
                    reference_dir
                ),
                file_name=(
                    dorsal_filename
                ),
            )
        )


    if dorsal_path is not None:

        raw = load_obj(
            dorsal_path,
            "DorsalFinReference",
        )


        dorsal = raw.transformed(
            rotate_y_180=True,
            name="DorsalFin",
        )


        low, high = (
            dorsal.bbox()
        )


        dorsal.translate_in_place(
            (
                0.0,
                -0.5
                * (
                    low[1]
                    + high[1]
                ),
                0.0,
            )
        )


        dorsal.orient_consistently_outward()


        source_status[
            "dorsal_fin"
        ] = {
            "mode":
                "fishsim_reference_mesh",

            "source":
                str(
                    dorsal_path
                ),
        }


    elif fallback:

        dorsal = (
            procedural_dorsal_fin(
                dorsal_config,
                "DorsalFin",
            )
        )


        source_status[
            "dorsal_fin"
        ] = {
            "mode":
                "procedural_fallback",

            "source":
                "BionicFishV1Config "
                "dimensions",
        }


    else:

        raise FileNotFoundError(
            "fishsim finTop.obj "
            "not found. Use:\n"
            "  --fishsim-root "
            "/path/to/fishsim\n"
            "or\n"
            "  --fetch-reference-meshes\n"
            "or\n"
            "  --allow-procedural-fin-fallback"
        )


    return (
        caudal,
        dorsal,
        source_status,
    )


# ================================================================
# Validation
# ================================================================

def mesh_stats(
    mesh: Mesh,
) -> dict:

    low, high = (
        mesh.bbox()
    )


    normals = (
        mesh.compute_vertex_normals()
    )


    return {
        "vertices":
            len(
                mesh.vertices
            ),

        "normals":
            len(
                normals
            ),

        "faces":
            len(
                mesh.faces
            ),

        "boundary_edges":
            mesh.boundary_edge_count(),

        "closed":
            mesh.boundary_edge_count()
            == 0,

        "volume_m3":
            mesh.volume(),

        "bbox_min_xyz_m":
            list(
                low
            ),

        "bbox_max_xyz_m":
            list(
                high
            ),

        "bbox_dimensions_xyz_m":
            list(
                mesh.bbox_size()
            ),
    }


def assert_closed(
    mesh: Mesh,
    label: str,
) -> None:

    count = (
        mesh.boundary_edge_count()
    )


    if count != 0:

        raise ValueError(
            f"{label} "
            "is not closed/watertight: "
            f"{count} boundary edges"
        )


def midpoint_int(
    values: Sequence[int],
    fallback: int,
) -> int:

    if not values:

        return fallback


    return int(
        round(
            0.5
            * (
                int(
                    values[0]
                )
                + int(
                    values[-1]
                )
            )
        )
    )


# ================================================================
# Generation
# ================================================================

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

    fish_config = (
        config[
            "bionic_fish_v1"
        ]
    )


    output_layout = (
        fish_config[
            "output_layout"
        ]
    )


    if output_override is not None:

        output_root = (
            output_override.resolve()
        )

        visual_dir = (
            output_root
            / "visual"
        )

        physics_dir = (
            output_root
            / "physics"
        )


    else:

        output_root = (
            repo_root
            / output_layout[
                "root"
            ]
        )


        visual_dir = (
            repo_root
            / output_layout[
                "visual_dir"
            ]
        )


        physics_dir = (
            repo_root
            / output_layout[
                "physics_dir"
            ]
        )


    reference_dir = (
        output_root
        / "reference"
        / "fishsim"
    )


    visual_dir.mkdir(
        parents=True,
        exist_ok=True,
    )


    physics_dir.mkdir(
        parents=True,
        exist_ok=True,
    )


    reference_dir.mkdir(
        parents=True,
        exist_ok=True,
    )


    pipeline = (
        fish_config[
            "mesh_pipeline"
        ]
    )


    visual_longitudinal = (
        midpoint_int(

            pipeline[
                "visual_mesh"
            ][
                "resolution_guideline"
            ][
                "longitudinal_sections_range"
            ],

            64,
        )
    )


    visual_circumferential = (
        midpoint_int(

            pipeline[
                "visual_mesh"
            ][
                "resolution_guideline"
            ][
                "points_per_section_range"
            ],

            28,
        )
    )


    physics_longitudinal = (
        midpoint_int(

            pipeline[
                "physics_mesh"
            ][
                "resolution_guideline"
            ][
                "longitudinal_sections_range"
            ],

            14,
        )
    )


    physics_circumferential = (
        midpoint_int(

            pipeline[
                "physics_mesh"
            ][
                "resolution_guideline"
            ][
                "points_per_section_range"
            ],

            10,
        )
    )


    # ============================================================
    # Body
    # ============================================================

    body_config = (
        fish_config[
            "body"
        ]
    )


    body_sections = (
        body_config[
            "cross_sections"
        ]
    )


    body_exponent = float(
        body_config[
            "cross_section_superellipse_exponent"
        ]
    )


    body_visual = (
        generate_closed_loft(
            name="Body",
            control_sections=(
                body_sections
            ),
            longitudinal_sections=(
                visual_longitudinal
            ),
            circumferential_points=(
                visual_circumferential
            ),
            exponent=(
                body_exponent
            ),
        )
    )


    body_physics = (
        generate_closed_loft(
            name="Body",
            control_sections=(
                body_sections
            ),
            longitudinal_sections=(
                physics_longitudinal
            ),
            circumferential_points=(
                physics_circumferential
            ),
            exponent=(
                body_exponent
            ),
        )
    )


    # ============================================================
    # Tail
    # ============================================================

    envelope = (

        fish_config[
            "tail"
        ][
            "smooth_mesh_envelope"
        ][
            "sections"
        ]
    )


    if len(
        envelope
    ) != 6:

        raise ValueError(
            "Expected six tail "
            "envelope stations."
        )


    tail_visual: List[Mesh] = []

    tail_physics: List[Mesh] = []


    for i in range(5):

        proximal = (
            envelope[i]
        )

        distal = (
            envelope[i + 1]
        )


        tail_visual.append(

            generate_tail_link(
                name=f"Tail{i}",
                proximal_section=(
                    proximal
                ),
                distal_section=(
                    distal
                ),
                longitudinal_sections=7,
                circumferential_points=max(
                    20,
                    visual_circumferential,
                ),
                exponent=2.0,
            )
        )


        tail_physics.append(

            generate_tail_link(
                name=f"Tail{i}",
                proximal_section=(
                    proximal
                ),
                distal_section=(
                    distal
                ),
                longitudinal_sections=3,
                circumferential_points=max(
                    8,
                    physics_circumferential,
                ),
                exponent=2.0,
            )
        )


    # ============================================================
    # Pectoral fins
    # ============================================================

    pectoral_config = (
        fish_config[
            "pectoral_fins"
        ]
    )


    common_pectoral = dict(

        span=float(
            pectoral_config[
                "span_m"
            ]
        ),

        root_chord=float(
            pectoral_config[
                "root_chord_m"
            ]
        ),

        tip_chord=float(
            pectoral_config[
                "tip_chord_m"
            ]
        ),

        thickness=float(
            pectoral_config[
                "thickness_m"
            ]
        ),

        sweep_angle_deg=float(
            pectoral_config[
                "sweep_angle_deg"
            ]
        ),
    )


    left_visual = (
        generate_pectoral_fin(
            name=(
                "LeftPectoralFin"
            ),
            side_sign=-1.0,
            span_stations=15,
            chord_half_samples=18,
            **common_pectoral,
        )
    )


    right_visual = (
        generate_pectoral_fin(
            name=(
                "RightPectoralFin"
            ),
            side_sign=1.0,
            span_stations=15,
            chord_half_samples=18,
            **common_pectoral,
        )
    )


    left_physics = (
        generate_pectoral_fin(
            name=(
                "LeftPectoralFin"
            ),
            side_sign=-1.0,
            span_stations=6,
            chord_half_samples=8,
            **common_pectoral,
        )
    )


    right_physics = (
        generate_pectoral_fin(
            name=(
                "RightPectoralFin"
            ),
            side_sign=1.0,
            span_stations=6,
            chord_half_samples=8,
            **common_pectoral,
        )
    )


    # ============================================================
    # fishsim fins
    # ============================================================

    (
        caudal,
        dorsal,
        reference_status,
    ) = (
        load_or_generate_reference_fins(
            config=(
                fish_config
            ),
            fishsim_root=(
                fishsim_root
            ),
            reference_dir=(
                reference_dir
            ),
            fetch=(
                fetch_reference_meshes
            ),
            fallback=(
                allow_fallback
            ),
        )
    )


    caudal_visual = (
        caudal.copy(
            "CaudalFin"
        )
    )

    caudal_physics = (
        caudal.copy(
            "CaudalFin"
        )
    )


    dorsal_visual = (
        dorsal.copy(
            "DorsalFin"
        )
    )

    dorsal_physics = (
        dorsal.copy(
            "DorsalFin"
        )
    )


    # ============================================================
    # Validate all closed meshes
    # ============================================================

    generated_meshes = [

        (
            "body_visual",
            body_visual,
        ),

        (
            "body_physics",
            body_physics,
        ),

        *[
            (
                f"tail{i}_visual",
                mesh,
            )
            for i, mesh
            in enumerate(
                tail_visual
            )
        ],

        *[
            (
                f"tail{i}_physics",
                mesh,
            )
            for i, mesh
            in enumerate(
                tail_physics
            )
        ],

        (
            "left_pectoral_visual",
            left_visual,
        ),

        (
            "right_pectoral_visual",
            right_visual,
        ),

        (
            "left_pectoral_physics",
            left_physics,
        ),

        (
            "right_pectoral_physics",
            right_physics,
        ),

        (
            "caudal_visual",
            caudal_visual,
        ),

        (
            "caudal_physics",
            caudal_physics,
        ),

        (
            "dorsal_visual",
            dorsal_visual,
        ),

        (
            "dorsal_physics",
            dorsal_physics,
        ),
    ]


    for label, mesh in (
        generated_meshes
    ):

        assert_closed(
            mesh,
            label,
        )


        # Also force normal validation now.
        normals = (
            mesh.compute_vertex_normals()
        )


        if (
            len(normals)
            != len(
                mesh.vertices
            )
        ):

            raise ValueError(
                f"{label}: invalid normals."
            )


    # ============================================================
    # Write OBJ files
    # ============================================================

    body_visual.write_obj(
        visual_dir
        / "body.obj",
        (
            "Body-local coordinates. "
            "BionicFish body frame."
        ),
    )


    body_physics.write_obj(
        physics_dir
        / "body.obj",
        (
            "Low-poly closed body "
            "physics mesh."
        ),
    )


    for i, mesh in enumerate(
        tail_visual
    ):

        mesh.write_obj(
            visual_dir
            / f"tail_{i}.obj",
            (
                f"Tail{i} link-local mesh. "
                f"Origin is Joint{i}."
            ),
        )


    for i, mesh in enumerate(
        tail_physics
    ):

        mesh.write_obj(
            physics_dir
            / f"tail_{i}.obj",
            (
                f"Low-poly Tail{i} "
                f"physics mesh."
            ),
        )


    left_visual.write_obj(
        visual_dir
        / "left_pectoral.obj",
        (
            "Left pectoral "
            "link-local hydrofoil."
        ),
    )


    right_visual.write_obj(
        visual_dir
        / "right_pectoral.obj",
        (
            "Right pectoral "
            "link-local hydrofoil."
        ),
    )


    left_physics.write_obj(
        physics_dir
        / "left_pectoral.obj",
        (
            "Low-poly left "
            "pectoral physics mesh."
        ),
    )


    right_physics.write_obj(
        physics_dir
        / "right_pectoral.obj",
        (
            "Low-poly right "
            "pectoral physics mesh."
        ),
    )


    caudal_visual.write_obj(
        visual_dir
        / "caudal_fin.obj",
        (
            "Caudal fin "
            "link-local mesh."
        ),
    )


    caudal_physics.write_obj(
        physics_dir
        / "caudal_fin.obj",
        (
            "Caudal fin "
            "physics mesh."
        ),
    )


    dorsal_visual.write_obj(
        visual_dir
        / "dorsal_fin.obj",
        (
            "Dorsal fin "
            "link-local mesh."
        ),
    )


    dorsal_physics.write_obj(
        physics_dir
        / "dorsal_fin.obj",
        (
            "Dorsal fin "
            "physics mesh."
        ),
    )


    # ============================================================
    # Neutral assembly preview
    # ============================================================

    preview_parts: List[Mesh] = [

        body_visual.copy(
            "Body"
        )
    ]


    tail_joint_positions = (

        fish_config[
            "tail"
        ][
            "joint_positions_body_xyz_m"
        ]
    )


    for i, mesh in enumerate(
        tail_visual
    ):

        translation = tuple(

            float(value)

            for value
            in tail_joint_positions[
                f"Joint{i}"
            ]
        )


        preview_parts.append(

            mesh.transformed(
                translate=(
                    translation
                ),
                name=f"Tail{i}",
            )
        )


    fin_root_x = float(
        envelope[-1][
            "x_m"
        ]
    )


    preview_parts.append(

        caudal_visual.transformed(
            translate=(
                fin_root_x,
                0.0,
                0.0,
            ),
            name="CaudalFin",
        )
    )


    dorsal_mount = tuple(

        float(value)

        for value
        in fish_config[
            "dorsal_fin"
        ][
            "bionic_mount_position_body_xyz_m"
        ]
    )


    preview_parts.append(

        dorsal_visual.transformed(
            translate=(
                dorsal_mount
            ),
            name="DorsalFin",
        )
    )


    left_mount = tuple(

        float(value)

        for value
        in pectoral_config[
            "left"
        ][
            "root_position_body_xyz_m"
        ]
    )


    right_mount = tuple(

        float(value)

        for value
        in pectoral_config[
            "right"
        ][
            "root_position_body_xyz_m"
        ]
    )


    preview_parts.append(

        left_visual.transformed(
            translate=(
                left_mount
            ),
            name=(
                "LeftPectoralFin"
            ),
        )
    )


    preview_parts.append(

        right_visual.transformed(
            translate=(
                right_mount
            ),
            name=(
                "RightPectoralFin"
            ),
        )
    )


    preview = combine_meshes(
        "BionicFishV1_NeutralAssembly",
        preview_parts,
    )


    if not no_preview:

        preview.write_obj(
            output_root
            / "assembly_preview.obj",
            (
                "Neutral-pose body-frame "
                "assembly for visualization only. "
                "Do not use as articulated "
                "Stonefish robot."
            ),
        )


    # ============================================================
    # Manifest
    # ============================================================

    manifest = {

        "generator":
            "generate_bionic_fish.py",

        "config_name":
            config.get(
                "config_name"
            ),

        "config_revision":
            config.get(
                "config_revision"
            ),

        "obj_normals":
            {
                "enabled": True,
                "mode":
                    "area_weighted_smooth_vertex_normals",
                "face_format":
                    "v//vn",
            },

        "units":
            "m",

        "coordinate_frame":
            {
                "x":
                    "forward/head",

                "y":
                    "right",

                "z":
                    "down",
            },

        "reference_meshes":
            reference_status,

        "resolution":
            {
                "visual":
                    {
                        "body_longitudinal_sections_requested":
                            visual_longitudinal,

                        "body_circumferential_points":
                            visual_circumferential,
                    },

                "physics":
                    {
                        "body_longitudinal_sections_requested":
                            physics_longitudinal,

                        "body_circumferential_points":
                            physics_circumferential,
                    },
            },

        "link_origins_body_xyz_m":
            {
                "Body":
                    [
                        0.0,
                        0.0,
                        0.0,
                    ],

                **{
                    f"Tail{i}":
                        list(
                            map(
                                float,
                                tail_joint_positions[
                                    f"Joint{i}"
                                ],
                            )
                        )

                    for i
                    in range(5)
                },

                "CaudalFin":
                    [
                        fin_root_x,
                        0.0,
                        0.0,
                    ],

                "DorsalFin":
                    list(
                        dorsal_mount
                    ),

                "LeftPectoralFin":
                    list(
                        left_mount
                    ),

                "RightPectoralFin":
                    list(
                        right_mount
                    ),
            },

        "visual_meshes":
            {
                "body.obj":
                    mesh_stats(
                        body_visual
                    ),

                **{
                    f"tail_{i}.obj":
                        mesh_stats(
                            mesh
                        )

                    for i, mesh
                    in enumerate(
                        tail_visual
                    )
                },

                "caudal_fin.obj":
                    mesh_stats(
                        caudal_visual
                    ),

                "dorsal_fin.obj":
                    mesh_stats(
                        dorsal_visual
                    ),

                "left_pectoral.obj":
                    mesh_stats(
                        left_visual
                    ),

                "right_pectoral.obj":
                    mesh_stats(
                        right_visual
                    ),
            },

        "physics_meshes":
            {
                "body.obj":
                    mesh_stats(
                        body_physics
                    ),

                **{
                    f"tail_{i}.obj":
                        mesh_stats(
                            mesh
                        )

                    for i, mesh
                    in enumerate(
                        tail_physics
                    )
                },

                "caudal_fin.obj":
                    mesh_stats(
                        caudal_physics
                    ),

                "dorsal_fin.obj":
                    mesh_stats(
                        dorsal_physics
                    ),

                "left_pectoral.obj":
                    mesh_stats(
                        left_physics
                    ),

                "right_pectoral.obj":
                    mesh_stats(
                        right_physics
                    ),
            },

        "assembly_preview":
            (
                mesh_stats(
                    preview
                )

                if not no_preview

                else {
                    "status":
                        "disabled"
                }
            ),

        "important":
            [
                (
                    "OBJ files contain explicit "
                    "smooth vertex normals."
                ),

                (
                    "assembly_preview.obj is only "
                    "a neutral-pose visualization."
                ),

                (
                    "The real Stonefish robot must "
                    "use the separate link meshes."
                ),

                (
                    "Tail link origins are their "
                    "proximal joints."
                ),
            ],
    }


    manifest_path = (
        output_root
        / "mesh_manifest.yaml"
    )


    with manifest_path.open(
        "w",
        encoding="utf-8",
    ) as file:

        yaml.safe_dump(
            manifest,
            file,
            sort_keys=False,
            allow_unicode=True,
            width=110,
        )


    # ============================================================
    # Reference attribution
    # ============================================================

    attribution = (
        output_root
        / "REFERENCE_SOURCES.md"
    )


    attribution.write_text(
        """# BionicFish V1 reference geometry

The BionicFish V1 mechanical reference is based in part on:

- ETH Zürich SRL `srl-ethz/fishsim`
- Repository: https://github.com/srl-ethz/fishsim
- Reference meshes:
  - `Geometry/Meshes/finTail.obj`
  - `Geometry/Meshes/finTop.obj`

Keep the upstream fishsim license and attribution when vendoring its files.

The BionicFish body, articulated tail envelopes, pectoral hydrofoils,
coordinate conversion and Stonefish-oriented layout are generated by the
RL-stimu-fish project and are not direct copies of the fishsim MuJoCo body
boxes.
""",
        encoding="utf-8",
    )


    # ============================================================
    # Summary
    # ============================================================

    print()

    print(
        "BionicFish V1 mesh "
        "generation complete"
    )

    print(
        "======================================"
    )

    print(
        f"Config revision : "
        f"{config.get('config_revision')}"
    )

    print(
        f"Output root     : "
        f"{output_root}"
    )

    print(
        f"Visual dir      : "
        f"{visual_dir}"
    )

    print(
        f"Physics dir     : "
        f"{physics_dir}"
    )

    print(
        f"Manifest        : "
        f"{manifest_path}"
    )


    if not no_preview:

        print(
            f"Assembly preview: "
            f"{output_root / 'assembly_preview.obj'}"
        )


    print()


    print(
        "Body physics volume: "
        f"{body_physics.volume():.6f} m^3 "
        f"("
        f"{body_physics.volume() * 1000.0:.3f} L"
        f")"
    )


    print(
        "OBJ normals       : ENABLED"
    )


    print(
        "OBJ face format   : v//vn"
    )


    print(
        "All generated meshes passed "
        "watertight and normal checks."
    )


    print()


    return output_root


# ================================================================
# CLI
# ================================================================

def build_arg_parser(
) -> argparse.ArgumentParser:

    parser = (
        argparse.ArgumentParser(
            description=(
                "Generate BionicFish V1 "
                "OBJ meshes from YAML."
            )
        )
    )


    parser.add_argument(
        "--config",
        type=Path,
        required=True,
        help=(
            "Path to "
            "bionic_fish_v1_config YAML."
        ),
    )


    parser.add_argument(
        "--repo-root",
        type=Path,
        default=None,
        help=(
            "RL-stimu-fish repository root. "
            "Auto-detected if omitted."
        ),
    )


    parser.add_argument(
        "--output-root",
        type=Path,
        default=None,
        help=(
            "Override output directory."
        ),
    )


    parser.add_argument(
        "--fishsim-root",
        type=Path,
        default=None,
        help=(
            "Optional local clone "
            "of srl-ethz/fishsim."
        ),
    )


    parser.add_argument(
        "--fetch-reference-meshes",
        action="store_true",
        help=(
            "Download finTail.obj and "
            "finTop.obj if not found locally."
        ),
    )


    parser.add_argument(
        "--allow-procedural-fin-fallback",
        action="store_true",
        help=(
            "Generate fallback fins if "
            "fishsim meshes are unavailable."
        ),
    )


    parser.add_argument(
        "--no-preview",
        action="store_true",
        help=(
            "Do not generate "
            "assembly_preview.obj."
        ),
    )


    parser.add_argument(
        "--validate-only",
        action="store_true",
        help=(
            "Validate config without "
            "generating meshes."
        ),
    )


    return parser


def validate_config(
    config: dict,
) -> None:

    fish = require(
        config,
        "bionic_fish_v1",
        "root",
    )


    body = require(
        fish,
        "body",
        "bionic_fish_v1",
    )


    sections = require(
        body,
        "cross_sections",
        "bionic_fish_v1.body",
    )


    if len(
        sections
    ) < 4:

        raise ValueError(
            "Body needs at least "
            "four cross-section stations."
        )


    tail = require(
        fish,
        "tail",
        "bionic_fish_v1",
    )


    if (
        int(
            tail[
                "segment_count"
            ]
        )
        != 5
    ):

        raise ValueError(
            "BionicFish V1 expects "
            "exactly five tail segments."
        )


    envelope = (

        tail[
            "smooth_mesh_envelope"
        ][
            "sections"
        ]
    )


    if len(
        envelope
    ) != 6:

        raise ValueError(
            "Tail smooth_mesh_envelope "
            "must contain 6 stations."
        )


    pectoral = require(
        fish,
        "pectoral_fins",
        "bionic_fish_v1",
    )


    if (
        int(
            pectoral[
                "count"
            ]
        )
        != 2
    ):

        raise ValueError(
            "BionicFish V1 expects "
            "two pectoral fins."
        )


    for name in (
        "span_m",
        "root_chord_m",
        "tip_chord_m",
        "thickness_m",
    ):

        if (
            float(
                pectoral[name]
            )
            <= 0.0
        ):

            raise ValueError(
                f"pectoral_fins."
                f"{name} "
                "must be positive."
            )


    print(
        "Config validation: OK"
    )

    print(
        f"  revision      : "
        f"{config.get('config_revision')}"
    )

    print(
        f"  body sections : "
        f"{len(sections)}"
    )

    print(
        f"  tail segments : "
        f"{tail['segment_count']}"
    )

    print(
        f"  pectoral span : "
        f"{float(pectoral['span_m']):.3f} m"
    )


def main(
    argv: Sequence[str] | None = None,
) -> int:

    args = (
        build_arg_parser()
        .parse_args(
            argv
        )
    )


    config_path = (
        args.config.resolve()
    )


    if (
        not config_path.is_file()
    ):

        raise FileNotFoundError(
            f"Config not found: "
            f"{config_path}"
        )


    config = (
        load_config(
            config_path
        )
    )


    validate_config(
        config
    )


    if (
        args.validate_only
    ):

        return 0


    repo_root = (
        discover_repo_root(
            config_path=(
                config_path
            ),
            explicit=(
                args.repo_root
            ),
        )
    )


    fishsim_root = (

        args.fishsim_root.resolve()

        if (
            args.fishsim_root
            is not None
        )

        else None
    )


    generate(
        config=config,
        repo_root=(
            repo_root
        ),
        fishsim_root=(
            fishsim_root
        ),
        fetch_reference_meshes=(
            args.fetch_reference_meshes
        ),
        allow_fallback=(
            args.allow_procedural_fin_fallback
        ),
        output_override=(
            args.output_root
        ),
        no_preview=(
            args.no_preview
        ),
    )


    return 0


if __name__ == "__main__":

    try:

        raise SystemExit(
            main()
        )

    except KeyboardInterrupt:

        print(
            "\nInterrupted.",
            file=sys.stderr,
        )

        raise SystemExit(
            130
        )

    except Exception as exc:

        print(
            f"\nERROR: {exc}",
            file=sys.stderr,
        )

        raise SystemExit(
            1
        )
