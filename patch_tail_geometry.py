from pathlib import Path
import re
import shutil
from datetime import datetime

SCN = Path(
    "simulation/data/robots/bionic_fish/bionic_fish.scn"
)

if not SCN.exists():
    raise SystemExit(f"ERROR: cannot find {SCN}")

# ============================================================
# Backup
# ============================================================

stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
backup = SCN.with_suffix(f".scn.backup_{stamp}")
shutil.copy2(SCN, backup)

text = SCN.read_text()

print(f"Input : {SCN}")
print(f"Backup: {backup}")


# ============================================================
# Helpers
# ============================================================

def replace_link(name: str, replacement: str):
    global text

    pattern = (
        r'<link\s+'
        r'name="' + re.escape(name) + r'"'
        r'.*?'
        r'</link>'
    )

    text_new, n = re.subn(
        pattern,
        replacement.strip(),
        text,
        count=1,
        flags=re.S,
    )

    if n != 1:
        raise RuntimeError(
            f"Expected exactly one link '{name}', found {n}"
        )

    text = text_new


def box_part(
    part_name: str,
    dimensions,
    position,
    mass: float,
    look="BionicFishTailLook",
):
    dx, dy, dz = dimensions
    x, y, z = position

    return f'''
            <external_part
                name="{part_name}"
                type="box"
                physics="submerged"
                buoyant="true">

                <dimensions
                    xyz="{dx:.8f} {dy:.8f} {dz:.8f}"/>

                <origin
                    xyz="0.0 0.0 0.0"
                    rpy="0.0 0.0 0.0"/>

                <material
                    name="$(arg material_name)"/>

                <look
                    name="{look}"/>

                <mass
                    value="{mass:.9f}"/>

                <compound_transform
                    xyz="{x:.8f} {y:.8f} {z:.8f}"
                    rpy="0.0 0.0 0.0"/>

            </external_part>
'''


def make_tail_link(
    name: str,

    # proximal thin connector
    prox_length: float,
    prox_height: float,

    # 15 mm PLA support
    support_height: float,

    # tendon guide
    guide_x: float,
    guide_y_center: float,
    guide_width: float,

    # distal thin connector
    distal_length: float,
    distal_height: float,

    # explicit masses
    support_mass: float,
    guide_mass_each: float,
):
    """
    Local x=0 is the proximal joint.

    Stonefish fish direction:
        tail extends along negative X.

    Geometry is the mirrored-X equivalent of fishsim.
    """

    support_length = 0.015
    support_thickness = 0.003

    guide_x_length = 0.006
    guide_z_height = 0.014

    spine_thickness_y = 0.001

    # -----------------------------
    # X locations
    # -----------------------------

    prox_x = -0.5 * prox_length

    support_x = -(
        prox_length
        + 0.5 * support_length
    )

    distal_x = -(
        prox_length
        + support_length
        + 0.5 * distal_length
    )

    parts = []

    # --------------------------------------------------------
    # Proximal thin spine / compliant-region representation
    #
    # Original MuJoCo connector mass = 1 g.
    # --------------------------------------------------------

    parts.append(
        box_part(
            f"{name}ProxSpine",
            (
                prox_length,
                spine_thickness_y,
                prox_height,
            ),
            (
                prox_x,
                0.0,
                0.0,
            ),
            0.001,
        )
    )

    # --------------------------------------------------------
    # Main 15 mm structural support.
    # --------------------------------------------------------

    parts.append(
        box_part(
            f"{name}Support",
            (
                support_length,
                support_thickness,
                support_height,
            ),
            (
                support_x,
                0.0,
                0.0,
            ),
            support_mass,
        )
    )

    # --------------------------------------------------------
    # Left and right tendon-guide arms.
    #
    # These reproduce the MuJoCo physical guide blocks.
    # Tendon mathematical guide points remain in C++ and
    # are NOT changed here.
    # --------------------------------------------------------

    parts.append(
        box_part(
            f"{name}GuideLeft",
            (
                guide_x_length,
                guide_width,
                guide_z_height,
            ),
            (
                -guide_x,
                -guide_y_center,
                0.0,
            ),
            guide_mass_each,
        )
    )

    parts.append(
        box_part(
            f"{name}GuideRight",
            (
                guide_x_length,
                guide_width,
                guide_z_height,
            ),
            (
                -guide_x,
                +guide_y_center,
                0.0,
            ),
            guide_mass_each,
        )
    )

    # --------------------------------------------------------
    # Distal thin spine.
    #
    # Original MuJoCo connector mass = 1 g.
    # --------------------------------------------------------

    parts.append(
        box_part(
            f"{name}DistSpine",
            (
                distal_length,
                spine_thickness_y,
                distal_height,
            ),
            (
                distal_x,
                0.0,
                0.0,
            ),
            0.001,
        )
    )

    return f'''
        <!-- ======================================================
             {name.upper()} - FISHSIM-STYLE COMPOUND TAIL

             Local origin = proximal revolute joint.

             Replaces the previous full-volume fish-shaped
             physics mesh with:

                 thin spine
                    +
                 15 mm support
                    +
                 tendon guide arms
                    +
                 thin spine

             Joint positions and tendon guide coordinates
             remain unchanged.
             ====================================================== -->

        <link
            name="{name}"
            type="compound"
            physics="submerged">

{''.join(parts)}
        </link>
'''


# ============================================================
# Tail0
#
# fishsim:
#
# body-tail0      = 14 mm
# proximal half   =  7 mm
# support         = 15 mm
# tail0-tail1     = 15 mm
# distal half     =  7.5 mm
#
# joint distance:
# 7 + 15 + 7.5 = 29.5 mm
# ============================================================

tail0 = make_tail_link(
    name="Tail0",

    prox_length=0.0070,
    prox_height=0.075,

    support_height=0.100,

    guide_x=0.0190,
    guide_y_center=0.02575,
    guide_width=0.0485,

    distal_length=0.0075,
    distal_height=0.070,

    support_mass=0.005625,
    guide_mass_each=0.0050925,
)

replace_link("Tail0", tail0)


# ============================================================
# Tail1
#
# proximal = 15/2 = 7.5 mm
# support  = 15 mm
# distal   = 17/2 = 8.5 mm
#
# total = 31.0 mm
# ============================================================

tail1 = make_tail_link(
    name="Tail1",

    prox_length=0.0075,
    prox_height=0.070,

    support_height=0.095,

    guide_x=0.0195,
    guide_y_center=0.02075,
    guide_width=0.0385,

    distal_length=0.0085,
    distal_height=0.065,

    support_mass=0.00534375,
    guide_mass_each=0.0040425,
)

replace_link("Tail1", tail1)


# ============================================================
# Tail2
#
# proximal = 17/2 = 8.5 mm
# support  = 15 mm
# distal   = 14/2 = 7 mm
#
# total = 30.5 mm
# ============================================================

tail2 = make_tail_link(
    name="Tail2",

    prox_length=0.0085,
    prox_height=0.065,

    support_height=0.090,

    guide_x=0.0205,
    guide_y_center=0.01575,
    guide_width=0.0285,

    distal_length=0.0070,
    distal_height=0.060,

    support_mass=0.0050625,
    guide_mass_each=0.0029925,
)

replace_link("Tail2", tail2)


# ============================================================
# Tail3
#
# proximal = 14/2 = 7 mm
# support  = 15 mm
# distal   = 6/2 = 3 mm
#
# total = 25.0 mm
# ============================================================

tail3 = make_tail_link(
    name="Tail3",

    prox_length=0.0070,
    prox_height=0.060,

    support_height=0.085,

    guide_x=0.0190,
    guide_y_center=0.01075,
    guide_width=0.0185,

    distal_length=0.0030,
    distal_height=0.055,

    support_mass=0.00478125,
    guide_mass_each=0.0019425,
)

replace_link("Tail3", tail3)


# ============================================================
# Tail4
#
# Original fishsim proximal half:
#     tail3-tail4 / 2 = 3 mm
#
# Main support:
#     15 mm
#
# Current verified Stonefish CaudalJoint position:
#     21.55 mm from TailJoint4
#
# Therefore for THIS diagnostic we preserve that already
# verified CaudalJoint location and use:
#
#     distal = 21.55 - 3 - 15
#            = 3.55 mm
#
# We DO NOT move the caudal fin in this experiment.
#
# Mass of this distal connector stays 1 g, matching the
# original fishsim connector-mass convention.
# ============================================================

tail4 = make_tail_link(
    name="Tail4",

    prox_length=0.0030,
    prox_height=0.055,

    support_height=0.080,

    guide_x=0.0150,
    guide_y_center=0.00575,
    guide_width=0.0085,

    distal_length=0.00355,
    distal_height=0.045,

    support_mass=0.004500,
    guide_mass_each=0.0008925,
)

replace_link("Tail4", tail4)


# ============================================================
# Restore WET physics.
#
# Current public master still contains an old diagnostic
# "surface" state. This test must use the stable wet model.
# ============================================================

text = text.replace(
    'physics="surface"',
    'physics="submerged"',
)

# MotorShaft remains SURFACE.
motor_pattern = (
    r'(<link\s+'
    r'name="MotorShaft"\s+'
    r'type="box"\s+'
    r'physics=")submerged(")'
)

text, n_motor = re.subn(
    motor_pattern,
    r'\1surface\2',
    text,
    count=1,
    flags=re.S,
)

if n_motor != 1:
    raise RuntimeError(
        f"MotorShaft physics patch failed: {n_motor}"
    )


# ============================================================
# CaudalFin must be rigidly fixed to Tail4.
#
# Keep the already verified Stonefish origin:
#     x = -0.02155 m
#
# This experiment is about Tail0..Tail4 geometry ONLY.
# ============================================================

caudal_fixed = '''
        <joint
            name="CaudalJoint"
            type="fixed">

            <parent
                name="Tail4"/>

            <child
                name="CaudalFin"/>

            <origin
                xyz="-0.02155 0.0 0.0"
                rpy="0.0 0.0 0.0"/>

        </joint>
'''

caudal_pattern = (
    r'<joint\s+'
    r'name="CaudalJoint"\s+'
    r'type="(?:fixed|revolute)">'
    r'.*?'
    r'</joint>'
)

text, n_caudal = re.subn(
    caudal_pattern,
    caudal_fixed.strip(),
    text,
    count=1,
    flags=re.S,
)

if n_caudal != 1:
    raise RuntimeError(
        f"CaudalJoint patch failed: {n_caudal}"
    )


# ============================================================
# Sanity checks
# ============================================================

expected_joint_origins = {
    "TailJoint0": "-0.1810",
    "TailJoint1": "-0.0295",
    "TailJoint2": "-0.0310",
    "TailJoint3": "-0.0305",
    "TailJoint4": "-0.0250",
}

for joint, x in expected_joint_origins.items():

    pattern = (
        rf'name="{joint}".*?'
        rf'<origin\s+'
        rf'xyz="{re.escape(x)} 0\.0 0\.0"'
    )

    if not re.search(
        pattern,
        text,
        flags=re.S,
    ):
        raise RuntimeError(
            f"Joint geometry changed unexpectedly: {joint}"
        )


for i in range(5):
    if not re.search(
        rf'<link\s+'
        rf'name="Tail{i}"\s+'
        rf'type="compound"\s+'
        rf'physics="submerged">',
        text,
        flags=re.S,
    ):
        raise RuntimeError(
            f"Tail{i} is not submerged compound"
        )


if not re.search(
    r'name="CaudalJoint"\s+'
    r'type="fixed"',
    text,
    flags=re.S,
):
    raise RuntimeError(
        "CaudalJoint is not fixed"
    )


# ============================================================
# Check SURFACE links.
#
# IMPORTANT:
# Inspect each opening <link ...> / <base_link ...> tag
# independently. Do not allow regex to cross into another link.
# ============================================================

surface_links = []

for match in re.finditer(
    r'<(?:base_link|link)\b[^>]*>',
    text,
    flags=re.S,
):
    tag = match.group(0)

    if 'physics="surface"' not in tag:
        continue

    name_match = re.search(
        r'\bname="([^"]+)"',
        tag,
    )

    if not name_match:
        raise RuntimeError(
            "Found surface link without name:\n"
            + tag
        )

    surface_links.append(
        name_match.group(1)
    )

if surface_links != ["MotorShaft"]:
    raise RuntimeError(
        "Unexpected surface links: "
        + repr(surface_links)
    )



# ============================================================
# Write
# ============================================================

SCN.write_text(text)

print()
print("============================================")
print("TAIL GEOMETRY PATCH COMPLETE")
print("============================================")
print("Tail0..Tail4 : compound / submerged")
print("CaudalJoint  : fixed")
print("MotorShaft   : surface")
print("Other fish   : submerged")
print()
print("Joint origins were NOT changed.")
print("Tendon routing was NOT changed.")
print("Tendon guide coordinates were NOT changed.")
print("Tendon actuator code was NOT changed.")
print("============================================")
