from pathlib import Path
from datetime import datetime
import re
import shutil


# ============================================================
# CONFIG
# ============================================================

SCN = Path(
    "simulation/data/robots/bionic_fish/bionic_fish.scn"
)

if not SCN.exists():
    raise SystemExit(
        f"ERROR: cannot find:\n{SCN}"
    )


# ============================================================
# BACKUP
# ============================================================

stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

backup = SCN.with_name(
    SCN.name + f".stage3_backup_{stamp}"
)

shutil.copy2(SCN, backup)

text = SCN.read_text()

print(f"Input : {SCN}")
print(f"Backup: {backup}")


# ============================================================
# HELPERS
# ============================================================

def replace_named_block(
    text,
    tag,
    name,
    replacement,
):
    """
    Replace one complete XML block such as:

        <link ... name="MotorShaft" ...>
            ...
        </link>

    Attribute order does not matter.
    """

    pattern = (
        rf'<{tag}\b'
        rf'(?=[^>]*\bname="{re.escape(name)}")'
        rf'[^>]*>'
        rf'.*?'
        rf'</{tag}>'
    )

    new_text, count = re.subn(
        pattern,
        lambda _: replacement.strip(),
        text,
        count=1,
        flags=re.S,
    )

    if count != 1:
        raise RuntimeError(
            f"Cannot replace <{tag}> "
            f'name="{name}": matches={count}'
        )

    return new_text


def opening_tag_exists(
    text,
    tag,
    name,
    required_attributes=None,
):
    """
    Check one opening tag without depending
    on XML attribute order.
    """

    required_attributes = (
        required_attributes or {}
    )

    pattern = (
        rf'<{tag}\b'
        rf'(?=[^>]*\bname="{re.escape(name)}")'
    )

    for key, value in required_attributes.items():
        pattern += (
            rf'(?=[^>]*\b'
            rf'{re.escape(key)}='
            rf'"{re.escape(value)}")'
        )

    pattern += r'[^>]*>'

    return (
        re.search(
            pattern,
            text,
            flags=re.S,
        )
        is not None
    )


# ============================================================
# MOTOR SHAFT
#
# Source fishsim approximation:
#
# central shaft:
#   15 x 120 x 15 mm
#   mass = 0.05 kg
#
# crank arms:
#   15 x 10 x 39.5 mm
#
# PLA density:
#   1250 kg/m^3
#
# arm mass:
#   0.015 * 0.010 * 0.0395 * 1250
#   = 0.00740625 kg
#
# Motor link local coordinates:
#
# left arm:
#   y = -0.055
#   z = -0.01975
#
# right arm:
#   y = +0.055
#   z = +0.01975
#
# Tendon attachment ends will be:
#
# left:
#   (0, -0.055, -0.0345)
#
# right:
#   (0, +0.055, +0.0345)
#
# Those final attachment coordinates are used
# in tendon_tail_actuator.cpp.
# ============================================================

motor_link = r'''
<link
    name="MotorShaft"
    type="compound"
    physics="submerged">

    <external_part
        name="MotorShaftCore"
        type="box"
        physics="submerged"
        buoyant="false">

        <dimensions
            xyz="0.015 0.120 0.015"/>

        <origin
            xyz="0.0 0.0 0.0"
            rpy="0.0 0.0 0.0"/>

        <material
            name="$(arg material_name)"/>

        <look
            name="BionicFishBodyLook"/>

        <mass
            value="0.050000000"/>

        <compound_transform
            xyz="0.0 0.0 0.0"
            rpy="0.0 0.0 0.0"/>

    </external_part>


    <external_part
        name="MotorArmLeft"
        type="box"
        physics="submerged"
        buoyant="false">

        <dimensions
            xyz="0.015 0.010 0.0395"/>

        <origin
            xyz="0.0 0.0 0.0"
            rpy="0.0 0.0 0.0"/>

        <material
            name="$(arg material_name)"/>

        <look
            name="BionicFishBodyLook"/>

        <mass
            value="0.007406250"/>

        <compound_transform
            xyz="0.0 -0.055 -0.01975"
            rpy="0.0 0.0 0.0"/>

    </external_part>


    <external_part
        name="MotorArmRight"
        type="box"
        physics="submerged"
        buoyant="false">

        <dimensions
            xyz="0.015 0.010 0.0395"/>

        <origin
            xyz="0.0 0.0 0.0"
            rpy="0.0 0.0 0.0"/>

        <material
            name="$(arg material_name)"/>

        <look
            name="BionicFishBodyLook"/>

        <mass
            value="0.007406250"/>

        <compound_transform
            xyz="0.0 0.055 0.01975"
            rpy="0.0 0.0 0.0"/>

    </external_part>

</link>
'''

text = replace_named_block(
    text,
    "link",
    "MotorShaft",
    motor_link,
)


# ============================================================
# M1 SERVO
#
# Source fishsim:
#
# velocity gain = 100
# max torque    = ±100 Nm
# max velocity  = ±2*pi*5 rad/s
#
# C++ later switches this servo explicitly
# to VELOCITY mode and commands 1.25 Hz.
# ============================================================

motor_servo = r'''
<actuator
    name="M1Servo"
    type="servo">

    <controller
        position_gain="0.0"
        velocity_gain="100.0"
        max_torque="100.0"
        max_velocity="31.41592653589793"/>

    <joint
        name="M1Joint"/>

    <initial
        position="0.0"/>

</actuator>
'''

text = replace_named_block(
    text,
    "actuator",
    "M1Servo",
    motor_servo,
)


# ============================================================
# SANITY: M1 JOINT
# ============================================================

if not opening_tag_exists(
    text,
    "joint",
    "M1Joint",
    {
        "type": "revolute",
    },
):
    raise RuntimeError(
        "ERROR: M1Joint is not revolute."
    )


# Extract complete M1Joint block so that
# axis checking cannot accidentally cross into
# another joint.

m1_match = re.search(
    r'<joint\b'
    r'(?=[^>]*\bname="M1Joint")'
    r'[^>]*>'
    r'.*?'
    r'</joint>',
    text,
    flags=re.S,
)

if not m1_match:
    raise RuntimeError(
        "ERROR: cannot find complete M1Joint."
    )

m1_block = m1_match.group(0)

if not re.search(
    r'<axis\b'
    r'[^>]*'
    r'xyz="0\.0\s+1\.0\s+0\.0"',
    m1_block,
    flags=re.S,
):
    raise RuntimeError(
        "ERROR: M1Joint axis is not +Y."
    )


# ============================================================
# SANITY: CAUDAL MUST REMAIN FIXED
# ============================================================

if not opening_tag_exists(
    text,
    "joint",
    "CaudalJoint",
    {
        "type": "fixed",
    },
):
    raise RuntimeError(
        "ERROR: CaudalJoint is not fixed."
    )


# ============================================================
# SANITY: TAIL LINKS MUST STILL EXIST
# ============================================================

for i in range(5):

    name = f"Tail{i}"

    if not opening_tag_exists(
        text,
        "link",
        name,
    ):
        raise RuntimeError(
            f"ERROR: {name} missing."
        )


# ============================================================
# SANITY: MOTOR PATCH
# ============================================================

if not opening_tag_exists(
    text,
    "link",
    "MotorShaft",
    {
        "type": "compound",
        "physics": "submerged",
    },
):
    raise RuntimeError(
        "ERROR: MotorShaft compound patch failed."
    )


for part in (
    "MotorShaftCore",
    "MotorArmLeft",
    "MotorArmRight",
):
    if f'name="{part}"' not in text:
        raise RuntimeError(
            f"ERROR: missing {part}"
        )


# ============================================================
# SANITY: SERVO PATCH
# ============================================================

servo_match = re.search(
    r'<actuator\b'
    r'(?=[^>]*\bname="M1Servo")'
    r'[^>]*>'
    r'.*?'
    r'</actuator>',
    text,
    flags=re.S,
)

if not servo_match:
    raise RuntimeError(
        "ERROR: cannot find M1Servo."
    )

servo_block = servo_match.group(0)

checks = (
    'velocity_gain="100.0"',
    'max_torque="100.0"',
    'max_velocity="31.41592653589793"',
    'name="M1Joint"',
)

for value in checks:
    if value not in servo_block:
        raise RuntimeError(
            "ERROR: M1Servo missing: "
            + value
        )


# ============================================================
# WRITE ONLY AFTER ALL CHECKS PASS
# ============================================================

SCN.write_text(text)


# ============================================================
# RESULT
# ============================================================

print()
print("============================================")
print("STAGE 3 MOTOR PATCH COMPLETE")
print("============================================")
print()
print("MotorShaft:")
print("  type           = compound")
print("  physics        = submerged")
print("  shaft mass     = 0.050000 kg")
print("  left arm mass  = 0.00740625 kg")
print("  right arm mass = 0.00740625 kg")
print()
print("M1Joint:")
print("  type = revolute")
print("  axis = +Y")
print()
print("M1Servo:")
print("  velocity gain = 100")
print("  max torque    = 100 Nm")
print("  max velocity  = 31.4159265 rad/s")
print("                = 5 Hz")
print()
print("CaudalJoint:")
print("  fixed")
print()
print("Tail0..Tail4:")
print("  untouched")
print()
print("Tendon routing:")
print("  untouched")
print()
print("Output:")
print(f"  {SCN}")
print()
print("Backup:")
print(f"  {backup}")
print()
print("============================================")
