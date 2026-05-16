#!/usr/bin/env bash
set -euo pipefail

TARGET_DIR="${1:-/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_robot/cyberdog_description/xacro}"
GAZEBO_XACRO="${TARGET_DIR}/gazebo.xacro"
MARKER="<!-- MiCup RGB camera sensor -->"

if [[ ! -f "${GAZEBO_XACRO}" ]]; then
  echo "gazebo.xacro not found: ${GAZEBO_XACRO}" >&2
  exit 1
fi

python3 - "${GAZEBO_XACRO}" "${MARKER}" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
marker = sys.argv[2]
text = path.read_text()

if marker in text:
    print(f"RGB camera sensor already exists in {path}")
    print("Expected topics: /rgb_camera/image_raw, /rgb_camera/camera_info")
    raise SystemExit(0)

insert = f"""

    {marker}
    <gazebo reference="RGB_camera_link">
        <sensor name="rgb_camera" type="camera">
            <always_on>true</always_on>
            <visualize>true</visualize>
            <update_rate>30</update_rate>
            <pose>0 0 0 0 0 0</pose>
            <camera name="rgb_camera">
                <horizontal_fov>1.047</horizontal_fov>
                <image>
                    <width>640</width>
                    <height>480</height>
                    <format>R8G8B8</format>
                </image>
                <clip>
                    <near>0.05</near>
                    <far>20.0</far>
                </clip>
            </camera>
            <plugin name="rgb_camera_controller" filename="libgazebo_ros_camera.so">
                <camera_name>rgb_camera</camera_name>
                <frame_name>RGB_camera_link</frame_name>
            </plugin>
        </sensor>
    </gazebo>
"""

closing = "\n</robot>"
if closing not in text:
    raise SystemExit("Could not find closing </robot> tag")

backup = path.with_suffix(path.suffix + ".bak")
if not backup.exists():
    backup.write_text(text)

path.write_text(text.replace(closing, insert + closing, 1))
print(f"Added RGB camera sensor to {path}")
print("Expected topics: /rgb_camera/image_raw, /rgb_camera/camera_info")
PY
