#!/usr/bin/env bash
set -euo pipefail

TARGET_DIR="${1:-/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_robot/cyberdog_description/xacro}"
GAZEBO_XACRO="${TARGET_DIR}/gazebo.xacro"
MARKER="<!-- MiCup D435 camera sensor -->"

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
    print(f"D435 camera sensor already exists in {path}")
    raise SystemExit(0)

insert = f"""

    {marker}
    <gazebo reference="D435_camera_link">
        <sensor name="d435_rgb_camera" type="camera">
            <always_on>true</always_on>
            <visualize>true</visualize>
            <update_rate>30</update_rate>
            <pose>0 0 0 0 0 0</pose>
            <camera name="d435_rgb_camera">
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
            <plugin name="d435_rgb_camera_controller" filename="libgazebo_ros_camera.so">
                <ros>
                    <namespace>/d435/color</namespace>
                </ros>
                <camera_name>image</camera_name>
                <frame_name>D435_camera_link</frame_name>
            </plugin>
        </sensor>

        <sensor name="d435_depth_camera" type="depth">
            <always_on>true</always_on>
            <visualize>true</visualize>
            <update_rate>30</update_rate>
            <pose>0 0 0 0 0 0</pose>
            <camera name="d435_depth_camera">
                <horizontal_fov>1.047</horizontal_fov>
                <image>
                    <width>640</width>
                    <height>480</height>
                    <format>R8G8B8</format>
                </image>
                <clip>
                    <near>0.05</near>
                    <far>10.0</far>
                </clip>
            </camera>
            <plugin name="d435_depth_camera_controller" filename="libgazebo_ros_camera.so">
                <ros>
                    <namespace>/d435/depth</namespace>
                </ros>
                <camera_name>image</camera_name>
                <frame_name>D435_camera_link</frame_name>
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
print(f"Added D435 RGB/depth camera sensors to {path}")
print("Expected topics include: /d435/color/image/image_raw, /d435/color/image/camera_info")
print("Expected depth topics depend on gazebo_ros_camera naming; check with: ros2 topic list | grep d435")
PY
