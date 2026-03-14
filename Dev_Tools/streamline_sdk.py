import os
import shutil
import re

# Paths
BASE_DIR = "/Users/zclb00103397/Documents/01_Code_Qiyuan/interface"
SOURCE_ROOT = os.path.join(BASE_DIR, "ros2/robot")
TARGET_ROOT = os.path.join(BASE_DIR, "primebot_sdk/aimdk_msgs")

# 330 Interface Mapping (Target file name -> Source relative path from ros2/robot)
INTERFACE_MAP = {
    "GetMcAction.srv": "mc/action/srv/GetMcAction.srv",
    "SetMcAction.srv": "mc/action/srv/SetMcAction.srv",
    "SetMcPresetMotion.srv": "mc/motion/srv/SetMcPresetMotion.srv",
    "McLocomotionVelocity.msg": "mc/motion/McLocomotionVelocity.msg",
    "SetMcInputSource.srv": "mc/motion/srv/SetMcInputSource.srv",
    "GetCurrentInputSource.srv": "mc/motion/srv/GetCurrentInputSource.srv",
    "McCommonState.msg": "mc/data/msg/McCommonState.msg",
    "JointCommand.msg": "hal/msg/JointCommand.msg",
    "JointState.msg": "hal/msg/JointState.msg",
    "PlayTts.srv": "interaction/srv/PlayTts.srv",
    "GetVolume.srv": "hal/audio/srv/GetVolume.srv",
    "SetVolume.srv": "hal/audio/srv/SetVolume.srv",
    "GetMute.srv": "hal/audio/srv/GetMute.srv",
    "SetMute.srv": "hal/audio/srv/SetMute.srv",
    "PlayAudioFile.srv": "hal/audio/srv/PlayAudioFile.srv",
    "PlayEmotion.srv": "interaction/srv/PlayEmotion.srv",
    "PlayVideo.srv": "face_ui/srv/PlayVideo.srv",
    "FaceEmojiStatus.msg": "face_ui/FaceEmojiStatus.msg",
    "TouchState.msg": "hal/msg/TouchState.msg",
    "PmuState.msg": "hal/msg/PmuState.msg",
    "SetRgbStrip.srv": "hal/srv/SetRgbStrip.srv",
}

# Known dependency locations (Mapping message name used in files -> Source relative path)
# This will be populated as we search. For simple cases, we search common, hal/msg, etc.
processed_files = set()
files_to_copy = set()

def find_source_file(msg_type):
    """Find the .msg file for a given type name in the ros2/robot directory tree."""
    # Special case for core types
    if msg_type in ["std_msgs", "geometry_msgs", "sensor_msgs", "nav_msgs"]:
        return None
        
    # Search for MessageName.msg
    for root, dirs, files in os.walk(SOURCE_ROOT):
        if f"{msg_type}.msg" in files:
            return os.path.relpath(os.path.join(root, f"{msg_type}.msg"), SOURCE_ROOT)
    return None

def get_dependencies(file_path):
    """Extract custom message dependencies from a .srv or .msg file."""
    deps = []
    if not os.path.exists(file_path):
        return deps
        
    with open(file_path, 'r') as f:
        content = f.read()
        
    # Matches patterns like "CommonRequest header" or "McActionStatus status"
    # Basic regex for ROS2 message fields
    lines = content.split('\n')
    for line in lines:
        line = line.strip()
        if not line or line.startswith('#') or line.startswith('---'):
            continue
        
        # Split by space and get the first part (type)
        parts = line.split()
        if not parts:
            continue
            
        msg_type = parts[0]
        # Remove array indicators []
        msg_type = msg_type.split('[')[0]
        
        # We only care about custom types (no int32, string, etc.)
        builtin_types = ["bool", "byte", "char", "float32", "float64", "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "string"]
        if msg_type not in builtin_types and '/' not in msg_type:
            deps.append(msg_type)
    return list(set(deps))

def collect_recursively(rel_path):
    """Recursively collect a file and its dependencies."""
    if rel_path in processed_files:
        return
    
    src_path = os.path.join(SOURCE_ROOT, rel_path)
    if not os.path.exists(src_path):
        print(f"Warning: Source file not found: {src_path}")
        return
        
    processed_files.add(rel_path)
    files_to_copy.add(rel_path)
    
    deps = get_dependencies(src_path)
    for dep in deps:
        dep_path = find_source_file(dep)
        if dep_path:
            collect_recursively(dep_path)

# Main logic
print("Starting recursive dependency collection...")
for target_name, rel_path in INTERFACE_MAP.items():
    collect_recursively(rel_path)

print(f"Total files identified to copy: {len(files_to_copy)}")

# Copying files
for rel_path in files_to_copy:
    src = os.path.join(SOURCE_ROOT, rel_path)
    dst = os.path.join(TARGET_ROOT, rel_path)
    
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copy2(src, dst)
    print(f"Copied: {rel_path}")

print("Streamlining complete.")
