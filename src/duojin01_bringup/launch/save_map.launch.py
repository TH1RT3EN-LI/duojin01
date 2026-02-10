import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess

def _next_numeric_map_name(output_dir: str) -> str:
    latest = -1
    if os.path.isdir(output_dir):
        for filename in os.listdir(output_dir):
            if not filename.endswith('.yaml'):
                continue
            stem = os.path.splitext(filename)[0]
            if stem.isdigit():
                latest = max(latest, int(stem))
    return str(latest + 1)


def _save_map_action(context: LaunchContext):
    
    bringup_share = get_package_share_directory('duojin01_bringup')
    default_output_dir = os.path.join(bringup_share, "maps")
    
    output_dir = LaunchConfiguration('output_dir').perform(context) or default_output_dir
    map_name_arg = LaunchConfiguration('map_name').perform(context).strip()
    map_name = _next_numeric_map_name(output_dir) if map_name_arg == "" or map_name_arg.lower() == "auto" else map_name_arg
    wait_timeout = LaunchConfiguration('wait_timeout').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    
    save_cmd = [
        'bash', '-lc',
        f'''
        set -e

        MAP_NAME="{map_name}"
        OUT_DIR="{output_dir}"
        TIMEOUT="{wait_timeout}"
        USE_SIM_TIME="{use_sim_time}"

        mkdir -p "$OUT_DIR"

        echo "[save_map] waiting for /map ... (timeout=${{TIMEOUT}}s)"
        deadline=$((SECONDS + TIMEOUT))
        while true; do
        if ros2 topic list | grep -qx "/map"; then
            break
        fi
        if [ "$TIMEOUT" -gt 0 ] && [ $SECONDS -ge $deadline ]; then
            echo "[save_map] ERROR: timeout waiting for /map"
            exit 1
        fi
        sleep 0.2
        done

        prefix="$OUT_DIR/$MAP_NAME"
        echo "[save_map] saving to ${{prefix}}.pgm/.yaml"
        ros2 run nav2_map_server map_saver_cli -f "$prefix" --ros-args -p use_sim_time:="$USE_SIM_TIME"
        echo "[save_map] done"
        '''
    ]
    return [ExecuteProcess(cmd=save_cmd, output='screen')]


def generate_launch_description():
    bringup_share = get_package_share_directory('duojin01_bringup')
    default_output_dir = os.path.join(bringup_share, "maps")

    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='auto'),
        DeclareLaunchArgument('output_dir', default_value=default_output_dir),
        DeclareLaunchArgument('wait_timeout', default_value='30'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_save_map_action),
    ])
    
