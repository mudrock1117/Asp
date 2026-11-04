from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("s3_60_eco_2024", package_name="estun_moveit2_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
