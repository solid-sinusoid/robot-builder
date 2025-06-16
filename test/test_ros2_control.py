import pytest
import yaml
from pathlib import Path
import os

from robot_builder.external.ros2_control import ControllerManager
from robot_builder.parser.urdf import URDF_parser

current_dir = os.path.dirname(__file__)
urdf_path = os.path.join(current_dir, "../urdf/current.urdf")

robot = URDF_parser.load(
    urdf_path,
    base_link_name="base_link",
    ee_link_name="gripper_grasp_point",
    merged_gripper_joint=True
)

@pytest.fixture
def sample_yaml(tmp_path: Path) -> Path:
    text = """
controller_manager:
  update_rate: {{ update_rate }}
  ros__parameters:
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

joint_trajectory_controller:
  ros__parameters:
    joints: {{ actuated_joint_names }}
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
"""
    p = tmp_path / "ur_config.yaml"
    p.write_text(text.strip())
    return p

def test_robot_yaml_patch(sample_yaml: Path, tmp_path: Path):
    out = tmp_path / "patched.yaml"

    mgr = ControllerManager(
        robot=robot,
        cfg_path=sample_yaml,
        var_map={
            "update_rate": 777,
        }
    )
    mgr.save(out)

    patched = yaml.safe_load(out.read_text())

    assert patched["controller_manager"]["update_rate"] == 777

    jt_params = patched["joint_trajectory_controller"]["ros__parameters"]
    assert jt_params["joints"] == robot.actuated_joint_names
    assert "position" in jt_params["state_interfaces"]
    assert "position" in jt_params["command_interfaces"]
