"""
Robot configuration protocol and dynamic loader for dora-moveit.

All robot-specific configuration must satisfy the RobotConfig protocol.
Library operators load the config at startup via load_config(), which reads
the ROBOT_CONFIG_MODULE environment variable to find the user's config class.

Example:
    # In your app's config module (e.g., my_app/config/my_robot.py):
    class MyRobotConfig:
        NUM_JOINTS = 6
        JOINT_LOWER_LIMITS = np.array([...])
        # ... all required attributes

    # In the dora dataflow YAML:
    env:
      ROBOT_CONFIG_MODULE: "my_app.config.my_robot"

    # In library operators (automatic):
    from dora_moveit.config import load_config
    config = load_config()
    print(config.NUM_JOINTS)  # 6
"""

import os
import importlib
import numpy as np
from typing import Dict, List, Tuple, runtime_checkable
from typing import Protocol


@runtime_checkable
class RobotConfig(Protocol):
    """Protocol that all robot configurations must satisfy.

    Library operators access the config through this interface.
    Implement this for your specific robot (GEN72, LM3, UR5, etc.).
    """

    NUM_JOINTS: int
    JOINT_LOWER_LIMITS: np.ndarray
    JOINT_UPPER_LIMITS: np.ndarray
    JOINT_VELOCITY_LIMITS: np.ndarray
    LINK_TRANSFORMS: List[Dict]
    COLLISION_GEOMETRY: List[tuple]
    COLLISION_MARGIN: float
    HOME_CONFIG: np.ndarray
    SAFE_CONFIG: np.ndarray
    NAMED_POSES: Dict[str, np.ndarray]

    @staticmethod
    def get_joint_limits() -> Tuple[np.ndarray, np.ndarray]: ...

    @staticmethod
    def is_config_valid(q: np.ndarray) -> bool: ...

    @staticmethod
    def clip_to_limits(q: np.ndarray) -> np.ndarray: ...


# Per-process cache
_cached_config = None


def load_config(env_var: str = "ROBOT_CONFIG_MODULE"):
    """Load robot configuration from the module specified by environment variable.

    The env var should point to a Python module containing a class whose name
    ends with 'Config' (e.g., 'GEN72Config', 'LM3Config'). The module must
    be importable from the current Python environment (pip install -e).

    Examples:
        ROBOT_CONFIG_MODULE=hunter_arm_demo.config.gen72
        ROBOT_CONFIG_MODULE=lebai_demo.config.lm3

    Caches the result per-process — subsequent calls return the same class.

    Returns:
        The robot config class satisfying RobotConfig protocol.

    Raises:
        RuntimeError: If env var is not set or module cannot be imported.
    """
    global _cached_config
    if _cached_config is not None:
        return _cached_config

    module_path = os.environ.get(env_var)
    if not module_path:
        raise RuntimeError(
            f"Environment variable {env_var} is not set. "
            f"Set it to a Python module path containing your robot config, "
            f"e.g., ROBOT_CONFIG_MODULE=my_app.config.my_robot"
        )

    try:
        module = importlib.import_module(module_path)
    except ImportError as e:
        raise RuntimeError(
            f"Cannot import robot config module '{module_path}': {e}. "
            f"Make sure it is installed (pip install -e .) or on PYTHONPATH."
        ) from e

    # Find the Config class in the module
    config_class = None
    for name in dir(module):
        obj = getattr(module, name)
        if (isinstance(obj, type) and name.endswith("Config")
                and name not in ("JointConfig", "DHParams")):
            config_class = obj
            break

    if config_class is None:
        raise RuntimeError(
            f"No class ending with 'Config' found in module '{module_path}'. "
            f"Expected a class like GEN72Config, LM3Config, etc."
        )

    _cached_config = config_class
    return _cached_config
