"""Configuration loader for YAML-based parameter management."""

from importlib import import_module
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml


class ConfigLoader:
    """Loads and manages configuration from YAML files."""

    def __init__(self, config_dir: Optional[str] = None):
        """
        Initialize the configuration loader.
        
        Args:
            config_dir: Directory containing YAML config files. 
                       If None, uses the default config directory.
        """
        if config_dir is None:
            # Default to the config directory in the package
            self.config_dir = Path(__file__).parent
        else:
            self.config_dir = Path(config_dir)

        self._config_cache: Dict[str, Any] = {}
        # Lazily populated driver caches
        self.ROBOT_ARM_DRIVERS: Dict[str, Any] = {}
        self.GRIPPER_DRIVERS: Dict[str, Any] = {}
        self.FORCE_SENSOR_DRIVERS: Dict[str, Any] = {}
        module_path = type(self).__module__
        root_parts = module_path.split(".config.")
        if len(root_parts) == 2:
            self._package_root = root_parts[0]
        else:
            self._package_root = module_path.rsplit(".", 1)[0]
        # Map driver types to module path and class name for runtime import
        self._driver_paths = {
            "robot_arm": {
                "ur": ("hardware_interface.drivers.ur_robot_driver", "UrRobotArmDriver"),
                "cobotta": ("hardware_interface.drivers.cobotta_robot_driver", "CobottaRobotArmDriver"),
            },
            "gripper": {
                "robotiq": ("hardware_interface.drivers.robotiq_gripper_driver", "RobotiqGripperDriver"),
            },
            "force_sensor": {
                "ur_builtin": ("hardware_interface.drivers.ur_force_sensor_driver", "UrForceSensorDriver"),
                "wacoh": ("hardware_interface.drivers.wacoh_force_sensor_driver", "WacohForceSensorDriver"),
            },
        }
        
    def load_config(self, config_name: str) -> Dict[str, Any]:
        """
        Load a configuration file.
        
        Args:
            config_name: Name of the config file (without .yaml extension)
            
        Returns:
            Dictionary containing the configuration
        """
        if config_name in self._config_cache:
            return self._config_cache[config_name]
            
        config_file = self.config_dir / f"{config_name}.yaml"
        
        if not config_file.exists():
            raise FileNotFoundError(f"Configuration file not found: {config_file}")
            
        with open(config_file, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
            
        self._config_cache[config_name] = config
        return config
    
    def reload_config(self, config_name: str) -> Dict[str, Any]:
        """
        Reload a configuration file, bypassing the cache.
        
        Args:
            config_name: Name of the config file (without .yaml extension)
            
        Returns:
            Dictionary containing the configuration
        """
        if config_name in self._config_cache:
            del self._config_cache[config_name]
        return self.load_config(config_name)
    
    def get_common_config(self) -> Dict[str, Any]:
        """Get common configuration parameters."""
        return self.load_config("common")
    
    def get_robot_config(self, robot_type: str) -> Dict[str, Any]:
        """
        Get robot-specific configuration.
        
        Args:
            robot_type: Type of robot (ur, cobotta)
            
        Returns:
            Dictionary containing robot configuration
        """
        return self.load_config(f"{robot_type}_robot")
    
    def get_sensor_config(self) -> Dict[str, Any]:
        """Get force sensor configuration."""
        return self.load_config("force_sensor")
    
    def get_gripper_config(self, gripper_type: str) -> Dict[str, Any]:
        """
        Get gripper-specific configuration.
        
        Args:
            gripper_type: Type of gripper (robotiq)
            
        Returns:
            Dictionary containing gripper configuration
        """
        return self.load_config(f"{gripper_type}_gripper")
    
    def get_driver_classes(self):
        """Get driver classes based on common configuration."""
        common_config = self.get_common_config()
        
        robot_arm_type = common_config.get("ROBOT_ARM_TYPE", "ur")
        gripper_type = common_config.get("GRIPPER_TYPE", "robotiq")
        force_sensor_type = common_config.get("FORCE_SENSOR_TYPE", "ur_builtin")

        robot_arm_class = self._get_driver_class("robot_arm", robot_arm_type)
        gripper_class = self._get_driver_class("gripper", gripper_type)
        force_sensor_class = self._get_driver_class("force_sensor", force_sensor_type)
        
        return {
            "robot_arm": robot_arm_class,
            "gripper": gripper_class,
            "force_sensor": force_sensor_class,
            "types": {
                "robot_arm": robot_arm_type,
                "gripper": gripper_type,
                "force_sensor": force_sensor_type,
            }
        }

    def _get_driver_class(self, category: str, driver_key: str):
        """Dynamically import and cache driver classes."""
        cache_map, path_map = self._get_cache_and_paths(category)

        if driver_key in cache_map:
            return cache_map[driver_key]

        try:
            rel_module_path, class_name = path_map[driver_key]
        except KeyError as exc:
            raise KeyError(f"Unknown {category} driver type '{driver_key}'") from exc

        module_path = f"{self._package_root}.{rel_module_path}"
        module = import_module(module_path)
        driver_class = getattr(module, class_name)
        cache_map[driver_key] = driver_class
        return driver_class

    def _get_cache_and_paths(self, category: str) -> Tuple[Dict[str, Any], Dict[str, Tuple[str, str]]]:
        """Return cache and path mapping for a driver category."""
        if category == "robot_arm":
            return self.ROBOT_ARM_DRIVERS, self._driver_paths["robot_arm"]
        if category == "gripper":
            return self.GRIPPER_DRIVERS, self._driver_paths["gripper"]
        if category == "force_sensor":
            return self.FORCE_SENSOR_DRIVERS, self._driver_paths["force_sensor"]
        raise KeyError(f"Unknown driver category '{category}'")


# Global instance for backward compatibility
_global_loader = None

def get_config_loader(config_dir: Optional[str] = None) -> ConfigLoader:
    """Get or create the global configuration loader."""
    global _global_loader
    if _global_loader is None:
        _global_loader = ConfigLoader(config_dir)
    return _global_loader