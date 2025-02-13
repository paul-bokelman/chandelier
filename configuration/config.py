from typing import TypedDict, Literal, Any, get_args
from enum import Enum
import yaml

class Environments(Enum):
    GENERAL = 'general'
    DEVELOPMENT = 'development'
    TESTING = 'testing'
    PRODUCTION = 'production'

ConfigurationKeys = Literal[
    'encoder_pins', 'wall_switch_pins', 'service_button_pin', 'reboot_button_pin', 'charging_pin', 'led_pin', 'up', 'down', 'modes', 'debug', 'max_counts', 'n_motors', 'suppress_count_logging', 'disabled_motors', 'random_state_duration', 'sequence_state_duration',
    'candles_per_charge_cycle', 'charge_cycle_time', 'available_charging_hours', 'throttle_offset', 'calibration_counts', 'calibration_file_path', "skip_find_home", "uncalibrated_up_throttle", "uncalibrated_down_throttle", "max_recovery_attempts", "recovery_counts", "default_allowable_up_cps", 'default_allowable_down_cps', 'stall_buffer_normal', 'stall_buffer_home', 'stall_buffer_random_mode', 'stall_buffer_sequence_mode', 'stall_buffer_recovery', 'borderline_stall_zone', 'duration_before_recalibration', 'dead_motors', "c1", "c2", "random_params", "wave_params", "alternating_params", "charging_buffer_distance", "button_wait_time", "switch_wait_time"
]

class ConfigurationFileSchema(TypedDict):
    development: dict[str, Any]
    testing: dict[str, Any]
    production: dict[str, Any]
    general: dict[str, Any]

class Config:
    _instance = None

    def __init__(self) -> None:
        self.config: dict[str, Any]

    def __new__(cls, env: Environments = Environments.DEVELOPMENT):
        if cls._instance is None:
            cls._instance = super(Config, cls).__new__(cls)
            cls._instance.load(env)
        return cls._instance

    def load(self, env: Environments = Environments.DEVELOPMENT):
        with open('configuration/config.yaml', 'r') as file:
            configs: ConfigurationFileSchema = yaml.safe_load(file)
            env_configuration = configs.get(env.value)
            general_configuration = configs.get('general')
            testing_configuration = configs.get('testing_mode')
            sequences_configuration = configs.get('sequences')

            if env_configuration is None:
                raise ValueError(f"Configuration for environment {env.value} not found")
            
            if general_configuration is None:
                raise ValueError("General configuration not found")
            
            if testing_configuration is None:
                raise ValueError("Testing configuration not found")
            
            if sequences_configuration is None:
                raise ValueError("Sequences configuration not found")

            self.config = {**general_configuration, **env_configuration, **testing_configuration, **sequences_configuration}

            for key in self.config.keys():
                if key not in list(get_args(ConfigurationKeys)):
                    raise ValueError(f"Invalid configuration key: {key}")
                
                if self.config[key] is None:
                    raise ValueError(f"Invalid configuration value for key: {key}")

    def get(self, key: ConfigurationKeys) -> Any:
        return self.config.get(key)

# Create a global config instance
config = Config()  # Default to development; will be overridden in main module