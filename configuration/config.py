from typing import TypedDict, Literal, cast, Any
from enum import Enum
import yaml

class Environments(Enum):
    GENERAL = 'general'
    DEVELOPMENT = 'development'
    TESTING = 'testing'
    PRODUCTION = 'production'

ConfigurationKeys = Literal[
    'encoder_pins', 'wall_switch_pins', 'service_button_pin', 'reboot_button_pin', 'charging_pin', 'led_pin', 'up', 'down', 'modes',
    'debug', 'max_counts', 'n_motors', 'suppress_count_logging', 'initial_disabled_motors', 'random_state_duration', 'sequence_state_duration',
    'candles_per_charge_cycle', 'charge_cycle_time', 'available_charging_hours', 'throttle_offset', 'uncalibrated_throttle', 'to_home_timeout',
    'to_position_timeout', 'max_time_between_encoder_readings', 'calibration_counts', 'calibration_speed_step', 'calibration_total_steps', 'calibration_file_path', "calibration_timeout"
]

# general configuration
class GeneralConfigurationSchema(TypedDict):
    encoder_pins: list[int]
    wall_switch_pins: list[int]
    service_button_pin: int
    reboot_button_pin: int
    charging_pin: int
    led_pin: int
    up: int
    down: int
    modes: list[str]

# environment specific configuration
class ConfigurationSchema(GeneralConfigurationSchema):
    debug: bool
    max_counts: int
    n_motors: int
    suppress_count_logging: bool
    initial_disabled_motors: list[int]
    random_state_duration: int
    sequence_state_duration: int
    candles_per_charge_cycle: int
    charge_cycle_time: int
    available_charging_hours: list[int]
    throttle_offset: float
    uncalibrated_throttle: float
    to_home_timeout: int
    to_position_timeout: int
    max_time_between_encoder_readings: int
    calibration_counts: int
    calibration_speed_step: float
    calibration_total_steps: int
    calibration_file_path: str
    calibration_timeout: int

class ConfigurationFileSchema(TypedDict):
    development: ConfigurationSchema
    testing: ConfigurationSchema
    production: ConfigurationSchema
    general: GeneralConfigurationSchema

class Config:
    _instance = None

    def __init__(self) -> None:
        self.config: ConfigurationSchema

    def __new__(cls, env: Environments = Environments.DEVELOPMENT):
        if cls._instance is None:
            cls._instance = super(Config, cls).__new__(cls)
            cls._instance.load(env)
        return cls._instance

    def load(self, env: Environments = Environments.DEVELOPMENT):
        with open('config.yml', 'r') as file:
            configs: ConfigurationFileSchema = yaml.safe_load(file)
            env_configuration = configs.get(env.value)
            general_configuration = configs.get('general')

            if env_configuration is None:
                raise ValueError(f"Configuration for environment {env.value} not found")
            
            if general_configuration is None:
                raise ValueError("General configuration not found")

            self.config = env_configuration + general_configuration

            for key in self.config.keys():
                if key not in ConfigurationSchema.__annotations__:
                    raise ValueError(f"Invalid configuration key: {key}")
                
                if self.config[key] is None:
                    raise ValueError(f"Invalid configuration value for key: {key}")

            self.config = cast(ConfigurationSchema, self.config)

    def get(self, key: ConfigurationKeys) -> Any:
        return self.config.get(key)

# Create a global config instance
config = Config()  # Default to development; will be overridden in main module