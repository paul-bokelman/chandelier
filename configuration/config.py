from typing import TypedDict, Literal, cast, Any
from enum import Enum
import yaml

class Environments(Enum):
    DEVELOPMENT = 'development'
    TESTING = 'testing'
    PRODUCTION = 'production'
    GENERAL = 'general'

ConfigurationKeys = Literal['debug']

# general configuration
class GeneralConfigurationSchema(TypedDict):
    up: int
    down: int

# environment specific configuration
class ConfigurationSchema(GeneralConfigurationSchema):#
    debug: bool
    
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
            cls._instance.load_config(env)
        return cls._instance

    def load_config(self, env: Environments = Environments.DEVELOPMENT):
        with open('config.yml', 'r') as file:
            configs: ConfigurationFileSchema = yaml.safe_load(file)
            env_configuration = configs.get(env.value)
            general_configuration = configs.get('general')
            assert env_configuration is not None, f"Configuration for environment {env.value} not found"
            assert general_configuration is not None, f"Gneral configuration not found"
            self.config = env_configuration + general_configuration

            for key in self.config.keys():
                assert key in ConfigurationSchema.__annotations__, f"Invalid configuration key: {key}"
                assert self.config[key] is not None, f"Invalid configuration value for key: {key}"

            self.config = cast(ConfigurationSchema, self.config)

    def get(self, key: ConfigurationKeys) -> Any:
        return self.config.get(key)

# Create a global config instance
config = Config()  # Default to development; will be overridden in main module