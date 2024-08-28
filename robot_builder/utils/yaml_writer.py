import yaml
from loguru import logger


class VerboseSafeDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def load_yaml_abs(filepath: str):
    try:
        with open((filepath), "r") as file:
            yamlfile = yaml.safe_load(file)
    except FileNotFoundError as e:
        logger.error(f"Error: {e}. Make sure the robot configuration file exists.")
        return {}
    except yaml.YAMLError as e:
        logger.error(
            f"Error: {e}. Invalid YAML format in the robot configuration file."
        )
        return {}
    return yamlfile


def write_yaml_abs(data, file_path):
    try:
        with open(file_path, "w") as file:
            yaml.dump(data, file, Dumper=VerboseSafeDumper)
        logger.info(f"Data has been successfully written to the file: {file_path}")
    except IOError as e:
        logger.info(f"IOError occurred while writing the file: {e}")
    except Exception as e:
        logger.info(f"An error occurred while writing the file: {e}")
