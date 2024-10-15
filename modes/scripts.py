import os
from lib.utils import log
import inquirer

def run():
    log.info("Running scripts mode")

    script_choices = [c.replace(".sh", "") for c in os.listdir("scripts")]

    # prompt: should reset all calibration data?
    question = [inquirer.List("script", message="What script would you like to run?", choices=script_choices)]
    answer = inquirer.prompt(question)

    # ensure valid input
    if answer is None:
        raise ValueError("Invalid input")

    # run script
    os.system(f"bash scripts/{answer['script']}.sh")