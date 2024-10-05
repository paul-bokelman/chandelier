import asyncio
from lib.controller import MotorController
from lib.utils import log
import inquirer
import constants

# todo: refactor to work as preflight module

def calibration_mode():
    log.info("Running calibration mode")

    initial_questions = [
        inquirer.Confirm("reset", message="Reset calibration data?", default=False),
    ]

    initial_answers = inquirer.prompt(initial_questions)

    if initial_answers is None:
        log.error("Invalid input")
        return

    try:
        if initial_answers["reset"]:
            log.warning("Resetting calibration data")
            mc = MotorController()
            asyncio.run(mc.calibrate(reset=True))
        else:
            followup_questions = [
                inquirer.Text("update", message="Calibrate individual motors? (Enter comma separated list)", default="")
            ]

            followup_answers = inquirer.prompt(followup_questions)

            if followup_answers is None:
                log.error("Invalid input")
                return

            mc = MotorController()
            updating_motors = [int(x) for x in followup_answers["update"].replace(" ", "").split(",")]

            if not all(motor in range(constants.n_motors) for motor in updating_motors):
                log.error("Invalid motor channel")
                return
            
            asyncio.run(mc.calibrate(update=updating_motors))
    except Exception as e:
        log.error(f"An error occurred: {e}")
    finally:
        mc.stop_all_motors()
        log.info("Calibrations mode complete")