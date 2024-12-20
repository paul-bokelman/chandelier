from typing import Union, Literal, List, get_args
from lib.controller import MotorController
from lib.utils import log
import inquirer
from configuration.config import config

CalibrationOptions = Literal["default", "prompt"]
calibration_options: List[CalibrationOptions] = list(get_args(CalibrationOptions))

async def preflight(option: CalibrationOptions = "default", skip: bool = False) -> MotorController:
    mc = MotorController()

    if skip:
        return mc

    log.info("Running calibration preflight")

    if option not in calibration_options:
        raise ValueError("Invalid calibration option")

    # calibrate all motors with no resets or prompts
    if option == 'default':
        await mc.calibrate()
        return mc
    
    reset: Union[bool, list[int]] = False
    
    # prompt: should reset all calibration data?
    initial_questions = [inquirer.Confirm("reset", message="Reset calibration data?", default=False)]
    initial_answers = inquirer.prompt(initial_questions)

    # ensure valid input
    if initial_answers is None:
        raise ValueError("Invalid input")

    # reset all calibration data
    if initial_answers["reset"]:
        reset = True
    else:
        # prompt: reset individual motors?
        followup_questions = [inquirer.Text("reset", message="Reset individual motors? (Enter comma separated list)", default="")]
        followup_answers = inquirer.prompt(followup_questions)
        
        # ensure valid input
        if followup_answers is None:
            raise ValueError("Invalid input")

        reset = [int(x) for x in followup_answers["reset"].replace(" ", "").split(",")]

        if not all(motor in range(config.get('n_motors')) for motor in reset):
            raise ValueError("Invalid motor channel")

    await mc.calibrate(reset) # calibrate motors

    mc.stop_all_motors()
    log.info("Calibration mode complete")
    return mc