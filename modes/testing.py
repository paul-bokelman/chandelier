from typing import Callable
from lib.controller import MotorController
import asyncio
import inquirer as iq
from lib.utils import log
from configuration import config

def apply_selective_controller(mc: MotorController) -> list[int]:
    """
    Choose motors and mutate MotorController instance to only use selected motors

    Parameters:
        mc (MotorController): Motor controller instance

    Returns:
        list[int]: List of disabled motor channels (for reversion)
    """
    questions = [iq.Text("motor_numbers", message="Enter test motors (comma separated, blank for all)", default="")]
    answers = iq.prompt(questions)
    
    # ensure valid input
    if answers is None:
        log.warning("No answer provided, using all motors")
        return []
    
    channels: list[int] = []
    
    # none selected, use all motors
    if answers["motor_numbers"] == "":
        channels = list(range(config.get('n_motors')))
    else: # parse selected motors
        channels = [int(x) for x in answers["motor_numbers"].replace(" ", "").split(",")]

    disabled_channels: list[int] = []

    # disable unselected motors for test
    for motor in mc.get_enabled_motors():
        if motor.channel not in channels:
            disabled_channels.append(motor.channel)
            motor.disable() 

    return disabled_channels

def revert_selective_controller(mc: MotorController, disabled_channels: list[int]):
    """
    Revert MotorController instance to be original state

    Parameters:
        mc (MotorController): Motor
    """
    # re-enable motors for normal operation
    for disabled_channel in disabled_channels:
        mc.motors[disabled_channel].enable()

async def c1(mc: MotorController):
    """
    Test case 1
        1. Move down configured distance
        2. Move up configured distance
        3. Repeat configured number of times
        4. Pause for configured duration
        5. Move to home position

    Parameters:
        mc (MotorController): Motor controller instance
    """
    log.info("Running test case 1")

    # apply selective controller (choose motors)
    disabled_channels = apply_selective_controller(mc)

    # run procedure
    down_counts, up_counts, n_times, pause_duration = config.get('c1')

    for _ in range(n_times):
        await mc.move_all_counts(down_counts, directions=config.get('down'))
        await mc.move_all_counts(up_counts, directions=config.get('up'))
        await asyncio.sleep(pause_duration)

    await mc.move_all_home() # move to home position

    # revert selective controller
    revert_selective_controller(mc, disabled_channels)

    log.success("Test case 1 complete")

def run(controller: MotorController):
    log.info("Running testing mode")

    test_cases: dict[str, Callable[[MotorController]]] = {
        "down➜up➜pause➜home": c1,
    }

    question = iq.List("test_case", message="Select a test case", choices=[key for key in test_cases.keys()] + ["exit"])

    while True:
        answer = iq.prompt(question)

        if answer is None:
            log.warning("No test case selected")
            continue
        
        if answer['test_case'] == "exit":
            break
        
        test_case = test_cases.get(answer['test_case'])

        if test_case is None:
            log.warning("Invalid test case")
            continue
        
        asyncio.run(test_case(controller))