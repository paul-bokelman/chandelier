from enum import Enum
import asyncio
import random
import RPi.GPIO as GPIO
import constants
from lib.mc.controller import MotorController
from lib.sequence import Sequence

class State(Enum):
    IDLE = 0
    RUNNING = 1
    SERVICE = 2

class ChargeState(Enum):
    REQUIRES_CHARGE = 0,
    CHARGING = 1,
    CHARGED = 2

def change_state(current_state: State, new_state: State):
    current_state = new_state

async def idle(state: State):
    """Idle state for charging and waiting for sequence to run"""
    charge_state = ChargeState.CHARGED

    # change state to running if power switch is pressed
    # GPIO.add_event_detect(constants.power_switch_pins[0], GPIO.FALLING, callback=change_state(state, State.RUNNING), bouncetime=300)

    # change state to service if service button is pressed
    # GPIO.add_event_detect(constants.service_button_pin, GPIO.FALLING, callback=change_state(state, State.SERVICE), bouncetime=300)

    time_since_last_charge = 0 # time elapsed since last charge
    elapsed_charge_time = 0 # time spend charging

    # check if state changed every second
    while True:
        if state == State.RUNNING: break # break back to main loop if state changed
        if state == State.SERVICE: break # break back to main loop if state changed

        # state is charged -> increment time since last charge and check if requires charge
        if charge_state == ChargeState.CHARGED:
            time_since_last_charge += 1
            if time_since_last_charge >= constants.charge_interval:
                charge_state = ChargeState.REQUIRES_CHARGE
                time_since_last_charge = 0

        # state requires charge -> change to charging and start charging
        if charge_state == ChargeState.REQUIRES_CHARGE:
            charge_state = ChargeState.CHARGING
            # await charge() # start charging

        # state is charging -> increment charge time and check if charged, if changed -> set to charged
        if charge_state == ChargeState.CHARGING:
            elapsed_charge_time += 1
            if elapsed_charge_time >= constants.max_charge_time:
                charge_state = ChargeState.CHARGED
                elapsed_charge_time = 0
        
        await asyncio.sleep(1) # sleep for 1 second and return back to loop

    return 

async def run(state: State):
    """Run state for running and maintaining sequences"""
    run_time_elapsed = 0 # time elapsed since sequence started 
    mc = MotorController() # motor controller
    sequence = Sequence() # sequence generator
    current_generator = sequence.random() # current generator for sequence

    # change state to service if service button is pressed
    # GPIO.add_event_detect(constants.service_button_pin, GPIO.FALLING, callback=change_state(state, State.SERVICE), bouncetime=300)

    # check if state changed every second
    while True:
        if state == State.IDLE: break # break back to main loop if state changed to idle
        if state == State.SERVICE: break # break back to main loop if state changed to service

        # elapsed time is greater than max run time -> change to idle
        if run_time_elapsed >= constants.max_run_time:
            change_state(state, State.IDLE)

        # run next iteration or get new generator
        try:
            positions, speeds = next(current_generator) # get next positions and speeds
            max_elapsed_time = await mc.move_all(positions, speeds)
            run_time_elapsed += max_elapsed_time # increment time elapsed to account for time taken to move
        except StopIteration:
            #/ should be abstracted to the sequence generator
            iterations = random.randint(30, 120) # randomize number of iterations
            current_generator = random.choice([sequence.random, sequence.wave, sequence.alternating])(iterations) # choose random sequence

        run_time_elapsed += 1 # increment time elapsed
        await asyncio.sleep(1) # sleep for 1 second and return back to loop

    return 

async def service(state: State):
    """Service state for servicing the chandelier"""
    # change state to idle if service button is pressed
    # GPIO.add_event_detect(constants.service_button_pin, GPIO.FALLING, callback=change_state(state, State.IDLE), bouncetime=300)
    mc = MotorController()

    # run service sequence
    await mc.move_all(1) # move all candles all the way down
    # flash_led() # flash led to indicate service mode
    # charging_power_off() # turn off charging power

    # check if state changed every second
    while True:
        if state == State.IDLE: break # break back to main loop if state changed to idle
        await asyncio.sleep(1) # sleep for 1 second and return back to loop

    return


async def main():
    state = State.IDLE
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(constants.encoder_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # GPIO.setup(constants.power_switch_pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # GPIO.setup(constants.service_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # while True:
    #     if state == State.IDLE:
    #         await idle(state)
    #     elif state == State.RUNNING:
    #         await run(state)
    #     elif state == State.SERVICE:
    #         await service(state)
    #     else:
    #         raise ValueError("Invalid state")

    mc = MotorController()

    await mc.calibrate(reset=True)

    # await mc.move_all_home()
    # await mc.move_all(0.7)

    # mc.stop_all_motors()

    GPIO.cleanup() # clean up for next session

if __name__ == "__main__":
    asyncio.run(main())