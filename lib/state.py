from enum import Enum
import random
import asyncio
import time
import RPi.GPIO as GPIO
import constants
from lib.mc.controller import MotorController
from lib.sequence import Sequence
from lib.utils import to_seconds
from lib.utils import log
from lib.led import LED

class State(Enum):
    IDLE = 0
    RANDOM = 1
    SEQUENCE = 2
    SERVICE = 3
    REBOOT = 4

class ChargeState(Enum):
    REQUIRES_CHARGE = 0,
    CHARGING = 1,
    CHARGED = 2

def test(channel):
    print("test", channel)
    GPIO.output(constants.led_pin, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(constants.led_pin, GPIO.LOW)

class StateMachine:
    """State machine for managing states"""
    def __init__(self) -> None:
        self.state = State.IDLE
        self.led = LED(constants.led_pin)
        self.mc = MotorController()
        self.switch_state = [False, False]

        # change state based on buttons and switches
        GPIO.add_event_detect(constants.service_button_pin, GPIO.FALLING, callback=self._handle_event, bouncetime=300)
        GPIO.add_event_detect(constants.reboot_button_pin, GPIO.FALLING, callback=self._handle_event, bouncetime=300)
        GPIO.add_event_detect(constants.wall_switch_pins[0], GPIO.BOTH, callback=self._handle_event, bouncetime=300)
        GPIO.add_event_detect(constants.wall_switch_pins[1], GPIO.BOTH, callback=self._handle_event, bouncetime=300)

        # detect initial state
        if not GPIO.input(constants.wall_switch_pins[0]):
            self._change_state(State.RANDOM)
        elif not GPIO.input(constants.wall_switch_pins[1]):
            self._change_state(State.SEQUENCE)
        else:
            self._change_state(State.IDLE)

        log.info(f"State machine initialized, initial state is {self.state}", override=True)


    def _change_state(self, new_state: State):
        """Change state from current state to new state"""
        log.info(f"Changing state from {self.state} to {new_state}", override=True)
        self.led.reset()
        self.state = new_state

    def _handle_event(self, channel):
        """Handle events from GPIO"""
        new_state = State.IDLE

        # detect button presses
        if channel == constants.service_button_pin:
            new_state = State.SERVICE
        if channel == constants.reboot_button_pin:
            new_state = State.REBOOT

        # detect random and sequence switches
        if channel == constants.wall_switch_pins[0]:
            if self.state == State.RANDOM: # already random -> change to idle
                new_state = State.IDLE
            else:
                new_state = State.RANDOM
        if channel == constants.wall_switch_pins[1]:
            if self.state == State.SEQUENCE: # already sequence -> change to idle
                new_state = State.IDLE
            else:
                new_state = State.SEQUENCE

        self._change_state(new_state)
    
    async def check(self):
        """Check current state and run appropriate state"""
        print("Checking state")
        print(self.state)
        await asyncio.sleep(1)

        # if self.state == State.IDLE:
        #     await self.idle()
        # elif self.state == State.SEQUENCE:
        #     await self.sequence()
        # elif self.state == State.RANDOM:
        #     await self.random()
        # elif self.state == State.SERVICE:
        #     await self.service()
        # elif self.state == State.REBOOT:
        #     await self.reboot()
        # else:
        #     raise ValueError("Invalid state")
        
    async def reboot(self):
        """Reboot state for rebooting the system"""
        log.info("Entering reboot state")
        log.warning("Rebooting system")

    async def idle(self):
        """Idle state for charging and waiting for sequence to run"""
        log.info("Entering idle state")
        self.led.on()
        charge_state = ChargeState.CHARGED

        time_since_last_charge = 0 # time elapsed since last charge
        elapsed_charge_time = 0 # time spend charging

        # check if state changed every second
        while True:
            if self.state != State.IDLE: break # break back to main loop if state changed

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

    async def sequence(self):
        """Sequence state for running timed sequences"""
        log.info("Entering sequence state")

        run_time_elapsed = 0 # time elapsed since sequence started 
        sequence = Sequence() # sequence generator
        current_generator = sequence.random() # current generator for sequence

        # check if state changed every second
        while True:
            if self.state != State.SEQUENCE: break # break back to main loop if state changed

            # elapsed time is greater than max run time -> change to idle
            if run_time_elapsed >= constants.max_run_time:
                self._change_state(State.IDLE)

            # run next iteration or get new generator
            try:
                positions, speeds = next(current_generator) # get next positions and speeds
                max_elapsed_time = await self.mc.move_all(positions, speeds)
                run_time_elapsed += to_seconds(max_elapsed_time) # increment time elapsed to account for time taken to move
            except StopIteration:
                #/ should be abstracted to the sequence generator
                iterations = random.randint(30, 120) # randomize number of iterations
                current_generator = random.choice([sequence.wave, sequence.alternating])(iterations) # choose random sequence

            run_time_elapsed += 1 # increment time elapsed
            await asyncio.sleep(1) # sleep for 1 second and return back to loop
    
    async def random(self):
        """Random state for running random sequences"""
        log.info("Entering random state")

        run_time_elapsed = 0 # time elapsed since sequence started 
        sequence = Sequence() # sequence generator
        current_generator = sequence.random() # current generator for sequence

        # check if state changed every second
        while True:
            if self.state != State.RANDOM: break # break back to main loop if state changed

            # elapsed time is greater than max run time -> change to idle
            if run_time_elapsed >= constants.max_run_time:
                self._change_state(State.IDLE)

            # run next iteration or get new generator
            try:
                positions, speeds = next(current_generator) # get next positions and speeds
                max_elapsed_time = await self.mc.move_all(positions, speeds)
                run_time_elapsed += to_seconds(max_elapsed_time) # increment time elapsed to account for time taken to move
            except StopIteration:
                #/ should be abstracted to the sequence generator
                iterations = random.randint(30, 120) # randomize number of iterations
                current_generator = random.choice([sequence.wave, sequence.alternating])(iterations) # choose random sequence

            run_time_elapsed += 1 # increment time elapsed
            await asyncio.sleep(1) # sleep for 1 second and return back to loop

    async def service(self):
        """Service state for servicing the chandelier"""
        log.info("Entering service state")

        # run service sequence
        await self.mc.move_all(1) # move all candles all the way down
        await self.led.blink(5)

        await asyncio.sleep(5)
        self.led.stop_blink()
        # charging_power_off() # turn off charging power

        # check if state changed every second
        while True:
            if self.state == State.IDLE: break # break back to main loop if state changed to idle
            await asyncio.sleep(1) # sleep for 1 second and return back to loop