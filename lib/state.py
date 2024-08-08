from enum import Enum
import os
import asyncio
import random
import time
import RPi.GPIO as GPIO
import constants
from lib.controller import MotorController
from lib.sequence import Sequence
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

        # reflect initial switch state
        if not GPIO.input(constants.wall_switch_pins[0]):
            self.switch_state[0] = True
            self._change_state(State.RANDOM)
        elif not GPIO.input(constants.wall_switch_pins[1]):
            self.switch_state[1] = True
            self._change_state(State.SEQUENCE)
        else:
            self._change_state(State.IDLE)

        log.info(f"State machine initialized, initial state is {self.state}", override=True)
        log.info("Calibrating motors, STATES WILL BE IGNORED UNTIL CALIBRATION IS COMPLETE", override=True)

        asyncio.run(self.mc.calibrate(reset=False)) # calibrate motors

    def _change_state(self, new_state: State):
        """Change state from current state to new state"""
        log.info(f"Changing state from {self.state} to {new_state}", override=True)
        self.led.off() # turn off LED for new state
        self._charger_off() # turn off charging for new state
        self.state = new_state

    def _handle_event(self, channel):
        """Handle events from GPIO"""
        new_state = State.IDLE

        # detect button presses
        if channel == constants.service_button_pin:
            new_state = State.SERVICE
        if channel == constants.reboot_button_pin:
            new_state = State.REBOOT

        # detect switch changes
        if channel in constants.wall_switch_pins:
            # detect random and sequence switches
            if channel == constants.wall_switch_pins[0]:
                self.switch_state[0] = not self.switch_state[0]
            if channel == constants.wall_switch_pins[1]:
                self.switch_state[1] = not self.switch_state[1]
            # set new state based on switch state
            if self.switch_state[0] and self.switch_state[1]:
                new_state = State.IDLE
            elif self.switch_state[0]:
                new_state = State.RANDOM
            elif self.switch_state[1]:
                new_state = State.SEQUENCE
            
        self._change_state(new_state)

    def _charger_off(self):
        """Turn off charging"""
        GPIO.output(constants.charging_pin, GPIO.LOW)
    
    def _charger_on(self):
        """Turn on charging"""
        GPIO.output(constants.charging_pin, GPIO.HIGH)
    
    async def check(self):
        """Check current state and run appropriate state"""
        print(f"CURRENT STATE: {self.state}")

        try: 
            if self.state == State.IDLE:
                await self.idle()
            elif self.state == State.SEQUENCE:
                await asyncio.gather(*[self.led.double_blink(0.5), self.sequence()])
            elif self.state == State.RANDOM:
                await asyncio.gather(*[self.led.blink(), self.random()])
            elif self.state == State.SERVICE:
                await asyncio.gather(*[self.led.blink(0.5), self.service()])
            elif self.state == State.REBOOT:
                await self.reboot()
            else:
                raise ValueError("Invalid state")
        except Exception as e:
            log.error(f"An error occurred, exiting state machine")
            raise Exception("State machine error")
        
    async def reboot(self):
        """Reboot state for rebooting the system"""
        log.info("Entering reboot state")
        log.warning("Rebooting system")
        self.mc.stop_all_motors()
        GPIO.cleanup()
        os.system('sudo reboot')

    async def idle(self):
        """Idle state for charging and waiting for sequence to run"""
        log.info("Entering idle state")

        charge_cycle_time = constants.charge_cycle_time if not constants.testing_mode else constants.testing_charge_cycle_time
        charge_interval = constants.charge_interval if not constants.testing_mode else constants.testing_charge_interval

        time_since_last_charge = time.time() # time elapsed since last charge

        self.led.on() # solid on for idle state
        charge_state = ChargeState.REQUIRES_CHARGE

        cycle = 0 # track current charge cycle
        current_cycle_elapsed_time = time.time() # track current cycle elapsed time

        # check if state changed every second
        while True:
             # break back to main loop if state changed
            if self.state != State.IDLE: break

            # state is charged -> increment time since last charge and check if requires charge
            if charge_state == ChargeState.CHARGED:
                log.info("CHARGED", override=True)
                if time.time() - time_since_last_charge >= charge_interval:
                    charge_state = ChargeState.REQUIRES_CHARGE
                    time_since_last_charge = time.time()

            # state requires charge -> change to charging and start charging
            if charge_state == ChargeState.REQUIRES_CHARGE:
                log.info("REQUIRES CHARGE", override=True)
                charge_state = ChargeState.CHARGING
                self._charger_on() # turn on charging power
                current_cycle_elapsed_time = time.time() # reset current cycle elapsed time

                # place candles in correct position start charging
                await self.mc.move_all_home()
                await self.mc.move_all(0.2) 

            # state is charging -> increment charge time and check if charged, if changed -> set to charged
            if charge_state == ChargeState.CHARGING:
                log.info("CHARGING", override=True)

                # current cycle complete or hasn't started -> start new cycle
                if time.time() - current_cycle_elapsed_time >= charge_cycle_time or cycle == 0:
                    cycle += 1

                    n_active_motors = len(self.mc._get_active_motors())

                    # all cycles complete -> all candles charged, change charge state
                    if cycle > n_active_motors // constants.candles_per_charge_cycle:
                        charge_state = ChargeState.CHARGED
                        cycle = 0
                        time_since_last_charge = time.time()
                        self._charger_off() # turn off charging power
                        await asyncio.gather(*[motor.to(0.2) for motor in self.mc.motors if not motor.disabled]) # move all candles to past charger
                        continue
                    
                    # bounds to slice motors to charge
                    lower_bound = (cycle - 1) * constants.candles_per_charge_cycle
                    upper_bound = lower_bound + constants.candles_per_charge_cycle

                    # ensure upper bound is in range
                    if upper_bound > n_active_motors:
                        upper_bound = n_active_motors

                    # select motors to charge
                    motors_to_charge = [m for m in self.mc.motors if not m.disabled][lower_bound:upper_bound]

                    # not first cycle -> move currently charging candles slightly past charger
                    if cycle != 1:
                        prev_lower_bound = lower_bound - constants.candles_per_charge_cycle
                        prev_upper_bound = prev_lower_bound + constants.candles_per_charge_cycle

                        currently_charging_motors = [m for m in self.mc.motors if not m.disabled][prev_lower_bound:prev_upper_bound]

                        # move previously charging candles to past charger (stop charging)
                        await asyncio.gather(*[motor.to(0.2) for motor in currently_charging_motors])

                    # move to next cycle of candles to charge
                    await asyncio.gather(*[motor.to_home() for motor in motors_to_charge])

                    current_cycle_elapsed_time = time.time() # reset current cycle elapsed time for next iteration

            await asyncio.sleep(1) # sleep 1 second before next check

    async def sequence(self):
        """Sequence state for running timed sequences"""
        log.info("Entering sequence state")

        max_run_time = constants.max_sequence_state_time if not constants.testing_mode else constants.testing_max_sequence_state_time
        elapsed_time = time.time() # time elapsed since sequence started 
        seq = Sequence() # sequence generator
        current_generator = seq.alternating() # current generator for sequence

        # check if state changed every second
        while True:
             # break back to main loop if state changed
            if self.state != State.SEQUENCE: break

            # elapsed time is greater than max run time -> change to idle
            if time.time() - elapsed_time >= max_run_time:
                self._change_state(State.IDLE)

            # run next iteration or get new generator
            try:
                positions, speeds = next(current_generator) # get next positions and speeds
                await self.mc.move_all(positions, speeds)
            except StopIteration:
                iterations = random.randint(30, 120) # randomize number of iterations
                current_generator = random.choice([seq.wave, seq.alternating])(iterations, self.mc.n_active_motors) # choose random sequence
    
    async def random(self):
        """Random state for running random sequences"""
        log.info("Entering random state", override=True)

        max_run_time = constants.max_random_state_time if not constants.testing_mode else constants.testing_max_random_state_time
        elapsed_time = time.time() # time elapsed since sequence started 
        seq = Sequence() # sequence generator

        # check if state changed every second
        while True:
            log.info(f"Entering random state loop", override=True)
            # break back to main loop if state changed
            if self.state != State.RANDOM: break

            # elapsed time is greater than max run time -> change to idle
            if time.time() - elapsed_time >= max_run_time:
                self._change_state(State.IDLE)

            # run next iteration
            positions, speeds = seq.random_iteration()

            log.info(f"Moving to positions: {positions} with speeds: {speeds}", override=True)
            await self.mc.move_all(positions, speeds)

    async def service(self):
        """Service state for servicing the chandelier"""
        log.info("Entering service state")

        # run service sequence
        self._charger_off() # turn off charging
        await self.mc.move_all(1.0) # move all candles all the way down

        # check if state changed every second
        while True:
            # break back to main loop if state changed
            if self.state != State.SERVICE: break
            await asyncio.sleep(1) # sleep 1 second before next check