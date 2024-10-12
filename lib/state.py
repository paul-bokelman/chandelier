from enum import Enum
import os
import time
import random
from datetime import datetime
import asyncio
from pynput import keyboard
from configuration.config import config
from lib.controller import MotorController
from lib.sequence import Sequence
from lib.led import LED
from lib.utils import log

try:
    import RPi.GPIO as GPIO # type: ignore
except ImportError:
    import Mock.GPIO as GPIO

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
    def __init__(self, controller: MotorController, manual = False) -> None:
        self.state = State.IDLE
        self.led = LED(config.get("led_pin"))
        self.mc = controller
        self.switch_state = [False, False]
        self.manual = manual

        # add event detection for all relevant GPIO pins
        GPIO.add_event_detect(config.get('service_button_pin'), GPIO.FALLING, self._handle_event, 300)
        GPIO.add_event_detect(config.get('reboot_button_pin'), GPIO.FALLING, self._handle_event, 300)
        GPIO.add_event_detect(config.get('wall_switch_pins')[0], GPIO.BOTH, self._handle_event, 300)
        GPIO.add_event_detect(config.get('wall_switch_pins')[1], GPIO.BOTH, self._handle_event, 300)

        # reflect initial switch state
        if not GPIO.input(config.get('wall_switch_pins')[0]):
            self.switch_state[0] = True
            self._change_state(State.RANDOM)
        elif not GPIO.input(config.get('wall_switch_pins')[1]):
            self.switch_state[1] = True
            self._change_state(State.SEQUENCE)
        else:
            self._change_state(State.IDLE)

        # set up manual mode if enabled
        if self.manual:
            # remove event detection if in manual mode
            GPIO.remove_event_detect(config.get('service_button_pin'))
            GPIO.remove_event_detect(config.get('reboot_button_pin'))
            for pin in config.get('wall_switch_pins'):
                GPIO.remove_event_detect(pin)

            # check for manual state changes (manual mode)
            def on_press(key):
                try:
                    if key.char == 'i':
                        log.info("Pressed i, changing to idle")
                        self._change_state(State.IDLE)
                    elif key.char == 'r':
                        log.info("Pressed r, changing to random")
                        self._change_state(State.RANDOM)
                    elif key.char == 's':
                        log.info("Pressed s, changing to sequence")
                        self._change_state(State.SEQUENCE)
                    elif key.char == 'c':
                        log.info("Pressed c, changing to service")
                        self._change_state(State.SERVICE)
                    elif key.char == 'q':
                        log.info("Pressed q, changing to reboot")
                        self._change_state(State.REBOOT)
                except AttributeError:
                    pass  # Ignore special keys

            with keyboard.Listener(on_press=on_press) as listener:
                listener.join()

        log.info(f"State machine initialized, initial state is {self.state}", override=True)

        if not self.mc.store._validate():
            log.error("Calibration data is invalid, please recalibrate")
            raise Exception("Invalid calibration data")

        self.mc.load_calibration_data() # load calibration data

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
        if channel == config.get('service_button_pin'):
            new_state = State.SERVICE
        if channel == config.get('reboot_button_pin'):
            new_state = State.REBOOT

        # detect switch changes
        if channel in config.get('wall_switch_pins'):
            # detect random and sequence switches
            if channel == config.get('wall_switch_pins')[0]:
                self.switch_state[0] = not self.switch_state[0]
            if channel == config.get('wall_switch_pins')[1]:
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
        GPIO.output(config.get('charging_pin'), GPIO.LOW)
    
    def _charger_on(self):
        """Turn on charging"""
        GPIO.output(config.get('charging_pin'), GPIO.HIGH)
    
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
            self.mc.stop_all_motors() # emergency stop
            log.error(f"An error occurred, exiting state machine. Error: {e}")
            raise Exception("State machine error")
        
    async def reboot(self):
        """Reboot state for rebooting the system"""
        log.info("Entering reboot state")
        log.warning("Rebooting system")
        self.mc.stop_all_motors()
        GPIO.cleanup()
        os.system('sudo bash ./reboot.sh') # todo: should utilize python subprocess module

    async def idle(self):
        """Idle state for charging and waiting for sequence to run"""
        log.info("Entering idle state")

        charge_cycle_time = config.get('charge_cycle_time')
        available_charging_hours = config.get('available_charging_hours')

        self.led.on() # solid on for idle state
        charge_state = ChargeState.CHARGED # persistent charge state

        completed_cycles = 0 # track current charge cycle
        current_cycle_elapsed_time = time.time() # track current cycle elapsed time

        returned_after_charging = False # track if candles have returned home this iteration

        # check if state changed every second
        while True:
             # break back to main loop if state changed
            if self.state != State.IDLE: break

            # state is charged -> increment time since last charge and check if requires charge
            if charge_state == ChargeState.CHARGED:
                log.info("CHARGED", override=True)
                
                if not returned_after_charging:
                    await self.mc.move_all_home() # move all candles to home position
                    returned_after_charging = True

                if datetime.now().time().hour in available_charging_hours:
                    charge_state = ChargeState.REQUIRES_CHARGE

            # state requires charge -> change to charging and start charging
            if charge_state == ChargeState.REQUIRES_CHARGE:
                log.info("REQUIRES CHARGE", override=True)
                charge_state = ChargeState.CHARGING
                current_cycle_elapsed_time = time.time() # reset current cycle elapsed time

                # place candles in correct position start charging
                await self.mc.move_all_home()
                await self.mc.move_all(6 / config.get('max_counts'))

                returned_after_charging = False # reset returned after charging

                self._charger_on() # turn on charging power

            # todo: requires attentions -> check if everything works logically
            # state is charging -> increment charge time and check if charged, if changed -> set to charged
            if charge_state == ChargeState.CHARGING:
                log.info("CHARGING", override=True)

                # current cycle complete or hasn't started -> start new cycle
                if time.time() - current_cycle_elapsed_time >= charge_cycle_time or completed_cycles == 0:
                    n_motors = len(self.mc.motors)

                    # all cycles complete -> all candles charged, change charge state
                    if completed_cycles * config.get('candles_per_charge_cycle') >= n_motors:
                        charge_state = ChargeState.CHARGED
                        completed_cycles = 0
                        self._charger_off() # turn off charging power
                        await asyncio.gather(*[motor.to(6 / config.get('max_counts')) for motor in self.mc.motors]) # move all candles to past charger
                        continue
                    
                    # bounds to slice motors to charge
                    lower_bound = completed_cycles * config.get('candles_per_charge_cycle')
                    upper_bound = lower_bound + config.get('candles_per_charge_cycle')

                    # ensure upper bound is in range
                    if upper_bound > n_motors:
                        upper_bound = n_motors

                    # select motors to charge
                    motors_to_charge = self.mc.motors[lower_bound:upper_bound]

                    # not first cycle -> move currently charging candles slightly past charger
                    if completed_cycles != 0:
                        prev_lower_bound = lower_bound - config.get('candles_per_charge_cycle')
                        prev_upper_bound = prev_lower_bound + config.get('candles_per_charge_cycle')

                        currently_charging_motors = self.mc.motors[prev_lower_bound:prev_upper_bound]

                        # move previously charging candles to past charger (stop charging)
                        await asyncio.gather(*[motor.to(0.2) for motor in currently_charging_motors])

                    # move to next cycle of candles to charge
                    await asyncio.gather(*[motor.to_home() for motor in motors_to_charge])

                    current_cycle_elapsed_time = time.time() # reset current cycle elapsed time for next iteration

                    completed_cycles += 1 # increment completed cycles

            await asyncio.sleep(1) # sleep 1 second before next check

    async def sequence(self):
        """Sequence state for running timed sequences"""
        log.info("Entering sequence state")

        max_run_time = config.get('sequence_state_duration')
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
                current_generator = random.choice([seq.wave, seq.alternating])(iterations) # choose random sequence
    
    async def random(self):
        """Random state for running random sequences"""
        log.info("Entering random state", override=True)

        max_run_time = config.get('random_state_duration')
        elapsed_time = time.time() # time elapsed since sequence started 
        seq = Sequence() # sequence generator

        # check if state changed every second
        while True:
            log.info(f"Entering random state loop", override=True)
            # break back to main loop if state changed
            if self.state != State.RANDOM: break
            else:
                # elapsed time is greater than max run time -> change to idle
                if time.time() - elapsed_time >= max_run_time:
                    self._change_state(State.IDLE)

            # run next iteration
            positions, speeds = seq.random_iteration()

            log.info(f"Moving to positions: {positions} with speeds: {speeds}")
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
