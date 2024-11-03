from enum import Enum
import os
import time
import random
from datetime import datetime
import asyncio
import threading
import sshkeyboard as keyboard
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
    __order__ = 'IDLE RANDOM SEQUENCE SERVICE REBOOT'
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
        self.button_timer = 0

        # add event detection for all relevant GPIO pins
        GPIO.add_event_detect(config.get('service_button_pin'), GPIO.BOTH, self._handle_button_event, 300)
        GPIO.add_event_detect(config.get('reboot_button_pin'), GPIO.BOTH, self._handle_button_event, 300)
        GPIO.add_event_detect(config.get('wall_switch_pins')[0], GPIO.BOTH, self._handle_switch_event, 300)
        GPIO.add_event_detect(config.get('wall_switch_pins')[1], GPIO.BOTH, self._handle_switch_event, 300)

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

            # Start keyboard listening in a separate thread
            self.keyboard_thread = threading.Thread(target=self._start_keyboard_listener, daemon=True)
            self.keyboard_thread.start()

        log.info(f"State machine initialized, initial state is {self.state}", override=True)

        if not self.mc.store._validate():
            log.error("Calibration data is invalid, please recalibrate")
            raise Exception("Invalid calibration data")

        self.mc.load_calibration_data() # load calibration data

    def _start_keyboard_listener(self):
        """Start keyboard listener in a separate thread"""
        keyboard.listen_keyboard(
            on_press=self._handle_keypress,
        )

    def _handle_keypress(self, key: str):
        """Handle keypresses"""
        state = self.state

        if key == 'i':
            state = State.IDLE
        elif key == 's':
            state = State.SEQUENCE
        elif key == 'r':
            state = State.RANDOM
        elif key == 'c':
            state = State.SERVICE
        elif key == 'q':
            state = State.REBOOT

        # Use asyncio.run_coroutine_threadsafe to call _change_state from this thread
        self._change_state(state)

    def _change_state(self, new_state: State):
        """Change state from current state to new state"""
        log.info(f"Changing state from {self.state} to {new_state}", override=True)
        self.led.off() # turn off LED for new state
        self._charger_off() # turn off charging for new state
        self.state = new_state

    def _handle_switch_event(self, channel):
        """Handle switch events from GPIO"""
        new_state = State.IDLE

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

    def _handle_button_event(self, channel):
        """Handle button events from GPIO"""
        new_state = State.IDLE

        # button is pressed -> start timer
        if GPIO.input(channel) == GPIO.HIGH:
            self.button_timer = time.time()
            return
        
        # button is released & time since press is greater than 5 seconds -> change state
        if GPIO.input(channel) == GPIO.LOW and time.time() - self.button_timer > config.get("button_wait_time"):
            if channel == config.get('service_button_pin'):
                new_state = State.SERVICE
            elif channel == config.get('reboot_button_pin'):
                new_state = State.REBOOT
        else: # otherwise, ignore button press
            self.button_timer = 0
            return

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

        # pull code from git
        os.system(f"bash  {os.getcwd()}/scripts/update-repo.sh")

        # reboot system
        os.system('sudo reboot')

    async def idle(self):
        """Idle state for charging and waiting for sequence to run"""
        log.info("Entering idle state")

        charge_cycle_time = config.get('charge_cycle_time')
        available_charging_hours = config.get('available_charging_hours')

        self.led.on() # solid on for idle state
        charge_state = ChargeState.CHARGED # persistent charge state

        recovery_attempted = False # track if recovery has been attempted
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
                    await self.mc.move_all_home() # go home and recalibrate all home positions
                    returned_after_charging = True

                if not recovery_attempted:
                    # attempt to recover motors from disabled state 
                    await self.mc.recover_all()
                    recovery_attempted = True

                if datetime.now().time().hour in available_charging_hours:
                    charge_state = ChargeState.REQUIRES_CHARGE

            # state requires charge -> change to charging and start charging
            if charge_state == ChargeState.REQUIRES_CHARGE:
                log.info("REQUIRES CHARGE", override=True)
                charge_state = ChargeState.CHARGING
                current_cycle_elapsed_time = time.time() # reset current cycle elapsed time

                #/ should move all motors by counts instead of scaled position, this ensures no extreme positions when max counts is high
                await self.mc.move_all(config.get('charging_buffer_distance')) # move all candles slightly past charger (buffer)

                returned_after_charging = False # reset returned after charging

                self._charger_on() # turn on charging power

            # state is charging -> increment charge time and check if charged, if changed -> set to charged
            if charge_state == ChargeState.CHARGING:
                log.info("CHARGING", override=True)

                # current cycle complete or hasn't started -> start new cycle
                if time.time() - current_cycle_elapsed_time >= charge_cycle_time or completed_cycles == 0:
                    enabled_motors = self.mc.get_enabled_motors()
                    n_enabled_motors = len(enabled_motors)

                    # all cycles complete -> all candles charged, change charge state
                    if completed_cycles * config.get('candles_per_charge_cycle') >= n_enabled_motors:
                        charge_state = ChargeState.CHARGED
                        completed_cycles = 0
                        self._charger_off() # turn off charging power
                        await asyncio.gather(*[motor.to(config.get('charging_buffer_distance')) for motor in enabled_motors]) # move all candles past charger
                        continue
                    
                    # bounds to slice motors to charge
                    lower_bound = completed_cycles * config.get('candles_per_charge_cycle')
                    upper_bound = lower_bound + config.get('candles_per_charge_cycle')

                    # ensure upper bound is in range
                    if upper_bound > n_enabled_motors:
                        upper_bound = n_enabled_motors

                    # select motors to charge
                    motors_to_charge = enabled_motors[lower_bound:upper_bound]

                    # not first cycle -> move currently charging candles slightly past charger
                    if completed_cycles != 0:
                        prev_lower_bound = lower_bound - config.get('candles_per_charge_cycle')
                        prev_upper_bound = prev_lower_bound + config.get('candles_per_charge_cycle')

                        currently_charging_motors = enabled_motors[prev_lower_bound:prev_upper_bound]

                        # move previously charging candles to past charger (stop charging)
                        await asyncio.gather(*[motor.to(config.get('charging_buffer_distance')) for motor in currently_charging_motors])

                    # move to next cycle of candles to charge
                    await asyncio.gather(*[motor.to_home() for motor in motors_to_charge])

                    current_cycle_elapsed_time = time.time() # reset current cycle elapsed time for next iteration

                    completed_cycles += 1 # increment completed cycles

            await asyncio.sleep(1) # sleep 1 second before next check

    async def sequence(self):
        """Sequence state for running timed sequences"""
        log.info("Entering sequence state")

        elapsed_time = time.time() # time elapsed since sequence started 
        elapsed_time_since_calibration = time.time() # time elapsed since last calibration
        seq = Sequence() # sequence generator
        current_generator = seq.alternating() # current generator for sequence

        # check if state changed every second
        while True:
             # break back to main loop if state changed
            if self.state != State.SEQUENCE: break

            # elapsed time is greater than max run time -> change to idle
            if time.time() - elapsed_time >= config.get('sequence_state_duration'):
                self._change_state(State.IDLE)

            # recalibrate home positions if time since last calibration is greater than duration before recalibration
            if time.time() - elapsed_time_since_calibration  >= config.get('duration_before_recalibration'):
                await self.mc.find_home_positions()
                elapsed_time_since_calibration = time.time() # reset elapsed time since calibration

            # run next iteration or get new generator
            try:
                positions, throttles = next(current_generator) # get next positions and speeds
                await self.mc.move_all(positions, throttles)
            except StopIteration:
                iterations = random.randint(30, 120) # randomize number of iterations
                current_generator = random.choice([seq.wave, seq.alternating])(iterations) # choose random sequence
    
    async def random(self):
        """Random state for running random sequences"""
        log.info("Entering random state", override=True)

        elapsed_time = time.time() # time elapsed since sequence started 
        elapsed_time_since_calibration = time.time() # time elapsed since last calibration
        seq = Sequence() # sequence generator

        # check if state changed every second
        while True:
            # break back to main loop if state changed
            if self.state != State.RANDOM: break

            # elapsed time is greater than max run time -> change to idle
            if time.time() - elapsed_time >= config.get('random_state_duration'):
                self._change_state(State.IDLE)

            # recalibrate home positions if time since last calibration is greater than duration before recalibration
            if time.time() - elapsed_time_since_calibration >= config.get('duration_before_recalibration'):
                await self.mc.find_home_positions()
                elapsed_time_since_calibration = time.time() # reset elapsed time since calibration

            # run next iteration
            positions, throttles = seq.random_iteration()
            await self.mc.move_all(positions, throttles)

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
