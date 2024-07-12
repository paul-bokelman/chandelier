import asyncio
import constants
from lib.store import Store, CalibrationData
from lib.mc.motor import Motor

class MotorController:
  """Main motor controller class for calibrating, saving, and manipulating servos"""
  def __init__(self, debug = False) -> None:
    self.debug = debug
    self.store = Store()
    self.motors = [Motor(i) for i in range(constants.n_motors)]

  def stop_all_motors(self):
    """Stop all motors by calling stop_motor for each motor"""
    for motor in self.motors:
      motor.stop()

  def move_all_home(self):
    """Move all motors to home position"""
    for motor in self.motors:
      motor.to_home()

  def calibrate(self, reset = False):
    """Find minimum speed and time between encoder counts for each motor"""
    if reset: self.store.reset()
    data = self.store.load()
    
    for motor in self.motors:
      motor.calibrate([data['counts'][motor.pin], data["cps_down"][motor.pin], data["cps_up"][motor.pin]])

  async def move_all(self, positions: list[float], speed: float = 0.05):
    """Move all motors to specific positions. Positions is a list of floats from 0 to 1 representing the position of each motor (0 is home, 1 is max)"""

    if speed < 0 or speed > 1:
      raise ValueError("Speed must be between 0 and 1")

    await asyncio.gather(*[motor.to(position, speed) for motor, position in zip(self.motors, positions)])
    # for i, motor in enumerate(self.motors):
    #   motor.to(positions[i], speed)

  def save_calibration(self):
    """Save calibration data with store"""
    data = CalibrationData(
      counts=[motor.counts for motor in self.motors],
      cps_down=[motor.cps_down for motor in self.motors],
      cps_up=[motor.cps_up for motor in self.motors]
    )
    self.store.save(data)
    
  # def motor_calibration_sequence(self, servo_num, speed, enc_counts, time_out):
  #   """Motor move sequence that reports back time between counts"""
  #   el_time_list = []
  #   encoder_num = constants.encoder_pins[servo_num]
  #   if speed > 0:
  #     enc_inc = 1
  #   else:
  #     enc_inc = -1
  #   encoder_counter = 0
    
  #   if GPIO.input(encoder_num) == False: #Encoder already triggered
  #     encoder_counter += 1 #encoder_counter + enc_inc
  #     self.enc_counts[servo_num] = self.enc_counts[servo_num] + enc_inc
  #     print("Encoder_counter:",encoder_counter,"  self.enc_counts:",self.enc_counts[servo_num])
  #   pwm.setServoPulse(servo_num,self._as_pulse(speed))
  #   start = time.time()
  #   time_since_last = start
  #   print("Setting Servo Motor",servo_num,"to",speed,"for",enc_counts,"counts")
  #   while True: 
  #     if GPIO.input(encoder_num) == False: #Encoder triggered
  #       encoder_counter += 1 #encoder_counter + enc_inc
  #       self.enc_counts[servo_num] = self.enc_counts[servo_num] + enc_inc
  #       #print("Encoder_counter 2:",encoder_counter,"  self.enc_counts:",self.enc_counts[servo_num])
  #       if encoder_counter in range(0,enc_counts):
  #         elapsed_time = time.time() - time_since_last
  #         time_since_last = time.time()
  #         if abs(encoder_counter) != 1:
  #           el_time_list.append(elapsed_time)
  #         #print("Encoder count 3",encoder_counter,"  self.enc_counts:",self.enc_counts[servo_num],"elapsed time:",round(elapsed_time,3))
  #       if abs(encoder_counter) == abs(enc_counts):
  #         break
  #       while GPIO.input(encoder_num) == False: #Encoder still triggered
  #         if time.time()-time_since_last > time_out:
  #           #print("Motor",servo_num,"timed out at encoder count",encoder_counter,"without reaching count",enc_counts)
  #           return(time_out)
  #           break
  #         continue
  #       if time.time()-time_since_last > time_out:
  #         #print("Motor",servo_num,"timed out at encoder count",encoder_counter,"without reaching count",enc_counts)
  #         return(time_out)
  #         break
  #     continue
  #   #print("Max time between encoder counts for speed",speed,"is: ",round(max(el_time_list),3),"seconds")
  #   pwm.setServoPulse(servo_num,1500)
  #   if len(el_time_list)>0:
  #     return(round(max(el_time_list),3))
  #   else:
  #     return(0)

  # def encoder_speed_calibration(self):
  #   """
  #   builds lists of motor move constants such as minimum speed up and down and max encoder timing by speed up and down 
  #   """
  #   tar_pos_list = [0] * 16
  #   speed_list = [0] * 16
  #   initial_offset = 10
  #   time_out = 8
  #   enc_counts_min_speed = 4
  #   enc_counts_time_cal = 8    
  #   speed_inc = -1
  #   min_speed_margin = 1
  #   time_between_counts_margin = 1.05

  #   for i in range(0,constants.n_motors):
  #     tar_pos_list[i] = initial_offset
  #     speed_list[i] = constants.Max_Speed
  #   #if self.debug: print("Tar_Pos:",tar_pos_list,"Speed:",speed_list)
  #   self.move_motors_counts(tar_pos_list,"",speed_list)

  #   for servo_num in range(0,constants.n_motors):
  #     #find minimum speed
  #     speed = constants.Max_Speed
  #     found_min_down = False
  #     found_min_up = False
      
  #     while True:
  #       if self.debug: print("Trying speed",speed,"for Servo number:",servo_num)
  #       if found_min_down == False:
  #         if self.motor_calibration_sequence(servo_num, speed, enc_counts_min_speed, time_out) != time_out:
  #           self.slow_speed_down[servo_num] = speed
  #         else:
  #           found_min_down = True
  #       if found_min_up == False:
  #         if self.motor_calibration_sequence(servo_num, -speed, enc_counts_min_speed, time_out) != time_out:
  #           self.slow_speed_up[servo_num] = -speed
  #         else:
  #           found_min_up = True
  #       speed += speed_inc
  #       if found_min_down and found_min_up:
  #         self.slow_speed_down[servo_num] += min_speed_margin
  #         self.slow_speed_up[servo_num] -= min_speed_margin
  #         self.med_speed_down[servo_num] = self.slow_speed_down[servo_num] + constants.Delta_Slow_Med_Speed
  #         self.med_speed_up[servo_num] = self.slow_speed_up[servo_num] - constants.Delta_Slow_Med_Speed
  #         self.fast_speed_down[servo_num] = self.slow_speed_down[servo_num] + constants.Delta_Slow_Fast_Speed
  #         self.fast_speed_up[servo_num] = self.slow_speed_up[servo_num] - constants.Delta_Slow_Fast_Speed
  #         print("Min speed down, Servo",servo_num,"= ",self.slow_speed_down[servo_num])
  #         print("Min speed up Servo",servo_num,"= ",self.slow_speed_up[servo_num])
  #         break
  #       continue
        
  #     #find time increments for 3 speeds
  #     time_bet_counts = self.motor_calibration_sequence(servo_num, self.slow_speed_down[servo_num], enc_counts_time_cal, time_out)
  #     if time_bet_counts != 0 and time_bet_counts != time_out:
  #       self.enc_time_bet_counts_slow_down[servo_num] = round(time_bet_counts * time_between_counts_margin,3)
  #     print("Time between counts for Servo",servo_num,"at slow speed down:",self.enc_time_bet_counts_slow_down[servo_num])
      
  #     time_bet_counts = self.motor_calibration_sequence(servo_num, self.slow_speed_up[servo_num], enc_counts_time_cal, time_out)
  #     if time_bet_counts != 0 and time_bet_counts != time_out:
  #       self.enc_time_bet_counts_slow_up[servo_num] = round(time_bet_counts * time_between_counts_margin,3)
  #     print("Time between counts for Servo",servo_num,"at slow speed up:",self.enc_time_bet_counts_slow_up[servo_num])
      
  #     time_bet_counts = self.motor_calibration_sequence(servo_num, self.med_speed_down[servo_num], enc_counts_time_cal, time_out)
  #     if time_bet_counts != 0 and time_bet_counts != time_out:
  #       self.enc_time_bet_counts_med_down[servo_num] = round(time_bet_counts * time_between_counts_margin,3)
  #     print("Time between counts for Servo",servo_num,"at medium speed down:",self.enc_time_bet_counts_med_down[servo_num])
      
  #     time_bet_counts = self.motor_calibration_sequence(servo_num, self.med_speed_up[servo_num], enc_counts_time_cal, time_out)
  #     if time_bet_counts != 0 and time_bet_counts != time_out:
  #       self.enc_time_bet_counts_med_up[servo_num] = round(time_bet_counts * time_between_counts_margin,3)
  #     print("Time between counts for Servo",servo_num,"at medium speed up:",self.enc_time_bet_counts_med_up[servo_num])    
      
  #     time_bet_counts = self.motor_calibration_sequence(servo_num, self.fast_speed_down[servo_num], enc_counts_time_cal, time_out)
  #     if time_bet_counts != 0 and time_bet_counts != time_out:
  #       self.enc_time_bet_counts_fast_down[servo_num] = round(time_bet_counts * time_between_counts_margin,3)
  #     print("Time between counts for Servo",servo_num,"at fast speed down:",self.enc_time_bet_counts_fast_down[servo_num])
      
  #     time_bet_counts = self.motor_calibration_sequence(servo_num,self.fast_speed_up[servo_num], enc_counts_time_cal, time_out)
  #     if time_bet_counts != 0 and time_bet_counts != time_out:
  #       self.enc_time_bet_counts_fast_up[servo_num] = round(time_bet_counts * time_between_counts_margin,3)
  #     print("Time between counts for Servo",servo_num,"at fast speed up:",self.enc_time_bet_counts_fast_up[servo_num])    
    
  #     self.store.save(CalibrationMode.SLOW_SPEED_DOWN,self.slow_speed_down)
  #     self.store.save(CalibrationMode.SLOW_SPEED_UP,self.slow_speed_up)
  #     self.store.save(CalibrationMode.MED_SPEED_DOWN,self.med_speed_down)
  #     self.store.save(CalibrationMode.MED_SPEED_UP,self.med_speed_up)
  #     self.store.save(CalibrationMode.FAST_SPEED_DOWN,self.fast_speed_down)
  #     self.store.save(CalibrationMode.FAST_SPEED_UP,self.fast_speed_up)
  #     self.store.save(CalibrationMode.ENC_TIME_BET_COUNTS_SLOW_DOWN,self.enc_time_bet_counts_slow_down)
  #     self.store.save(CalibrationMode.ENC_TIME_BET_COUNTS_SLOW_UP,self.enc_time_bet_counts_slow_up)
  #     self.store.save(CalibrationMode.ENC_TIME_BET_COUNTS_MED_DOWN,self.enc_time_bet_counts_med_down)
  #     self.store.save(CalibrationMode.ENC_TIME_BET_COUNTS_MED_UP,self.enc_time_bet_counts_med_up)
  #     self.store.save(CalibrationMode.ENC_TIME_BET_COUNTS_FAST_DOWN,self.enc_time_bet_counts_fast_down)
  #     self.store.save(CalibrationMode.ENC_TIME_BET_COUNTS_FAST_UP,self.enc_time_bet_counts_fast_up)

  #     print("Encoder counts",self.enc_counts)  
  #     #move_motors_home()
  #     self.stop_all_motors()
    

# ---------------------------------- UNUSED ---------------------------------- #

# def shutdown():
#   #Try to save data and shutdown before power loss
#   save_calibration(self.enc_counts)
#   call("sudo nohup shutdown -h now", shell=True)
  
# def reboot():
#   #Try to save data and shutdown before power loss
#   save_calibration(self.enc_counts)
#   call("sudo nohup shutdown -r now", shell=True)