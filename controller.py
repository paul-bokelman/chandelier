import time
import RPi.GPIO as GPIO
import constants
from PCA9685 import PCA9685
from lib.data import CalibrationMode, CalibrationData

pwm = PCA9685(0x40, debug=False)
pwm.setPWMFreq(50)

class MotorController:
  """
  Main motor controller class for calibrating, saving, and manipulating servos
  """
  def __init__(self, debug = False) -> None:
    self.debug = debug
    self.store = CalibrationData()

    self.pwm = PCA9685(0x40, debug=False)
    self.pwm.setPWMFreq(50)

    self.enc_counts = self.store.get(CalibrationMode.ENC_COUNTS) # motor encoder count - Initialize to 0
    self.slow_speed_down = self.store.get(CalibrationMode.SLOW_SPEED_DOWN)# minimum motor speed in down direction - Initialize to 0
    self.slow_speed_up = self.store.get(CalibrationMode.SLOW_SPEED_UP)# minimum motor speed in up direction - Initialize to 0
    self.med_speed_down = self.store.get(CalibrationMode.MED_SPEED_DOWN)# medium motor speed in down direction - Initialize to 0
    self.med_speed_up = self.store.get(CalibrationMode.MED_SPEED_UP)# medium motor speed in up direction - Initialize to 0
    self.fast_speed_down = self.store.get(CalibrationMode.FAST_SPEED_DOWN)# fast motor speed in down direction - Initialize to 0
    self.fast_speed_up = self.store.get(CalibrationMode.FAST_SPEED_UP)# fast motor speed in up direction - Initialize to 0
    self.enc_time_bet_counts_slow_down = self.store.get(CalibrationMode.ENC_TIME_BET_COUNTS_SLOW_DOWN) # max time between encoder counts for slow speed in down direction - Initialize to 0
    self.enc_time_bet_counts_slow_up = self.store.get(CalibrationMode.ENC_TIME_BET_COUNTS_SLOW_UP)# max time between encoder counts for slow speed in up direction - Initialize to 0
    self.enc_time_bet_counts_med_down = self.store.get(CalibrationMode.ENC_TIME_BET_COUNTS_MED_DOWN)# max time between encoder counts for medium speed in down direction - Initialize to 0
    self.enc_time_bet_counts_med_up = self.store.get(CalibrationMode.ENC_TIME_BET_COUNTS_MED_UP)# max time between encoder counts for medium speed in up direction - Initialize to 0
    self.enc_time_bet_counts_fast_down = self.store.get(CalibrationMode.ENC_TIME_BET_COUNTS_FAST_DOWN)# max time between encoder counts for fast speed in down direction - Initialize to 0
    self.enc_time_bet_counts_fast_up = self.store.get(CalibrationMode.ENC_TIME_BET_COUNTS_FAST_UP)# max time between encoder counts for fast speed in up direction - Initialize to 0

  def set_motor(self, id: int, speed: float):
    """
    Set a specific motor to a specific speed
    """
    mtr_pulse = self.to_pulse(speed)
    pwm.setServoPulse(id, mtr_pulse)
    print("Setting Servo Motor",id,"to",speed)

  def stop_motor(self, id: int):
    """
    Stop a specific motor given the motor number
    """
    pwm.setServoPulse(id, 1500) 

  def stop_all_motors(self):
    """
    Stop all motors by calling stop_motor for each motor
    """
    for i in range(constants.n_motors):
      self.stop_motor(i)

  def set_all_motors(self, speed: float):
    """
    Set all motors to a specific speed
    """
    for i in range(constants.n_motors):
      self.set_motor(i,speed)

  def to_pulse(self, percent_speed: float)->float:
    """
    Convert motor speed and direction into Servo pulse
    """
    if constants.Up_Dir_CCW:
      return -8*(percent_speed-187.5) 
    else:
      return 8*(percent_speed+187.5) 
    
  def move_motors_counts(self, tar_pos,mode,speed=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]):
    """
    Move list of motors a set number of counts at given speeds\n
    Returns a list of the motors that failed to reach their targets either from stalling or timeout
    """
  
    start = time.time()
    time_out = 40 # how long to wait for encoder counts before timing out
    old_enc_counts: list[float] = [0] * 16 # old encoder counts
    motors_stopped = [] # stopped motors
    motors_stalled = [] # stalled motors
    motors_successful = [] # successful motors
    motors_unsuccessful = [] # unsuccessful motors
    last_count_time = [0.0] * 16 # last time there was an encoder count
    last_count_limit = [0.0] * 16 # last time limit between encoder counts
    total_move = [0] * 16 # total move distances
    startup_time_factor = 1.5 # correction factor for first encoder count
    stall_detect = True # stall detection flag
    
    # set speeds and last count limits
    for i in range(0,len(tar_pos)):
      total_move.append(abs(self.enc_counts[i]-tar_pos[i])) # append total move distance (in counts)

      # target position greater than current position -> go down
      if tar_pos[i] > self.enc_counts[i]:
        if mode == "s": # slow mode
          speed[i] = self.slow_speed_down[i] # set speed to slow speed down 
          last_count_limit[i] = self.enc_time_bet_counts_slow_down[i]  # set last count limit to slow speed down
        elif mode == "m": # medium mode
          speed[i] = self.med_speed_down[i]
          last_count_limit[i] = self.enc_time_bet_counts_med_down[i]
        elif mode == "f": # fast mode
          speed[i] = self.fast_speed_down[i]
          last_count_limit[i] = self.enc_time_bet_counts_fast_down[i]
        else:
          stall_detect = False

      # target position less than current position -> go up
      elif tar_pos[i] < self.enc_counts[i]:
        if mode == "s": # slow mode
          speed[i] = self.slow_speed_up[i]
          last_count_limit[i] = self.enc_time_bet_counts_slow_up[i]
        elif mode == "m": # medium mode
          speed[i] = self.med_speed_up[i]
          last_count_limit[i] = self.enc_time_bet_counts_med_up[i]
        elif mode == "f": # fast mode
          speed[i] = self.fast_speed_up[i]
          last_count_limit[i] = self.enc_time_bet_counts_fast_up[i]
        else:
          if speed[i] > 0: speed[i] = -speed[i]
          stall_detect = False
      # already at target position -> stop motor (speed 0)
      else:
        speed[i] = 0
      if self.debug: print(i,":Speed =",speed[i],"  Last_count_limit =",last_count_limit[i])

    # set time out based on max encoder counts and max total move
    if stall_detect: time_out = max(last_count_limit) * max(total_move)

    # set speeds and start motors for motors that need to move
    for i in range(0,len(tar_pos)):
      if tar_pos[i] != self.enc_counts[i] and speed[i] != 0:
        pwm.setServoPulse(i,self.to_pulse(speed[i]))
        print("Motor",i,"set at speed",speed[i])
      else:
        motors_stopped.append(i)
        motors_successful.append(i)
        print("Motor",i,"already at target position")
    
    # set old encoder counts and last count time
    for i in range(0,16):
      old_enc_counts[i] = self.enc_counts[i]
      last_count_time[i] = time.time()
    
    print("Starting encoder counts",self.enc_counts)

    # loop through motors and check for encoder counts
    while True: 
      for i in range(0,len(tar_pos)):
        if i not in motors_stopped and GPIO.input(constants.encoder_pins[i]) == False: #Encoder triggered
          if self.enc_counts[i] == old_enc_counts[i]:
            if self.enc_counts[i] < tar_pos[i]:
              self.enc_counts[i] += 1
            else:
              self.enc_counts[i] -= 1
            total_move[i] += 1
            last_count_time[i] = time.time()
            print("Encoder counts",self.enc_counts)
        else:
            old_enc_counts[i] = self.enc_counts[i]
        if i not in motors_stopped and abs(self.enc_counts[i] - tar_pos[i]) == 0:
          self.stop_motor(i)
          motors_stopped.append(i)
          if not i in motors_successful: motors_successful.append(i)
          print("Motor",i,"reached encoder count",self.enc_counts[i])
        if stall_detect:
          for j in range(0,len(tar_pos)):
            #if self.debug: print("Time elaspsed for motor",j,":",(time.time()-last_count_time[j])," Time limit =",last_count_limit[j])
            if (total_move[j] > 2 and (time.time()-last_count_time[j]) > last_count_limit[j]) or (total_move[j] < 3 and (time.time()-last_count_time[j]) > startup_time_factor*last_count_limit[j]): #motor j stalled
              self.stop_motor(j)
              if not j in motors_stopped: 
                motors_stopped.append(j)
                if not j in motors_stalled:
                  motors_stalled.append(j)
                  print("Motor",j,"stalled at encoder count",self.enc_counts[j])
    
      if len(motors_stopped) == len(tar_pos):
        print("All motors stopped. Motors",motors_stalled,"stalled.")
        break
      if time.time()-start > time_out:
        print(time_out,"seconds elapsed.")
        break
      continue
    
    self.stop_all_motors()
    print("Final encoder counts",self.enc_counts)
    print("Motors",motors_successful,"reached their encoder targets.")
    for i in range(0,len(tar_pos)):
      if not i in motors_successful:
        motors_unsuccessful.append(i)
    return(motors_unsuccessful)
    
  def move_motors_home(self):
    #Move all motors home (0 encoder)
    
    return_val = []
    #cmd_speeds = [0] * CC.n_motors    # Initialize
    home_counts = [0] * constants.n_motors    # Set all 0's as target home position
    if self.debug: print("Returing all motors home.")
    return_val = self.move_motors_counts(home_counts,"s")
    if len(return_val) == 0:
      if self.debug: print("All motors home.")
    else:
      if self.debug: print("Motors",return_val,"failed to return home")
    
  def home_motors(self):
    """
    Calibrate absolute encoder positions for all motors
    """
    move_inc = -5
    time_out = 60
    timed_out = False
    return_val = []
    motors_stalled = []
    motors_timed_out = []
    tar_positions = [0] * constants.n_motors
    start_time = time.time()
    
    while len(motors_stalled) != constants.n_motors:
      for i in range(0,constants.n_motors):
        self.enc_counts[i] = 0
        if not i in motors_stalled:
          tar_positions[i] = move_inc
        else:
          tar_positions[i] = 0
      return_val = self.move_motors_counts(tar_positions,"s")
      if self.debug: print("Return_val:",return_val)
      for k in range(0,len(return_val)):
        if not return_val[k] in motors_stalled:
          motors_stalled.append(return_val[k])
          self.enc_counts[k] = 0
      if self.debug: print("Stalled motors:",motors_stalled)
      if time.time() - start_time > time_out: 
        for j in range(0,constants.n_motors):
          if j not in motors_stalled: motors_timed_out.append(j)
        timed_out = True
        break
    if self.debug:
      if timed_out:
        print("Motors homed:",motors_stalled,"Motors timed out:",motors_timed_out)
      else:  
        print("All motors homed:",self.enc_counts)
    return motors_timed_out
      
  def motor_calibration_sequence(self, servo_num, speed, enc_counts, time_out):
    """
    Motor move sequence that reports back time between counts
    """
    el_time_list = []
    encoder_num = constants.encoder_pins[servo_num]
    if speed > 0:
      enc_inc = 1
    else:
      enc_inc = -1
    encoder_counter = 0
    
    if GPIO.input(encoder_num) == False: #Encoder already triggered
      encoder_counter += 1 #encoder_counter + enc_inc
      self.enc_counts[servo_num] = self.enc_counts[servo_num] + enc_inc
      print("Encoder_counter:",encoder_counter,"  self.enc_counts:",self.enc_counts[servo_num])
    pwm.setServoPulse(servo_num,self.to_pulse(speed))
    start = time.time()
    time_since_last = start
    print("Setting Servo Motor",servo_num,"to",speed,"for",enc_counts,"counts")
    while True: 
      if GPIO.input(encoder_num) == False: #Encoder triggered
        encoder_counter += 1 #encoder_counter + enc_inc
        self.enc_counts[servo_num] = self.enc_counts[servo_num] + enc_inc
        #print("Encoder_counter 2:",encoder_counter,"  self.enc_counts:",self.enc_counts[servo_num])
        if encoder_counter in range(0,enc_counts):
          elapsed_time = time.time() - time_since_last
          time_since_last = time.time()
          if abs(encoder_counter) != 1:
            el_time_list.append(elapsed_time)
          #print("Encoder count 3",encoder_counter,"  self.enc_counts:",self.enc_counts[servo_num],"elapsed time:",round(elapsed_time,3))
        if abs(encoder_counter) == abs(enc_counts):
          break
        while GPIO.input(encoder_num) == False: #Encoder still triggered
          if time.time()-time_since_last > time_out:
            #print("Motor",servo_num,"timed out at encoder count",encoder_counter,"without reaching count",enc_counts)
            return(time_out)
            break
          continue
        if time.time()-time_since_last > time_out:
          #print("Motor",servo_num,"timed out at encoder count",encoder_counter,"without reaching count",enc_counts)
          return(time_out)
          break
      continue
    #print("Max time between encoder counts for speed",speed,"is: ",round(max(el_time_list),3),"seconds")
    pwm.setServoPulse(servo_num,1500)
    if len(el_time_list)>0:
      return(round(max(el_time_list),3))
    else:
      return(0)

  def encoder_speed_calibration(self):
    """
    builds lists of motor move constants such as minimum speed up and down and max encoder timing by speed up and down 
    """
    tar_pos_list = [0] * 16
    speed_list = [0] * 16
    initial_offset = 10
    time_out = 8
    enc_counts_min_speed = 4
    enc_counts_time_cal = 8    
    speed_inc = -1
    min_speed_margin = 1
    time_between_counts_margin = 1.05

    for i in range(0,constants.n_motors):
      tar_pos_list[i] = initial_offset
      speed_list[i] = constants.Max_Speed
    #if self.debug: print("Tar_Pos:",tar_pos_list,"Speed:",speed_list)
    self.move_motors_counts(tar_pos_list,"",speed_list)

    for servo_num in range(0,constants.n_motors):
      #find minimum speed
      speed = constants.Max_Speed
      found_min_down = False
      found_min_up = False
      
      while True:
        if self.debug: print("Trying speed",speed,"for Servo number:",servo_num)
        if found_min_down == False:
          if self.motor_calibration_sequence(servo_num, speed, enc_counts_min_speed, time_out) != time_out:
            self.slow_speed_down[servo_num] = speed
          else:
            found_min_down = True
        if found_min_up == False:
          if self.motor_calibration_sequence(servo_num, -speed, enc_counts_min_speed, time_out) != time_out:
            self.slow_speed_up[servo_num] = -speed
          else:
            found_min_up = True
        speed += speed_inc
        if found_min_down and found_min_up:
          self.slow_speed_down[servo_num] += min_speed_margin
          self.slow_speed_up[servo_num] -= min_speed_margin
          self.med_speed_down[servo_num] = self.slow_speed_down[servo_num] + constants.Delta_Slow_Med_Speed
          self.med_speed_up[servo_num] = self.slow_speed_up[servo_num] - constants.Delta_Slow_Med_Speed
          self.fast_speed_down[servo_num] = self.slow_speed_down[servo_num] + constants.Delta_Slow_Fast_Speed
          self.fast_speed_up[servo_num] = self.slow_speed_up[servo_num] - constants.Delta_Slow_Fast_Speed
          print("Min speed down, Servo",servo_num,"= ",self.slow_speed_down[servo_num])
          print("Min speed up Servo",servo_num,"= ",self.slow_speed_up[servo_num])
          break
        continue
        
      #find time increments for 3 speeds
      time_bet_counts = self.motor_calibration_sequence(servo_num, self.slow_speed_down[servo_num], enc_counts_time_cal, time_out)
      if time_bet_counts != 0 and time_bet_counts != time_out:
        self.enc_time_bet_counts_slow_down[servo_num] = round(time_bet_counts * time_between_counts_margin,3)
      print("Time between counts for Servo",servo_num,"at slow speed down:",self.enc_time_bet_counts_slow_down[servo_num])
      
      time_bet_counts = self.motor_calibration_sequence(servo_num, self.slow_speed_up[servo_num], enc_counts_time_cal, time_out)
      if time_bet_counts != 0 and time_bet_counts != time_out:
        self.enc_time_bet_counts_slow_up[servo_num] = round(time_bet_counts * time_between_counts_margin,3)
      print("Time between counts for Servo",servo_num,"at slow speed up:",self.enc_time_bet_counts_slow_up[servo_num])
      
      time_bet_counts = self.motor_calibration_sequence(servo_num, self.med_speed_down[servo_num], enc_counts_time_cal, time_out)
      if time_bet_counts != 0 and time_bet_counts != time_out:
        self.enc_time_bet_counts_med_down[servo_num] = round(time_bet_counts * time_between_counts_margin,3)
      print("Time between counts for Servo",servo_num,"at medium speed down:",self.enc_time_bet_counts_med_down[servo_num])
      
      time_bet_counts = self.motor_calibration_sequence(servo_num, self.med_speed_up[servo_num], enc_counts_time_cal, time_out)
      if time_bet_counts != 0 and time_bet_counts != time_out:
        self.enc_time_bet_counts_med_up[servo_num] = round(time_bet_counts * time_between_counts_margin,3)
      print("Time between counts for Servo",servo_num,"at medium speed up:",self.enc_time_bet_counts_med_up[servo_num])    
      
      time_bet_counts = self.motor_calibration_sequence(servo_num, self.fast_speed_down[servo_num], enc_counts_time_cal, time_out)
      if time_bet_counts != 0 and time_bet_counts != time_out:
        self.enc_time_bet_counts_fast_down[servo_num] = round(time_bet_counts * time_between_counts_margin,3)
      print("Time between counts for Servo",servo_num,"at fast speed down:",self.enc_time_bet_counts_fast_down[servo_num])
      
      time_bet_counts = self.motor_calibration_sequence(servo_num,self.fast_speed_up[servo_num], enc_counts_time_cal, time_out)
      if time_bet_counts != 0 and time_bet_counts != time_out:
        self.enc_time_bet_counts_fast_up[servo_num] = round(time_bet_counts * time_between_counts_margin,3)
      print("Time between counts for Servo",servo_num,"at fast speed up:",self.enc_time_bet_counts_fast_up[servo_num])    
    
      self.store.save(CalibrationMode.SLOW_SPEED_DOWN,self.slow_speed_down)
      self.store.save(CalibrationMode.SLOW_SPEED_UP,self.slow_speed_up)
      self.store.save(CalibrationMode.MED_SPEED_DOWN,self.med_speed_down)
      self.store.save(CalibrationMode.MED_SPEED_UP,self.med_speed_up)
      self.store.save(CalibrationMode.FAST_SPEED_DOWN,self.fast_speed_down)
      self.store.save(CalibrationMode.FAST_SPEED_UP,self.fast_speed_up)
      self.store.save(CalibrationMode.ENC_TIME_BET_COUNTS_SLOW_DOWN,self.enc_time_bet_counts_slow_down)
      self.store.save(CalibrationMode.ENC_TIME_BET_COUNTS_SLOW_UP,self.enc_time_bet_counts_slow_up)
      self.store.save(CalibrationMode.ENC_TIME_BET_COUNTS_MED_DOWN,self.enc_time_bet_counts_med_down)
      self.store.save(CalibrationMode.ENC_TIME_BET_COUNTS_MED_UP,self.enc_time_bet_counts_med_up)
      self.store.save(CalibrationMode.ENC_TIME_BET_COUNTS_FAST_DOWN,self.enc_time_bet_counts_fast_down)
      self.store.save(CalibrationMode.ENC_TIME_BET_COUNTS_FAST_UP,self.enc_time_bet_counts_fast_up)

      print("Encoder counts",self.enc_counts)  
      #move_motors_home()
      self.stop_all_motors()
    

# ---------------------------------- UNUSED ---------------------------------- #

# def shutdown():
#   #Try to save data and shutdown before power loss
#   save_calibration(self.enc_counts)
#   call("sudo nohup shutdown -h now", shell=True)
  
# def reboot():
#   #Try to save data and shutdown before power loss
#   save_calibration(self.enc_counts)
#   call("sudo nohup shutdown -r now", shell=True)


# def set_GPIO_as_input(self, GPIO_List):
#   #Sets the passed in list of GPIO pins as input
#   for i in range(0,len(GPIO_List)):
#     print("Setting GPIO pin",i,"as input")
#     GPIO.setup(GPIO_List[i], GPIO.IN, pull_up_down=GPIO.PUD_UP)

# def set_GPIO_as_output(self, GPIO_List):
#   #Sets the passed in list of GPIO pins as output
#   for i in range(0,len(GPIO_List)):
#     print("Setting GPIO pin",i,"as output")
#     GPIO.setup(GPIO_List[i], GPIO.OUT)


  # def move_motor_time(self, servo_num,speed,move_time):
  #   """
  #   Move the servo motor at the input speed for set time
  #   """
  #   mtr_pulse = self.to_pulse(speed)
  #   pwm.setServoPulse(servo_num,mtr_pulse)
  #   print("Setting Servo Motor",servo_num,"to",speed,"for",move_time,"seconds")
  #   time.sleep(move_time)
  #   pwm.setServoPulse(servo_num,1500)


  # def move_motor_enc(self, servo_num,encoder_num,speed,enc_counts,time_out):
  #   """
  #   Move the servo motor at the input speed for set number of counts
  #   """
  #   mtr_pulse = self.to_pulse(speed)
  #   start = time.time()
  #   if speed > 0:
  #     enc_inc = 1
  #   else:
  #     enc_inc = -1
  #   encoder_counter = 0
  #   if GPIO.input(encoder_num) == False: #Encoder already triggered
  #     encoder_counter = encoder_counter - enc_inc
  #   pwm.setServoPulse(servo_num,mtr_pulse)
  #   print("Setting Servo Motor",servo_num,"to",speed,"for",enc_counts,"revolutions")
  #   while True: 
  #     if GPIO.input(encoder_num) == False: #Encoder triggered
  #       encoder_counter = encoder_counter + enc_inc
  #       print("Encoder counts",encoder_counter)
  #       if abs(encoder_counter) == abs(enc_counts):
  #         break
  #       while GPIO.input(encoder_num) == False: #Encoder still triggered
  #         continue
  #       if time.time()-start > time_out:
  #         print("Motor",servo_num,"timed out at encoder count",encoder_counter,"without reaching count",enc_counts)
  #         break
  #     continue
  #   pwm.setServoPulse(servo_num,1500)