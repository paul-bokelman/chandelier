import time
import RPi.GPIO as GPIO
import constants
from subprocess import call
from main import pwm
# from adafruit_pca9685 import PCA9685 #  Adafruit PCA9685 library

EnCounts = [0] * 16                 #Global variable for motor encoder count - Initialize to 0
SlowSpeedDown = [0] * 16            #Global variable for minimum motor speed in down direction - Initialize to 0
SlowSpeedUp = [0] * 16              #Global variable for minimum motor speed in up direction - Initialize to 0
MedSpeedDown = [0] * 16             #Global variable for medium motor speed in down direction - Initialize to 0
MedSpeedUp = [0] * 16               #Global variable for medium motor speed in up direction - Initialize to 0
FastSpeedDown = [0] * 16            #Global variable for fast motor speed in down direction - Initialize to 0
FastSpeedUp = [0] * 16              #Global variable for fast motor speed in up direction - Initialize to 0
EncTimeBetCountsSlowDown = [0] * 16 #Global variable for max time between encoder counts for slow speed in down direction - Initialize to 0
EncTimeBetCountsSlowUp = [0] * 16   #Global variable for max time between encoder counts for slow speed in up direction - Initialize to 0
EncTimeBetCountsMedDown = [0] * 16  #Global variable for max time between encoder counts for medium speed in down direction - Initialize to 0
EncTimeBetCountsMedUp = [0] * 16    #Global variable for max time between encoder counts for medium speed in up direction - Initialize to 0
EncTimeBetCountsFastDown = [0] * 16 #Global variable for max time between encoder counts for fast speed in down direction - Initialize to 0
EncTimeBetCountsFastUp = [0] * 16   #Global variable for max time between encoder counts for fast speed in up direction - Initialize to 0

Debug = True

def shutdown():
  #Try to save data and shutdown before power loss
  data_output(EnCounts)
  call("sudo nohup shutdown -h now", shell=True)
  
def reboot():
  #Try to save data and shutdown before power loss
  data_output(EnCounts)
  call("sudo nohup shutdown -r now", shell=True)

def data_output(save_mode, data_out):
  #Save data to a text file
  valid_save_mode = True
  
  if save_mode == 1:  #Save the current encoder positions
    save_file = "/home/kevin/Documents/Code/Data/encoder_data.txt"
  elif save_mode == 2:  #Save the slow speed down data
    save_file = "/home/kevin/Documents/Code/Data/slowspeed_down_data.txt"
  elif save_mode == 3:  #Save the slow speed up data
    save_file = "/home/kevin/Documents/Code/Data/slowspeed_up_data.txt"
  elif save_mode == 4:  #Save the med speed down data
    save_file = "/home/kevin/Documents/Code/Data/medspeed_down_data.txt"
  elif save_mode == 5:  #Save the med speed up data
    save_file = "/home/kevin/Documents/Code/Data/medspeed_up_data.txt"
  elif save_mode == 6:  #Save the fast speed down data
    save_file = "/home/kevin/Documents/Code/Data/fastspeed_down_data.txt"
  elif save_mode == 7:  #Save the fast speed up data
    save_file = "/home/kevin/Documents/Code/Data/fastspeed_up_data.txt"
  elif save_mode == 8:  #Save the time between encoder counts slow down data
    save_file = "/home/kevin/Documents/Code/Data/enc_time_bet_counts_slow_down_data.txt"
  elif save_mode == 9:  #Save the time between encoder counts slow up data
    save_file = "/home/kevin/Documents/Code/Data/enc_time_bet_counts_slow_up_data.txt"
  elif save_mode == 10:  #Save the time between encoder counts med down data
    save_file = "/home/kevin/Documents/Code/Data/enc_time_bet_counts_med_down_data.txt"
  elif save_mode == 11:  #Save the time between encoder counts med up data
    save_file = "/home/kevin/Documents/Code/Data/enc_time_bet_counts_med_up_data.txt"
  elif save_mode == 12:  #Save the time between encoder counts fast down data
    save_file = "/home/kevin/Documents/Code/Data/enc_time_bet_counts_fast_down_data.txt"
  elif save_mode == 13:  #Save the time between encoder counts fast up data
    save_file = "/home/kevin/Documents/Code/Data/enc_time_bet_counts_fast_up_data.txt"
  else:
    print("Invalid save mode")
    valid_save_mode = False
    
  if valid_save_mode:
    with open(save_file, "w+") as fp:
      for items in data_out:
        fp.write('%s\n' %items)  # write elements of list
      print("File written successfully")
  fp.close()


def initialize():
  #gather data from saved files
  global Encounts, SlowSpeedDown, SlowSpeedUp, MedSpeedDown, MedSpeedUp, FastSpeedDown, FastSpeedDown
  global EncTimeBetCountsSlowDown, EncTimeBetCountsSlowUp,EncTimeBetCountsMedDown, EncTimeBetCountsMedUp, EncTimeBetCountsFastDown, EncTimeBetCountsFastUp
  
  EnCounts = data_input(1)
  SlowSpeedDown = data_input(2)
  SlowSpeedUp = data_input(3)
  MedSpeedDown = data_input(4)
  MedSpeedDown = data_input(5)
  FastSpeedDown = data_input(6)
  FastSpeedUp = data_input(7)
  EncTimeBetCountsSlowDown = data_input(8)
  EncTimeBetCountsSlowUp = data_input(9)
  EncTimeBetCountsMedDown = data_input(10)
  EncTimeBetCountsMedUp = data_input(11)
  EncTimeBetCountsFastDown = data_input(12)
  EncTimeBetCountsFastUp = data_input(13)


def data_input(open_mode):
  #Get data from a text file
  valid_open_mode = True
  
  if open_mode == 1:  #Save the current encoder positions
    open_file = "/home/kevin/Documents/Code/Data/encoder_data.txt"
  elif open_mode == 2:  #Save the slow speed down data
    open_file = "/home/kevin/Documents/Code/Data/slowspeed_down_data.txt"
  elif open_mode == 3:  #Save the slow speed up data
    open_file = "/home/kevin/Documents/Code/Data/slowspeed_up_data.txt"
  elif open_mode == 4:  #Save the med speed down data
    open_file = "/home/kevin/Documents/Code/Data/medspeed_down_data.txt"
  elif open_mode == 5:  #Save the med speed up data
    open_file = "/home/kevin/Documents/Code/Data/medspeed_up_data.txt"
  elif open_mode == 6:  #Save the fast speed down data
    open_file = "/home/kevin/Documents/Code/Data/fastspeed_down_data.txt"
  elif open_mode == 7:  #Save the fast speed up data
    open_file = "/home/kevin/Documents/Code/Data/fastspeed_up_data.txt"
  elif open_mode == 8:  #Save the time between encoder counts slow down data
    open_file = "/home/kevin/Documents/Code/Data/enc_time_bet_counts_slow_down_data.txt"
  elif open_mode == 9:  #Save the time between encoder counts slow up data
    open_file = "/home/kevin/Documents/Code/Data/enc_time_bet_counts_slow_up_data.txt"
  elif open_mode == 10:  #Save the time between encoder counts med down data
    open_file = "/home/kevin/Documents/Code/Data/enc_time_bet_counts_med_down_data.txt"
  elif open_mode == 11:  #Save the time between encoder counts med up data
    open_file = "/home/kevin/Documents/Code/Data/enc_time_bet_counts_med_up_data.txt"
  elif open_mode == 12:  #Save the time between encoder counts fast down data
    open_file = "/home/kevin/Documents/Code/Data/enc_time_bet_counts_fast_down_data.txt"
  elif open_mode == 13:  #Save the time between encoder counts fast up data
    open_file = "/home/kevin/Documents/Code/Data/enc_time_bet_counts_fast_up_data.txt"
  else:
    print("Invalid open mode")
    valid_open_mode = False

  if valid_open_mode:  #Read the last known encoder positions
    try:
      with open(open_file, "r") as fp:
        tmp_enc = []
        for line in fp.readlines():
          if open_mode in range(0,8):
            tmp_enc.append(int(line.strip()))
          elif open_mode in range(8,14):
            tmp_enc.append(float(line.strip()))
      fp.close()
      print("File mode",open_mode,"read successfully",tmp_enc)
      return tmp_enc
    except FileNotFoundError as e:
      print("File mode",open_mode,"failed. File not found.")
 
def set_GPIO_as_input(GPIO_List):
  #Sets the passed in list of GPIO pins as input
  for i in range(0,len(GPIO_List)):
    print("Setting GPIO pin",i,"as input")
    GPIO.setup(GPIO_List[i], GPIO.IN, pull_up_down=GPIO.PUD_UP)
 
def set_GPIO_as_output(GPIO_List):
  #Sets the passed in list of GPIO pins as output
  for i in range(0,len(GPIO_List)):
    print("Setting GPIO pin",i,"as output")
    GPIO.setup(GPIO_List[i], GPIO.OUT)

def stop_motor(mtr_num):
  #Stop specific motor
  pwm.setServoPulse(mtr_num,1500) 

def stop_all_motors(Num_Mtrs):
  #Stop all motors
  for i in range(0,Num_Mtrs):
    stop_motor(i)

def mtr_speed(Percent_Speed)->int:
  #Convert motor speed and direction into Servo pulse
  if constants.Up_Dir_CCW:
    return -8*(Percent_Speed-187.5) 
  else:
    return 8*(Percent_Speed+187.5) 
     
def move_motor_time(servo_num,speed,move_time):
  #Move the servo motor at the input speed for set time
  mtr_pulse = mtr_speed(speed)
  pwm.setServoPulse(servo_num,mtr_pulse)
  print("Setting Servo Motor",servo_num,"to",speed,"for",move_time,"seconds")
  time.sleep(move_time)
  pwm.setServoPulse(servo_num,1500)

def move_motor_enc(servo_num,encoder_num,speed,enc_counts,time_out):
  #Move the servo motor at the input speed for set number of counts
  mtr_pulse = mtr_speed(speed)
  start = time.time()
  if speed > 0:
    enc_inc = 1
  else:
    enc_inc = -1
  encoder_counter = 0
  if GPIO.input(encoder_num) == False: #Encoder already triggered
    encoder_counter = encoder_counter - enc_inc
  pwm.setServoPulse(servo_num,mtr_pulse)
  print("Setting Servo Motor",servo_num,"to",speed,"for",enc_counts,"revolutions")
  while True: 
    if GPIO.input(encoder_num) == False: #Encoder triggered
      encoder_counter = encoder_counter + enc_inc
      print("Encoder counts",encoder_counter)
      if abs(encoder_counter) == abs(enc_counts):
        break
      while GPIO.input(encoder_num) == False: #Encoder still triggered
        continue
      if time.time()-start > time_out:
        print("Motor",servo_num,"timed out at encoder count",encoder_counter,"without reaching count",enc_counts)
        break
    continue
  pwm.setServoPulse(servo_num,1500)
  
def move_motors_counts(tar_pos,mode,speed=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]):
  #Move list of motors a set number of counts at given speeds
  #Returns a list of the motors that failed to reach their targets either from stalling or timeout
 
  global EnCounts
  start = time.time()
  time_out = 40
  old_enc_counts = [0] * 16     #Initialize old count list to 0
  motors_stopped = []           #Initialize a blank list of stopped motors
  motors_stalled = []           #Initialize a blank list of stalled motors
  motors_successful = []        #Initialize a blank list of successful motors
  motors_unsuccessful = []      #Initialize a blank list of unsuccessful motors
  last_count_time = [0.0] * 16  #Initialize a list for last time there was an encoder count
  last_count_limit = [0.0] * 16 #Initialize a list for encoder time outs
  total_move = [0] * 16         #Initial a list of total move distances
  startup_time_factor = 1.5     #Correction factor for first encoder count
  stall_detect = True
 
  for i in range(0,len(tar_pos)):
    total_move.append(abs(EnCounts[i]-tar_pos[i]))
    if tar_pos[i] > EnCounts[i]:  #Go down
      if mode == "s": #Slow
        speed[i] = SlowSpeedDown[i]
        last_count_limit[i] = EncTimeBetCountsSlowDown[i]
      elif mode == "m": #Medium
        speed[i] = MedSpeedDown[i]
        last_count_limit[i] = EncTimeBetCountsMedDown[i]
      elif mode == "f": #Fast
        speed[i] = FastSpeedDown[i]
        last_count_limit[i] = EncTimeBetCountsFastDown[i]
      else:
        stall_detect = False
    elif tar_pos[i] < EnCounts[i]:  #Go up
      if mode == "s": #Slow
        speed[i] = SlowSpeedUp[i]
        last_count_limit[i] = EncTimeBetCountsSlowUp[i]
      elif mode == "m": #Medium
        speed[i] = MedSpeedUp[i]
        last_count_limit[i] = EncTimeBetCountsMedUp[i]
      elif mode == "f": #Fast
        speed[i] = FastSpeedUp[i]
        last_count_limit[i] = EncTimeBetCountsFastUp[i]
      else:
        if speed[i] > 0: speed[i] = -speed[i]
        stall_detect = False
    else: #at target position
      speed[i] = 0
    if Debug: print(i,":Speed =",speed[i],"  Last_count_limit =",last_count_limit[i])
  if stall_detect: time_out = max(last_count_limit) * max(total_move)
 
  for i in range(0,len(tar_pos)):
    if tar_pos[i] != EnCounts[i] and speed[i] != 0:
      pwm.setServoPulse(i,mtr_speed(speed[i]))
      print("Motor",i,"set at speed",speed[i])
    else:
      motors_stopped.append(i)
      motors_successful.append(i)
      print("Motor",i,"already at target position")
  
  for i in range(0,16):
    old_enc_counts[i] = EnCounts[i]
    last_count_time[i] = time.time()
  
  print("Starting encoder counts",EnCounts)

  while True: 
    for i in range(0,len(tar_pos)):
      if i not in motors_stopped and GPIO.input(constants.MEGM[i]) == False: #Encoder triggered
        if EnCounts[i] == old_enc_counts[i]:
          if EnCounts[i] < tar_pos[i]:
            EnCounts[i] += 1
          else:
            EnCounts[i] -= 1
          total_move[i] += 1
          last_count_time[i] = time.time()
          print("Encoder counts",EnCounts)
      else:
          old_enc_counts[i] = EnCounts[i]
      if i not in motors_stopped and abs(EnCounts[i] - tar_pos[i]) == 0:
        stop_motor(i)
        motors_stopped.append(i)
        if not i in motors_successful: motors_successful.append(i)
        print("Motor",i,"reached encoder count",EnCounts[i])
      if stall_detect:
        for j in range(0,len(tar_pos)):
          #if Debug: print("Time elaspsed for motor",j,":",(time.time()-last_count_time[j])," Time limit =",last_count_limit[j])
          if (total_move[j] > 2 and (time.time()-last_count_time[j]) > last_count_limit[j]) or (total_move[j] < 3 and (time.time()-last_count_time[j]) > startup_time_factor*last_count_limit[j]): #motor j stalled
            stop_motor(j)
            if not j in motors_stopped: 
              motors_stopped.append(j)
              if not j in motors_stalled:
                motors_stalled.append(j)
                print("Motor",j,"stalled at encoder count",EnCounts[j])
  
    if len(motors_stopped) == len(tar_pos):
      print("All motors stopped. Motors",motors_stalled,"stalled.")
      break
    if time.time()-start > time_out:
      print(time_out,"seconds elapsed.")
      break
    continue
   
  stop_all_motors(constants.Num_Motors)
  print("Final encoder counts",EnCounts)
  print("Motors",motors_successful,"reached their encoder targets.")
  for i in range(0,len(tar_pos)):
    if not i in motors_successful:
      motors_unsuccessful.append(i)
  return(motors_unsuccessful)
   
def move_motors_home():
  #Move all motors home (0 encoder)
  
  return_val = []
  #cmd_speeds = [0] * CC.Num_Motors    #Initialize
  home_counts = [0] * constants.Num_Motors    #Set all 0's as target home position
  if Debug: print("Returing all motors home.")
  return_val = move_motors_counts(home_counts,"s")
  if len(return_val) == 0:
    if Debug: print("All motors home.")
  else:
    if Debug: print("Motors",return_val,"failed to return home")
  
def home_motors():
  #Home all motors and reset EnCounts home position
  global EnCounts
  move_inc = -5
  time_out = 60
  timed_out = False
  return_val = []
  motors_stalled = []
  motors_timed_out = []
  tar_positions = [0] * constants.Num_Motors
  start_time = time.time()
  
  while len(motors_stalled) != constants.Num_Motors:
    for i in range(0,constants.Num_Motors):
      EnCounts[i] = 0
      if not i in motors_stalled:
        tar_positions[i] = move_inc
      else:
        tar_positions[i] = 0
    return_val = move_motors_counts(tar_positions,"s")
    if Debug: print("Return_val:",return_val)
    for k in range(0,len(return_val)):
      if not return_val[k] in motors_stalled:
        motors_stalled.append(return_val[k])
        EnCounts[k] = 0
    if Debug: print("Stalled motors:",motors_stalled)
    if time.time() - start_time > time_out: 
      for j in range(0,constants.Num_Motors):
        if j not in motors_stalled: motors_timed_out.append(j)
      timed_out = True
      break
  if Debug:
    if timed_out:
      print("Motors homed:",motors_stalled,"Motors timed out:",motors_timed_out)
    else:  
      print("All motors homed:",EnCounts)
  return motors_timed_out
    
def motor_calibration_sequence(servo_num, speed, enc_counts, time_out):
  #Motor move sequence that reports back time between counts
  global Encounts
  el_time_list = []
  encoder_num = constants.MEGM[servo_num]
  if speed > 0:
    enc_inc = 1
  else:
    enc_inc = -1
  encoder_counter = 0
   
  if GPIO.input(encoder_num) == False: #Encoder already triggered
    encoder_counter += 1 #encoder_counter + enc_inc
    EnCounts[servo_num] = EnCounts[servo_num] + enc_inc
    print("Encoder_counter:",encoder_counter,"  EnCounts:",EnCounts[servo_num])
  pwm.setServoPulse(servo_num,mtr_speed(speed))
  start = time.time()
  time_since_last = start
  print("Setting Servo Motor",servo_num,"to",speed,"for",enc_counts,"counts")
  while True: 
    if GPIO.input(encoder_num) == False: #Encoder triggered
      encoder_counter += 1 #encoder_counter + enc_inc
      EnCounts[servo_num] = EnCounts[servo_num] + enc_inc
      #print("Encoder_counter 2:",encoder_counter,"  EnCounts:",EnCounts[servo_num])
      if encoder_counter in range(0,enc_counts):
        elapsed_time = time.time() - time_since_last
        time_since_last = time.time()
        if abs(encoder_counter) != 1:
          el_time_list.append(elapsed_time)
        #print("Encoder count 3",encoder_counter,"  EnCounts:",EnCounts[servo_num],"elapsed time:",round(elapsed_time,3))
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

def encoder_speed_calibration():
  #builds lists of motor move constants such as minimum speed up and down
  #and max enocder timing by speed up and down 
  tar_pos_list = [0] * 16
  speed_list = [0] * 16
  initial_offset = 10
  time_out = 8
  enc_counts_min_speed = 4
  enc_counts_time_cal = 8    
  speed_inc = -1
  min_speed_margin = 1
  time_between_counts_margin = 1.05

  for i in range(0,constants.Num_Motors):
    tar_pos_list[i] = initial_offset
    speed_list[i] = constants.Max_Speed
  #if Debug: print("Tar_Pos:",tar_pos_list,"Speed:",speed_list)
  move_motors_counts(tar_pos_list,"",speed_list)

  for servo_num in range(0,constants.Num_Motors):
    #find minimum speed
    speed = constants.Max_Speed
    found_min_down = False
    found_min_up = False
    
    while True:
      if Debug: print("Trying speed",speed,"for Servo number:",servo_num)
      if found_min_down == False:
        if motor_calibration_sequence(servo_num, speed, enc_counts_min_speed, time_out) != time_out:
          SlowSpeedDown[servo_num] = speed
        else:
          found_min_down = True
      if found_min_up == False:
        if motor_calibration_sequence(servo_num, -speed, enc_counts_min_speed, time_out) != time_out:
          SlowSpeedUp[servo_num] = -speed
        else:
          found_min_up = True
      speed += speed_inc
      if found_min_down and found_min_up:
        SlowSpeedDown[servo_num] += min_speed_margin
        SlowSpeedUp[servo_num] -= min_speed_margin
        MedSpeedDown[servo_num] = SlowSpeedDown[servo_num] + constants.Delta_Slow_Med_Speed
        MedSpeedUp[servo_num] = SlowSpeedUp[servo_num] - constants.Delta_Slow_Med_Speed
        FastSpeedDown[servo_num] = SlowSpeedDown[servo_num] + constants.Delta_Slow_Fast_Speed
        FastSpeedUp[servo_num] = SlowSpeedUp[servo_num] - constants.Delta_Slow_Fast_Speed
        print("Min speed down, Servo",servo_num,"= ",SlowSpeedDown[servo_num])
        print("Min speed up Servo",servo_num,"= ",SlowSpeedUp[servo_num])
        break
      continue
      
    #find time increments for 3 speeds
    time_bet_counts = motor_calibration_sequence(servo_num, SlowSpeedDown[servo_num], enc_counts_time_cal, time_out)
    if time_bet_counts != 0 and time_bet_counts != time_out:
      EncTimeBetCountsSlowDown[servo_num] = round(time_bet_counts * time_between_counts_margin,3)
    print("Time between counts for Servo",servo_num,"at slow speed down:",EncTimeBetCountsSlowDown[servo_num])
    
    time_bet_counts = motor_calibration_sequence(servo_num, SlowSpeedUp[servo_num], enc_counts_time_cal, time_out)
    if time_bet_counts != 0 and time_bet_counts != time_out:
      EncTimeBetCountsSlowUp[servo_num] = round(time_bet_counts * time_between_counts_margin,3)
    print("Time between counts for Servo",servo_num,"at slow speed up:",EncTimeBetCountsSlowUp[servo_num])
    
    time_bet_counts = motor_calibration_sequence(servo_num, MedSpeedDown[servo_num], enc_counts_time_cal, time_out)
    if time_bet_counts != 0 and time_bet_counts != time_out:
      EncTimeBetCountsMedDown[servo_num] = round(time_bet_counts * time_between_counts_margin,3)
    print("Time between counts for Servo",servo_num,"at medium speed down:",EncTimeBetCountsMedDown[servo_num])
    
    time_bet_counts = motor_calibration_sequence(servo_num, MedSpeedUp[servo_num], enc_counts_time_cal, time_out)
    if time_bet_counts != 0 and time_bet_counts != time_out:
      EncTimeBetCountsMedUp[servo_num] = round(time_bet_counts * time_between_counts_margin,3)
    print("Time between counts for Servo",servo_num,"at medium speed up:",EncTimeBetCountsMedUp[servo_num])    
    
    time_bet_counts = motor_calibration_sequence(servo_num, FastSpeedDown[servo_num], enc_counts_time_cal, time_out)
    if time_bet_counts != 0 and time_bet_counts != time_out:
      EncTimeBetCountsFastDown[servo_num] = round(time_bet_counts * time_between_counts_margin,3)
    print("Time between counts for Servo",servo_num,"at fast speed down:",EncTimeBetCountsFastDown[servo_num])
    
    time_bet_counts = motor_calibration_sequence(servo_num,FastSpeedUp[servo_num], enc_counts_time_cal, time_out)
    if time_bet_counts != 0 and time_bet_counts != time_out:
      EncTimeBetCountsFastUp[servo_num] = round(time_bet_counts * time_between_counts_margin,3)
    print("Time between counts for Servo",servo_num,"at fast speed up:",EncTimeBetCountsFastUp[servo_num])    
  
  data_output(2,SlowSpeedDown)
  data_output(3,SlowSpeedUp)
  data_output(4,MedSpeedDown)
  data_output(5,MedSpeedUp)
  data_output(6,FastSpeedDown)
  data_output(7,FastSpeedUp)
  data_output(8,EncTimeBetCountsSlowDown)
  data_output(9,EncTimeBetCountsSlowUp)
  data_output(10,EncTimeBetCountsMedDown)
  data_output(11,EncTimeBetCountsMedUp)
  data_output(12,EncTimeBetCountsFastDown)
  data_output(13,EncTimeBetCountsFastUp)
  
  print("Encoder counts",EnCounts)  
  #move_motors_home()
  stop_all_motors(constants.Num_Motors)
  
   
# if __name__=='__main__':
 
#   pwm = PCA9685(0x40, debug=False)
#   pwm.setPWMFreq(50)
#   GPIO.setmode(GPIO.BCM)
#   GPIO.setup(CC.MEGM, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  
#   return_val = []
  
#   #data_output(1,EnCounts)
#   #move_motors_home(30)
#   #move_motor_time(0,20,2)
#   #move_motors_home(30)
#   #encoder_speed_calibration()  
#   initialize()
#   home_motors()
#   motor_speeds = [20,0,0,0]
#   tar_positions = [5,0,0,0]
#   #move_motors_counts(tar_positions,"",motor_speeds)
#   return_val = move_motors_counts(tar_positions,"s")
#   move_motors_home()
#   stop_all_motors(CC.Num_Motors)



 
  
  
 
   
    
