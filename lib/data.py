from enum import Enum

class CalibrationMode(Enum):
  ENCODER_DATA = 1
  SLOW_SPEED_DOWN_DATA = 2
  SLOW_SPEED_UP_DATA = 3
  MED_SPEED_DOWN_DATA = 4
  MED_SPEED_UP_DATA = 5
  FAST_SPEED_DOWN_DATA = 6
  FAST_SPEED_UP_DATA = 7
  ENC_TIME_BET_COUNTS_SLOW_DOWN_DATA = 8
  ENC_TIME_BET_COUNTS_SLOW_UP_DATA = 9
  ENC_TIME_BET_COUNTS_MED_DOWN_DATA = 10
  ENC_TIME_BET_COUNTS_MED_UP_DATA = 11
  ENC_TIME_BET_COUNTS_FAST_DOWN_DATA = 12
  ENC_TIME_BET_COUNTS_FAST_UP_DATA = 13


class CalibrationData:
  def __init__(self) -> None:
    pass
  def save() -> None:
    pass
  def read() -> None:
    pass



def save_calibration(save_mode, data_out):
  """
  Save input calibration data to respective row in csv file
  """
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


def get_calibration(open_mode) -> list[int]:
  """
  Read data from a text file
  """
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
      return []
      

  return []