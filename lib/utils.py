from termcolor import colored

def info(msg):
    print(colored(msg, 'grey'))

def success(msg):
    print(colored(msg, 'green'))

def error(msg):
    print(colored(msg, 'red'))

def warning(msg):
    print(colored(msg, 'yellow'))

def debug(msg):
    print(colored(msg, 'blue'))