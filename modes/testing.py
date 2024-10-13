from lib.utils import log
from lib.controller import MotorController
# todo: populate with tests
#/ could be manual tests; tests that move motors to specific positions but pause for user input (physical verification)

def run(controller: MotorController):
    log.info("Running testing mode")
    