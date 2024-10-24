import asyncio
from lib.state import StateMachine
from lib.controller import MotorController
from lib.utils import log

def run(controller: MotorController):
    log.info("Running testing mode")
    sm = StateMachine(controller=controller)

    # run state machine forever
    while True:
        asyncio.run(sm.check())