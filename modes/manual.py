import asyncio
from lib.state import StateMachine
from lib.utils import log

def run():
    log.info("Running manual mode")
    sm = StateMachine(manual=True)

    # run state machine forever
    while True:
        asyncio.run(sm.check())