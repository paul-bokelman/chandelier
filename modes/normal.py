import asyncio
from lib.state import StateMachine
from lib.utils import log

def run():
    log.info("Running normal mode")
    sm = StateMachine()

    # run state machine forever
    while True:
        asyncio.run(sm.check())