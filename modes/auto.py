import asyncio
from lib.state import StateMachine
from lib.utils import log

def auto_mode():
    log.info("Running auto test mode")
    log.info("Running normal mode")
    sm = StateMachine(auto=True)

    # run state machine forever
    while True:
        asyncio.run(sm.check())