import asyncio
from lib.state import StateMachine
from lib.utils import log

def run():
    log.info("""
    Running manual mode
    Controls (press button): 
        i: idle state
        r: random state
        s: sequence state
        c: service state
        q: reboot
    """)


    sm = StateMachine(manual=True)

    # run state machine forever
    while True:
        asyncio.run(sm.check())