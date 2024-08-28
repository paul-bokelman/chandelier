import asyncio
from lib.controller import MotorController
from lib.utils import log

def blank_mode():
    log.info("Running blank test mode")
    mc = MotorController()
    asyncio.run(mc.move_all_home(-0.2))
    asyncio.run(mc.move_all(0.1, 0.5))
    asyncio.run(mc.move_all_home(-0.2))
    mc.stop_all_motors()
    log.info("Blank test mode complete")