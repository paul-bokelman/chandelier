import asyncio
from lib.controller import MotorController
from lib.utils import log

def encoders_mode():
    log.info("Running encoders test mode")
    mc = MotorController()
    asyncio.run(mc.move_all_home(-0.2))
    asyncio.run(mc.move_all(0.1, 0.5))
    asyncio.run(mc.move_all_home(-0.2))
    mc.stop_all_motors()

    disabled_motors = [m.channel for m in mc.motors if m.disabled]

    if len(disabled_motors) > 0:
        log.error(f"Disabled motors: {disabled_motors}")
    else:
        log.info("All motors are enabled")