from dobot_driver.dobot_handle import bot
import time

bot.set_end_effector_laser(True, True, queue=False)
time.sleep(2)
bot.set_end_effector_laser(False, False, queue=False)