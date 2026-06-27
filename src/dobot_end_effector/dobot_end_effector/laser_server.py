from dobot_msgs.srv import LaserControl
import rclpy
from rclpy.node import Node
from dobot_driver.dobot_handle import bot
import time


class LaserService(Node):

    def __init__(self):
        super().__init__('dobot_laser_srv')
        self.srv = self.create_service(LaserControl, 'dobot_laser_service', self.laser_callback)
        self.laser_latency = 0.500


    def laser_callback(self, request, response):

        if request.enable_laser == True:
            bot.set_end_effector_laser(True, True, queue=False)
            time.sleep(self.laser_latency)

        elif request.enable_laser == False:
            bot.set_end_effector_laser(False, False, queue=False)
            time.sleep(self.laser_latency)

        else:
            response.success = False
            response.message = "Invalid service request"
            return response

        response.success = True
        response.message = "Laser state has been changed"
        return response




def main(args=None):
    rclpy.init(args=args)

    minimal_service = LaserService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
