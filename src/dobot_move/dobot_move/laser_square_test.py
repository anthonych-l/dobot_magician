import rclpy
from rclpy.node import Node
from dobot_driver.dobot_handle import bot
import time

# Valor del protocolo Dobot para movimiento lineal cartesiano (MOVL_XYZ)
MOVL_XYZ = 2

# --- AJUSTA ESTOS VALORES A TU SETUP ANTES DE EJECUTAR ---
Z_SAFE = 0.0      # altura segura (mm) para desplazarse SIN tocar la superficie, láser apagado
Z_ENGRAVE = -50.0  # altura de grabado (mm) - DEBES calibrarla físicamente con tu superficie
R = 0.0            # rotación de la muñeca, normalmente irrelevante para láser

# Vértices del cuadrado en el plano XY (mm), en el sistema de coordenadas del robot.
# Cuadrado de 20mm x 20mm (2cm x 2cm). Ajusta estos 4 puntos a una zona segura
# y alcanzable de tu espacio de trabajo (verifica que caigan sobre tu superficie real).
SQUARE_POINTS = [
    (200.0, -10.0),
    (200.0,  10.0),
    (220.0,  10.0),
    (220.0, -10.0),
]

MOVE_LATENCY = 1.0   # segundos de espera tras cada movimiento PTP (ajusta si el brazo aún se mueve)
LASER_LATENCY = 0.3  # segundos de espera tras encender/apagar el láser

# Velocidad/aceleración del end-effector en el espacio cartesiano (mm/s y mm/s^2 aprox).
# Estos valores son de ejemplo - bajos a propósito para una primera prueba segura.
# Si el trazo sale muy tenue (poco quemado) bajalos mas; si tarda demasiado, subelos
# con cuidado y de poco en poco.
COORD_VELOCITY = 50.0
COORD_ACCELERATION = 50.0
EFFECTOR_VELOCITY = 50.0
EFFECTOR_ACCELERATION = 50.0


class LaserSquareTest(Node):

    def __init__(self):
        super().__init__('laser_square_test')
        self.get_logger().info('Iniciando prueba de grabado: trazo de un cuadrado.')
        self.run_square()

    def move_to(self, x, y, z):
        bot.set_point_to_point_command(MOVL_XYZ, x, y, z, R, queue=False)
        time.sleep(MOVE_LATENCY)

    def laser_on(self):
        bot.set_end_effector_laser(True, True, queue=False)
        time.sleep(LASER_LATENCY)

    def laser_off(self):
        bot.set_end_effector_laser(False, False, queue=False)
        time.sleep(LASER_LATENCY)

    def run_square(self):
        first_x, first_y = SQUARE_POINTS[0]

        self.get_logger().info('Configurando velocidad/aceleracion del end-effector...')
        bot.set_point_to_point_coordinate_params(
            COORD_VELOCITY, EFFECTOR_VELOCITY,
            COORD_ACCELERATION, EFFECTOR_ACCELERATION,
            queue=False
        )
        time.sleep(0.2)

        self.get_logger().info('Moviendo a posicion segura sobre el primer vertice...')
        self.move_to(first_x, first_y, Z_SAFE)

        self.get_logger().info('Bajando a altura de grabado...')
        self.move_to(first_x, first_y, Z_ENGRAVE)

        self.get_logger().info('Encendiendo laser...')
        self.laser_on()

        # Recorre los demas vertices en linea recta (MOVL), con el laser ya encendido
        for (x, y) in SQUARE_POINTS[1:] + [SQUARE_POINTS[0]]:
            self.get_logger().info(f'Trazando hacia ({x}, {y})...')
            self.move_to(x, y, Z_ENGRAVE)

        self.get_logger().info('Apagando laser...')
        self.laser_off()

        self.get_logger().info('Subiendo a posicion segura...')
        self.move_to(first_x, first_y, Z_SAFE)

        self.get_logger().info('Prueba completada.')


def main(args=None):
    rclpy.init(args=args)
    node = LaserSquareTest()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()