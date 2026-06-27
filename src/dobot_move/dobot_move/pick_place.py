import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

# Importamos los mensajes necesarios
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import GripperControl

class DobotSecuenciaSegura(Node):
    def __init__(self):
        super().__init__('dobot_secuencia_segura')
        
        # Cliente para mover el brazo (Action)
        self.ptp_client = ActionClient(self, PointToPoint, '/PTP_action')
        
        # Cliente para controlar el gripper (Service)
        self.gripper_client = self.create_client(GripperControl, '/dobot_gripper_service')
        
        # Definición de coordenadas fijas de trabajo [X, Y, Z, R]
        self.pos_home  = [150.0, 0.0, 100.0, 0.0]
        self.pos_pick  = [139.0, 176.0, -8.0, 8.0]
        self.pos_place = [186.0, -129.0, -6.0, -78.0]
        
        # Altura Z de tránsito seguro para evitar colisiones
        self.z_seguro = 100.0 

        # Iniciar la secuencia automática
        self.esperar_sistemas()

    def esperar_sistemas(self):
        self.get_logger().info('Esperando que los servidores del Dobot estén listos...')
        self.ptp_client.wait_for_server()
        while not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio del gripper no disponible, esperando...')
        
        # Comenzamos el primer paso
        self.ir_a_posicion_pick()

    # --- PASO 1: Mover al punto de recogida ---
    def ir_a_posicion_pick(self):
        self.get_logger().info('PASO 1: Moviéndose al punto de recogida (Pick)...')
        self.enviar_movimiento(self.pos_pick, callback_final=self.cerrar_gripper)

    # --- PASO 2: Cerrar Gripper ---
    def cerrar_gripper(self):
        self.get_logger().info('PASO 2: Llegada exitosa. Cerrando el gripper...')
        self.enviar_comando_gripper('close', callback_final=self.retraer_alta_seguridad_pick)

    # --- PASO 3: Elevación de seguridad tras Pick ---
    def retraer_alta_seguridad_pick(self):
        self.get_logger().info('PASO 3: Retrayendo eje Z para tránsito seguro...')
        # Mantenemos X e Y del Pick, pero subimos Z a la zona segura
        pos_segura = [self.pos_pick[0], self.pos_pick[1], self.z_seguro, self.pos_pick[3]]
        self.enviar_movimiento(pos_segura, callback_final=self.ir_a_posicion_place)

    # --- PASO 4: Mover al punto de descarga ---
    def ir_a_posicion_place(self):
        self.get_logger().info('PASO 4: Desplazándose horizontalmente hacia el punto de descarga (Place)...')
        self.enviar_movimiento(self.pos_place, callback_final=self.abrir_gripper)

    # --- PASO 5: Abrir Gripper ---
    def abrir_gripper(self):
        self.get_logger().info('PASO 5: Llegada al destino. Abriendo el gripper...')
        self.enviar_comando_gripper('open', callback_final=self.retraer_alta_seguridad_place)

    # --- PASO 6: Elevación de seguridad tras Place ---
    def retraer_alta_seguridad_place(self):
        self.get_logger().info('PASO 6: Retrayendo eje Z antes de ir a Home...')
        pos_segura = [self.pos_place[0], self.pos_place[1], self.z_seguro, self.pos_place[3]]
        self.enviar_movimiento(pos_segura, callback_final=self.regresar_a_home)

    # --- PASO 7: Retorno Seguro a Home ---
    def regresar_a_home(self):
        self.get_logger().info('PASO 7: Finalizando secuencia. Regresando a Home...')
        self.enviar_movimiento(self.pos_home, callback_final=self.secuencia_terminada)

    def secuencia_terminada(self):
        self.get_logger().info('¡Secuencia de Pick & Place completada de manera segura!')
        rclpy.shutdown()

    # =======================================================
    # MÉTODOS AUXILIARES PARA ENCADENAR ACCIONES Y SERVICIOS
    # =======================================================
    def enviar_movimiento(self, coordenadas, callback_final):
        goal_msg = PointToPoint.Goal()
        goal_msg.motion_type = 1 
        goal_msg.target_pose = coordenadas
        goal_msg.velocity_ratio = 0.1       # Velocidad moderada por seguridad
        goal_msg.acceleration_ratio = 0.1

        self.future_movimiento = self.ptp_client.send_goal_async(goal_msg)
        
        # Cuando el servidor acepta o rechaza la meta
        def respuesta_meta_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Meta de movimiento rechazada.')
                return
            
            # Si es aceptada, esperamos el resultado final del movimiento
            self.future_resultado = goal_handle.get_result_async()
            self.future_resultado.add_done_callback(lambda f: callback_final())

        self.future_movimiento.add_done_callback(respuesta_meta_callback)

    def enviar_comando_gripper(self, estado, callback_final):
        req = GripperControl.Request()
        req.gripper_state = estado
        req.keep_compressor_running = True if estado == 'close' else False
        
        future_service = self.gripper_client.call_async(req)
        
        # Cuando el servicio responde, se ejecuta el siguiente paso de la secuencia
        def respuesta_servicio_callback(future):
            try:
                future.result()
                time.sleep(1.0) # Pequeña pausa física para asegurar el agarre/soltado
                callback_final()
            except Exception as e:
                self.get_logger().error(f'Fallo al llamar al servicio del gripper: {e}')

        future_service.add_done_callback(respuesta_servicio_callback)


def main(args=None):
    rclpy.init(args=args)
    nodo = DobotSecuenciaSegura()
    rclpy.spin(nodo)

if __name__ == '__main__':
    main()