import os
import rclpy
from rclpy.node import Node
from dobot_driver.dobot_handle import bot
import cv2
import time
import sys

MOVL_XYZ = 2

class LaserEngraver(Node):
    def __init__(self):
        super().__init__('laser_engraver')
        
        self.declare_parameter('image_path', '/home/r11/magician_ros2_control_system_ws/src/dobot_move/dobot_move/pictures/prueba2.png')
        self.declare_parameter('size_mm', 60.0)
        self.declare_parameter('offset_x', 206.0)
        self.declare_parameter('offset_y', 25.0)
        self.declare_parameter('z_focal', -15.0)
        self.declare_parameter('z_safe', 0.0) 
        self.declare_parameter('coord_velocity', 50.0)
        self.declare_parameter('coord_acceleration', 50.0)
        # Separación de parámetros para el efector (Punto 5)
        self.declare_parameter('effector_velocity', 50.0)
        self.declare_parameter('effector_acceleration', 50.0)
        
        self.get_logger().info('Iniciando nodo grabador láser...')
        self.run_engraver()

    def process_image(self, image_path, size_mm):
        if not os.path.exists(image_path):
            self.get_logger().error(f'Imagen no encontrada: {image_path}')
            return None, None, None, None
            
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f'Fallo al cargar la imagen con OpenCV: {image_path}')
            return None, None, None, None

        _, bw = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            self.get_logger().error('No se detectaron contornos en la imagen tras binarizarla.')
            return None, None, None, None

        simplified_contours = []
        for cnt in contours:
            epsilon = 0.002 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            simplified_contours.append(approx)
        
        height, width = img.shape
        scale = size_mm / max(width, height)
        
        self.get_logger().info(f'Procesamiento exitoso. Se grabarán {len(simplified_contours)} contornos.')
        return simplified_contours, width, height, scale

    def run_engraver(self):
        image_path = self.get_parameter('image_path').value
        size_mm = self.get_parameter('size_mm').value
        offset_x = self.get_parameter('offset_x').value
        offset_y = self.get_parameter('offset_y').value
        z_focal = self.get_parameter('z_focal').value
        z_safe = self.get_parameter('z_safe').value
        coord_vel = self.get_parameter('coord_velocity').value
        coord_acc = self.get_parameter('coord_acceleration').value
        eff_vel = self.get_parameter('effector_velocity').value
        eff_acc = self.get_parameter('effector_acceleration').value

        contours, w, h, scale = self.process_image(image_path, size_mm)
        # Punto 3: Log mudo solucionado. Si no hay contornos, el error se loguea en process_image.
        if not contours:
            return

        # Punto 4: Manejo de excepciones robusto
        try:
            # Punto 2: Asegurar la limpieza dando tiempo y manejando fallos silenciosos
            self.get_logger().info("Limpiando alarmas y cola...")
            bot.clear_alarms_state()
            time.sleep(0.5)
            bot.clear_queue()
            time.sleep(0.5)
            bot.start_queue()
            
            # Reset the local command counter
            queued_commands = 0

            def read_queue_index():
                # get_current_queue_index devuelve un entero escalar (queuedCmdIndex)
                # o None si la respuesta no se pudo leer/parsear.
                idx = bot.get_current_queue_index()
                if isinstance(idx, (list, tuple)):
                    idx = idx[0] if len(idx) > 0 else None
                return idx

            def wait_for_queue_space():
                # Limitar a ~100 comandos pendientes. Tope por tiempo: si el índice
                # no se puede leer, avanzamos en vez de colgarnos para siempre (lo que
                # dejaría al robot congelado con el láser encendido).
                start = time.time()
                while rclpy.ok():
                    current_idx = read_queue_index()
                    if current_idx is not None and queued_commands - current_idx < 100:
                        break
                    if time.time() - start > 2.0:
                        break
                    rclpy.spin_once(self, timeout_sec=0.05)

            bot.set_point_to_point_coordinate_params(
                coord_vel, eff_vel,
                coord_acc, eff_acc,
                queue=True
            )
            queued_commands += 1

            # Asegurar láser apagado al inicio
            bot.set_end_effector_laser(False, False, queue=True)
            queued_commands += 1

            bot.set_point_to_point_command(MOVL_XYZ, offset_x, offset_y, z_safe, 0.0, queue=True)
            queued_commands += 1
            bot.set_point_to_point_command(MOVL_XYZ, offset_x, offset_y, z_focal, 0.0, queue=True)
            queued_commands += 1

            for cnt in contours:
                if len(cnt) < 2:
                    continue
                    
                for i, point in enumerate(cnt):
                    wait_for_queue_space()
                    
                    px, py = point[0]
                    rob_y = offset_y - ((px - (w/2)) * scale)
                    rob_x = offset_x + ((py - (h/2)) * scale)

                    if i == 0:
                        # Ir al primer punto del contorno con láser apagado (ya está apagado desde el loop anterior)
                        # Punto 6: Evitamos doble comando de apagado redundante
                        bot.set_point_to_point_command(MOVL_XYZ, rob_x, rob_y, z_focal, 0.0, queue=True)
                        queued_commands += 1
                        
                        # Encender láser para empezar el trazo
                        bot.set_end_effector_laser(True, True, queue=True)
                        queued_commands += 1
                    else:
                        bot.set_point_to_point_command(MOVL_XYZ, rob_x, rob_y, z_focal, 0.0, queue=True)
                        queued_commands += 1

                # Apagar láser al terminar este contorno
                bot.set_end_effector_laser(False, False, queue=True)
                queued_commands += 1

            # Volver a zona segura
            wait_for_queue_space()
            bot.set_point_to_point_command(MOVL_XYZ, offset_x, offset_y, z_safe, 0.0, queue=True)
            queued_commands += 1
            
            # Iniciar ejecución (ya iniciado)
            self.get_logger().info(f"Enviando {queued_commands} comandos a la cola de hardware. Ejecución en progreso...")

            # Punto 1: Esperar activamente a que el robot vacíe la cola antes de destruir el nodo
            self.get_logger().info("Esperando a que el robot complete el trabajo...")
            last_idx = -1
            stall_start = time.time()
            while rclpy.ok():
                current_idx = read_queue_index()
                if current_idx is not None:
                    # La cola del Dobot empieza en 0 y sube. Si current_idx alcanza el total enviado (o casi), terminó.
                    if current_idx >= queued_commands - 1:
                        self.get_logger().info("¡Trabajo de grabado finalizado con éxito!")
                        break
                    # Reiniciar el detector de atasco cada vez que la cola avanza.
                    if current_idx != last_idx:
                        last_idx = current_idx
                        stall_start = time.time()

                # Seguridad: si la cola no avanza en mucho tiempo, salir para
                # apagar el láser (en el finally) en vez de quedarnos colgados.
                if time.time() - stall_start > 15.0:
                    self.get_logger().warn("La cola no avanza; finalizando por seguridad.")
                    break

                # Checkeamos rclpy.spin_once por si hay callbacks pendientes o si se presiona Ctrl+C
                rclpy.spin_once(self, timeout_sec=0.5)

        except Exception as e:
            self.get_logger().error(f"Fallo crítico durante la ejecución: {str(e)}")
        finally:
            # Apagar el láser incondicionalmente en modo NO COLA para seguridad absoluta
            self.get_logger().info("Asegurando apagado seguro del láser...")
            try:
                bot.set_end_effector_laser(False, False, queue=False)
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = LaserEngraver()
    # Ahora sí podemos destruir el nodo tranquilamente porque run_engraver espera a que termine.
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
