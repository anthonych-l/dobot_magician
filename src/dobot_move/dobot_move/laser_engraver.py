import os
import rclpy
from rclpy.node import Node
from dobot_driver.dobot_handle import bot
import cv2
import time
import sys

MOVJ_XYZ = 1   # Movimiento por articulaciones: para REPOSICIONAR (saltos largos entre figuras).
MOVL_XYZ = 2   # Movimiento lineal (línea recta): solo para los TRAZOS de grabado.

class LaserEngraver(Node):
    def __init__(self):
        super().__init__('laser_engraver')
        
        self.declare_parameter('image_path', '/home/r11/magician_ros2_control_system_ws/src/dobot_move/dobot_move/pictures/prueba6.png')
        # size_mm es el LADO MAYOR del grabado. La escala es UNIFORME (misma en
        # X e Y), así que una imagen rectangular conserva su proporción: NO se
        # estira a un cuadrado. Ej.: imagen 800x400 px con size_mm=60 -> 60x30 mm.
        self.declare_parameter('size_mm', 60.0)
        # Alternativa: caja máxima disponible en mm (0 = usar size_mm). Si se
        # define, el grabado se ajusta DENTRO de la caja sin deformarse.
        self.declare_parameter('max_width_mm', 0.0)
        self.declare_parameter('max_height_mm', 0.0)
        self.declare_parameter('offset_x', 182.0)
        self.declare_parameter('offset_y', 107.0)
        self.declare_parameter('z_focal', -50.0)
        self.declare_parameter('z_safe', 0.0) 
        self.declare_parameter('coord_velocity', 12.0)
        self.declare_parameter('coord_acceleration', 50.0)
        # Separación de parámetros para el efector (Punto 5)
        self.declare_parameter('effector_velocity', 50.0)
        self.declare_parameter('effector_acceleration', 50.0)
        
        self.get_logger().info('Iniciando nodo grabador láser...')

    def process_image(self, image_path, size_mm):
        if not os.path.exists(image_path):
            self.get_logger().error(f'Imagen no encontrada: {image_path}')
            return None, None, None, None
            
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f'Fallo al cargar la imagen con OpenCV: {image_path}')
            return None, None, None, None

        # Otsu elige el umbral automáticamente: no pierde líneas claras/grises como
        # el umbral fijo 127 (otra causa de detalle faltante en figuras complejas).
        _, bw = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # Cerrar pequeños huecos para unir trazos casi-conectados ("poco cerrados").
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, kernel)

        # ESQUELETIZAR: adelgazar cada trazo a 1 px para seguir su CENTRO (una sola
        # pasada por línea) en vez de contornear sus dos bordes (que dibujaba cada
        # línea doble). Es lo correcto para dibujos de línea (line-art).
        # NOTA: asume trazos, no rellenos sólidos (un relleno se reduciría a su eje).
        traced = bw
        try:
            traced = cv2.ximgproc.thinning(bw)
        except Exception as e:
            self.get_logger().warn(f"Sin thinning (ximgproc no disponible): uso contorno. {e}")

        # RETR_LIST recupera TODOS los trazos (externos E internos). Con RETR_EXTERNAL
        # se perdía el detalle interior (melena, ojos, líneas dentro de la silueta).
        contours, _ = cv2.findContours(traced, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self.get_logger().error('No se detectaron trazos en la imagen tras procesarla.')
            return None, None, None, None

        height, width = img.shape

        # Escala UNIFORME (idéntica en X e Y) -> la proporción de la imagen se
        # conserva SIEMPRE y una imagen rectangular sale rectangular.
        max_w_mm = float(self.get_parameter('max_width_mm').value)
        max_h_mm = float(self.get_parameter('max_height_mm').value)
        if max_w_mm > 0.0 or max_h_mm > 0.0:
            # Ajustar dentro de la caja disponible (el lado más restrictivo manda).
            ratios = []
            if max_w_mm > 0.0:
                ratios.append(max_w_mm / width)
            if max_h_mm > 0.0:
                ratios.append(max_h_mm / height)
            scale = min(ratios)
        else:
            # size_mm dimensiona el lado MAYOR; el menor queda proporcional.
            scale = size_mm / max(width, height)

        self.get_logger().info(
            f'Imagen {width}x{height} px -> grabado {width * scale:.1f} x '
            f'{height * scale:.1f} mm (proporción conservada).')

        # Descartar contornos de ruido demasiado pequeños (< ~1.5 mm de perímetro),
        # para no quemar motas sueltas en imágenes con detalle.
        min_perimeter_px = (1.5 / scale) if scale > 0 else 0.0

        simplified_contours = []
        for cnt in contours:
            if cv2.arcLength(cnt, True) < min_perimeter_px:
                continue
            epsilon = 0.002 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            if len(approx) >= 2:
                simplified_contours.append(approx)

        if not simplified_contours:
            self.get_logger().error('Todos los contornos quedaron filtrados por tamaño.')
            return None, None, None, None

        simplified_contours = self.order_contours(simplified_contours)
        self.get_logger().info(f'Procesamiento exitoso. Se grabarán {len(simplified_contours)} contornos.')
        return simplified_contours, width, height, scale

    def order_contours(self, contours):
        # Ordenar por vecino más cercano: cada contorno empieza cerca de donde
        # terminó el anterior. Menos viajes en vacío -> grabado más rápido y
        # menos reposicionamientos largos del brazo.
        remaining = list(contours)
        ordered = []
        cur = (0.0, 0.0)
        while remaining:
            best = min(
                range(len(remaining)),
                key=lambda i: (remaining[i][0][0][0] - cur[0]) ** 2
                            + (remaining[i][0][0][1] - cur[1]) ** 2)
            cnt = remaining.pop(best)
            ordered.append(cnt)
            cur = (float(cnt[0][0][0]), float(cnt[0][0][1]))
        return ordered

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

        # Aviso preventivo: si alguna esquina del grabado queda fuera del alcance
        # del Magician (~315 mm de radio), el firmware congelará la cola con
        # alarma a mitad de trazo y el dibujo saldrá incompleto/deformado.
        half_x = (h / 2) * scale   # la altura de la imagen se traza sobre el eje X del robot
        half_y = (w / 2) * scale   # el ancho de la imagen se traza sobre el eje Y del robot
        r_far = ((abs(offset_x) + half_x) ** 2 + (abs(offset_y) + half_y) ** 2) ** 0.5
        if r_far > 315.0:
            self.get_logger().warn(
                f'La esquina más lejana del grabado queda a {r_far:.0f} mm de la base '
                f'(límite ~315 mm): reduce size_mm o acerca offset_x/offset_y.')

        # ENFOQUE CON COLA (queue=True): se encolan todos los comandos y el firmware
        # los ejecuta en orden. La sincronización/fin se hace con índices ABSOLUTOS de
        # la cola (queuedCmdIndex), sin asumir que clear_queue reinicie el contador a 0.
        try:
            self.get_logger().info("Limpiando alarmas y cola...")
            bot.clear_alarms_state()
            time.sleep(0.5)
            bot.clear_queue()
            time.sleep(0.5)
            bot.start_queue()

            last_index = None        # queuedCmdIndex del último comando encolado con éxito
            OUTSTANDING_MAX = 32     # máx. de comandos por delante del que se ejecuta

            def remember(result):
                # Cada comando encolado devuelve su queuedCmdIndex.
                # Dependiendo de la función, puede ser un int o una lista [index].
                nonlocal last_index
                if isinstance(result, (list, tuple)) and len(result) > 0:
                    last_index = result[0]
                elif isinstance(result, int):
                    last_index = result

            def read_current_index():
                idx = bot.get_current_queue_index()
                if isinstance(idx, (list, tuple)):
                    idx = idx[0] if len(idx) > 0 else None
                return idx if isinstance(idx, int) else None

            def active_alarm_codes():
                # Lista de códigos de alarma activos. La cola del firmware se CONGELA
                # cuando salta cualquiera; así sabemos cuál es y por qué se detuvo.
                state = bot.get_alarms_state()
                codes = []
                if isinstance(state, (list, tuple)):
                    for i, byte in enumerate(state):
                        for j in range(8):
                            if int(byte) & (1 << j):
                                codes.append(8 * i + j)
                return codes

            def throttle():
                # Esperar a no encolar demasiado por delante (evita desbordar la cola).
                # Tope por tiempo para no colgarnos nunca.
                if last_index is None:
                    return
                start = time.time()
                while rclpy.ok():
                    cur = read_current_index()
                    if cur is not None and (last_index - cur) < OUTSTANDING_MAX:
                        break
                    if time.time() - start > 120.0:
                        break
                    rclpy.spin_once(self, timeout_sec=0.05)

            # --- helpers de sincronización láser/movimiento ---
            def queue_move(mode, x, y, z):
                # Encola un movimiento y devuelve su índice en la cola (o None).
                remember(bot.set_point_to_point_command(mode, x, y, z, 0.0, queue=True))
                return last_index

            def wait_queue_reach(target_idx, stall_timeout=15.0):
                # Espera a que la cola EJECUTE hasta target_idx (ese movimiento terminó).
                # True si llegó; False si salta una alarma o el índice no avanza en
                # stall_timeout s (atasco). No usa tope absoluto: una traza larga sigue
                # avanzando el índice, así que no se aborta por error.
                if target_idx is None:
                    return False
                last = None
                last_change = time.time()
                while rclpy.ok():
                    cur = read_current_index()
                    if cur is not None:
                        if cur >= target_idx:
                            return True
                        if cur != last:
                            last = cur
                            last_change = time.time()
                    stalled = time.time() - last_change
                    if stalled > 2.0 and active_alarm_codes():
                        return False
                    if stalled > stall_timeout:
                        return False
                    rclpy.spin_once(self, timeout_sec=0.05)
                return False

            def point_to_robot(point):
                px, py = point[0]
                rob_y = offset_y - ((px - (w / 2)) * scale)
                rob_x = offset_x + ((py - (h / 2)) * scale)
                return rob_x, rob_y

            # Parámetros de velocidad (encolado) y láser apagado (INMEDIATO, seguridad).
            remember(bot.set_point_to_point_coordinate_params(
                coord_vel, eff_vel, coord_acc, eff_acc, queue=True))
            bot.set_end_effector_laser(False, False, queue=False)

            # Posición segura inicial (reposicionamiento -> MOVJ)
            queue_move(MOVJ_XYZ, offset_x, offset_y, z_safe)

            total = len(contours)
            for c_idx, cnt in enumerate(contours):
                if len(cnt) < 2:
                    continue

                # 1) Reposicionar al PRIMER punto (MOVJ, láser apagado) y ESPERAR a que
                #    el robot llegue de verdad antes de tocar el láser.
                rob_x, rob_y = point_to_robot(cnt[0])
                idx_start = queue_move(MOVJ_XYZ, rob_x, rob_y, z_focal)
                if not wait_queue_reach(idx_start):
                    codes = active_alarm_codes()
                    self.get_logger().warn(
                        f"Contorno {c_idx+1}/{total}: no se alcanzó el inicio "
                        f"(alarma {[hex(c) for c in codes]}); se omite.")
                    bot.set_end_effector_laser(False, False, queue=False)
                    bot.clear_alarms_state()
                    continue

                # 2) Robot YA en el inicio -> LÁSER ON inmediato (perfectamente sincronizado).
                bot.set_end_effector_laser(True, True, queue=False)

                # 3) Encolar toda la traza del contorno (movimiento suave, MOVL),
                #    cerrando el contorno al volver al primer punto.
                idx_end = idx_start
                for point in list(cnt[1:]) + [cnt[0]]:
                    throttle()
                    rob_x, rob_y = point_to_robot(point)
                    idx_end = queue_move(MOVL_XYZ, rob_x, rob_y, z_focal)

                # 4) Esperar a que termine la traza y LÁSER OFF inmediato (también si se
                #    congela: el comando inmediato apaga el láser aunque la cola esté frita).
                reached = wait_queue_reach(idx_end)
                bot.set_end_effector_laser(False, False, queue=False)
                if reached:
                    self.get_logger().info(f"Contorno {c_idx+1}/{total} completado.")
                else:
                    codes = active_alarm_codes()
                    self.get_logger().warn(
                        f"Contorno {c_idx+1}/{total}: traza interrumpida "
                        f"(alarma {[hex(c) for c in codes]}).")
                    bot.clear_alarms_state()

            # Volver a zona segura
            self.get_logger().info("Grabado finalizado. Volviendo a zona segura...")
            idx_final = queue_move(MOVJ_XYZ, offset_x, offset_y, z_safe)
            wait_queue_reach(idx_final)
            self.get_logger().info("¡Trabajo de grabado finalizado con éxito!")

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
    node.run_engraver()
    # Ahora sí podemos destruir el nodo tranquilamente porque run_engraver espera a que termine.
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
