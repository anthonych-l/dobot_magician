import sys
import time

# Añadimos la ruta de tu workspace para que encuentre la libreria
sys.path.append('/magician_ros2_control_system_ws/src/dobot_driver/dobot_driver') 

try:
    from interface import Interface
    print("Intentando conectar al puerto /dev/ttyACM0...")
    bot = Interface('/dev/ttyACM0')
    print("¡Conexión exitosa! El brazo está respondiendo.")

    pose = bot.get_pose()
    print(f"Posición actual del DOBOT: {pose}")
    bot.close()

except Exception as e:
    print(f"FALLA DE COMUNICACIÓN: {e}")