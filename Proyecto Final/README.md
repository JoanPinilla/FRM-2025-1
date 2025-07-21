# PROYECTO FINAL FUNDAMENTOS DE ROBÓTICA MÓVIL
# RUMBA CON EL RUMBA

## INTEGRANTES
* Eduardo Cuadros
* Andrés Serna
* Nicolás Moreno
* Joan Pinilla

## Objetivo
Este proyecto tiene como objetivo la implementación de un sistema colaborativo de locomoción y guiado entre dos plataformas móviles autónomas: un robot iRobot Create 2 (Roomba) y un robot LEGO EV3, mediante comunicación no estructurada basada en señales ultrasónicas.

El Roomba será responsable de liderar el trayecto, emitiendo señales ultrasónicas de forma continua. El LEGO EV3, equipado con un sensor ultrasónico, interpretará esta señal como referencia para ajustar su desplazamiento y mantener el seguimiento del Roomba.

## Materiales
* iRobot Create 2 (Roomba)
* LEGO Mindstorms EV3
* NVIDIA Jetson Nano (ROS 2, control del Roomba)
* Sensor ultrasónico (en el EV3 y en el Roomba)

## Herramientas de softaware utilizadas

## Resultados obtenidos con soporte de imagenes y videos

## Dificultades

## Scripts utilizados o paquetes creados

Codigo empleado en el EV3, implementado en python para el sistema EV3DEV2.

Lógica de navegación

Seguir señal: 

   Cuando detecta la señal objetivo (con el ultrasónico), avanza recto.

Evitar obstáculos: 

   Si encuentra un obstáculo frontal:
   Retrocede un poco
   Gira aproximadamente 80 grados
   Intenta rodear el obstáculo manteniendo una distancia óptima de la pared lateral

Búsqueda aleatoria: 
   
   Cuando no detecta la señal, alterna entre:
   Avanzar recto por un tiempo
   Girar en su lugar por un tiempo

```python
#!/usr/bin/env python3
from ev3dev2.motor import OUTPUT_B, OUTPUT_C, MoveTank
from ev3dev2.sensor.lego import InfraredSensor, GyroSensor, UltrasonicSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from time import sleep, time as now

# === Motores ===
tank = MoveTank(OUTPUT_B, OUTPUT_C)

# === Sensores ===
ir_front = InfraredSensor(INPUT_3)
ir_side  = InfraredSensor(INPUT_1)
gyro     = GyroSensor(INPUT_2)
usl      = UltrasonicSensor(INPUT_4)
usl.mode = 'US-LISTEN'
gyro.mode = 'GYRO-ANG'

# === Constantes ===
OBSTACLE_THRESHOLD = 17    # distancia frontal
WALL_TOO_CLOSE = 15        # pared lateral demasiado cercana
WALL_OPTIMAL = 20          # zona deseada
WALL_TOO_FAR = 25          # demasiado lejos de la pared
ROTATION_SPEED = 45
FORWARD_SPEED = 70
GIRO_TIMEOUT = 3.5         # tiempo máx de giro en segundos
EVASION_TIMEOUT = 5       # tiempo máx de evasión
M_LINE_TOLERANCE = 5       # tolerancia de ángulo para detectar M-line


# === Variables globales ===
initial_angle = 0
maxahead = 30 #tiempo que se mueve hacia delante en el modo busqueda
maxturn = 10  #tiempo que se gira en el modo busqueda
ahead = 0
turn = maxturn

# === Funciones auxiliares ===

def other_sensor_present():
    return usl.value()

def rotar_derecha(grados):
    print("Girando", grados, "grados")
    gyro.reset()
    sleep(0.2)  # estabilizar
    tank.on(30, -30)
    start_time = now()
    while abs(gyro.angle) < grados:
        if now() - start_time > GIRO_TIMEOUT:
            print("Giro excede el tiempo, abortando")
            break
        sleep(0.01)
    tank.off()

def alejar_de_obstaculo_lateral():
    print("Pared lateral muy cercana. Ajustando ruta...")
    while ir_side.proximity < WALL_TOO_CLOSE:
        tank.on(20, 15)
        sleep(0.05)
    tank.off()
    print("Distancia lateral segura restablecida.")

def seguir_signal():
    print("Siguiendo")
    tank.on(FORWARD_SPEED, FORWARD_SPEED)

def buscar_signal():
    print("Buscando...")
    global ahead, turn
    if ahead < maxahead:          #modo busqueda moverse adelante
            
            tank.on(FORWARD_SPEED, FORWARD_SPEED)
            ahead = ahead + 1
            print("frente",ahead)
            
            if ahead == maxahead:      #cambia a modo giro
                turn = 0
            
    elif turn < maxturn:           #modo busqueda girar
            
            tank.on(-ROTATION_SPEED, ROTATION_SPEED)
            turn = turn +1
            print("giro",turn)
            
            if turn == maxturn:        #cambia a modo adelante
                ahead = 0
     
    sleep(0.05)
        

def retroceder_suavemente(tiempo=0.5, velocidad=-20):
    print("Retrocediendo")
    tank.on(velocidad, velocidad)
    sleep(tiempo)
    tank.off()
    print("Retroceso completado.")

def esquivar_obstaculo():
    print("Iniciando evasion...")
    retroceder_suavemente()
    rotar_derecha(80)  # ángulo más suave
    start_time = now()
    evasión_completada = False

    while True:
        front = ir_front.proximity
        side = ir_side.proximity

        if front < OBSTACLE_THRESHOLD:
            print("Obstaculo detectado de nuevo. Ajustando direccion")
            retroceder_suavemente()
            rotar_derecha(75)
            start_time = now()

        elif side < WALL_TOO_CLOSE:
            print("Muy cerca de la pared lateral. Girando a la derecha.")
            tank.on(20, 15)

        elif side > WALL_TOO_FAR:
            if front > OBSTACLE_THRESHOLD + 10:
                print("Demasiado lejos de la pared y sin obstaculo al frente. Buscando pared...")
                tank.on(15, 20)  # giro fuerte a la izquierda
            else:
                print("Demasiado lejos, pero posible obstaculo al frente. Ajustando suavemente.")
                #retroceder_suavemente()
                tank.on(20, 15)
        else:
            print("Distancia lateral optima. Avanzando recto.")
            tank.on(FORWARD_SPEED, FORWARD_SPEED)

        current_angle = gyro.angle
        if abs(current_angle - initial_angle) < M_LINE_TOLERANCE and other_sensor_present():
            print("Retorno a la M-line con signall presente.")
            evasión_completada = True
            break

        if now() - start_time > EVASION_TIMEOUT:
            print("Evasion muy larga. Abortando.")
            break

        sleep(0.05)

    tank.off()
    if evasión_completada:
        seguir_signal()
    else:
        buscar_signal()

# === Función principal ===

def main():
    print("Iniciando programa Bug 2 mejorado...")
    gyro.reset()
    sleep(0.2)
    global initial_angle
    initial_angle = gyro.angle
    print("Angulo inicial registrado (M-line):", initial_angle)

    while True:
        print("Frontal:", ir_front.proximity, "| Lateral:", ir_side.proximity, "| Signal:", other_sensor_present())

        if other_sensor_present() and ir_front.proximity < OBSTACLE_THRESHOLD:
            print("Signal detectada, pero obstaculo frontal bloquea el paso")
            esquivar_obstaculo()

        elif ir_side.proximity < WALL_TOO_CLOSE:
            alejar_de_obstaculo_lateral()
            continue

        elif ir_front.proximity < OBSTACLE_THRESHOLD:
            print("Obstaculo detectado frontalmente")
            esquivar_obstaculo()

        elif other_sensor_present():
            seguir_signal()

        else:
            buscar_signal()

# === Entrada principal ===

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        tank.off()
        print("Programa interrumpido.")
```


## Autoevalución del equipo y personal
### Autoevaluación de equipo - 5.0
Como equipo, nos evaluamos con una calificación de 5.0. Logramos una colaboración muy buena durante todo el desarrollo del proyecto. Hubo buena comunicación, compromiso por parte de todos y una distribución equilibrada de las tareas. Cada integrante asumió un rol claro y aportó desde sus fortalezas. Supimos resolver los problemas técnicos que aparecieron y llegamos a un sistema funcional que cumplió con los objetivos planteados. Además, nos apoyamos mutuamente en las etapas más exigentes y logramos mantener una buena organización y motivación.

### Autoevaluación de Eduardo Cuadros - 5.0
Considero que hice un excelente trabajo dentro del equipo. Me enfoqué especialmente en la parte de hardware y en el funcionamiento del EV3, participando activamente en las pruebas de seguimiento y asegurándome de que la lectura del sensor ultrasónico fuera estable. Me mantuve comprometido durante todas las etapas del proyecto y colaboré de forma constante con mis compañeros.

### Autoevaluación de Andrés Serna - 5.0
Tuve un papel clave en la parte del Roomba, trabajando en la emisión de la señal ultrasónica y en el control de movimiento. Aporté ideas para resolver varios problemas técnicos y siempre estuve disponible para trabajar en equipo. Me siento satisfecho con mi desempeño y el resultado final del sistema.

### Autoevaluación de Nicolás Moreno - 5.0
Tuve un papel clave en la parte del Roomba, trabajando en la emisión de la señal ultrasónica y en el control de movimiento. Aporté ideas para resolver varios problemas técnicos y siempre estuve disponible para trabajar en equipo. Me siento satisfecho con mi desempeño y el resultado final del sistema.

### Autoevaluación de Joan Pinilla - 5.0
Me involucré desde el inicio en el diseño general del sistema, especialmente en cómo lograr la comunicación entre los dos robots. Aporté soluciones en la integración del software y el control de seguimiento, y estuve pendiente de que todo funcionara bien en conjunto. Me sentí muy motivado y comprometido con el proyecto.

## Bibliografia y enlace de recursos utilizados 
