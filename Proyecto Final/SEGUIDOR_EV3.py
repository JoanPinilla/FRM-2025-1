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
