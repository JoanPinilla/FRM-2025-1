# Laboratorio No. 3

Integrantes:
- Eduardo Cuadros Montealegre
- Andrés S Serna
- Joan Pinilla
- Nicolás Moreno

# Tabla de contenido

- 1. Búsqueda bibliográfica
    - 1.1. Características de navegación 
    - 1.2. Investigaciones destacadas
    - 1.3. Algoritmos de planificación de rutas
    - 1.4. Algoritmos Bug
    - 1.5. Algoritmos Maze 
- 2. Misión 1
    - 2.1. Algoritmo solución
    - 2.2. Video
- 3. Misión 2
    - 3.1. Algoritmo solución
    - 3.2. Video

---

# 1. Búsqueda bibliográfica


**1.1. Características de la navegación**
---




**1.2. Investigaciones destacadas**
---



**1.3. Algoritmos de planificación de rutas**
---




**1.4. Algoritmos Bug**
---




**1.5. Algoritmo de solución de laberintos**
---




# 2. Misión 1 

**2.1. Descripción**
La misión consiste en utilizar un algoritmo Bug para navegar desde un punto de partida hasta una meta con dos obstáculos interceptando la línea que une ambos puntos. El espacio de trabajo escogido, visualizado en la siguiente foto, consta de una línea negra trazada entre el inicio y la meta. Dicha línea hace parte del algoritmo de solución usado, el cual es el *Bug 2*.

![espacio1](https://github.com/user-attachments/assets/9db73f86-8878-4133-b359-8a46799abb41)


El robot se preparó con 4 sensores: Un sensor de color apuntando al piso encargado de seguir la línea negra; un sensor ultrasónico apuntando al frente y detectando obstáculos cercanos; sensor infrarrojo apuntando hacia el lado izquierdo actuando cuando el robot bordea un obstáculo para verificar distancias adecuadas entre el robot y este; y un giroscopio para hacer un control más preciso del cambio de orientación del robot. Dicha configuración se visualiza en la siguiente foto.

![Imagen de WhatsApp 2025-06-16 a las 18 53 03_9228d340](https://github.com/user-attachments/assets/2f95dd4b-9cfe-4249-ad3b-79777d5bd5bc)


**2.2. Algoritmo de solución**

El algoritmo Bug 2 implementado para esta misión es el siguiente:

~~~
Inicio:
    - Resetear ángulo
    - Definir B y C como puertos de motores
    - EncontrarLinea()

    Mientras lectura de color diferente a meta (rojo)
        Si distancia frontal (ultrasonido) < 15 cm entonces
            Ejecución EsquivarObstáculo()
        Sino
            Ejecución SeguirLinea()
    Fin Mientras

    Detener motores
    Mostrar éxito
Fin
~~~

Dentro de este algoritmo se definieron una serie de funciones, las cuales son:

~~~
Función EncontrarLinea():
    Iniciar rotación antihoraria
    Si lectura de color = negro
        Parar rotación
        Resetear ángulo
    Fin Si
Fin Función

Función SeguirLinea():
    Si lectura de color  = negro
        Avanzar hacia la izquierda
    Sino 
        Avanzar hacia la derecha
    Fin Si
Fin Función

Función EsquivarObstáculo():
    RotarDerecha(80):
    Mientras color ≠ negro
        Si distancia lateral(infrarrojo) < 15%
            RotarDerecha(5)
            Avanzar con velocidad 12% y ángulo de 7° hacia izquierda
        Sino
            Avanzar con velocidad 12% y ángulo de 20° hacia derecha  
    RotarDerecha(30)
    EncontrarLinea()
Fin Función

Función RotarDerecha(grados):
    Resetear ángulo
    Mientras  ángulo < grados
        Avanzar con velocidad 12% y ángulo 15° hacia derecha
    Fin Mientras
Fin Función
~~~

Este algoritmo, en resumen, inicia configurando los motores y reiniciando el giroscopio. Luego, el robot gira hasta encontrar la línea negra en el suelo. Mientras no haya llegado a la meta (color rojo), sigue la línea o esquiva obstáculos si detecta algo al frente con el sensor ultrasónico. Para esquivarm bordea el obstáculo usando el sensor infrarrojo lateral y el giroscopio, hasta volver a encontrar la línea. Al llegar a la meta, se detiene y muestra una señal de éxito.

Aunque la misión se desarrolló de manera satisfactoria, se tuvieron varios problemas con la velocidad de los motores y las lecturas de los sensores. Por esta razón se hizo un trabajo iterativo de ajustar cada valor según el comportamiento real de cada componente.

**2.3. Video**


[![Demo Video](Laboratorio_No3/mision1/thumbnail.png)](Laboratorio_No3/mision1/video_mision1.mp4)

# 3. Misión 2 

**3.1. Descripción**
_Utilizar uno de los algoritmos MAZE para ir desde la entrada P1 hasta la salida P2 del laberinto._

En este caso, el algoritmo a usar será el de mano en la pared.

**3.2. Algoritmo de solución**
Se implementó el algoritmo de mano en la pared con un sensor de toque en la parte frontal del EV3 y un sensor infrarrojo en la parte posterior, apuntando a la derecha del carro.
El código mostrado a continuación esta destinado a funcionar en un EV3 que tenga el sistema operativo EV3dev, e implementado con python.

```python
#!/usr/bin/env python3
from ev3dev2.sensor.lego import InfraredSensor, TouchSensor, GyroSensor
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_D
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3
import time

# Inicializa sensores y motores
ir = InfraredSensor(INPUT_2)
touch = TouchSensor(INPUT_1)
gyro = GyroSensor(INPUT_3)
tank = MoveTank(OUTPUT_B, OUTPUT_D)  # Motor Izquierdo y Derecho
gyro.mode = 'GYRO-ANG' #modo del giroscopio (angulo)

# Parametros
WALL_DISTANCE_THRESHOLD = 30  # cm 
SPEED = 30  # Velocidad del motor (0-100)

#Función principal

def follow_wall():
    while True:
        
        ir_distance = ir.proximity  # Distancia actual del infrarrojo a la pared 
        is_touched = touch.is_pressed  # Estado del sensor de toque
	
        # Logica si encuentra una pared frontal
        if is_touched:
            if ir_distance < WALL_DISTANCE_THRESHOLD: # pared a la derecha?
                tank.off()
                gyro.reset()
                tank.on(-SPEED, SPEED) #Giro izquierda
            
            	# Este ciclo espera hasta que el vehiculo gire hasta la posición deseada (90°)
                while abs(gyro.angle) < 79: #° valor modificado para obtener un valor cercano al angulo deseado
                    time.sleep(0.001)
        	
                tank.off()
            else:
                tank.off()
                gyro.reset()
                tank.on(SPEED, -SPEED) #Giro Derecha
            
                while abs(gyro.angle) < 79:
                    time.sleep(0.001)
        	
                tank.on_for_degrees(SPEED, SPEED,730)
                tank.off()
        
        # Pared a la derecha → Movimiento frontal                       
        elif ir_distance < WALL_DISTANCE_THRESHOLD:
            tank.on(SPEED, SPEED)
            
        # No Pared a la derecha → Movimiento en un cuadrado, intentando encontrar la pared
        else:
            gyro.reset()
            tank.on(SPEED, -SPEED)
            
            while abs(gyro.angle) < 79:
                time.sleep(0.001)
            
            tank.on_for_degrees(SPEED, SPEED,730)

        time.sleep(0.1)  # Pequeño delay
        
#main

if __name__ == '__main__':
    try:
        follow_wall()
    except KeyboardInterrupt:
        tank.off()
        print("Program stopped.")
```

En palabras simples, el algoritmo cumple las siguientes condiciones:
* Cuando se activa el sensor de toque y se detecta pared a la derecha con el sensor infrarrojo, el carro gira a la izquierda.
* Cuando se activa el sensor de toque y no se detecta pared a la derecha con el sensor infrarrojo, el carro gira a la derecha.
* Cuando no se detecta pared después de cierto tiempo, el carro gira comienza a hacer una búsqueda, esta consiste en hacer una trayectoria cuadrada hasta encontrar nuevamente una pared.

**3.3. Video**

[![YouTube Video](https://img.youtube.com/vi/ItjqDdtpoGM/mqdefault.jpg)](https://youtu.be/ItjqDdtpoGM)
