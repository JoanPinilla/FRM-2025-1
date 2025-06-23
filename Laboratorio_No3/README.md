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
En robótica móvil, existen dos enfoques principales de navegación: la navegación planeada (deliberativa) y la navegación basada en comportamientos (reactiva).

La navegación planificada se basa en un modelo global del entorno, generalmente representado como un mapa completo y estático. Utiliza algoritmos como A* o Dijkstra para calcular rutas óptimas antes de ejecutar el movimiento. Este enfoque es ideal para entornos conocidos y estructurados, donde se requiere una planificación precisa. Sin embargo, su desventaja es la baja adaptabilidad a cambios dinámicos y su mayor requerimiento computacional, lo que reduce la rapidez de respuesta ante imprevistos [1][2].

Por otro lado, la navegación basada en comportamientos permite que el robot tome decisiones en tiempo real utilizando información local capturada por sensores. Se apoya en un conjunto de reglas simples como evitar obstáculos, seguir paredes o dirigirse hacia la meta. Esto le confiere mayor adaptabilidad y rapidez de respuesta, siendo ideal para entornos desconocidos o parcialmente observables. Este enfoque es utilizado comúnmente en robots con capacidades limitadas o que operan en entornos impredecibles [1][3].


**1.2. Investigaciones destacadas**
---
Rodney Brooks es uno de los pioneros en robótica reactiva. Introdujo la arquitectura de subsunción en 1986, la cual está basada en capas jerárquicas de comportamientos que operan sin necesidad de modelos internos del entorno. Este enfoque dio origen a lo que se conoce como IA situada, cuyo principio es que el entorno físico puede ser utilizado directamente como fuente de información en lugar de representarlo simbólicamente [4].

Brooks también es conocido por ser cofundador de iRobot (creador del robot aspirador Roomba) y de Rethink Robotics, responsable de los robots colaborativos como Baxter. Entre sus robots más emblemáticos están Genghis y Attila, los cuales demostraron que comportamientos complejos pueden emerger de reglas simples aplicadas localmente [5][4].

Mark Tilden es reconocido por desarrollar la robótica BEAM (Biology, Electronics, Aesthetics, Mechanics), un enfoque minimalista que evita el uso de microprocesadores y se basa en circuitos analógicos inspirados en redes neuronales simples. Sus robots, diseñados con principios biológicos, son energéticamente eficientes y adaptativos.

Entre sus creaciones más conocidas se encuentran RoboSapien y RoboRaptor, desarrollados durante su colaboración con WowWee. Su trabajo promueve la construcción de robots autónomos y autoestabilizados con componentes simples y económicos, aptos para aplicaciones tanto educativas como prácticas [6][7].

**1.3. Algoritmos de planificación de rutas**
---
Existen diversos algoritmos utilizados para planificar rutas en presencia de obstáculos. Entre los más relevantes se encuentran:

- A* (A estrella): Algoritmo heurístico que busca el camino más corto en un grafo ponderado, combinando el costo real desde el inicio con una estimación heurística del costo restante. Muy eficiente en entornos discretos [2].

- Dijkstra: Algoritmo de búsqueda de costo mínimo para grafos ponderados. Es exhaustivo y garantiza encontrar el camino más corto, aunque su eficiencia disminuye en entornos grandes [8].

- RRT (Rapidly-Exploring Random Tree): Método probabilístico que construye un árbol de exploración rápida en el espacio libre. Es útil en espacios de alta dimensionalidad, aunque no garantiza optimalidad [9].

- PRM (Probabilistic Roadmap): Crea una red de caminos válidos conectando puntos aleatorios del espacio libre. Es efectivo en entornos estáticos donde se requieren múltiples consultas de trayectoria [9].

- Diagramas de Voronoi: Utiliza regiones equidistantes a obstáculos para trazar rutas seguras. Suele producir trayectorias suaves y centradas en pasadizos amplios [9].


**1.4. Algoritmos Bug**
---
Los algoritmos Bug son estrategias clásicas para navegación con percepción local (sensor de contacto o proximidad):

- Bug 0: El robot avanza directamente hacia la meta. Al encontrar un obstáculo, lo bordea hasta poder continuar en línea recta. No garantiza llegar si la meta es inaccesible, y puede quedar atrapado en ciclos [10].

- Bug 1: Al encontrar un obstáculo, el robot lo recorre completamente, identifica el punto más cercano a la meta y desde ahí continúa. Asegura encontrar la meta si es alcanzable [11].

- Bug 2: Similar al Bug 1, pero el robot abandona el obstáculo tan pronto como intersecta la línea directa entre el punto inicial y la meta. Es más eficiente, pero no siempre encuentra la mejor ruta [12].

Cada algoritmo tiene ventajas según el escenario: por ejemplo, A*/Dijkstra funcionan bien en entornos discretizados con pocos grados de libertad, mientras que RRT/PRM son preferibles en espacios complejos y continuos.

**1.5. Algoritmo de solución de laberintos**
---
Un método clásico en robótica móvil es el seguimiento de pared (regla de la mano derecha/izquierda): el robot mantiene contacto constante con una pared del laberinto, girando siempre en una dirección fija en cada cruce. Esto garantiza encontrar la salida en laberintos simplemente conectados (sin “islas” independientes). Para laberintos arbitrarios se usa el algoritmo de Trémaux: el robot “marca” cada pasaje recorrido (p. ej. con cruces o detección) para no repetirlo. Las reglas básicas de Trémaux son:
No recorrer dos veces el mismo camino.
1. Si al llegar a un cruce elige un camino nuevo, márcalo.
2. Si un camino nuevo lleva a un cruce ya visitado o callejón sin salida, retrocede hasta la entrada del camino.
3. Si un camino ya marcado conduce a otro cruce visitado, elige otro camino no marcado (o cualquiera si no hay nuevos).
Siguiendo estas reglas, el robot explora sistemáticamente el laberinto y eventualmente encontrará la salida (o regresará al inicio si no hay solución). Este algoritmo garantiza escapar de laberintos complejos evitando ciclos infinitos al “no repetir caminos”. En la práctica robótica se implementa trazando virtualmente marcas en el entorno (memoria o señales) según estos pasos para guiar al robot [13].


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

**Referencias**
[1] Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011). Introduction to Autonomous Mobile Robots. MIT Press.
[2] Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A Formal Basis for the Heuristic Determination of Minimum Cost Paths. IEEE Transactions on Systems Science and Cybernetics, 4(2), 100–107.
[3] Arkin, R. C. (1998). Behavior-Based Robotics. MIT Press.
[4] Brooks, R. A. (1986). A Robust Layered Control System for a Mobile Robot. IEEE Journal on Robotics and Automation, 2(1), 14–23.
[5] Brooks, R. A. (1990). Elephants Don't Play Chess. Robotics and Autonomous Systems, 6(1–2), 3–15.
[6] Tilden, M. (1997). Biomorphic Robotics. Proceedings of SPIE 3201, Microrobotics and Microassembly, 1–11.
[7] BEAM Robotics. (2003). Principles of BEAM: Biology, Electronics, Aesthetics and Mechanics. Technical Report.
[8] LaValle, S. M. (2006). Planning Algorithms. Cambridge University Press.
[9] Thrun, S., Burgard, W., & Fox, D. (2005). Probabilistic Robotics. MIT Press.
[10] Lumelsky, V., & Stepanov, A. (1986). Path-Planning Strategies for a Point Mobile Automaton Moving amidst Unknown Obstacles of Arbitrary Shape. Algorithmica, 2, 403–430.
[11] Latombe, J. C. (1991). Robot Motion Planning. Springer.
[12] Cormen, T. H., Leiserson, C. E., Rivest, R. L., & Stein, C. (2009). Introduction to Algorithms (3rd ed.). MIT Press.
[13] LaValle, S. M. (2006). Planning Algorithms. Cambridge University Press. (Capítulo sobre exploración de laberintos)

