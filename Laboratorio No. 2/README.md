# Laboratorio No. 2

Integrantes:
- Eduardo Cuadros Montealegre
- Andrés S Serna
- Joan Pinilla
- Nicolás Moreno

## Contenido
- [Busqueda bibliografica](#busqueda-bibliografica)
- [Sensores](#sensores)
  - [Sensor Hokuyo](#sensor-hokuyo)
  - [Sensor RPLidar](#sensor-rplidar)
  - [Sensor ultrasonido](#sensor-ultrasonido)
  - [Sensores Lego](#sensor-lego)
    - [Sensor infrarrojo](#sensor-infrarrojo)
    - [Sensor Encoder](#sensor-encoder)
- [ROS](#ros)
  - [Uso de ROS](#uso-de-ros)
  - [ROS Kuboki](#ros-kobuki)
  - [ROS Lego EV3](#ros-lego-ev3)


## Busqueda bibliografica
- **¿Qué es el Vocabulario Internacional de Metrología (VIM)?**
  
Según el Centro Español de Metrología, el VIM es un diccionario terminológico que contiene las denominaciones y definiciones que conciernen a la metrología, la ciencia de las mediciones y sus aplicaciones. Este VIM abarca princios relativos de las magnitudes y unidades.

- **Según el VIM, defina los siguientes conceptos:**
  
  - Exactitud de medida: Proximidad entre un valor medido y un valor verdadero de un mensurando.
  - Precisión de medida: Proximidad entre los valores medidos obtenidos en mediciones repetidas de un mismo objeto bajo condiciones especificas.
  - Error de medida: Diferencia entre un valor medido de una magnitud y un valor de referencia.
  - Incertidumbre de medida: Parámetro no negativo que caracteriza la dispersión de los valores atribuidos a un mensurando, a partir de la información que se utiliza.

- **Explique la diferencia entre un error sistemático y un error aleatorio**
  
El error sistemático de medida es un componente del error de medida que, en mediciones repetidas, permanece constante o varía de manera predecible. Mientras que el error aleatorio es el otro componente que, en mediciones repetidas, varía de manera impredecible.


## Sensores

### Sensor Hokuyo

Comenzando con la práctica para el sensor Hokuyo URG-04LX-UG01, se creó el espacio de trabajo que se puede ver en la _Figura 2.1.1_.

---
_Figura 2.1.1: Plano del espacio de trabajo:_

![mapaReal](https://github.com/user-attachments/assets/9e1cf951-c2d1-4c10-948b-69d7b0edeba3)
---
Se escogieron las 3 poses siguientes:

- **Pose 1**
  - **Coordenada X:** 24.5 cm
  - **Coordenada Y:** 26.0 cm
  - **Dirección theta:** 45°
- **Pose 2**
  - **Coordenada X:** 64 cm
  - **Coordenada Y:** 46.5 cm
  - **Dirección theta:** 32°
- **Pose 3**
  - **Coordenada X:** 65.5 cm
  - **Coordenada Y:** 65.3 cm
  - **Dirección theta:** 18°
 
Para la toma de datos primero se hizo la configuración del lidar, iniciando el objeto en MATLAB. Luego se realizaron 3 escaneos con 3 segundos de diferencia entre ellos. Esto haciendo uso de la función **LidarScan.m**. Con esta función se manda un mensaje al lidar mediante el protocolo SCIP 2.0. Este mensaje es _GD0044072500_ el cual hace la petición de toma de datos en el ángulo establecido. Luego de hacer los escaneos se crean los arrays para cada pose. Estos arrays tienen forma _[num_escaneos, n_pasos]_. 

Para la visualización de los datos primero se hizo un promedio entre las medidas de los 3 escaneos para cada paso,que a su vez se realizó para cada pose. Se generó un vector de angulos, el cual tiene los rangos de los angulos escaneados y su longitud es igual a la cantidad de pasos en dicho escaneo. Con estos dos arrays, se usaron las funciones **lidarScan** para visualizar los puntos escaneados (_Figura 2.1.2_) y **occumancyMap** para ver el mapa de ocupación local con la pose del lidar definida (_Figuras 2.1.3, 2.1.4 y 2.1.5_). Una vez se tenían los 3 mapas de ocupación referentes a las 3 poses, se usó **buildMap** para crear el mapa global uniendo los 3 mapas locales (_Figura 2.1.6_).

---
_Figura 2.1.2: Escaneo para Pose 1:_

![scan1](https://github.com/user-attachments/assets/6a059f93-ba7e-42d3-8fec-66644a1e17a3)
---
_Figura 2.1.3: Mapa local para Pose 1:_

![mapa1](https://github.com/user-attachments/assets/8990bf12-f871-4ed2-9d1c-545b81878495)
---
_Figura 2.1.4: Mapa local para Pose 2:_

![mapa2](https://github.com/user-attachments/assets/f88b2b5d-a809-47c6-a9c8-cf98151317f7)
---
_Figura 2.1.5: Mapa local para Pose 3:_

![mapa3](https://github.com/user-attachments/assets/b1e24ec7-96ac-4d53-858a-2b8b823154aa)
---
_Figura 2.1.6: Mapa global:_

![mapaT](https://github.com/user-attachments/assets/40691bf0-33a8-47f6-b707-0146b9ada72c)
---

En general los mapas son muy parecidos a los espacios físicos. Hay algunas paredes se sobreponen y otras que tienen ondulaciones. También se puede observar que, aunque el lidar tiene una zona muerta de 120°, cuando se combinan los mapas de las 3 poses, esa zona muerta se reconstruye con las otras perspectivas.

---
__
### Sensor RPLidar
Para la práctica de este sensor se organizó el espacio de trabajo como se puede ver en la siguiente _Figura 2.2.1_. 

---
_Figura 2.2.1: Plano del espacio de trabajo del RPLidar._

![workspace_rplidar](https://github.com/user-attachments/assets/29e7432f-f9cc-4a0b-922a-e5aafba2866c) 
---

Siguiendo el marco de referencia visto en la imágen, las tres poses escogidas tienen los siguientes componentes:

- **Pose 1**:
  - **Coordenada X**: 76.2 cm. 
  - **Coordenada Y**: 32.9 cm. 
  - **Dirección theta**: 113°.
- **Pose 2**:
  - **Coordenada X**: 30.1 cm
  - **Coordenada Y**: 20.0 cm
  - **Dirección theta**: -33°
- **Pose 3**:
  - **Coordenada X**: 30.1 cm
  - **Coordenada Y**: 55.5 cm
  - **Dirección theta**: 67°

Se realizó la toma de datos siguiendo el algoritmo facilitado por el monitor (:D) **ScanExportLidar.py**. Este algoritmo recorre un bucle en el que el sensor toma los datos haciendo 3 giros, guarda dichos datos en un archivo .csv y presenta un gráfico. 
Teniendo los archivos con los datos, lo siguiente es unir los 3 mapas para crear el mapa de todo el laberinto. Para esto se utilizan funciones como **lidarScan**, **occupancyMap** y **buildMap**. Con estas 3 funciones pasamos de escaneo (_Figura 2.2.2_), pasando por el mapa local (_Figura 2.2.3_) y, juntando estos tres (_Figura 2.2.4 y 2.2.5_), llegar hasta el mapa global (_Figura 2.2.6_).

---
_Figura 2.2.2: Escaneo para Pose 1:_

![scan1](https://github.com/user-attachments/assets/da092947-435d-4b89-895e-1a837e4900f3)
---
_Figura 2.2.3: Mapa de ocupación para Pose 1:_

![occupancymap1](https://github.com/user-attachments/assets/c9562caf-d682-42ca-94f2-6ba5fef72d1d)
---

_Figura 2.2.4: Mapa de ocupación parar Pose 2:_

![scan2](https://github.com/user-attachments/assets/3f9c6fe1-7c91-4e80-9cc0-d1d773e82968)
---

_Figura 2.2.5: Mapa de ocupación para Pose 3:_

![occ3](https://github.com/user-attachments/assets/8fefd9c6-bfaf-4845-b034-551a26ba7e83)
---

_Figura 2.2.6: Mapa de ocupación global:_

![occT](https://github.com/user-attachments/assets/db84ed38-4f17-46f9-abb7-fe0c73682c91)
---

Como se puede observar, el mapa de ocupación es en gran parte semejane con el espacio real. Sin embargo, presenta alguas dimensiones erroneas, cerca a la Pose 3. Estos problemas se deben a que en la creación del  espacio de trabajo por equivocación se modificó el mapa de manera que entre las Poses hay ligeras diferencias. A pesar de esto, cada mapa local representa de forma acertada los espacios que tomaron en la maqueta.

---
#### Fotos
_Figura 2.2.7: Pose 1:_

![Pose1](https://github.com/user-attachments/assets/54f2b5f2-6de2-4e84-9812-230a60632f9c)
---
_Figura 2.2.8: Pose 2:_

![Pose2](https://github.com/user-attachments/assets/54949db9-3b22-4f69-a9b3-b8f0837fd605)
---
_Figura 2.2.9: Pose 3:_

![Ñls](https://github.com/user-attachments/assets/b4b101ca-fe98-4a30-a260-62295c14396c)
---

#### Fotos 
_Figura 2.2.7: Pose 1_

![Pose1raw](https://github.com/user-attachments/assets/688266bf-f409-4d40-8958-b169ecdffbb0)
---

_Figura 2.2.8: Pose 2_

![Pose2raw](https://github.com/user-attachments/assets/1c02a293-fd44-4464-90e0-19e076a12b6c)
---

_Figura 2.2.9: Pose 3_

![Pose3raw](https://github.com/user-attachments/assets/650eed18-f9e3-465f-99cb-664aa9155ea2)
---

### Sensor ultrasonido

# 100 cm
Media: 97.1886  
Desviación estándar: 0.3109  
Error absoluto medio: 2.8114  
Error relativo medio: 2.81%  

![data100](https://raw.githubusercontent.com/JoanPinilla/FRM-2025-1/main/Laboratorio%20No.%202/imagenes/100_data.svg)

![hist100](https://raw.githubusercontent.com/JoanPinilla/FRM-2025-1/main/Laboratorio%20No.%202/imagenes/100_hist.svg)

# 140 cm
Media: 132.9971  
Desviación estándar: 9.8831  
Error absoluto medio: 7.0029  
Error relativo medio: 5.00%  

![data140](https://raw.githubusercontent.com/JoanPinilla/FRM-2025-1/main/Laboratorio%20No.%202/imagenes/140_data.svg)

![hist140](https://raw.githubusercontent.com/JoanPinilla/FRM-2025-1/main/Laboratorio%20No.%202/imagenes/140_hist.svg)

# 150 cm
Media: 145.5017  
Desviación estándar: 5.5401  
Error absoluto medio: 4.4983  
Error relativo medio: 3.00%  

![data150](https://raw.githubusercontent.com/JoanPinilla/FRM-2025-1/main/Laboratorio%20No.%202/imagenes/150_data.svg)

![hist150](https://raw.githubusercontent.com/JoanPinilla/FRM-2025-1/main/Laboratorio%20No.%202/imagenes/150_hist.svg)

# Comparación

![comp](https://raw.githubusercontent.com/JoanPinilla/FRM-2025-1/main/Laboratorio%20No.%202/imagenes/comp.svg)

### Sensores Lego
## ROS

### Uso de ROS

Durante el desarrollo del laboratorio se implementaron y ejecutaron diversos scripts en Python con el objetivo de familiarizarnos con el uso de ROS desde este lenguaje. En primer lugar, el script pysubpose.py se encarga de suscribirse al tópico /turtle1/pose para recibir continuamente la posición y orientación actual de la tortuga. Para lograr esto, se utiliza la función rospy.Subscriber, que permite recibir mensajes del tipo Pose. Estos mensajes son impresos en la consola a medida que llegan. El nodo se inicializa mediante rospy.init_node, y se mantiene activo mediante rospy.spin, que garantiza que el programa no termine hasta que se detenga manualmente.

Por otro lado, el script pypubvel.py tiene la función opuesta: en lugar de suscribirse, publica mensajes en el tópico /turtle1/cmd_vel, lo que permite controlar directamente la velocidad lineal y angular de la tortuga. En este caso, se publica un mensaje del tipo Twist con componentes aleatorias, generadas mediante el módulo random. Esto hace que la tortuga se mueva de manera errática dentro del entorno. Para controlar la frecuencia de publicación se emplea rospy.Rate, estableciendo una tasa de 1 Hz.

Con el fin de determinar los límites del espacio de simulación de Turtlesim, se emplearon los programas turtle_teleop_key y pysubpose.py. Al mover manualmente la tortuga hacia los bordes de la pantalla y observar los valores de posición reportados, se determinó que las coordenadas del plano varían aproximadamente entre 0 y 11 tanto para la posición en X como en Y. 

**Uso de Servicios en Python**

ROS permite la utilización de servicios, los cuales proporcionan una forma de ejecutar tareas bajo demanda mediante la comunicación sincrónica entre nodos. En Python, los servicios pueden utilizarse mediante rospy.ServiceProxy, que actúa como un cliente para un servicio determinado. Antes de utilizarlo, es necesario asegurarse de que el servicio esté disponible utilizando rospy.wait_for_service.

Por ejemplo, para limpiar la pantalla de Turtlesim se puede utilizar el servicio clear, como se muestra a continuación:

rospy.wait_for_service('clear')
clear = rospy.ServiceProxy('clear', Empty)
clear()

Este fragmento de código espera a que el servicio esté disponible, lo conecta mediante un proxy y finalmente lo ejecuta. Esta mecánica se repite en muchos servicios disponibles en ROS, incluyendo spawn, kill, teleport_absolute, entre otros.

**Ejecución y Análisis del Script pycuadrado.py**

El script pycuadrado.py fue proporcionado como ejemplo para demostrar cómo se pueden utilizar servicios y comandos de velocidad para lograr que la tortuga dibuje una figura específica. En este caso, el programa está diseñado para que turtle1 dibuje un cuadrado en la pantalla.

#### RECORRIDO DE 1 METRO AL 30% DE VELOCIDAD
El montaje de este experimento donde se quiere comprobar la incertidumbre de medida en los sensores y actuadores utilizados en los kits LEGO EV3 se puede ver en la siguiente figura:

<div align="center">
  <img src="https://github.com/user-attachments/assets/60e28985-ea41-4998-9c28-0af91cfdec5d" width="500">
</div>

El montaje consiste en el robot EV3, la cinta métrica y una pared. 

<div align="center">
  <img src="https://github.com/user-attachments/assets/fa8dcd5c-821f-4785-877c-070e7e753b1a" width="500">
</div>

<div align="center">
  <img src="https://github.com/user-attachments/assets/d01ce5f9-caac-4e1f-999f-5ffce5d967a1" width="500">
</div>

#### RECORRIDO DE 1 METRO AL 100% DE VELOCIDAD

<div align="center">
  <img src="https://github.com/user-attachments/assets/efd0d2b8-1c33-466c-ac83-fb44798f2c98" width="500">
</div>

#### MEDIDAS DE ÁNGULOS
<div align="center">
  <a href="https://youtu.be/eh_VdQ6DnPY">
    <img src="https://img.youtube.com/vi/eh_VdQ6DnPY/0.jpg" alt="Prueba de video" width="500">
  </a>
</div>

Para lograr esto, el script emplea una combinación de comandos Twist enviados al tópico /turtle1/cmd_vel y pausas mediante rospy.sleep para asegurar que la tortuga avance la distancia deseada antes de girar. Además, al inicio del script se hace uso del servicio /clear para borrar la pantalla y del servicio /turtle1/teleport_absolute para posicionar a la tortuga en un punto de inicio determinado.

Este ejemplo fue fundamental para comprender la estructura de un programa que combina servicios con publicación de mensajes, y sirvió como base para el desarrollo del script final solicitado por la guía.

**Creación de una Segunda Tortuga**

Para añadir una segunda tortuga al entorno simulado, se utilizó el servicio /spawn. Este servicio permite crear una nueva instancia de tortuga en una posición y orientación dadas. En la terminal, se ejecutó el siguiente comando:

rosservice call /spawn 2.0 2.0 0.0 "turtle2"

Esto crea una nueva tortuga llamada turtle2 en las coordenadas (2.0, 2.0) con una orientación de 0 radianes. Posteriormente, se puede interactuar con esta tortuga de forma independiente utilizando los tópicos /turtle2/cmd_vel, /turtle2/pose, entre otros.

**Desarrollo del Script Final draw_shapes.py**

El objetivo final fue desarrollar un script en Python, draw_shapes.py, que permita a dos tortugas (turtle1 y turtle2) dibujar secuencialmente un triángulo equilátero y un cuadrado. El comportamiento del programa se estructuró en dos etapas principales. En la primera etapa, turtle1 dibuja ambas figuras utilizando una combinación de comandos Twist para moverse hacia adelante y rotar, junto con pausas para sincronizar los movimientos. Una vez completada esta etapa, se procede a crear a turtle2 mediante el servicio spawn. Posteriormente, se repite el mismo procedimiento de dibujo con esta nueva tortuga.

Para modularizar el código y facilitar su lectura, se implementaron funciones independientes para el dibujo de cada figura. Esto permitió reutilizar el código tanto para la primera como para la segunda tortuga. Las velocidades lineales y angulares fueron  ajustadas para mantener las proporciones de las figuras dentro de los límites del entorno de simulación.

Con el fin de automatizar el proceso de ejecución del nodo de simulación (turtlesim_node) y del script draw_shapes.py, se implementó un archivo de lanzamiento denominado draw_shapes.launch. Este archivo contiene dos nodos: uno para iniciar la simulación y otro para ejecutar el script de dibujo.

El contenido del archivo es el siguiente:

<launch>
    <node pkg="turtlesim" type="turtlesim_node" name="sim"/>
    <node pkg="laboratorio_2" type="draw_shapes.py" name="draw_shapes" output="screen"/>
</launch>

Este archivo puede ejecutarse mediante el comando:

roslaunch laboratorio_2 draw_shapes.launch

Al hacerlo, se lanza simultáneamente el entorno gráfico de Turtlesim y el script que comanda a las tortugas para dibujar las figuras requeridas. Esta automatización permite simplificar la puesta en marcha del sistema y asegura que todas las dependencias necesarias estén disponibles en el momento de la ejecución.

### ROS Kobuki

En esta sección se buscó desarrollar un programa que permite al usuario operar el robot Kobuki de forma remota usando el teclado, mientras el sistema supervisa continuamente el sensor de acantilado. Si se detecta una caída potencial (borde de mesa, escalón, etc.), el robot reproduce un sonido de alerta (Para este caso, decidimos usar el sonido de Error del Kobuki). Esta funcionalidad debía integrarse sin afectar la capacidad de controlar el robot mediante teleoperación por teclado. Se buscó una solución funcional y simple que no implicara la creación de nuevos nodos, archivos launch, ni modificaciones adicionales a la configuración base del sistema.

Para cumplir el objetivo planteado, se decidió modificar directamente el archivo fuente keyop_core.cpp del paquete kobuki_keyop, el cual ya implementa la funcionalidad de teleoperación mediante teclado. Esta elección se tomó para evitar la creación de nuevos nodos, archivos .launch o entradas adicionales en el CMakeLists.txt y el package.xml, manteniendo así la simplicidad del sistema.

Dentro del archivo keyop_core.cpp, se implementó una nueva función cliffEventCallback, la cual se suscribe al tópico /mobile_base/events/cliff. Este tópico es publicado por el nodo base de Kobuki (kobuki_node) y emite mensajes del tipo kobuki_msgs/CliffEvent cuando se detecta un evento asociado al sensor de acantilado. Al detectar un evento con estado CLIFF, la función reproduce uno de los sonidos integrados del robot utilizando el tópico /mobile_base/commands/sound.


https://github.com/user-attachments/assets/02837dc4-d0a3-4f1b-a31c-32c4e30120ed


Para ello, se añadió un publisher adicional en el mismo nodo keyop_core para publicar mensajes del tipo kobuki_msgs/Sound. La publicación del sonido se realiza utilizando uno de los valores predefinidos por el mensaje Sound, como por ejemplo kobuki_msgs::Sound::ERROR, lo cual hace que el robot emita un sonido interno de advertencia sin necesidad de usar archivos de audio externos ni reproducirlos desde el computador.

Estas modificaciones fueron realizadas directamente en la siguiente ruta del espacio de trabajo:
kobuki/kobuki_keyop/src/keyop_core.cpp
Además, se incluyó la suscripción en el método init() del mismo archivo, y se declaró el nuevo publisher sound_publisher_ como atributo de la clase KeyOpCore. No se realizaron modificaciones adicionales en los archivos CMakeLists.txt ni package.xml, dado que las dependencias requeridas ya estaban incluidas en el paquete original.

￼

