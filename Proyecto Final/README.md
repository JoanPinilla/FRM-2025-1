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
<div align="center">
  <img height="450" alt="Roomba" src="https://github.com/user-attachments/assets/b5befe65-2fa0-4ec7-9279-18f83ea317a2" />
</div>

* NVIDIA Jetson Nano usada para ROS2 y control del Roomba
* LEGO Mindstorms EV3
<div align="center">
  <img height="450" alt="EV3" src="https://github.com/user-attachments/assets/b663d2bf-210f-4c15-a182-3d9f400be014" />
</div>

Con los siguientes sensores:
   * Sensor ultrasónico
   * Gyro
   * Sensores infrarrojos

## Herramientas de softaware utilizadas

### ROS2
La integración del Roomba con el entorno ROS 2 se realizó mediante el uso del framework _ros2_control_, que permite una interfaz estandarizada y extensible para el control de hardware robótico. Esta arquitectura facilita el desarrollo modular y el acoplamiento con algoritmos de control.

El iRobot Create 2 se conecta la Jetson Nano por comunicación serial. A través de esta conexión, es posible enviar comandos de velocidad y recibir datos de sensores internos del robot, como los encoders.

<div align="center">
   <img height="500" alt="comunicaciones" src="https://github.com/user-attachments/assets/8d95c319-f2bc-4b29-b1a6-10a9e69edcea" />  
</div>

El control del Roomba se estructura en tres capas principales:

<div align="center">
   <img height="300" alt="control_roomba" src="https://github.com/user-attachments/assets/01d2f281-bdd6-4183-bd2a-7e0bed2db530" />
</div>

Interfaz de hardware (RoombaSystemInterface): Implementa la comunicación serial con el robot y traduce los comandos de velocidad lineal y angular recibidos desde ROS 2 al protocolo nativo del Roomba. También publica la información sensorial disponible.

Controladores (controller_manager): Se emplearon controladores estándar del paquete _ros2_control_, particularmente _diff_cont_ para el control teleoperado.

Planificación y emisión de trayectorias: El Roomba lidera el movimiento de manera teleoperada. El nodo _robot_starte_publisher_ publica constantemente los estados de los encoders y también los recibe de _joint_broad_

<div align="center">
   <img height="250" alt="ros" src="https://github.com/user-attachments/assets/252acc4d-962f-42a9-af46-25055a174b8e" />
</div>

### GAZEBO
Con este software de simulación se pudo implementar el modelo cineático del robot para simular su movimiento correctamente. Además, se realizó el mapeo de una zona con un LIDAR. El mapeo de este espacio también se puede observar en la simulación.

<div align="center">
<img width="968" height="580" alt="mapeo" src="https://github.com/user-attachments/assets/22e1c67d-362b-4ed8-95cd-8eef81bd5b11" />
</div>

<div align="center">
<img width="408" height="337" alt="gazebo" src="https://github.com/user-attachments/assets/11e359e0-5b0a-4dc6-bfc9-a057d86d3b78" />
</div>


## Resultados obtenidos con soporte de imagenes y videos
<div align="center">
  <a href="https://youtu.be/fB5QnzeEPsw">
    <img src="https://img.youtube.com/vi/fB5QnzeEPsw/maxresdefault.jpg" alt="Video Tutorial - Haz clic para ver" style="max-width: 100%; border-radius: 8px;" width="720">
  </a>
  <br>
  <sub>En este video se muestra el seguimiento puro del ultrasonido</sub>
</div>

## Lecciones aprendidas

## Scripts utilizados o paquetes creados

Codigo empleado en el EV3, implementado en python para el sistema EV3DEV2 se encuentra [acá](FRM-2025-1/Proyecto_Final/SEGUIDOR_EV3.py).
Las funciones principales de este código son:

<div align="center">
  <img width="770" height="621" alt="seguimiento" src="https://github.com/user-attachments/assets/87ac395c-18f2-49ab-9040-ad25dc2df1c4" />
</div>

### Seguir señal: 

   Cuando detecta la señal objetivo (con el ultrasónido), avanza recto.

### Evitar obstáculos: 

   Si se encuentra un obstáculo frontal,
   retrocede un poco
   y gira aproximadamente 90 grados,
   a continucación intenta rodear el obstáculo manteniendo una distancia óptima de la pared lateral

### Búsqueda aleatoria: 
   
   Cuando no detecta la señal, alterna entre
   avanzar recto por un tiempo dado
   y girar en su lugar por un tiempo dado



## Autoevalución del equipo y personal
### Autoevaluación de equipo - 4.0
Logramos una colaboración muy buena durante todo el desarrollo del proyecto. Hubo buena comunicación, compromiso por parte de todos y una distribución equilibrada de las tareas. Cada integrante asumió un rol claro y aportó desde sus fortalezas. Supimos resolver los problemas técnicos que aparecieron y llegamos a un sistema que presenta algunos problemas pero es funcional y cumple de manera aceptable el objetivo planteado.

### Autoevaluación de Eduardo Cuadros - 4.5
Tuve un papel clave en la parte del Roomba, trabajando en la emisión de la señal ultrasónica y en el control de movimiento. Aporté ideas para resolver varios problemas técnicos y siempre estuve disponible para trabajar en equipo. Me siento satisfecho con mi desempeño y el resultado final del sistema.

### Autoevaluación de Andrés Serna - 4.5
Hice un excelente trabajo dentro del equipo. Me enfoqué especialmente en la parte de hardware y en el funcionamiento del EV3, participando activamente en las pruebas de seguimiento y asegurándome de que la lectura del sensor ultrasónico fuera estable. Me mantuve comprometido durante todas las etapas del proyecto y colaboré de forma constante con mis compañeros.


### Autoevaluación de Nicolás Moreno - 4.5
Me involucré desde el inicio en el diseño general del sistema, especialmente en cómo lograr la comunicación entre los dos robots. Aporté soluciones en la integración del software y el control de seguimiento, y estuve pendiente de que todo funcionara bien en conjunto. Me sentí muy motivado y comprometido con el proyecto.

### Autoevaluación de Joan Pinilla - 4.2
Estuve involucrado tanto en la parte del EV3 como en la parte del Roomba. Aporté mis ideas para el flujo del código de seguimiento y me encargué de hacer la presentación.
