# GUÍA 1: Conociendo los robots kuboki y Ev3

## 1. ¿Qué es un robot móvil? Definir qué es un robot y cuáles son sus principales características.
### ¿Qué es un robot móvil?
A diferencia de los robots estacionarios, como muchos robots industriales, los robots móviles pueden navegar por diferentes espacios, adaptándose a diversas condiciones del terreno .

### Características de los robots móviles
#### Movilidad:
Pueden desplazarse utilizando ruedas, patas, orugas u otros mecanismos, lo que les permite operar en diversos entornos.
#### Percepción del entorno:
Equipados con sensores que les permiten detectar obstáculos, identificar rutas y adaptarse a cambios en el entorno.
#### Navegación autónoma:
Capacidad para planificar rutas y desplazarse sin intervención humana, utilizando algoritmos de navegación y mapeo.
#### Adaptabilidad:
Pueden ajustar su comportamiento en función de las condiciones del entorno, como terrenos irregulares o presencia de obstáculos, por lo que tienen aplicaciones en sectores como la logística, exploración espacial, agricultura, seguridad y asistencia en el hogar.

## 2. Presentación de los Robots: Descripción detallada de los robots Kuboki y EV3, incluyendo sus características físicas y capacidades.

### Robot Kuboki

El kuboki es una base robótica móvil diseñada como plataforma de desarrollo para investigación y prototipado.

#### Características:
- Dimensiones: 35 cm de diámetro y 14 cm de altura.
- Peso: Aproximadamente 2.5 kg (sin carga).
- Chasis: Ruedas diferenciales.
- Sensores:
  -  Bumpers
  -  Sensores de proximidad
  -  Giroscopio y acelerómetro
- Puerto USB y serial

Este robot tiene la capacidad de implementar paquetes de navegación autónoma, haciendo uso de ROS y sensores adicionales, lo que a su vez permite que el robot pueda detectar obstáculos. Una de las mayores ventajas del robot es la capacidad de personalización que tiene, ya que se le pueden montar distintos tipos de cámaras, sensores LIDAR, brazos robóticos, etc.

### Robot Lego Mindstorm EV3

El EV3 es parte de la serie Mindstorms de Lego, el cual es un kit educativo diseñado para introducir a los niños a la robótica y la programación. Su ventaja principal es su modularidad, lo que permite construir múltiples robots con los mismos componentes de Lego.

#### Características:
- La unidad central EV3 Brick posee una pantalla LCD, botónes de control, altavos y 4 puertos de entrada para sensores y 4 puertos de salida para motores.
- Incluye sensores de color, contacto, ultrasónicos y giroscopio.
- La estructura es completamente hecha con piezas de Lego.

A través del software de Lego se logra la programación del movimiento del robot de forma sencilla. Sin embargo, también tiene la capacidad de ser programado en lenguajes de programación comunes como Python o Java, lo que eleva la capacidad de experimentación a un nivel mayor.

## 3. Estado actual del robot y sistema de control.
### Kobuki
Físicamente, el robot es completamente funcional a pesar de que se nota cierto desgaste. A través del API o de ROS se puede acceder al sistema de control del Kobuki y programar rutinas como se verá en secciones posteriores.

### EV3
El robot en general está en buen estado, junto con sus piezas, motores y sensores. Con el brick de control del EV3 se pueden hacer conexiones hacia la aplicación de LEGO con el cual el sistema de control se simplifica mucho, pero también presenta restricciones al ser software privado. 

## 4. APIs y lenguajes de programación: Identificar las APIs o librerías disponibles para programar los robots. Enumerar los lenguajes de programación compatibles con los robots.

### Bibliotecas y lenguajes de programación de Kobuki
#### Kobuki Driver:
Biblioteca de bajo nivel C++ que permite interactura con los sensores, motores y otros elementos del robot. El driver esta disponible en GitHub con documentación. 

#### ROS:
El kobuki es compatible en su totalidad con ROS1 (especialmente con Melodic y Kinetic). El robot también cuenta con soporte por parte de la comunidad para ROS2. Entre las bibliotecas destacadas esta _kobuki_node_, _yocs_cmd_vel_mux_, _ecl_ , entre otros. Estas bibliotecas permiten la integración de la navegación, odometría, SLAM, etc.

#### TurtleBot 2 Stack:
El kobuki forma parte de la plataforma TurbleBot 2, por lo que puede usar los paquetes de ROS de TurtleBot.

El principal lenguaje de programación para manejar al Kobuki es C++, presenta bibliotecas de bajo nivel y muchos de los nodos de ROS en este lenguaje. También se usa ampliamente Python para ROS, teniendo muchos scripts y nodos escritos. Sin embargo, también se puede usar Java o Lua, mediante ROS, para integrar los sistemas.

### Bibliotecas y lenguajes de programación de EV3
#### Software EV3
El EV3 presenta un entorno de programación basado en bloques, muy usado en educación por su fácil entendimiento y acceso para todas las edades.

#### EV3Dev
Este es un sistema operativo basado en Linux en el cual se pueden ejecutar programas en diversos lenguajes (Python, C++, Java, etc).

#### Pybriks
Es un firmware alternativo que permite programar directamente en Python y usando Bluetooth, sin necesidad de instalar EV3Dev.

Como se mencionó anteriormente, el EV3 es compatible con Python a través de _ev3dev-lang-python_. También es compatible con Java usando _LeJOS_ o _EV3Dev_. Para algunas versiones educativas también se puede usar Scratch, el cual es un lenguaje basado en bloques.

## 5. Herramientas de desarrollo propias: Resumir las herramientas propias que disponen los robots para facilitar la programación y el control.
## Kobuki
La clase principal de la API es kobuki::Kobuki, que encapsula la comunicación con el hardware del robot. Para utilizarla, se debe inicializar con un objeto kobuki::Parameters, especificando configuraciones como el puerto serial (device_port) y parámetros de batería. Una vez inicializada, la clase proporciona métodos para:
* Control de movimiento: setBaseControl(linear_velocity, angular_velocity) permite establecer velocidades lineales y angulares.
* Gestión de LEDs y sonidos: setLed() y playSoundSequence() controlan los indicadores visuales y auditivos.
* Acceso a sensores: Métodos como getCoreSensorData(), getCliffData() y getInertiaData() proporcionan información de los sensores integrados.
* Odometría: updateOdometry() calcula la posición del robot basándose en los datos de los encoders y el giroscopio.
* Estado del robot: Funciones como isAlive(), isEnabled() y batteryStatus() informan sobre el estado operativo y energético del robot.

## EV3
### Tipos de bloques de programación

El entorno de programación EV3 se organiza en paletas de bloques, cada una con funciones específicas:

- **Bloques de acción**:  
  Permiten controlar motores y emitir sonidos. Incluyen bloques para mover motores individuales o múltiples, ajustar la potencia, la dirección y la duración del movimiento.

- **Bloques de sensores**:  
  Procesan datos de sensores como el táctil, de color, giroscópico, ultrasónico, entre otros. Estos bloques permiten que el robot reaccione a estímulos del entorno.

  ![image](https://github.com/user-attachments/assets/158a1203-c5bd-4a94-bb62-ed2edb69cffc)


- **Bloques de flujo**:  
  Controlan la lógica del programa mediante estructuras como bucles y condicionales. Por ejemplo, el bloque "Switch" permite ejecutar diferentes acciones según condiciones específicas.

  ![image](https://github.com/user-attachments/assets/d9487243-062a-4a43-8015-fc13f858ff4f)


- **Bloques de datos**:  
  Manejan variables, constantes, operaciones matemáticas y lógicas. Estos bloques son esenciales para almacenar y manipular información durante la ejecución del programa.

- **Bloques avanzados**:  
  Ofrecen funcionalidades como acceso a archivos, registro de datos, comunicación Bluetooth y más, ampliando las capacidades del robot.

- **My Blocks**:  
  Permiten crear bloques personalizados que encapsulan secuencias de comandos reutilizables, facilitando la organización y reutilización del código.


### Personalización avanzada

El software EV3 permite una personalización avanzada mediante las siguientes herramientas:

- **My Blocks con parámetros**:  
  Al crear un My Block, se pueden definir parámetros de entrada y salida, como números, texto o valores lógicos, lo que permite adaptar el bloque a diferentes situaciones.

- **Variables y constantes**:  
  Se pueden definir variables para almacenar datos que cambian durante la ejecución del programa, y constantes para valores fijos, facilitando la gestión de la información.

- **Editor de contenido**:  
  Herramienta que permite personalizar proyectos, agregar notas, imágenes o videos, y adaptar actividades a diferentes niveles educativos.

- **Programación avanzada**:  
  Además de la interfaz gráfica, EV3 es compatible con lenguajes como Python y Java, permitiendo a usuarios avanzados escribir código más complejo y detallado.


## 6. Sensores del robot Identificar los sensores incorporados en los robots y explicar su funcionamiento. Que compatibildiad tienes con otros sensores.
El robot Kobuki es una plataforma de base móvil d equipada con una serie de sensores básicos integrados, que permiten tareas de navegación, control y seguridad. Entre los sensores integrados en el Kobuki se encuentran: (1) sensores de contacto o bumpers, (2) sensores de caída de rueda, (3) sensores de acantilado, (4) encoders en las ruedas, (5) giroscopio y (6) receptor infrarrojo.

Los sensores de contacto, conocidos como bumpers, están ubicados en la parte frontal del robot y se dividen en tres secciones: izquierda, central y derecha. Funcionan mediante interruptores mecánicos que se activan al detectar colisiones físicas. Estos sensores son fundamentales para la detección de obstáculos directos y la prevención de daños estructurales durante el desplazamiento del robot.
Los sensores de caída de rueda, ubicados en el chasis cerca de cada rueda motriz, permiten detectar si alguna de las ruedas ha perdido contacto con el suelo. Si el robot se encuentra al borde de una superficie elevada o si ha sido levantado, estos sensores envían una señal que generalmente detiene el movimiento para evitar accidentes.
Los sensores de acantilado, también distribuidos en tres zonas (izquierda, centro y derecha), emplean haces de luz infrarroja orientados hacia el suelo. Si uno de estos haces no refleja luz de vuelta al sensor —lo que ocurriría, por ejemplo, si hay un escalón o un vacío— el robot interpreta esto como un "acantilado" y se detiene para evitar caídas.
El giroscopio del robot estima su orientación y complementar los datos de los encoders mediante fusión sensorial. Los encoders, integrados en las ruedas, permiten medir con precisión la rotación de cada una. Finalmente, el receptor infrarrojo permite que el robot reciba señales desde controles remotos estándar. Aunque su funcionalidad es limitada en comparación con otros sensores, puede usarse para pruebas, activación manual de rutinas o como mecanismo de parada de emergencia.

En cuanto a compatibilidad, el Kobuki ha sido diseñado con una arquitectura abierta que facilita la integración con sensores externos. Es común conectar sensores como cámaras RGB-D (por ejemplo, Intel RealSense o Kinect), utilizadas para percepción tridimensional, SLAM y navegación autónoma. También se integran con frecuencia sensores LIDAR, como el RPLIDAR o el Hokuyo, que permiten la generación de mapas bidimensionales precisos del entorno.

Asimismo, puede conectarse con IMUs externas para mejorar la estimación de orientación mediante técnicas como el filtro de Kalman extendido (EKF). Sensores ambientales (como de temperatura, humedad o detección de gases) pueden integrarse a través de microcontroladores como Arduino o Raspberry Pi, comunicándose con el sistema principal mediante USB o interfaces seriales compatibles con ROS.

Gracias a su compatibilidad con ROS, el Kobuki puede aprovechar una amplia gama de drivers y paquetes existentes que permiten la incorporación fluida de nuevos sensores y módulos, ampliando significativamente sus capacidades tanto en entornos controlados como en aplicaciones reales de investigación.

## 7. Práctica de identificación y uso de los sensores integrados en los robots, explicando cómo interactúan con el entorno.
Durante esta práctica se utilizó la herramienta `rqt_robot_monitor` disponible en ROS 1 para observar el estado en tiempo real de los sensores y componentes del robot Kobuki. Esta interfaz permite validar el funcionamiento correcto del sistema y visualizar la actividad de cada sensor.

En la sección inferior del monitor, se muestra un resumen del estado de los sensores y subsistemas del robot. A continuación se detallan los sensores integrados identificados:

- **Cliff Sensor (Sensor de acantilado)**  
  Este sensor utiliza haces de luz infrarroja para detectar cambios bruscos de altura frente al robot. Si no detecta reflejo, lo interpreta como un borde o escalón, y puede activar una parada de emergencia para prevenir caídas.

- **Gyro Sensor (Giroscopio)**  
  Muestra la orientación angular del robot con respecto a un eje de referencia. En la prueba, el robot reportó un `heading` de 29.81 grados. Este sensor es esencial para la navegación, el cálculo de odometría y las rotaciones precisas.

- **Motor Current (Corriente del motor)**  
  Su estado fue "All right", lo cual indica que los motores están funcionando correctamente y no presentan sobrecarga. Este sensor permite detectar posibles problemas mecánicos u obstrucciones.

- **Wall Sensor (Sensor de pared)**  
  Permite detectar obstáculos cercanos, como paredes o muebles, para evitar colisiones. Funciona mediante proximidad y puede ser empleado en tareas de navegación autónoma o mapeo.

- **Wheel Drop (Sensor de caída de ruedas)**  
  Detecta si alguna rueda ha perdido contacto con el suelo. Es útil para garantizar la estabilidad del robot en terrenos irregulares o cuando es levantado manualmente.
  
![Sensores del Kobuki](https://github.com/user-attachments/assets/272c64ba-477e-4f86-a0b6-80a190370ee0)

Además, se confirmó que los siguientes subsistemas del robot se encontraban operando de forma normal:
- El estado de los motores fue reportado como habilitado (`Motors Enabled`).
- El `Watchdog` se encontraba activo (`Alive`).
- El sistema de baterías estaba saludable (`Battery: Healthy`).
- El sistema de potencia general fue reportado como `OK`.
- Los puertos de entrada analógica `[4095, 4095, 4095, 4095]` y digital `[0, 0, 0]` también funcionaban correctamente.

A continuación se presentan imágenes que muestran el comportamiento de los sensores del robot Kobuki frente a diversas condiciones de entorno simuladas manualmente.

- **Respuesta del sensor Wheel Drop**  
  Se levantó el robot del suelo, provocando que se activara el sensor de caída de ruedas (`Wheel Drop`). En la interfaz `rqt_robot_monitor`, se puede observar cómo cambia su estado de `All right` a un estado de alerta `Wheel Drop!`.
  ![Respuesta de Wheel Drop](https://github.com/user-attachments/assets/1b1c373d-a930-49c8-9407-c7fab5d5bd9b)

- **Respuesta del Cliff Sensor al inclinar el robot**  
  El robot fue inclinado en un ángulo mayor a 20°, simulando la aproximación a un borde. El sensor de acantilado (`Cliff Sensor`) se activa, y se refleja en el monitoreo del sistema como una advertencia.
  ![Sensor de acantilado activado](https://github.com/user-attachments/assets/416df7fe-7d3f-42e1-b5b0-59e8dfb910a5)
  
- **Lectura del giroscopio (Gyro Sensor) al girar el robot**  
  Al girar el robot sobre su eje, el `Gyro Sensor` muestra una variación en el `heading`. En el ejemplo mostrado, se registra un cambio hacia 29.81 grados. Esta lectura es útil para tareas de localización y navegación.
  ![Giroscopio en acción](https://github.com/user-attachments/assets/224b0d04-6430-431f-9387-6dedbf973e9f)


## 8. Modelado del robot real: Realizar el modelado del robot Kuboki y EV3, en coopeliasim.
Se Tomaron modelos del robot [EV3](https://github.com/albmardom/EV-R3P/tree/master) y [Kobuki](https://grabcad.com/library/interbotix-turtlebot-2i-1), y se importaron en el simulador CoppeliaSim, con el fin de simular el comportamiento de los robots utilizados en laboratorio, dicho modelo se modificó creando distintos objetos para cada una de las partes, y se añadieron junturas de revolución para simular el torque generado.  

Se aseguró de que las medidas fueran las mismas de los robots disponibles, haciendo especial énfasis en el distanciamiento de las ruedas, su diámetro y su ancho, al ser uno de los factores claves para el éxito de la simulación, y se tomó un estimado del peso de estos, para la simulación.

Al no contar con el software de cada uno de los robots dentro del simulador CoppeliaSim, se genero un Script en formato LUA para poder observar su comportamiento, dicho Script realiza una rutina básica de movimiento en forma de cuadrado, teniendo por entradas relevantes el distanciamiento entre ruedas “axleLength”, el radio de las ruedas y las velocidades linear y angular deseadas; dicho Script se observa a continuación, y se modifican los parámetros de inicio según las especificaciones de cada uno de los robots.

```lua
function sysCall_init()
    -- Handles
    leftMotor = sim.getObjectHandle("RJ")  --object asignation 
    rightMotor = sim.getObjectHandle("LJ")
    robotBase = sim.getObjectAssociatedWithScript(sim.handle_self)

    -- Robot parameters
    wheelRadius = 0.035 --m
    axleLength = 0.23   --m

    -- Motion parameters
    linearSpeed = 0.4-- m/s
    angularSpeed = 0.5 -- rad/s
    wheelLinearVelocity = linearSpeed / wheelRadius
    wheelAngularVelocity = (angularSpeed * axleLength / 2) / wheelRadius

    -- State control
    step = 1
    segment = 0
    moving = false
end

function startForward()
    startPos = sim.getObjectPosition(robotBase, -1)
    sim.setJointTargetVelocity(leftMotor, wheelLinearVelocity)
    sim.setJointTargetVelocity(rightMotor, wheelLinearVelocity)
    moving = true
end

function checkForward()
    currentPos = sim.getObjectPosition(robotBase, -1)
    dx = currentPos[1] - startPos[1]
    dy = currentPos[2] - startPos[2]
    distance = math.sqrt(dx*dx + dy*dy)
    if distance >= 1.0 then
        sim.setJointTargetVelocity(leftMotor, 0)
        sim.setJointTargetVelocity(rightMotor, 0)
        moving = false
        return true
    end
    return false
end

function startTurn()
    startAngle = sim.getObjectOrientation(robotBase, -1)[3]
    sim.setJointTargetVelocity(leftMotor, -wheelAngularVelocity)
    sim.setJointTargetVelocity(rightMotor, wheelAngularVelocity)
    moving = true
end

function checkTurn()
    currentAngle = sim.getObjectOrientation(robotBase, -1)[3]
    deltaAngle = currentAngle - startAngle
    if deltaAngle < -math.pi then deltaAngle = deltaAngle + 2 * math.pi end
    if deltaAngle > math.pi then deltaAngle = deltaAngle - 2 * math.pi end
    if math.abs(deltaAngle) >= math.pi / 2 then
        sim.setJointTargetVelocity(leftMotor, 0)
        sim.setJointTargetVelocity(rightMotor, 0)
        moving = false
        return true
    end
    return false
end

function sysCall_actuation()
    if step <= 8 then
        if not moving then
            if step % 2 == 1 then
                startForward()
            else
                startTurn()
            end
        else
            if step % 2 == 1 then
                if checkForward() then step = step + 1 end
            else
                if checkTurn() then step = step + 1 end
            end
        end
    end
end

```

los resultados de las simulaciones pueden ser visto en lo iguientes videos:

### Simulacion Lego EV3
[![Simulación LEGO EV3](https://img.youtube.com/vi/ogT6Gf_eBSU/0.jpg)](https://www.youtube.com/watch?v=ogT6Gf_eBSU)

### Simulacion KOBUKI

[![Simulación KOBUKI](https://img.youtube.com/vi/zChXzVlUzIU/0.jpg)](https://www.youtube.com/watch?v=zChXzVlUzIU)

Las escenas completas en CoppeliaSim se pueden encontrar en:

[Lego EV3](https://github.com/JoanPinilla/FRM-2025-1/blob/main/Laboratorio%20No.%201/Lego.ttt)

[KOBUKI](https://github.com/JoanPinilla/FRM-2025-1/blob/main/Laboratorio%20No.%201/Kobuki.ttt)


## 9. Programa simple de movimientos: Utilizando las herramientas propias del robot, crear un programa sencillo que indique movimientos básicos del robot, como desplazarse hacia adelante, girar a la derecha, etc.
### KOBUKI

![Rutina Autónoma en Trayectoria Pentagonal del Kobuki](https://github.com/user-attachments/assets/c241380d-987c-46e5-8bfc-fd59fd765ce8)


Se implementó una rutina simple de movimiento autónomo utilizando **ROS 1 (Melodic)** y el nodo `keyop_core` del paquete `kobuki_keyop`. El objetivo fue demostrar la capacidad del robot **Kobuki** para ejecutar comandos de movimiento de forma programada sin intervención directa del usuario. Para ello, se desarrolló una función que permite al robot desplazarse en forma de pentágono regular, con lados de **1 metro** de longitud.

La lógica del programa consiste en alternar movimientos de avance con rotaciones angulares precisas. En un pentágono regular, cada ángulo interno es de **108°**, por lo que para cerrar la figura el robot debe girar dicho ángulo luego de avanzar cada lado. Dado que el robot gira sobre su eje y sin corrección de trayectoria, se usa ese mismo valor como ángulo de giro.

El código se agregó al archivo `keyop_core.cpp`, en la definición del método `KeyOpCore::executeAutonomousSequence()`:

```cpp
void KeyOpCore::executeAutonomousSequence()
{
  ROS_INFO("Starting autonomous sequence...");

  if (!power_status) {
    enable();
    ros::Duration(1.0).sleep(); // Esperar para que los motores se activen
  }

  double linear_speed = 0.3;         // m/s
  double angular_speed = 1.2;        // rad/s
  double distance = 1.0;             // metros
  double angle_deg = 108.0;          // grados
  double angle_rad = angle_deg * M_PI / 180.0; // radianes

  double linear_duration = distance / linear_speed;
  double angular_duration = angle_rad / angular_speed;

  for (int i = 0; i < 5; ++i) {
    // Avanzar 1 metro
    cmd->linear.x = linear_speed;
    velocity_publisher_.publish(cmd);
    ros::Duration(linear_duration).sleep();

    // Detener
    cmd->linear.x = 0.0;
    velocity_publisher_.publish(cmd);
    ros::Duration(0.5).sleep();

    // Girar 108 grados
    cmd->angular.z = angular_speed;
    velocity_publisher_.publish(cmd);
    ros::Duration(angular_duration).sleep();

    // Detener
    cmd->angular.z = 0.0;
    velocity_publisher_.publish(cmd);
    ros::Duration(0.5).sleep();
  }

  // Detener al final
  cmd->linear.x = 0.0;
  velocity_publisher_.publish(cmd);
  cmd->angular.z = 0.0;
  velocity_publisher_.publish(cmd);

  ROS_INFO("Autonomous sequence complete.");
}
```
- **Activación de la Secuencia**  
  Para activar esta rutina, se asoció la tecla `a` dentro del mismo archivo `keyop_core.cpp`, dentro del switch encargado de la interpretación de las teclas. Se añadió el siguiente fragmento:
  ```
  case 'a':
  executeAutonomousSequence();
  break;
  ```
  Esto permite que al presionar la tecla `a` mientras se ejecuta el nodo `kobuki_keyop`, el robot inicie automáticamente la rutina descrita, avanzando y girando hasta formar una figura regular con cinco lados (pentágono).
- **Detalles Técnicos Adicionales**
  Utilizamos el Kobuki TurtleBot base con Ubuntu con ROS 1 (Melodic) en C++, modificando el nodo kobuki_keyop/src/keyop_core.cpp. La secuencia autónoma se activa mediante la tecla `a`, la cual fue añadida al `switch` de interpretación de teclas. Al presionar esta tecla, el robot comienza a ejecutar la secuencia programada, donde se definieron las siguientes velocidades para los movimientos del robot: Velocidad lineal de 0.3 m/s, lo que indica la velocidad a la que el robot avanza en línea recta; y Velocidad angular de 1.2 rad/s, la cual se utiliza para controlar la velocidad de rotación del robot. Las acciones realizadas por el robot tienen tiempos de ejecución obtenidos a partir de la división entre la distancia (Lineal o angular) y la velocidad respectiva. La duración total de la rutina autónoma, incluyendo pausas entre movimientos, es de aproximadamente **30 segundos**. Esto incluye los ciclos de avance y giro necesarios para completar la figura geométrica deseada. La lógica de ejecución del robot se basa en un ciclo repetitivo, que consiste en los siguientes pasos:
1. Avanzar en línea recta 1 metro.
2. Detenerse por 0.5 segundos.
3. Girar 108° en sentido antihorario.
4. Repetir este ciclo cinco veces para formar un pentágono.

El robot utiliza un **publicador de velocidad** denominado `velocity_publisher_` para enviar mensajes del tipo `geometry_msgs::Twist`. Estos mensajes contienen las velocidades lineales y angulares que controlan el movimiento del robot. Para garantizar que el robot ejecute las acciones de manera secuencial, se emplea `ros::Duration().sleep()`. Este método permite controlar la duración de cada acción, evitando la necesidad de temporizadores más complejos.

 ![Rutina Autónoma del Kobuki](https://github.com/user-attachments/assets/423595c9-d2e0-4f96-9dcc-da4f62ebf6b0)

### EV3
[![Moviemiento del robot LEGO EV3](https://img.youtube.com/vi/IjDYNgJ66YU/0.jpg)](https://www.youtube.com/watch?v=IjDYNgJ66YU)

Como se puede observar en el video, el EV3 está programado para realizar una trayectoria hexagonal. Esto se logra con el siguiente código:

![image](https://github.com/user-attachments/assets/bc7c10e0-4121-4e00-a9b9-26c24136419b)

El programa empieza por hacer un "beep" para indicar que ha iniciado, luego asigana los motores a sus salidas correspondientes y finalmente repite un ciclo 6 veces, 1 por cada lado del hexágono.

Dentro del ciclo se puede ver que está la instrucción para mover el robot hacia adelante, y luego para girar primero mueve una rueda. Idealmente, sería suficiente con sólo hacer mover la rueda A. Sin embargo, los giros no son tan consistentes por el terreno que es liso y desnivelado. Por este motivo es que se añade la pequeña compensación al giro con la rueda B.

Durante el último lado del hexágono, la trayectoria tiene un desvío considerable, donde se puede notar más el impacto que tiene el piso sobre el agarre de las ruedas. 

## 10. Reflexión y Discusión: experiencias, aprendizajes y posibles mejoras en el uso del robot Kuboki en aplicaciones prácticas.
### Kobuki
Como se pudo ver en los videos, el control de este robot se puede hacer de manera precisa a pesar de tener un mayor grado de dificultad. Este tipo de robot móviles, al ser no tan rápido pero precisos y capaces de albergar gran cantidad de sensores, pueden tener aplicaciones avanzadas en logística y distribución. También, en ambientes controlados, pueden llegar a ser grandes robots colaborativos. 

### EV3
El robot de LEGO fue bastante fácil y divertido de usar. Hacer las conexiones es intuitivo y programarlo también. Es un muy buen elemento de aprendizaje para temas como física clásica, sensores, programación y robótica.
