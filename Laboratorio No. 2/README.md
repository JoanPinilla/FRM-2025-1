# FRM-2025-1
Repositorio para llevar registro de las actividades realizadas en el laboratorio de Fundamentos de Robótica Móvil.

## Contenido
- Búsqueda bibliográfica
- Sensores
  - Sensor Hokuyo
  - Sensor RPLidar
  - Sensor ultrasonido
  - Sensores Lego
- ROS
  - Uso de ROS
  - ROS Kuboki
  - ROS Lego EV3


## 1. Búsqueda bibliográfica

## 2. Sensores

### 2.1 Sensor Hokuyo
Comenzando con la práctica para el sensor Hokuyo URG-04LX-UG01, se creó el espacio de trabajo que se puede ver en la _Figura 2.1.1_.
INSERTAR FIGURA

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
 
Para la toma de datos primero se hizo la configuración del lidar, iniciando el objeto en MATLAB. Luego se realizaron 3 escaneos con 3 segundos de diferencia entre ellos. Esto haciendo uso de la función **LidarScan.m**. Con esta función se manda un mensaje al lidar mediante el protocolo SCIP 2.0. Este mensaje es "GD0044072500" el cual hace la petición de toma de datos en el ángulo establecido. Luego de hacer los escaneos se crean los arrays para cada pose. Estos arrays tienen forma _[num_escaneos, n_pasos]_. 

Para la visualización de los datos primero se hizo un promedio entre las medidas de los 3 escaneos para cada paso,que a su vez se realizó para cada pose. Se generó un vector de angulos, el cual tiene los rangos de los angulos escaneados y su longitud es igual a la cantidad de pasos en dicho escaneo. Con estos dos arrays, se usaron las funciones **lidarScan** para visualizar los puntos escaneados y **occumancyMap** para ver el mapa de ocupación local con la pose del lidar definida. Una vez se tenían los 3 mapas de ocupación referentes a las 3 poses, se usó **buildMap** para crear el mapa global uniendo los 3 mapas locales.

---
_Figura 2.2.2: Escaneo para Pose 1:_

![scan1](https://github.com/user-attachments/assets/6a059f93-ba7e-42d3-8fec-66644a1e17a3)
---
_Figura 2.2.3: Mapa local para Pose 1:_

![mapa1](https://github.com/user-attachments/assets/8990bf12-f871-4ed2-9d1c-545b81878495)
---
_Figura 2.2.4: Mapa local para Pose 2:_

![mapa2](https://github.com/user-attachments/assets/f88b2b5d-a809-47c6-a9c8-cf98151317f7)
---
_Figura 2.2.5: Mapa local para Pose 3:_

![mapa3](https://github.com/user-attachments/assets/b1e24ec7-96ac-4d53-858a-2b8b823154aa)
---
_Figura 2.2.6: Mapa global:_

![mapaT](https://github.com/user-attachments/assets/40691bf0-33a8-47f6-b707-0146b9ada72c)
---

### 2.2 Sensor RPLidar
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

#### Fotos 
Figura 2.2.7: Pose 1

![Pose1raw](https://github.com/user-attachments/assets/688266bf-f409-4d40-8958-b169ecdffbb0)
---

Figura 2.2.8: Pose 2

![Pose2raw](https://github.com/user-attachments/assets/1c02a293-fd44-4464-90e0-19e076a12b6c)
---

Figura 2.2.9: Pose 3

![Pose3raw](https://github.com/user-attachments/assets/650eed18-f9e3-465f-99cb-664aa9155ea2)
---

### 2.3 Sensor ultrasonido

### 2.4 Sensores Lego

## 3. ROS

### 3.1 Uso de ROS
