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


## Búsqueda bibliográfica

## Sensores

### Sensor Hokuyo

### Sensor RPLidar
Para la práctica de este sensor se organizó el espacio de trabajo como se puede ver en la siguiente _Figura 1_. 

![workspace_rplidar](https://github.com/user-attachments/assets/29e7432f-f9cc-4a0b-922a-e5aafba2866c) 

_Figura 1: Plano del espacio de trabajo del RPLidar._



Siguiendo el marco de referncia visto en la imágen, las tres poses escogidas tienen los siguientes componentes:

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
Teniendo los archivos con los datos, lo siguiente es unir los 3 mapas para crear el mapa de todo el laberinto. Para esto se utilizan funciones como **lidarScan**, **occupancyMap** y **buildMap**. Con estas 3 funciones pasamos de escaneo (_Figura 2_), pasando por el mapa local (_Figura 3_) y, juntando estos tres (_Figura 4 y 5_), llegar hasta el mapa global (_Figura 6_).

---
_Figura 2: Escaneo para Pose 1:_

![scan1](https://github.com/user-attachments/assets/da092947-435d-4b89-895e-1a837e4900f3)
---
_Figura 3: Mapa de ocupación para Pose 1:_

![occupancymap1](https://github.com/user-attachments/assets/c9562caf-d682-42ca-94f2-6ba5fef72d1d)
---

_Figura 4: Mapa de ocupación parar Pose 2:_

![scan2](https://github.com/user-attachments/assets/3f9c6fe1-7c91-4e80-9cc0-d1d773e82968)
---

_Figura 5: Mapa de ocupación para Pose 3:_

![occ3](https://github.com/user-attachments/assets/8fefd9c6-bfaf-4845-b034-551a26ba7e83)
---

_Figura 6: Mapa de ocupación global:_

![occT](https://github.com/user-attachments/assets/db84ed38-4f17-46f9-abb7-fe0c73682c91)
---

Como se puede observar, el mapa de ocupación es en gran parte semejane con el espacio real. Sin embargo, presenta alguas dimensiones erroneas, cerca a la Pose 3. Estos problemas se deben a que en la creación del  espacio de trabajo por equivocación se modificó el mapa de manera que entre las Poses hay ligeras diferencias. A pesar de esto, cada mapa local representa de forma acertada los espacios que tomaron en la maqueta.

#### Fotos 
Figura 7: Pose 1

![Pose1raw](https://github.com/user-attachments/assets/688266bf-f409-4d40-8958-b169ecdffbb0)
---

Figura 8: Pose 2

![Pose2raw](https://github.com/user-attachments/assets/1c02a293-fd44-4464-90e0-19e076a12b6c)
---

Figura 9: Pose 3
![Pose3raw](https://github.com/user-attachments/assets/650eed18-f9e3-465f-99cb-664aa9155ea2)
---

### Sensor ultrasonido

### Sensores Lego

## ROS

### Uso de ROS
