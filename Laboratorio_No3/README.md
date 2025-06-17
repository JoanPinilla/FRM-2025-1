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


**3.2. Algoritmo de solución**

**3.3. Video**
