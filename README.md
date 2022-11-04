# Laboratorio 5 de Robótica
## Universiad Nacional de Colombia
## 2022-2
***
### Autores
- Andrés Holguín Restrepo 
- Julián Andrés Caipa Prieto
### Profesores encargados
- Ing. Ricardo Emiro Ramírez H.
- Ing. Jhoan Sebastian Rodriguez R.
***


## Descripción

### Cinemática inversa

Para resolver la cinemática inversa del Pincher, se va a retomar su modelo DHstd realizado en el laboratorio 4.


![robot marcos](https://github.com/aholguinr/Lab4_Robotica_Caipa_Holguin/blob/main/Imagenes/dhrobot.png?raw=true)

Donde se determinaron los siguientes parámetros

| Theta    | d | a | alpha | Offset |
| :----:   | :----:      | :----:     | :----:     | :----:     |
|Theta 1         |  L1        |    0     |   90     |    0°     | 
|Theta 2         |    0      |     L2    |    0°    |    90°     |   
|Theta 3         |    0      |     L3   |  0°     |    0°     | 
|Theta 4         |      0    |     0    | 90°     |     90°    |   
|Theta 5         |      L4    |     0    | 0°        |     0°    |  


Donde las longitudes de los eslabones son:

L1=137 mm, L2=L3=105 mm y L4=95 mm


Ahora bien, se puede simplificar bastante el problema de cinemática inversa a partir de estas definiciones y se pueden resolver por separado.

#### Theta 1

$\theta_1$ es bastante sencillo de definir, ya que este va a depender únicamente de los valores deseados en dirección $x$ y $y$, donde están descritos por la tangente de $\theta_1$, motivo por el cual se tiene que:



```math
\theta_1=atan2(P_y,P_x)
```


#### Theta 2 y 3

$\theta_2$ y $\theta_3$ están descritos por un robot 2R, donde la altura y la distancia horizontal del funto final van a estar definidas por todas las variables involucradas. El ángulo de ataque $\beta$ afecta ambas ya que dependiendo de su valor, la componente correspondiente de la longitud $L_4$ se va a restar del punto final del pincher. Así mismo, la altura también se afecta por la longitud de $L_1$ y su valor deseado $P_z$, por lo que para el 2R se tiene que: 

$P_z-L_4sin(\beta)-L_1$

Ahora, para la componente horizontal, se tiene un parámetro #R# que está asociado a la suma vectorial de los puntos asociados $P_x$ y $P_y$. Además, semejante a la altura, se descompone $L_4$ en esta dirección, resultando que:


$r=\sqrt{P_x^2+P_y^2}$    

$D_r=r-L_4*cos(\beta)$

Con todo y lo anterior, ya se pueden definir $\theta_2$ y $\theta_3$ como un 2R común, teniendo en cuenta la orientación inicial de sus eslabones, donde el offset de $\theta_2$ es de $90°$ y de $\theta_3$ es de $-90°$. Con todo y lo anterior, mediante simplificaciones geométricas y algebráicas, se puede establecer que:


```math
     \theta_2=-2 atan((2 D_r L_2 - \sqrt{- D_r^4 - 2 D_r^2 D_z^2 + 2 D_r^2 L_2^2 + 2 D_r^2 L_3^2 - D_z^4 + 2 D_z^2 L_2^2 + 2 D_z^2 L_3^2 - L_2^4 + 2 L_2^2 L_3^2 - L_3^4})/(D_r^2 + D_z^2 + 2 D_z L_2 + L_2^2 - L_3^2))
```

```math
    \theta_3=-2atan((\sqrt{(- D_r^2 - D_z^2 + L_2^2 + 2 L_2 L_3 + L_3^2)(D_r^2 + D_z^2 - L_2^2 + 2L_2L_3 - L_3^2)} - 2L_2L_3)/(D_r^2 + D_z^2 - L_2^2 - L_3^2))
```

Donde se tiene que:
```math
\begin{align}
r=\sqrt{P_x^2+P_y^2}\\
    D_r=r-L_4 cos(\beta)\\
    D_z=P_z-L_4 sin(\beta)-L_1\\

\end{align}
```

Estas ecuaciones también fueron determinadas mediante Matlab simbólico, donde sus resultados correspondieron a estos recien expuestos.

#### Theta 4

En este punto, ya se tienen valores de $\theta_1$, $\theta_2$ y $\theta_3$, por lo que determinar $\theta_4$ es bastante sencillo teniendo en cuenta que geométricamente la suma de estos tres ángulos deben de dar $\beta$. De este modo:

```math
\begin{align}
\theta_4=\beta-\theta_2-\theta_3
\end{align}
```

#### Theta 5

Este ángulo está asociado a la herramienta y determina el movimiento del gripper. Con base a diferentes análisis realizados, en $\theta_5=0°$ se tiene una apertura completa, mientras que un valor de $\theta_5\pm70°$ es suficiente para generar el cierre necesario para el marcador.


## Restricciones

* Se debe utilizar las bases en madera y marcadores borrables para la practica, evite realizar escritura en otras superficies o con otros instrumentos.

Debido a esta restricción, solo se va a utilizar la superficie de las bases de tablero, logrando una separación adecuada entre cada rutina de dibujo.

* Se debe configurar correctamente la apertura y cierre del gripper para sujetar el marcador, si es necesario agregue al marcador recubrimientos para asegurar la rigidez de la sujeción.

Se agregan empaques al marcador para que estos otorguen una mejor capacidad de agarre para el gripper.

* Se debe cuidar la integridad física y eléctrica del brazo y demás elementos del sistema en todo momento. 

Debido al cuidado que toca tener con los gripper, las rutinas se realizan en tiempos prolongados con el fin de evitar velocidades altas que puedan resultar en impactos del gripper contra algún objeto o incluso contra si mismo.

* El trazado de letras y figuras se puede hacer en cualquier posición dentro del área de trabajo. Configure la posición, tamaño y orientación de los trazos de forma que todas las rutinas puedan quedar marcadas sobre la superficie sin tener que borrar otros trazos.

Se intenta mantener una uniformidad en los dibujos, motivo por el cual se alinean todos excepto el de trazo libre.


## Base portaherramientas
Cree una base porta herramienta para el marcador de marca Expo (genérico) de forma que cuando el brazo tome y suelte el marcador la orientación respecto a la herramienta sea válida para escribir sobre la superficie plana, esta base debe estar sujeta a la tabla de trabajo mediante un acople temporal, no dañe la superficie de la tabla.

![Base portaheramientas](zelda)


Para este laboratorio se creó una base portaherramientas tal que sostenga la tapa del marcador de manera vertical. La idea de esta base es que logre soportar la tapa y el mismo marcador de forma vertical, por lo que se diseñó en Inventor y luego se imprimió en 3D en material PLA. Esta base es lo suficientemente rígida para poder sostener verticalmente el marcador. 

![Base portaheramientas cargada](zelda)

Ahora bien, debido a las limitaciones de torque y rigidez del Pincher, se determina que el marcador no debe estar con la tapa puesta, sino simplemente apoyada sobre esta para que se de un buen agarre del brazo. Así mismo, debido a la baja controlabilidad del robot, la base del marcador debe sostenerse en el suelo para que esta no vaya a moverse a causa del brazo o de cualquier movimiento de la base. Por eso, lo que se realizó fue colocar cinta en sus bordes hacia el tablero, tal que se sostenga al suelo lo suficientemente fuerte para poder soportar movimientos del Pincher, pero que sea fácil de montar y de desmontar del tablero. Cabe resaltar que funciona incluso sin cinta, sin embargo se recomienda para no tener que devolver el portaherramientas a la posición del tablero designada a la carga y descarga.

![Base portaheramientas cargada con el marcador suelto](zelda)


## Rutinas

Para la parte de rutinas y de operación del Pincher se realiza un único script en Python y desde ahí se realizan todas las operaciones. En el video se muestra todo el procedimiento. A continuación puede ver el video que se realizó en la ejecución de todas las funcionalidades requeridas para el laboratorio.

[![video](https://i9.ytimg.com/vi_webp/urgtAmz2foA/mq2.webp?sqp=CMzQkJsG-oaymwEmCMACELQB8quKqQMa8AEB-AGUA4AC0AWKAgwIABABGGUgZShlMA8=&rs=AOn4CLASYZi-GAaXVjwizHQAFC0Swfcj1A)](https://youtu.be/urgtAmz2foA)

Además, en el repositorio se encuentra el archivo de Python donde también se comenta una breve explicación de todas las rutinas que se realizaron y demás instancias del archivo. [Click aquí para dirigirse al archivo Lab5.py](https://github.com/aholguinr/lab5_Robotica_Caipa_Holguin/blob/main/Catkin/src/dynamixel_one_motor/scripts/lab5.py)

Sin embargo, aún con estos recursos, de todos modos se van a explicar de manera generalizada las rutinas en este informe. A final de la explicación, se analizan los resultados.

### Cargar herramienta: 

El brazo se desplaza a la base portaherramienta, sujeta el marcador y se ubica en
una posición de espera.

Para realizar esto, se tienen tres puntos en el espacio claves. El home, un punto de seguridad que está ubicado encima del marcador y la posición de agarre. Con base a esto, se realiza el mapeo de puntos y mediante movimientos lentos se realiza la carga. Una vez el Pincher está ubicado para apretar el gripper, se tiene una parada de advertencia donde el usuario debe presionar alguna tecla para proseguir con la rutina. Se hace de este modo con el fin de que se pueda asegurar que el marcador se encuentra en la posición adecuada para su agarre. Una vez se prosigue con la rutina, vuelve a la posición de seguridad y luego a home.



![Posicion de agarre del marcador](zelda)

Dicho esto, en la práctica se encontraron diversas dificultades que no solo afectaron esta rutina sino todas las demás. La primera es debido a que se utilizaron los pincher viejos, la rigidez de estos es bastante cuestionable, motivo por el cual entre más se alejaba el centro de masa del pincher de su base de apoyo, su propio peso hacía doblar sus eslabones y por ende la herramienta bajaba más de lo que se proyectaba con la cinemática inversa. Esto generó a que en la mayoría de puntos específicos dentro del espacio de trabajo, se ajustaran los valores de $z$ con el fin de que esto generara la compensación necesaria para que el problema de la caida del pincher se corrijiera de la mejor manera. Sin embargo, esto no siempre se solucionaba, debido a que cada vez que se cambiaba a otro pincher, se comportaba diferente y esta compensación variaba, motivo por el cual no se intentó perfeccionar esta corrección.

Otro error que se presentó en diversas ocasiones es que pareciese que cada vez que se actualizaba el publisher para dirijir el pincher a un punto específico, por un pequeño instante de tiempo se reseteaban los torques de los servomotores, motivo por el cual el brazo tenía una caida bastante notoria a medida que su centro de masa se alejaba más de su base, semejante al problema anterior.

Estos dos errores también se mencionaron durante el video.

### Espacio de trabajo: 

El brazo dibuja dos arcos que representan los límites de espacio de trabajo diestro
plano sobre la superficie y regresa a una posición de espera.

Después de hacer pruebas con el pincher, se determina que los radios mínimos y máximos al centro del Pincher son de 17cm y de 29cm. Cabe resaltar que podrían se más extremos, pero se consideran adecuados con el fin de que el ángulo $\beta$ pueda ser $0$ sin mayores dificultades. También se tienen alturas diferentes para cada uno de los dos extremos de los arcos debido al error mencionado anteriormente.  Para el arco interior el $z$ es de $11cm$ mientras que para el arco exterior son valores de $16.3cm$, se pudo haber corregido aún más, sin embargo fueron valores adecuados para generar los arcos.

![Espacio de trabajo](zelda)

### Dibujo de Iniciales: 

El brazo dibuja al menos dos letras, iniciales de los nombres de los estudiantes, sobre
la superficie y retorna a una posición de espera.

En este caso se tienen las iniciales A y J, cada una de ellas va a estar inscrita en un cuadrado de 5cm de lado y ubicado dentro del espacio de trabajo. Mediante el uso de Matlab se logran determinar adeucadamente las coordenadas $x$ y $y$ de cada punto asociado a las letras. Debido a la velocidad de trabajo del pincher y al recorrido relativamente corto que debía hacer, no se usaron puntos intermedios ya que se obtenían resultados muy semejantes y más demorados. Dicho esto, se generó un resumen de puntos y se realizó la rutina para estos, donde al inicio y final de cada inicial, hay una posición de seguridad asociada con una altura por encima del suelo para que la entrada al tablero sea suavizada.

![Foto de iniciales]()


### Dibujo de figuras geométricas: 

Se dibuja sobre la superficie un triángulo equilátero, una circunferencia y
3 líneas rectas paralelas y regresa a una posición de espera.

Siguiendo con la escritura generada de las iniciales, el triángulo, circunferencia y las líneas se realizan a la derecha en fila de estas letras ya explicadas. Para el triángulo y líneas paralelas, semejante a las iniciales, con los puntos extremos es suficiente para realizar adecuadamente el dibujo de las figuras. Ahora bien, para el circulo es un caso totalmente diferente. Este se parametriza asociado a un número de puntos N, que en el video se realizó con $N=50$. Debido a esto, todos los puntos que se ejecutan en la rutina deben ser parametrizados, incluyendo el más dificl de todos, el parámetro $z$ debido a que este varía dependiendo de la extensión del pincher a la hora de dibujar. 

Dicho esto, a los puntos $P_x$ $P_y$ se asocian funciones de transormación de coordenadas polares a cartesianas para una circunferencia de radio $25mm$ con centro en $[0.225 -0.025]$. Para los $P_z$ se tiene una función de la forma $Zpiso-0.01+0.005*cos(Rads)$, donde $Rads$ es el arreglo de ángulos desde 0 a $2\pi$. Como se puede ver, este $z$ varía semejante a la propia coordenada $x$ del círculo, lo cual es válido debido a que el $x$ debe ser máximo en su inicio de trayectoria que es el punto más alejado, disminuye a un mínimo cuando está en $Rads=\pi$, y luego vuelve a su máximo.

### Dibujo de puntos: 

El brazo dibuja 5 puntos equidistantes y regresa a una posición de espera.

Para este caso, lo que se realizan son 5 puntos colineales que esten distanciados equitatiavmente entre sus puntos aledaños. Están inscritos sobre una recta vertical respecto a los ejes sobre los cuales se han estado generando las rutinas. Se realiza todo el mapeo de puntos para tener un Z de seguridad y luego un Z donde el marcador pueda simplemente hacer contacto con el tablero y luego volver a subir. Para esto se tuvo en cuenta el error mencionado anteriormente que con cada cambio de movimiento, se reinicia el torque de los motores y se cae. Se aprovechó este movimiento para que la caida generada por el error sea lo suficientemente precisa para que solo se genere un leve contacto. Luego de las cinco iteraciones, se devuelve a Home.

### Dibujo figura libre: 

Se dibuja una figura libre que utilice trazos rectos tanto curvos y regresa a una
posición de espera.

Para la rutina de figura libre se realiza una rosa estilizada de cuatro pétalos la cual está definida por la ecuación

$R^2=\sqrt{\abs{cos(2\theta)}}

Se realiza un mapeo de N puntos (50 en el caso del video), y se descomponen en sus coordenadas $x$ y $y$ con centro en coordenadas $[125 200]mm$. Para los puntoz de $z$, se realiza un semejante a lo realizado para la circunferencia, donde va a ser descrito por la función:

$z=Zpiso-0.005+0.01*cos(theta-\pi/2))$

Se realiza ese desfase de 90° debido a la orientación del robot. El Z debe ser máximo cuando sea su máxima elongación respecto a la base, lo cual sucede cercano a este desfase. De este modo, se tienen las 50 rutinas que se deben realizar para obtener la rosa. Esta rutina la antecede y precede un movimiento a una posición de seguridad para no empezar el contacto con el tablero de forma abrupta, y lo mismo de salida antes de dirigirse a home.


### Descarga de la herramienta: 

El brazo se desplaza a la base porta herramienta, suelta el marcador y se
ubica en una posición de Home.

Para la descarga de la herramieneta se genera una rutina equivalente a la de la carga de la herramienta, sin embargo este empieza con el gripper apretado y luego se suelta cuando el marcador se encuentra de nuevo en el portaherramientas. Cabe resaltar que se tienen unos desfases de $x$ y $y$ debido a que con el marcador cargado, las posiciones difieren a si este se hubiera realizado sin él. Una vez el pincher está en posición de descarga, se espera la entrada de teclado del usuario para efectivamente soltar la herramienta y que esta no se caiga sino que se coloque adecuadamente en el portaherramientas.



# Interacción con el usuario: 
## Script

Para realizar la GUI se realizó todo mediante consola. Al inicio se le pide al usuario seleccionar alguna de las rutinas mencionadas anteriormente. Cabe resaltar que primero se debe cargar el marcador para realizar las demás rutinas. Luego de seleccionar el número correspondiente y de teclear “enter”, empieza la rutina. Una vez este acabe, la consola vuelve a pedirle al usuario alguna de las rutinas existentes, o, salir del programa. Además, en caso de que se realice una entrada de datos no valida, se entiende que no se realiza ninguna rutina y se sale del programa.

## Se deben mostrar mensajes al usuario en pantalla (consola) en cada una de las etapas de operación: 
### Estado de la herramienta (cargada o descargada). 

Se le imprime al usuario si la herramienta está cargada o descargada antes de que este seleccione alguna rutina del programa.

### Posición actual del brazo. 

La posición “actual” del brazo se muestra al final de cada rutina. Se dice “actual” entre comillas ya que esta que se muestra es calculada mediante la cinemática directa, sin embargo como ya se ha mencionado, la rigidez del robot hace que estas coordenadas sean un poco diferentes al caso real.
### Rutina seleccionada. 
Cuando el usuario selecciona alguna de las diferentes rutinas, se le hace saber al usuario esta que seleccionó.

### Rutina finalizada. 

Al finalizar los movimientos de la rutina, se imprime un mensaje de finalización de la operación.

### Tiempo de ejecución de la última rutina.
Para mostrar el tiempo de ejecución de la última rutina, una vez que se seleccione alguna rutina, se hace una toma del tiempo, y luego se realiza otra al finalizar. Se calcula esta diferencia y se imprime, por lo que se puede conocer el tiempo de ejecución que tardó el Pincher en realizar la operación deseada.

## Demostraciones

Para mostrar el funcionamiento del Pincher, a continuación, se muestra el video completo de las operaciones a realizar en el laboratorio.
Video



## Verificación dimensional


### Se debe verificar las dimensiones, calidad de trazo, rectitud, radio y homogeneidad de todos los trazos. 


### Escoja e implemente una metodología para medir la precisión de los trazos. 


### Tome imágenes de los trazos y compárelas digitalmente con imágenes de las trayectorias ideales. 
A partir de las imágenes mostradas anteriormente, puede hacerse una comparación con imágenes de las trayectorias deseadas graficadas desde Matlab y comparar los resultados teóricos con los prácticos.


### Analice si el trazo es igual haciendo múltiples rutinas de escritura descargando y cargando la herramienta.

