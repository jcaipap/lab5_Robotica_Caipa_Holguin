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


## Base portaherramientas
Cree una base porta herramienta para el marcador de marca Expo (genérico) de forma que cuando el brazo tome y suelte el marcador la orientación respecto a la herramienta sea válida para escribir sobre la superficie plana, esta base debe estar sujeta a la tabla de trabajo mediante un acople temporal, no dañe la superficie de la tabla.

Para este laboratorio se creó una base portaherramientas tal que sostenga la tapa del marcador de manera vertical. La idea de esta base es que logre soportar la tapa y el mismo marcador de forma vertical, por lo que se diseñó en Inventor y luego se imprimió en 3D en material PLA. Esta base es lo suficientemente rígida para poder sostener verticalmente el marcador. 

Ahora bien, debido a las limitaciones de torque y rigidez del Pincher, se determina que el marcador no debe estar con la tapa puesta, sino simplemente apoyada sobre esta para que se de un buen agarre del brazo. Así mismo, debido a la baja controlabilidad del robot, la base del marcador debe sostenerse en el suelo para que esta no vaya a moverse a causa del brazo o de cualquier movimiento de la base. Por eso, lo que se realizó fue colocar cinta en sus bordes hacia el tablero, tal que se sostenga al suelo lo suficientemente fuerte para poder soportar movimientos del Pincher, pero que sea fácil de montar y de desmontar del tablero.


### Cargar herramienta: 

El brazo se desplaza a la base porta herramienta, sujeta el marcador y se ubica en
una posición de espera.



### Espacio de trabajo: 

El brazo dibuja dos arcos que representan los límites de espacio de trabajo diestro
plano sobre la superficie y regresa a una posición de espera.

### Dibujo de Iniciales: 

El brazo dibuja al menos dos letras, iniciales de los nombres de los estudiantes, sobre
la superficie y retorna a una posición de espera.

### Dibujo de figuras geométricas: 

Se dibuja sobre la superficie un triángulo equilátero, una circunferencia y
3 líneas rectas paralelas y regresa a una posición de espera.

### Dibujo de puntos: 

El brazo dibuja 5 puntos equidistantes y regresa a una posición de espera.
### Dibujo figura libre: 

Se dibuja una figura libre que utilice trazos rectos tanto curvos y regresa a una
posición de espera.
### Descarga de la herramienta: 

El brazo se desplaza a la base porta herramienta, suelta el marcador y se
ubica en una posición de Home.

# Interacción con el usuario: 
## Se debe crear un programa (script) que permita seleccionar que rutina se desea ejecutar, el control de selección se debe hacer mediante comandos o teclas específicas del teclado del computador. Las rutinas de escritura no se deben poder ejecutar si no se ha cargado la herramienta (marcador). 
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

