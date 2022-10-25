from cmath import pi
import numpy as np
from numpy import interp
from numpy import pi
import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from dynamixel_workbench_msgs.srv import DynamixelCommand

__author__ = "Andres Holguin"

############################################################
#Funciones

def theta_1(P_x,P_y):
    return np.arctan2(P_y,P_x)

def theta_2(P_x,P_y,P_z,beta,L_1,L_2,L_3,L_4):
    R=np.sqrt(P_x**2+P_y**2)
    D_r=R-L_4*np.cos(beta)
    D_z=P_z-L_4*np.sin(beta)-L_1
    return 2*np.arctan((2*D_z*L_2 + np.sqrt(- D_r**4 - 2*D_r**2*D_z**2 + 2*D_r**2*L_2**2 + 2*D_r**2*L_3**2 - D_z**4 + 2*D_z**2*L_2**2 + 2*D_z**2*L_3**2 - L_2**4 + 2*L_2**2*L_3**2 - L_3**4))/(D_r**2 + 2*D_r*L_2 + D_z**2 + L_2**2 - L_3**2))

def theta_3(P_x,P_y,P_z,beta,L_1,L_2,L_3,L_4):
    R=np.sqrt(P_x**2+P_y**2)
    D_r=R-L_4*np.cos(beta)
    D_z=P_z-L_4*np.sin(beta)-L_1
    return -2*np.arctan(np.sqrt((- D_r**2 - D_z**2 + L_2**2 + 2*L_2*L_3 + L_3**2)*(D_r**2 + D_z**2 - L_2**2 + 2*L_2*L_3 - L_3**2))/(D_r**2 + D_z**2 - L_2**2 + 2*L_2*L_3 - L_3**2))

def theta_4(beta,theta_2,theta_3):
    return beta-theta_2-theta_3

def ikine4r(P_x,P_y,P_z,beta):
    L_1=0.137
    L_2=0.105
    L_3=0.105
    L_4=0.095
    T1=theta_1(P_x*0.001,P_y*0.001)
    T2=theta_2(P_x*0.001,P_y*0.001,P_z*0.001,beta,L_1,L_2,L_3,L_4)
    T3=theta_3(P_x*0.001,P_y*0.001,P_z*0.001,beta,L_1,L_2,L_3,L_4)
    T4=theta_4(beta,T2,T3)
    return np.array([T1,T2,T3,T4])

def ConvGra2An(q,q_Home):
    ListDouble=q+q_Home
    for i in range(len(ListDouble)):
        ListDouble[i]=int(np.interp(ListDouble[i],[0, 300],[0, 1023]))
    return ListDouble

def ConvRad2An(q,q_Home):
    ListDouble=q+q_Home
    for i in range(len(ListDouble)):
        ListDouble[i]=int(np.interp(ListDouble[i],[0, 300*np.pi/180],[0, 1023]))
    return ListDouble

def ConvGra2Q(Gra,Home):

    return 0

def ConvPosArr2QArr(PosArr):
    [r,c]=PosArr.shape
    QArr=np.zeros([r,c])
    for i in range(r):
        QArr[i,:]=ikine4r(PosArr[i,0],PosArr[i,1],PosArr[i,2],PosArr[i,3])    
    return QArr

def mapAn(theta):
    return int(interp(theta,[-150, 150],[0, 1023]))

#Función que cambia valores de registros de los motores del Pincher.
def jointCommand(command, id_num, addr_name, value, time):
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

#Función callback que se llama en el listener. Cambia la variable global de la posición de los ángulos de los motores.
#Se realiza el ajuste a grados y a la posición home que se estableció
def callback(data):
    global PosActual
    PosActual=np.multiply(data.position,180/pi)
    PosActual[2]=PosActual[2]-90 #Puede ser - o + dependiendo del pincher

#Función que genera el subscriber para obtener los estados de las articulaciones
def listener():
    rospy.init_node('joint_listener', anonymous=True)
    rospy.Subscriber("/dynamixel_workbench/joint_states", JointState,callback)

#Rutina de movimiento con puntos intermedios. Uno define el número de movimientos N para llegar a un punto.
#Se ejecuta en un ciclo for hasta llegar al punto final.
def movPartido(j,Goal,Actual):
    N=5
    delta=((Goal-Actual)/N)
    for i in range(N):
        jointCommand('', (j+1), 'Goal_Position', int(Actual+delta*(i+1)), 0.1)
        time.sleep(0.1)

def RutHome(PosHome):
    print('Ir a home\n')
    for i in range(5):
        jointCommand('', (i+1), 'Goal_Position', PosHome[i], 1)
        print('Moviento eslabon: '+str(i+1)+'\n')
        time.sleep(0.5)
    print('En home\n')

#Función de impresión de opciones
def PrintOpciones():
    print("Seleccione la rutína a ejecutar:")
    print("1:\tCargar herramienta")
    print("2:\tEspacio de trabajo")
    print("3:\tDibujo de Iniciales")
    print("4:\tDibujo de figuras geom´etricas")
    print("5:\tDibujo de puntos")
    print("6:\tDibujo figura libre")
    print("7:\tDescarga de la herramienta")
    print("8:\tSalr")

############################################################
############################################################

# Valores

#Arreglo de torques
Torques=[1023,1023,1023,1023,1023]

#Arreglo de offsets de Home
AngHomeGrad=np.array([150,150,60,150,150])
AngHomeRad=np.multiply(AngHomeGrad,np.pi/180)

#Ángulos de posición home con y sin herramienta montada
HomeS=np.array([0,0,0,0,0])
HomeC=np.array([0,0,0,0,45])

#Alturas Z en metros y ángulo de ataque
Zpiso=0.095
Zportaherramientas=0.1
Zseguridad=0.15
beta=0

#Valores analogos de Home
PosAnHomeS=ConvGra2An(HomeS,AngHomeGrad)
PosAnHomeC=ConvGra2An(HomeC,AngHomeGrad)


#Angulos de portaherramientas
AngPortaHerramientasArriba=ikine4r(0.1,-0.2,Zseguridad,0)
AngPortaHerramientasAbajo=ikine4r(0.1,-0.2,Zportaherramientas,0)



#Angulos para arcos de espacio de trabajo diestro

lim=70*np.pi/180
AngArcMinIzq=ikine4r(0.168,0,Zpiso,0)
AngArcMinIzq[0]=lim
AngArcMinDer=ikine4r(0.168,0,Zpiso,0)
AngArcMinDer[0]=-lim

AngArcMaxIzq=ikine4r(0.3,0,Zpiso,0)
AngArcMaxIzq[0]=lim
AngArcMaxDer=ikine4r(0.3,0,Zpiso,0)
AngArcMaxDer[0]=-lim

# Mapa de puntos
PuntosA=np.array([[0.200,0.150,Zpiso,beta],[0.250,0.125,Zpiso,beta],[0.200,0.100,Zpiso,beta],[0.225,0.1125,Zpiso,beta],[0.225,0.1375,Zpiso,beta]])
PuntosJ=np.array([[0.250,0.100,Zpiso,beta],[0.250,0.050,Zpiso,beta],[0.250,0.075,Zpiso,beta],[0.200,0.075,Zpiso,beta],[0.200,0.100,Zpiso,beta]])
PuntosTri=np.array([[0.200,0,Zpiso,beta],[0.200,0.050,Zpiso,beta],[0.2433,0.025,Zpiso,beta],[0.200,0,Zpiso,beta]])
PuntosR1=np.array([[0.200,-0.050,Zpiso,beta],[0.200,-0.100,Zpiso,beta]])
PuntosR2=np.array([[0.225,-0.050,Zpiso,beta],[0.225,-0.100,Zpiso,beta]])
PuntosR3=np.array([[0.250,-0.050,Zpiso,beta],[0.250,-0.100,Zpiso,beta]])
PuntosEQTablero=np.array([[0.200,-0.125,Zpiso,beta],[0.210,-0.125,Zpiso,beta],[0.220,-0.125,Zpiso,beta],[0.230,-0.125,Zpiso,beta],[0.240,-0.125,Zpiso,beta],[0.250,-0.125,Zpiso,beta]])
PuntosEQSeguridad=np.array([[0.200,-0.125,Zseguridad,beta],[0.210,-0.125,Zseguridad,beta],[0.220,-0.125,Zseguridad,beta],[0.230,-0.125,Zseguridad,beta],[0.240,-0.125,Zseguridad,beta],[0.250,-0.125,Zseguridad,beta]])

#Qpuntos
QPuntosA=ConvPosArr2QArr(PuntosA)
QPuntosJ=ConvPosArr2QArr(PuntosJ)
QPuntosTri=ConvPosArr2QArr(PuntosTri)
QPuntosR1=ConvPosArr2QArr(PuntosR1)
QPuntosR2=ConvPosArr2QArr(PuntosR2)
QPuntosR3=ConvPosArr2QArr(PuntosR3)
QPuntosEQTablero=ConvPosArr2QArr(PuntosEQTablero)
QPuntosEQSeguridad=ConvPosArr2QArr(PuntosEQSeguridad)

############################################################


#Main
if __name__ == '__main__':
    try:
        #Activar el subscriber.
        listener()

        print("Bienvenido al laboratorio 5 de robótica!")
        print("Por favor espere a que el robot se dirija a home")
        RutHome(PosHomeS)

        PrintOpciones()
        caso=0
        while(caso!=8):
            
            caso=int(input())
            if caso==1:
                print("Se seleccionó: Cargar herramienta")


                time.sleep(0.5)
                RutHome(PosHomeC)
            elif caso==2:
                print("Se seleccionó: Espacio de trabajo")


                time.sleep(0.5)
                RutHome(PosHomeC)
            elif caso==3:
                print("Se seleccionó: Dibujo de Iniciales")


                time.sleep(0.5)
                RutHome(PosHomeC)
            elif caso==4:
                print("Se seleccionó: Dibujo de figuras geométricas")


                time.sleep(0.5)
                RutHome(PosHomeC)
            elif caso==5:
                print("Se seleccionó: Dibujo de puntos")


                time.sleep(0.5)
                RutHome(PosHomeC)
            elif caso==6:
                print("Se seleccionó: Dibujo figura libre")


                time.sleep(0.5)
                RutHome(PosHomeC)
            elif caso==7:
                print("Se seleccionó: Descarga de la herramienta")



                time.sleep(0.5)
                RutHome(PosHomeS)
            elif caso==8:
                print("Se seleccionó: Salr")
            else:
                print("Opción no válida. Saliendo.")
                break


        print("Finalizando rutinas")

        #Definir los límites de torque de los motores.
        for i in range(5):    
            jointCommand('', (i+1), 'Torque_Limit', Torques[i], 0)
            

        P_x=200
        P_y=0
        P_z=145
        beta=np.pi/6

        q=ikine4r(P_x,P_y,P_z,beta)
        print(q)
        
        for i in range(4):
            print(mapAn(q[i]))
        input()


        #Rutina para ir al home.#Rutina para ir al home.
        RutHome(PosHome)


        







#Cargar herramienta: el brazo se desplaza a la base porta herramienta, sujeta el marcador y se ubica en
#una posicion de espera.


#Espacio de trabajo: el brazo dibuja dos arcos que representan los limites de espacio de trabajo diestro
#plano sobre la superficie y regresa a una posicion de espera.


#Dibujo de Iniciales: El brazo dibuja al menos dos letras, iniciales de los nombres de los estudiantes, sobre
#la superficie y retorna a una posicion de espera.


#Dibujo de figuras geometricas: Se dibuja sobre la superficie un triangulo equilatero, una circunferencia y
#3 lineas rectas paralelas y regresa a una posicion de espera.


#Dibujo de puntos: El brazo dibuja 5 puntos equidistantes y regresa a una posicion de espera.
#Dibujo figura libre: Se dibuja una figura libre que utilice trazos rectos tanto curvos y regresa a una
#posicion de espera.


#Descarga de la herramienta: el brazo se desplaza a la base porta herramienta, suelta el marcador y se
#ubica en una posicion de Home.
#





    except rospy.ROSInterruptException:
        pass



