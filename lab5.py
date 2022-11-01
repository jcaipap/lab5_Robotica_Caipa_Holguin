from cmath import pi
import numpy as np
from numpy import interp
from numpy import pi
import rospy
import roboticstoolbox as rt

import time
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from dynamixel_workbench_msgs.srv import DynamixelCommand
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

__author__ = "Andres Holguin"

############################################################
#Funciones

#Arreglo de offsets de Home
AngHomeGrad=np.array([0,0,-90,0,0])
AngHomeRad=np.multiply(AngHomeGrad,np.pi/180)



# Funcion de cinemática inversa de theta 1
def theta_1(P_x,P_y):
    return np.arctan2(P_y,P_x)

# Funcion de cinemática inversa de theta 2
def theta_2(P_x,P_y,P_z,beta,L_1,L_2,L_3,L_4):
    R=np.sqrt(P_x**2+P_y**2)
    D_r=R-L_4*np.cos(beta)
    D_z=P_z-L_4*np.sin(beta)-L_1
    return -2*np.arctan((2*D_r*L_2 - np.sqrt(- D_r**4 - 2*D_r**2*D_z**2 + 2*D_r**2*L_2**2 + 2*D_r**2*L_3**2 - D_z**4 + 2*D_z**2*L_2**2 + 2*D_z**2*L_3**2 - L_2**4 + 2*L_2**2*L_3**2 - L_3**4))/(D_r**2 + D_z**2 + 2*D_z*L_2 + L_2**2 - L_3**2))

# Funcion de cinemática inversa de theta 3
def theta_3(P_x,P_y,P_z,beta,L_1,L_2,L_3,L_4):
    R=np.sqrt(P_x**2+P_y**2)
    D_r=R-L_4*np.cos(beta)
    D_z=P_z-L_4*np.sin(beta)-L_1
    return -2*np.arctan((np.sqrt((- D_r**2 - D_z**2 + L_2**2 + 2*L_2*L_3 + L_3**2)*(D_r**2 + D_z**2 - L_2**2 + 2*L_2*L_3 - L_3**2)) - 2*L_2*L_3)/(D_r**2 + D_z**2 - L_2**2 - L_3**2))

# Funcion de cinemática inversa de theta 4
def theta_4(beta,theta_2,theta_3):
    return beta-theta_2-theta_3

# Funcion de cinemática inversa del pincher
def ikine4r(P_x,P_y,P_z,beta):
    L_1=0.137
    L_2=0.105
    L_3=0.105
    L_4=0.095
    T1=theta_1(P_x,P_y)
    T2=theta_2(P_x,P_y,P_z,beta,L_1,L_2,L_3,L_4)
    T3=theta_3(P_x,P_y,P_z,beta,L_1,L_2,L_3,L_4)
    T4=theta_4(beta,T2,T3)
    return np.array([round(T1,3),round(T2,3),round(T3,3),round(T4,3)])

# Funcion de cinemática directa del Pincher
def fkine(theta_1,theta_2,theta_3,theta_4,theta_5):
    L_1=0.137
    L_2=0.105
    L_3=0.105
    L_4=0.095
    beta=np.arccos(np.cos(theta_2 + theta_3 + theta_4)*np.cos(theta_1))
    P_x=np.cos(theta_1)*(L_3*np.cos(theta_2 + theta_3) - L_2*np.sin(theta_2) + L_4*np.cos(theta_2 + theta_3 + theta_4))
    P_y=np.sin(theta_1)*(L_3*np.cos(theta_2 + theta_3) - L_2*np.sin(theta_2) + L_4*np.cos(theta_2 + theta_3 + theta_4))
    P_z=L_1 + L_3*np.sin(theta_2 + theta_3) + L_2*np.cos(theta_2) + L_4*np.sin(theta_2 + theta_3 + theta_4)
    if theta_5>40*np.pi/180:
        Grip=0
    else:
        Grip=1
    return np.array([P_x,P_y,P_z,beta,Grip])

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

def ActualizarRegistros(P_o):
    for i in range(len(P_o)):
        jointCommand('', (i+1), 'Goal_Position', int(P_o[i]), 0)
        s=i
    print(P_o)



#Función callback que se llama en el listener. Cambia la variable global de la posición de los ángulos de los motores.
#Se realiza el ajuste a grados y a la posición home que se estableció
def callback(data):
    global PosActual
    PosActual=np.multiply(data.position,180/pi)
    PosActual[2]=PosActual[2]-90 #Puede ser - o + dependiendo del pincher

#Función que genera el subscriber para obtener los estados de las articulaciones
def listener():
    rospy.init_node('joint_listener', anonymous=False)
    rospy.Subscriber("/dynamixel_workbench/joint_states", JointState,callback)

pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
rospy.init_node('joint_publisher', anonymous=False)
state = JointTrajectory() 
state.header.stamp = rospy.Time.now()
state.joint_names = ["joint_1","joint_2","joint_3","joint_4","joint_4"]


def joint_publisher(q,t):
    state = JointTrajectory()
    state.header.stamp = rospy.Time.now()
    state.joint_names = ["joint_1","joint_2","joint_3","joint_4","joint_5"]
    point = JointTrajectoryPoint()
    point.positions = q 
    point.time_from_start = rospy.Duration(t)
    state.points.append(point)
    pub.publish(state)
    print('Cambio de punto q:')  
    print(q) 
    time.sleep(4*t)



def Rut2p(Pos,t,tool):
    q=np.concatenate(((ikine4r(Pos[0],Pos[1],Pos[2],Pos[3])+AngHomeRad[0:4]),np.array([tool])), axis=0)
    time.sleep(2.5*t)
    joint_publisher(q,t)
    time.sleep(2.5*t)


#Función para calcular la trayectoria entre dos puntos mediante un perfil trapezoidal con N puntos
def TrajRectDosPuntos(P_i,P_f,N):
    c=len(P_i)
    td=np.zeros([N,c])
    for i in range(c):
        if P_i[i]!=P_f[i]:
            tg=rt.trapezoidal(P_i[i],P_f[i],N)
            td[:,i]=tg.s
        else:
            td[:,i]=P_i[i]
    return td

def CompletoPuntos(Puntos,N):
    [r,c]=Puntos.shape
    Trayectoria=np.zeros([r-1,N,c])
    for i in range(r-1):
        Pi=Puntos[i,:]
        Pf=Puntos[i+1,:]
        Trayectoria[i,:,:]=TrajRectDosPuntos(Pi,Pf,N)
    Arreglo=Trayectoria[0,:,:]
    for i in range(r-2):
        Arreglo=np.concatenate((Arreglo,Trayectoria[i+1,:,:]), axis=0)
    return Arreglo

def RealizarRutina(Puntos,N,t,tool):
    PuntosTrayectoria=CompletoPuntos(Puntos,N)
    C1=ConvPosArr2QArr(PuntosTrayectoria[:,:])
    [r,c]=C1.shape
    for i in range(r):
        time.sleep(2.5*t)
        Rut2p(C1[i,:],t,tool)


#Función de impresión de opciones
def PrintOpciones():
    print("Seleccione la rutína a ejecutar:")
    print("1:\tCargar herramienta")
    print("2:\tEspacio de trabajo")
    print("3:\tDibujo de Iniciales")
    print("4:\tDibujo de figuras geométricas")
    print("5:\tDibujo de puntos")
    print("6:\tDibujo figura libre")
    print("7:\tDescarga de la herramienta")
    print("8:\tSalr")

############################################################
############################################################

# Valores

#Arreglo de torques

#Ángulos de posición home con y sin herramienta montada

posHome=np.array([0.2,0,0.242,0])

#Alturas Z en metros y ángulo de ataque

Zpiso=0.138
ZpisoInt=0.11
ZpisoEx=0.16
Zportaherramientas=0.155
Zseguridad=0.2
ZseguridadEx=0.24
ZseguridadPH=0.25
beta=0*np.pi/180
betaEx=10*np.pi/180

#Posiciones
coorPortaHerramientasArriba=np.array([0.1,-0.2,ZseguridadPH,betaEx])
coorPortaHerramientasAbajo=np.array([0.1,-0.2,Zportaherramientas,betaEx])

#Angulos para arcos de espacio de trabajo diestro
lim=70*np.pi/180

#Valores de ángulo de cierre del gripper
gripClose=70*np.pi/180
gripOpen=0

#Radios de espacio de trabajo mínimo y máximo
rmin=0.17
rmax=0.29

#Posiciones físicas del arco menor y mayor
PosArcMinIzq=np.array([[rmin*np.cos(lim),rmin*np.sin(lim),Zseguridad,0],[rmin*np.cos(lim),rmin*np.sin(lim),ZpisoInt,0]])
PosArcMinDer=np.array([[rmin*np.cos(-lim),rmin*np.sin(-lim),ZpisoInt,0],[rmin*np.cos(-lim),rmin*np.sin(-lim),Zseguridad,0]])
PosArcMaxDer=np.array([[rmax*np.cos(-lim),rmax*np.sin(-lim),Zseguridad,betaEx],[rmax*np.cos(-lim),rmax*np.sin(-lim),ZpisoEx,betaEx]])
PosArcMaxIzq=np.array([[rmax*np.cos(lim),rmax*np.sin(lim),ZpisoEx-0.02,betaEx],[rmax*np.cos(lim),rmax*np.sin(lim),Zseguridad+0.015,betaEx]])

# Mapa de puntos

PuntosA=np.array([[0.200,0.150,Zseguridad,betaEx],[0.200,0.150,Zpiso,betaEx],[0.250,0.125,Zpiso,betaEx],[0.200,0.100,Zpiso-0.005,betaEx],[0.225,0.1125,Zpiso,betaEx],[0.225,0.1375,Zpiso,betaEx],[0.225,0.1375,Zseguridad,betaEx]])

PuntosJ=np.array([[0.250,0.100,Zseguridad+0.01,betaEx],[0.250,0.100,Zpiso+0.01,betaEx],[0.250,0.050,Zpiso,betaEx],[0.250,0.075,Zpiso,betaEx],[0.200,0.075,Zpiso-0.012,betaEx],[0.200,0.100,Zpiso-0.012,betaEx],[0.200,0.100,Zseguridad,betaEx]])

PuntosTri=np.array([[0.200,0,Zseguridad,betaEx],[0.200,0,Zpiso-0.02,betaEx],[0.200,0.050,Zpiso-0.017,betaEx],[0.2433,0.025,Zpiso-0.005,betaEx],[0.200,0,Zpiso-0.015,betaEx],[0.200,0,Zseguridad,betaEx]])

PuntosRectasParalelas=np.array([[0.200,-0.050,Zseguridad,betaEx],[0.200,-0.050,Zpiso-0.01,betaEx],[0.200,-0.100,Zpiso-0.01,betaEx],[0.200,-0.100,Zseguridad,betaEx],[0.225,-0.100,Zseguridad,betaEx],[0.225,-0.100,Zpiso,betaEx],[0.225,-0.050,Zpiso,betaEx],[0.225,-0.050,Zseguridad,betaEx],[0.250,-0.050,Zseguridad,betaEx],[0.250,-0.050,Zpiso,betaEx],[0.250,-0.100,Zpiso,betaEx],[0.250,-0.100,Zseguridad,betaEx]])

PuntosEQTablero=np.array([[0.200,-0.125,Zseguridad,betaEx],[0.200,-0.125,Zpiso+0.016,betaEx],[0.200,-0.125,Zseguridad-0.005,betaEx],[0.210,-0.125,Zseguridad-0.005,betaEx],[0.210,-0.125,Zpiso+0.016,betaEx],[0.210,-0.125,Zseguridad-0.005,betaEx],[0.220,-0.125,Zseguridad-0.005,betaEx],[0.220,-0.125,Zpiso+0.022,betaEx],[0.220,-0.125,Zseguridad-0.005,betaEx],[0.230,-0.125,Zseguridad-0.005,betaEx],[0.230,-0.125,Zpiso+0.027,betaEx],[0.230,-0.125,Zseguridad-0.005,betaEx],[0.240,-0.125,Zseguridad-0.005,betaEx],[0.240,-0.125,Zpiso+0.029,betaEx],[0.240,-0.125,Zseguridad+0.03,betaEx]])

############################################################

#Main
if __name__ == '__main__':
    try:
        #Activar el subscriber.
        #listener()

        #Definir los límites de torques (se dejan en el máximo)
        Torques=[1023,1023,1023,1023,1023]
        for i in range(5):    
            jointCommand('', (i+1), 'Torque_Limit', Torques[i], 0)

        print("Bienvenido al laboratorio 5 de robótica!")
        print("Por favor espere a que el robot se dirija a home")

        Rut2p(posHome,1,gripOpen)
        Cargado=0

        caso=0
        while(caso!=8):
            PrintOpciones()
            caso=int(input())
            if caso==1:
                if Cargado==0: 
                    print("Se seleccionó: Cargar herramienta")
                    start_time = time.time()

                    Rut2p(coorPortaHerramientasArriba,0.3,gripOpen)
                    Rut2p(coorPortaHerramientasAbajo,0.5,gripOpen)
                    input()
                    print("Apretar gripper")
                    Rut2p(coorPortaHerramientasAbajo,0.5,gripClose)
                    print("Volviendo a home")
                    Rut2p(coorPortaHerramientasArriba,1,gripClose)
                    Rut2p(posHome,1,gripClose)
                    #Apretar el gripper

                    end_time = time.time()
                    Tiempo=end_time-start_time
                    print("\ntiempo de ejecucion: %.2f s" % Tiempo)
                    Cargado=1
                else: 
                    print("La herramienta ya está cargada")
                
            elif caso==2:
                if Cargado==1:
                    #print("Se seleccionó: Espacio de trabajo")
                    #start_time = time.time()
                    #Rut2p(PosArcMinIzq[0,:],3,gripClose)
                    #input()
                    #Rut2p(PosArcMinIzq[1,:],0.5,gripClose)
                    #input()
                    #Rut2p(PosArcMinDer[0,:],0.5,gripClose)
                    #input()
                    #Rut2p(PosArcMinDer[1,:],0.5,gripClose)
                    #input()
                    #Rut2p(PosArcMaxDer[0,:],1,gripClose)
                    #input()
                    #Rut2p(PosArcMaxDer[1,:],2,gripClose)
                    #input()
                    #Rut2p(PosArcMaxIzq[0,:],1.5,gripClose)
                    #input()
                    #Rut2p(PosArcMaxIzq[1,:],1.5,gripClose)
                    #input()
                    #Rut2p(PosArcMinIzq[0,:],1,gripClose)
                    #input()
                    #Rut2p(posHome,1.5,gripClose)

                    ArrPos=[PosArcMinIzq[0,:],PosArcMinIzq[1,:],PosArcMinDer[0,:],PosArcMinDer[1,:],PosArcMaxDer[0,:],PosArcMaxDer[1,:],PosArcMaxIzq[0,:],PosArcMaxIzq[1,:],PosArcMinIzq[0,:],posHome]
                    ArrTiempo=[1.5,0.5,0.5,0.5,1,2,1.5,1.5,0.5,1]
                    for i in range(len(ArrPos)):
                        input()
                        Rut2p(ArrPos[i],ArrTiempo[i],gripClose)
                    input()
                    Rut2p(posHome,1.5,gripClose)

                    end_time = time.time()
                    Tiempo=end_time-start_time
                    print("\ntiempo de ejecucion: %.2f s" % Tiempo)
                else: 
                    print("La herramienta no está cargada")

            elif caso==3:
                if Cargado==1:
                    print("Se seleccionó: Dibujo de Iniciales")
                    start_time = time.time()

                    #Rut2p(PuntosA[0,:],1.5,gripClose)
                    #Rut2p(PuntosA[1,:],1.5,gripClose)
                    #Rut2p(PuntosA[2,:],1.5,gripClose)
                    #Rut2p(PuntosA[3,:],0.5,gripClose)
                    #Rut2p(PuntosA[4,:],0.5,gripClose)
                    #Rut2p(PuntosA[5,:],0.5,gripClose)
                    #Rut2p(PuntosA[6,:],1,gripClose)
                    #time.sleep(0.5)
                    #Rut2p(PuntosJ[0,:],1,gripClose)
                    #Rut2p(PuntosJ[1,:],1.5,gripClose)
                    #Rut2p(PuntosJ[2,:],0.5,gripClose)
                    #Rut2p(PuntosJ[3,:],0.5,gripClose)
                    #Rut2p(PuntosJ[4,:],0.5,gripClose)
                    #Rut2p(PuntosJ[5,:],0.5,gripClose)
                    #Rut2p(PuntosJ[6,:],1,gripClose)
                    #Rut2p(posHome,1,gripClose)
                    
                    #ArrPos=[PuntosA[0,:],PuntosA[1,:],PuntosA[2,:],PuntosA[3,:],PuntosA[4,:],PuntosA[5,:],PuntosA[6,:],PuntosJ[0,:],PuntosJ[1,:],PuntosJ[2,:],PuntosJ[3,:],PuntosJ[4,:],PuntosJ[5,:],PuntosJ[6,:],posHome]
                   
                    ArrTiempo=[1.5,1.5,1.5,0.5,0.5,0.5,1,1,1.5,0.5,0.5,0.5,0.5,1,1]
                    for i in range(7):
                        Rut2p(PuntosA[i,:],ArrTiempo[i],gripClose)

                    time.sleep(1)

                    for i in range(7):
                        Rut2p(PuntosJ[i,:],ArrTiempo[i],gripClose)

                    end_time = time.time()
                    Tiempo=end_time-start_time
                    print("\ntiempo de ejecucion: %.2f s" % Tiempo)
                else: 
                    print("La herramienta no está cargada")

            elif caso==4:
                if Cargado==1:
                    print("Se seleccionó: Dibujo de figuras geométricas")
                    start_time = time.time()

                    #TRIANGULO
                    #Rut2p(PuntosTri[0,:],1.5,gripClose)
                    #Rut2p(PuntosTri[1,:],1.5,gripClose)
                    #Rut2p(PuntosTri[2,:],1.5,gripClose)
                    #Rut2p(PuntosTri[3,:],1.5,gripClose)
                    #Rut2p(PuntosTri[4,:],1.5,gripClose)
                    
                    ArrTiempo=[1.5,1.5,1.5,1.5,1.5]
                    for i in range(5):
                        Rut2p(PuntosTri[i,:],ArrTiempo[i],gripClose)

                    #CIRCULO
                    N=50
                    Rads=np.linspace(0,2*np.pi,N)
                    ZPuntos=np.ones(N)*(Zpiso-0.012)
                    BetaPuntos=np.ones(N)*betaEx
                    t=0.2
                    ArrTiempo=np.ones(N)*t
                    PuntosCirc=np.zeros([N,4])
                    PuntosCirc[:,0]=0.225+0.025*np.cos(Rads)
                    PuntosCirc[:,1]=-0.025+0.025*np.sin(Rads)
                    PuntosCirc[:,2]=ZPuntos
                    PuntosCirc[:,3]=BetaPuntos
                    PosIni=[0.25,0,ZseguridadEx,betaEx]

                    Rut2p(PosIni,1.5,gripClose)
                    input()
                    for i in range(N):
                        Rut2p(PuntosCirc[i,:],ArrTiempo[i],gripClose)
                    input()
                    Rut2p(posHome,2,gripClose)


                    #RECTAS PARALELAS
                    #Rut2p(PuntosRectasParalelas[0,:],1,gripClose)
                    #Rut2p(PuntosRectasParalelas[1,:],1,gripClose)
                    #Rut2p(PuntosRectasParalelas[2,:],1,gripClose)
                    #Rut2p(PuntosRectasParalelas[3,:],1,gripClose)
                    #Rut2p(PuntosRectasParalelas[4,:],1,gripClose)
                    #Rut2p(PuntosRectasParalelas[5,:],1,gripClose)
                    #Rut2p(PuntosRectasParalelas[6,:],1,gripClose)
                    #Rut2p(PuntosRectasParalelas[7,:],1,gripClose)
                    #Rut2p(PuntosRectasParalelas[8,:],1,gripClose)
                    #Rut2p(PuntosRectasParalelas[9,:],1,gripClose)
                    #Rut2p(PuntosRectasParalelas[10,:],1,gripClose)
                    #Rut2p(PuntosRectasParalelas[11,:],1,gripClose)
                    #Rut2p(posHome,1,gripClose)

                    #ArrPos=[PuntosRectasParalelas[0,:],PuntosRectasParalelas[1,:],PuntosRectasParalelas[2,:],PuntosRectasParalelas[3,:],PuntosRectasParalelas[4,:],PuntosRectasParalelas[5,:],PuntosRectasParalelas[6,:],PuntosRectasParalelas[7,:],PuntosRectasParalelas[8,:],PuntosRectasParalelas[9,:],PuntosRectasParalelas[10,:],PuntosRectasParalelas[11,:],posHome]
                    ArrTiempo=[1,1,1,1,1,1,1,1,1,1,1,1,1]
                    for i in range(len(ArrTiempo)):
                        Rut2p(PuntosRectasParalelas[i,:],ArrTiempo[i],gripClose)
                    Rut2p(posHome,1,gripClose)

                    end_time = time.time()  
                    Tiempo=end_time-start_time
                    print("\ntiempo de ejecucion: %.2f s" % Tiempo)
                else: 
                    print("La herramienta no está cargada")

            elif caso==5:
                if Cargado==1:
                    print("Se seleccionó: Dibujo de puntos")
                    start_time = time.time()
                    
                    #Rut2p(PuntosEQTablero[0,:],1,gripClose)
                    #Rut2p(PuntosEQTablero[1,:],1,gripClose)
                    #Rut2p(PuntosEQTablero[2,:],1,gripClose)
                    #Rut2p(PuntosEQTablero[3,:],1,gripClose)
                    #Rut2p(PuntosEQTablero[4,:],1,gripClose)
                    #Rut2p(PuntosEQTablero[5,:],1,gripClose)
                    #Rut2p(PuntosEQTablero[6,:],1,gripClose)
                    #Rut2p(PuntosEQTablero[7,:],1,gripClose)
                    #Rut2p(PuntosEQTablero[8,:],1,gripClose)
                    #Rut2p(PuntosEQTablero[9,:],1,gripClose)
                    #Rut2p(PuntosEQTablero[10,:],1,gripClose)
                    #Rut2p(PuntosEQTablero[11,:],1,gripClose)
                    #Rut2p(PuntosEQTablero[12,:],1,gripClose)
                    #Rut2p(PuntosEQTablero[13,:],1,gripClose)
                    #Rut2p(PuntosEQTablero[14,:],1,gripClose)
                    #Rut2p(posHome,1,gripClose)

                    #ArrPos=[PuntosEQTablero[0,:],PuntosEQTablero[1,:],PuntosEQTablero[2,:],PuntosEQTablero[3,:],PuntosEQTablero[4,:],PuntosEQTablero[5,:],PuntosEQTablero[6,:],PuntosEQTablero[7,:],PuntosEQTablero[8,:],PuntosEQTablero[9,:],PuntosEQTablero[10,:],PuntosEQTablero[11,:],PuntosEQTablero[12,:],PuntosEQTablero[13,:],PuntosEQTablero[14,:],posHome]
                    ArrTiempo=[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
                    for i in range(len(ArrTiempo)):
                        Rut2p(PuntosEQTablero[i,:],ArrTiempo[i],gripClose)
                    Rut2p(posHome,1,gripClose)


                    end_time = time.time()
                    Tiempo=end_time-start_time
                    print("\ntiempo de ejecucion: %.2f s" % Tiempo)
                else: 
                        print("La herramienta no está cargada")

            elif caso==6:
                if Cargado==1:
                    print("Se seleccionó: Dibujo figura libre")
                    start_time = time.time()

                    N=50
                    theta=np.linspace(0,2*pi,N)
                    R=np.sqrt(abs(np.cos(2*theta)))
                    x=R*np.cos(theta)/18+0.125
                    y=R*np.sin(theta)/18+0.2

                    BetaPuntos=np.ones(N)*betaEx
                    ZPuntos=np.ones(N)*(Zpiso-0.012)
                    t=0.2
                    ArrTiempo=np.ones(N)*t
                    PuntosRosa=np.zeros([N,4])
                    PuntosRosa[:,0]=0.225+0.025*np.cos(Rads)
                    PuntosRosa[:,1]=-0.025+0.025*np.sin(Rads)
                    PuntosRosa[:,2]=ZPuntos
                    PuntosRosa[:,3]=BetaPuntos
                    PosIni=[0.125,0.2,ZseguridadEx,betaEx]

                    Rut2p(PosIni,1.5,gripClose)
                    input()
                    for i in range(N):
                        Rut2p(PuntosRosa[i,:],ArrTiempo[i],gripClose)
                    input()
                    Rut2p(posHome,2,gripClose)


                    end_time = time.time()
                    Tiempo=end_time-start_time
                    print("\ntiempo de ejecucion: %.2f s" % Tiempo)
                else: 
                    print("La herramienta no está cargada")

            elif caso==7:
                if Cargado==1:
                    print("Se seleccionó: Descarga de la herramienta")
                    start_time = time.time()
                    time.sleep(1)
                    Rut2p(coorPortaHerramientasArriba,0.3,gripClose)
                    Rut2p(coorPortaHerramientasAbajo,0.5,gripClose)
                    input()
                    print("Apretar gripper")
                    Rut2p(coorPortaHerramientasAbajo,0.5,gripOpen)
                    print("Volviendo a home")
                    Rut2p(coorPortaHerramientasArriba,1,gripOpen)
                    Rut2p(posHome,1,gripOpen)

                    end_time = time.time()
                    Tiempo=end_time-start_time
                    print("\ntiempo de ejecucion: %.2f s" % Tiempo)
                    Cargado=0
                else: 
                    print("La herramienta ya está descargada")
                
            elif caso==8:
                print("Se seleccionó: Salr")
            
            else:
                print("Opción no válida. Saliendo.")
                break
        
        print("Finalizando rutinas")

    except rospy.ROSInterruptException:
        pass