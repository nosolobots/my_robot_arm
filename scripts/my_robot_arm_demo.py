#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Módulo controlador del brazo articulado.
# --------------------------------------------------------
# autor: Gabriel Montoiro Varela
# ver:   0.1

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import my_robot_arm_controller as rob

def clear_screen():
    # códigos de escape ANSI para borrar el terminal (ESC [ 2J) y 
    # posicionarse en la primera línea (ESC [ H)
    print "\033[H\033[2J"

def show_header():
    clear_screen()
    print "\t##########################################"
    print "\t#            F I S I C A   II            #"
    print "\t#                                        #"
    print "\t# - Proyecto: Brazo Articulado 6DOF      #"
    print "\t# - Autor: Gabriel Montoiro Varela       #"
    print "\t# - Curso: 2020/21                       #"
    print "\t##########################################"
    print

def show_menu():
    show_header()
    print "\t\tE - Mostrar estado"
    print "\t\tP - Posicionar brazo (CD)"
    print "\t\tG - Generar gráfica (CD)"
    print "\t\tS - Salir"
    print
    
def get_menu_option():
    opc = None
    while opc not in ('E', 'P', 'G', 'S'):
        show_menu()
        opc = raw_input("\tOpción > ").upper()
    return opc

def end_option():
    raw_input("\n\tPulsa [Ret] para salir")

def opc_mostrar_estado(arm):
    """
    Parámetros
    ----------
    arm : MyArmController
        instancia del brazo articulado
    """
    show_header()
    print "\tMostrando estado del robot:\n"
    print arm # imprime el estado completo del robot
    end_option()

def opc_genera_grafica(arm):
    """
    Genera gráficas 3D por cinemética directa de las posiciones del elemento terminal por
    rotación de las articulaciones 1 y 2 y traslación de la articulación 3
    Las rotaciones de la articulación esférica (4, 5, 6) no se consideran

    - Para las rotaciones el rango máximo debe ser [0, 2PI]
    - Para la traslación el rango máximo debe ser [-L3/2, L3/2]

    Parámetros
    ----------
    arm : MyArmController
        instancia del brazo articulado
    """
    show_header()
    print "\tGenerador de gráficas:\n"
    print "\tIndica las articulaciones a mover, sus valores inicial y final, y el paso"
    print "\t - Para las rotaciones el rango máximo debe ser [0, 2*PI]"
    print "\t - Para la traslación el rango máximo debe ser [-L3/2, L3/2]\n"

    # estructura para los rangos de valores [ini, fin, paso] de las 3 articulaciones
    th_range = np.zeros((3,3))

    # obtenemos los rangos y paso de movimiento
    for i in range(3):
        if raw_input("\tQuieres variar theta" + str(i) + " [S/N]? ").upper() == 'S':
            th_range[i,0] = float(raw_input("\tValor inicial: ")) 
            th_range[i,1] = float(raw_input("\tValor final: ")) 
            th_range[i,2] = float(raw_input("\tPaso: ")) 

    # valores para las articulaciones
    if th_range[0,0] != th_range[0,1]:
        th1_values = np.arange(th_range[0,0], th_range[0,1], th_range[0,2])
    else:
        th1_values = [0.0]
    
    if th_range[1,0] != th_range[1,1]:
        th2_values = np.arange(th_range[1,0], th_range[1,1], th_range[1,2])
    else:
        th2_values = [0.0]
    
    if th_range[2,0] != th_range[2,1]:
        th3_values = np.arange(th_range[2,0], th_range[2,1], th_range[2,2])
    else:
        th3_values = [0.0]

    # obtenemos las coordenadas del elemento terminal para cada combinación de las articulaciones
    pos = []
    old_theta = arm.get_theta()
    arm.reset()
    for t1 in th1_values:
        for t2 in th2_values:
            for t3 in th3_values:
                arm.set_theta([t1, t2, t3, 0., 0., 0.]) # establece valores articulaciones
                pos.append(arm.get_terminal_pos())  # obtiene posición del elemento terminal
    arm.set_theta(old_theta)   
 
    # generamos la gráfica
    arr_coord = np.array(pos) # array de coordenadas generadas
    x = arr_coord[:,0] # lista de coordenada x 
    y = arr_coord[:,1] # lista de coordenada y 
    z = arr_coord[:,2] # lista de coordenada z 

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlabel('Eje X')
    ax.set_ylabel('Eje Y')
    ax.set_zlabel('Eje Z')
    ax.scatter(x, y, z)
    plt.show()

    end_option()

if __name__ == '__main__':
    # creamos una instancia del brazo articulado a partir de las dimensiones de los eslabones
    arm = rob.MyArmController(5, 5, 5)
    
    # bucle principal
    opc = None
    while opc != 'S':
        opc = get_menu_option()

        if opc == 'E':
            opc_mostrar_estado(arm)

        elif opc == 'G':
            opc_genera_grafica(arm)     



