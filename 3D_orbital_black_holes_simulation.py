# Importamos las librerías necesarias para la simulación y visualización
import numpy as np  # Librería para operaciones numéricas y manejo de arrays
import vpython as vp  # Librería para visualización en 3D

# Definimos las constantes físicas y parámetros de la simulación
G = 1  # Constante gravitacional (en unidades simplificadas)
masa_agujero_negro_1 = 1.0  # Masa del primer agujero negro
masa_agujero_negro_2 = 0.5  # Masa del segundo agujero negro
num_pasos = 5000  # Número total de pasos para la simulación

# Establecemos las condiciones iniciales de nuestros agujeros negros en 3D
posicion_agujero_negro_1 = np.array([-5.0, 0.0, 0.0])  # Posición inicial del primer agujero negro
posicion_agujero_negro_2 = np.array([5.0, 0.0, 0.0])   # Posición inicial del segundo agujero negro
velocidad_agujero_negro_1 = np.array([0.0, -0.1, 0.0])  # Velocidad inicial del primer agujero negro
velocidad_agujero_negro_2 = np.array([0.0, 0.1, 0.0])   # Velocidad inicial del segundo agujero negro

radio_colision = 1.0  # Distancia a la que consideramos que ocurre la colisión
dt = 1.6  # Intervalo de tiempo entre cada paso de la simulación
fusionado = False  # Bandera para indicar si los agujeros negros se han fusionado

# Configuramos la visualización en 3D
escena = vp.canvas()  # Crea una nueva escena para la visualización
agujero_negro_1 = vp.sphere(pos=vp.vector(*posicion_agujero_negro_1), radius=0.5, color=vp.color.red)  # Primer agujero negro
agujero_negro_2 = vp.sphere(pos=vp.vector(*posicion_agujero_negro_2), radius=0.5, color=vp.color.blue)  # Segundo agujero negro

# Listas para almacenar las trayectorias
posiciones_agujero_negro_1 = []  # Lista para guardar las posiciones del primer agujero negro
posiciones_agujero_negro_2 = []  # Lista para guardar las posiciones del segundo agujero negro

def calcular_aceleracion(pos1, pos2, masa1, masa2):
    """
    Calcula la aceleración gravitacional que un objeto (pos1, masa1) ejerce sobre otro objeto (pos2, masa2).
    
    :param pos1: Posición del primer objeto (array de coordenadas x, y, z)
    :param pos2: Posición del segundo objeto (array de coordenadas x, y, z)
    :param masa1: Masa del primer objeto
    :param masa2: Masa del segundo objeto
    :return: Aceleración en pos1 y en pos2 debida a la fuerza gravitacional
    """
    distancia = np.linalg.norm(pos2 - pos1)  # Calcula la distancia entre los dos objetos
    if distancia == 0:  # Si la distancia es cero, retornamos aceleraciones nulas para evitar división por cero
        return np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])
    
    # Calcula la fuerza gravitacional usando la ley de gravitación de Newton
    fuerza = G * masa1 * masa2 / distancia**2
    # Calcula la aceleración de cada objeto debido a la fuerza gravitacional
    aceleracion1 = fuerza * (pos2 - pos1) / (masa1 * distancia)
    aceleracion2 = -fuerza * (pos2 - pos1) / (masa2 * distancia)
    
    return aceleracion1, aceleracion2  # Retorna las aceleraciones en los dos objetos

def paso_rk4(pos1, pos2, vel1, vel2, masa1, masa2, dt, func_aceleracion):
    """
    Realiza un paso de integración numérica usando el método de Runge-Kutta 4 para actualizar posiciones y velocidades.
    
    :param pos1: Posición del primer objeto
    :param pos2: Posición del segundo objeto
    :param vel1: Velocidad del primer objeto
    :param vel2: Velocidad del segundo objeto
    :param masa1: Masa del primer objeto
    :param masa2: Masa del segundo objeto
    :param dt: Intervalo de tiempo para cada paso
    :param func_aceleracion: Función que calcula la aceleración entre los objetos
    :return: Nuevas posiciones y velocidades de los objetos
    """
    def f(pos1, pos2, vel1, vel2):
        """
        Calcula el cambio en posiciones y velocidades.
        """
        aceleracion1, aceleracion2 = func_aceleracion(pos1, pos2, masa1, masa2)
        return np.array([vel1, vel2, aceleracion1, aceleracion2])
    
    # Calcula las pendientes (k1, k2, k3, k4) para el método de Runge-Kutta 4
    k1 = f(pos1, pos2, vel1, vel2)
    k2 = f(pos1 + 0.5*dt*k1[0], pos2 + 0.5*dt*k1[1], vel1 + 0.5*dt*k1[2], vel2 + 0.5*dt*k1[3])
    k3 = f(pos1 + 0.5*dt*k2[0], pos2 + 0.5*dt*k2[1], vel1 + 0.5*dt*k2[2], vel2 + 0.5*dt*k2[3])
    k4 = f(pos1 + dt*k3[0], pos2 + dt*k3[1], vel1 + dt*k3[2], vel2 + dt*k3[3])
    
    # Actualiza las posiciones y velocidades usando una combinación ponderada de las pendientes
    pos1_nueva = pos1 + (dt/6) * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0])
    pos2_nueva = pos2 + (dt/6) * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1])
    vel1_nueva = vel1 + (dt/6) * (k1[2] + 2*k2[2] + 2*k3[2] + k4[2])
    vel2_nueva = vel2 + (dt/6) * (k1[3] + 2*k2[3] + 2*k3[3] + k4[3])
    
    return pos1_nueva, pos2_nueva, vel1_nueva, vel2_nueva  # Retorna las nuevas posiciones y velocidades

def energia_total(pos1, pos2, vel1, vel2, masa1, masa2):
    """
    Calcula la energía total del sistema, que es la suma de la energía cinética y potencial.
    
    :param pos1: Posición del primer objeto
    :param pos2: Posición del segundo objeto
    :param vel1: Velocidad del primer objeto
    :param vel2: Velocidad del segundo objeto
    :param masa1: Masa del primer objeto
    :param masa2: Masa del segundo objeto
    :return: Energía total del sistema
    """
    distancia = np.linalg.norm(pos2 - pos1)  # Calcula la distancia entre los dos objetos
    energia_cinetica1 = 0.5 * masa1 * np.sum(vel1**2)  # Energía cinética del primer objeto
    energia_cinetica2 = 0.5 * masa2 * np.sum(vel2**2)  # Energía cinética del segundo objeto
    energia_potencial = -G * masa1 * masa2 / distancia  # Energía potencial gravitacional
    return energia_cinetica1 + energia_cinetica2 + energia_potencial  # Retorna la energía total

def actualizar():
    """
    Actualiza la posición y velocidad de los agujeros negros para cada fotograma de la animación.
    """
    global posicion_agujero_negro_1, posicion_agujero_negro_2, velocidad_agujero_negro_1, velocidad_agujero_negro_2, fusionado

    if not fusionado:  # Si los agujeros negros no se han fusionado
        # Actualiza posiciones y velocidades usando el método Runge-Kutta 4
        posicion_agujero_negro_1, posicion_agujero_negro_2, velocidad_agujero_negro_1, velocidad_agujero_negro_2 = paso_rk4(
            posicion_agujero_negro_1, posicion_agujero_negro_2, velocidad_agujero_negro_1, velocidad_agujero_negro_2,
            masa_agujero_negro_1, masa_agujero_negro_2, dt, calcular_aceleracion
        )
        # Actualiza la visualización
        agujero_negro_1.pos = vp.vector(*posicion_agujero_negro_1)
        agujero_negro_2.pos = vp.vector(*posicion_agujero_negro_2)
        
        # Añade las posiciones a las listas para trazar las trayectorias
        posiciones_agujero_negro_1.append(posicion_agujero_negro_1.copy())
        posiciones_agujero_negro_2.append(posicion_agujero_negro_2.copy())
        
        # Verifica si los agujeros negros se han fusionado
        distancia = np.linalg.norm(posicion_agujero_negro_2 - posicion_agujero_negro_1)
        if distancia < radio_colision:
            fusionado = True
            print("¡Los agujeros negros se han fusionado!")
            agujero_negro_1.color = vp.color.orange
            agujero_negro_2.color = vp.color.orange
    else:  # Si los agujeros negros ya se han fusionado
        # Actualiza la visualización de las trayectorias de los agujeros negros fusionados
        for pos1, pos2 in zip(posiciones_agujero_negro_1, posiciones_agujero_negro_2):
            vp.curve(pos=[vp.vector(*pos1), vp.vector(*pos2)], color=vp.color.green, radius=0.02)

    # Calcula y muestra la energía total del sistema
    energia = energia_total(posicion_agujero_negro_1, posicion_agujero_negro_2, velocidad_agujero_negro_1, velocidad_agujero_negro_2, masa_agujero_negro_1, masa_agujero_negro_2)
    print(f"Energía Total: {energia}")

# Animación y actualización del sistema
while True:
    vp.rate(50)  # Controla la velocidad de la animación (fotogramas por segundo)
    actualizar()