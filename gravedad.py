# ----------------- importar librerias --------------------- 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import random
import colorsys
import decimal
import os
import ctypes
import json
import numpy as np
import time
import string

# habilitar el soporte de colores ANSI en la consola de Windows
kernel32 = ctypes.windll.kernel32
kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 7)

# precisión de los datos
decimal.getcontext().prec = 250

# Constante de gravitación universal
G = decimal.Decimal('0.01')

# Modo de tiempo
TIME_MODE = True  # si es True se usa delta_time si es False es constante 
# valor de delta_time si el time_mode es false
UNIFORM_TIME = 0.1

# escala de los puntos
DOT_SCALE = 1

# escala del campo de vision
VIEW_SCALE = decimal.Decimal(150), decimal.Decimal(150)

# guardado de la simulacion
ACTIVE_SAVE = False
SAVE_IN = ""
CYCLES = 1

# ------------ crear cuerpos -------------------------
class Body:
    def __init__(self, name, position, velocity, mass, diameter=None, exact=True):
        self.name = name
        self.position = position
        self.velocity = velocity
        self.mass = mass
        self.tag = f"{random.randint(0, 99999):05}"
        self.color = colorsys.hsv_to_rgb(random.uniform(0, 1), 1, 1)
        self.diameter = diameter
        self.exact = exact
        self.positions = []

        if diameter is not None:
            self.diameter = diameter
        else:
            self.diameter = float(mass) * DOT_SCALE

    def save_pos(self):
        self.positions.append(self.position)

    def apply_force(self, force, time):
        acceleration = force / self.mass
        self.velocity += acceleration * time

    def apply_gravity(self, other, delta_time):
        force = universo.physics.gravity(self, other)
        self.apply_force(Vector2(force.x, force.y) * -1, delta_time)
        other.apply_force(Vector2(force.x, force.y), delta_time)

    def move(self, delta_time):
        self.position += self.velocity * delta_time

class Vector2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def magnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def __str__(self):
        return f"({self.x}, {self.y})"

def gravity(body_a, body_b):
    '''devuelve el vector de la fuerza que recibe los cuerpos'''
    # calcular la fuerza usando la formula de newton
    distance = decimal_distance(body_a.position, body_b.position)
    force_total = G * ((body_a.mass * body_b.mass) / (distance ** 2))

    # calcular el vector de la direccion usando trigonometria
    angle = math.atan2(body_a.position.y - body_b.position.y, body_a.position.x - body_b.position.x)
    force_x = force_total * math.cos(angle)
    force_y = force_total * math.sin(angle)

    return Vector2(force_x, force_y)

def decimal_distance(vector_a, vector_b):
    return decimal.Decimal(math.sqrt((vector_a.x - vector_b.x) ** 2 + (vector_a.y - vector_b.y) ** 2))

def create_random_string(length):
    letters = string.ascii_lowercase
    return ''.join(random.choice(letters) for i in range(length))

def create_color_rgb():
    return colorsys.hsv_to_rgb(random.uniform(0, 1), 1, 1)

def create_asteroid_belt(count, min_distance, max_distance):
    asteroids = []
    for i in range(count):
        distance = random.uniform(min_distance, max_distance)
        angle = random.uniform(0, 2 * math.pi)
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        velocity = random.uniform(10000, 30000)

        asteroid = {
            "name": f"Asteroid_{i+1}",
            "position": {"x": x, "y": y},
            "velocity": {"x": -velocity * math.sin(angle), "y": velocity * math.cos(angle)},
            "mass": "1e+15",
            "diameter": "5"
        }
        asteroids.append(asteroid)
    return asteroids

class Universe:
    def __init__(self, physics, grid):
        self.physics = physics
        self.grid = grid

class Physics:
    def __init__(self, g):
        self.g = g

    def gravity(self, body_a, body_b):
        '''devuelve el vector de la fuerza que recibe los cuerpos '''
        # calcular la fuerza usando la formula de newton
        distance = decimal_distance(body_a.position, body_b.position)
        force_total = self.g * ((body_a.mass * body_b.mass) / (distance ** 2))

        # calcular el vector de la direccion usando trigonometria
        angle = math.atan2(body_a.position.y - body_b.position.y, body_a.position.x - body_b.position.x)
        force_x = force_total * math.cos(angle)
        force_y = force_total * math.sin(angle)

        return Vector2(force_x, force_y)

    def calculate_orbital_velocity(self, central_body, orbiting_body):
        v = math.sqrt((self.g * central_body.mass) / decimal_distance(central_body.position, orbiting_body.position))
        return v

    def calculate_acceleration(self, force_applied, mass, time):
        '''aplicar la formula F = ma'''
        acceleration = force_applied / mass
        return acceleration * time

class Grid:
    def __init__(self, width, height, scale):
        self.width = width
        self.height = height
        self.scale = scale

    def update(self, bodies):
        # actualizar la grilla con los cuerpos
        pass

class Points:
    def __init__(self):
        # lista con los cuerpos que tiene que visualizar
        self.bodies = []
        # crear las variables para la ventana y los ejes
        self.fig, self.ax = plt.subplots()

        # establecer la variable para el grafico de dispersión
        self.scatter = self.ax.scatter([], [])

        # establecer el tamaño que muestran los ejes
        self.ax.set_xlim(-VIEW_SCALE[0], VIEW_SCALE[0])
        self.ax.set_ylim(-VIEW_SCALE[1], VIEW_SCALE[1])

        # establecer el color de fondo del grafico a negro
        self.ax.set_facecolor('black')
        # establecer el color de fondo de la ventana a negro
        self.fig.patch.set_facecolor('black')

        # ajustar la visualizacion para que 1 unidad en X mida lo mismo en la pantalla que 1 unidad en Y
        self.ax.set_aspect('equal', adjustable='datalim')

        # cambiar el color de los ejes a gris
        self.ax.tick_params(axis='x', colors='gray')
        self.ax.tick_params(axis='y', colors='gray')

        # cambiar el color del recuadro a gris
        for spine in self.ax.spines.values():
            spine.set_color('gray')

        # activar el modo interactivo en el grafico
        plt.ion()

        # crear una animacion
        self.ani = animation.FuncAnimation(self.fig, self.update_graph, interval=16.7 / 2, frames=60)

    def add_bodies(self, bodies):
        for body in bodies:
            self.bodies.append(body)

    def update_graph(self):
        # establecer las variables que se van a usar en el grafico
        xs, ys, names, exacts = zip(*[(body.position.x, body.position.y, body.name, body.exact) for body in self.bodies])
        # actualizar las posiciones
        self.scatter.set_offsets(list(zip(xs, ys)))

        # Eliminar etiquetas anteriores
        for annotation in self.ax.texts:
            annotation.remove()

        # Mostrar etiquetas en blanco en cada punto
        for x, y, name, exact in zip(xs, ys, names, exacts):
            if exact:
                self.ax.annotate(name, (x, y), ha='center', va='top', color='white', fontsize=8, xytext=(0, -10), textcoords='offset points')

        # mostrar cambios
        self.fig.canvas.get_tk_widget().update()

    def start_animation(self):
        plt.show()

        diameters, colors = zip(*[(body.diameter, body.color) for body in self.bodies])

        # actualizar las diametros
        self.scatter.set_sizes(list(diameters))
        # actualizar el color
        self.scatter.set_color(list(colors))

# crear la grilla
grid = Grid(300, 300, VIEW_SCALE[0])

# crear la física
physics = Physics(G)

# crear el universo
universo = Universe(physics, grid)

# crear los cuerpos
bodies = []

# crear asteroides aleatorios en todo el sistema solar
for i in range(100):
    position = Vector2(decimal.Decimal(random.uniform(-4558857000000, 4558857000000)), decimal.Decimal(random.uniform(-4558857000000, 4558857000000)))
    velocity = Vector2(0, 0)
    body = Body(str(i), position, velocity, decimal.Decimal(2.8e21))
    bodies.append(body)

# crear asteroides en el cinturón de asteroides
for i in range(100):
    position = Vector2(decimal.Decimal(random.uniform(-508632758000, 508632758000)), decimal.Decimal(random.uniform(-508632758000, 508632758000)))
    while decimal_distance(Vector2(0, 0), position) < decimal.Decimal(3.291e+11) or decimal_distance(Vector2(0, 0), position) > decimal.Decimal(4.787e+11):
        position = Vector2(decimal.Decimal(random.uniform(-508632758000, 508632758000)), decimal.Decimal(random.uniform(-508632758000, 508632758000)))

    velocity = Vector2(0, 0)
    body = Body(str(i) + " Cinturón", position, velocity, decimal.Decimal(2.8e21), diameter=3, exact=False)
    bodies.append(body)

# crear asteroides en el cinturón de Kuiper
for i in range(100):
    position = Vector2(decimal.Decimal(random.uniform(-7.48e+12, 7.48e+12)), decimal.Decimal(random.uniform(-7.48e+12, 7.48e+12)))
    while decimal_distance(Vector2(0, 0), position) < decimal.Decimal(4488000000000) or decimal_distance(Vector2(0, 0), position) > decimal.Decimal(7480000000000):
        position = Vector2(decimal.Decimal(random.uniform(-7.48e+12, 7.48e+12)), decimal.Decimal(random.uniform(-7.48e+12, 7.48e+12)))

    velocity = Vector2(0, 0)
    body = Body(str(i) + " Cinturón", position, velocity, decimal.Decimal(2.8e21), diameter=3, exact=False)
    bodies.append(body)

# agregar los cuerpos a la grilla
grid.update(bodies)

# crear la ventana
points = Points()

# agregar los cuerpos a la ventana
points.add_bodies(bodies)

# iniciar la animación
points.start_animation()

# variable para el delta time
last_time = time.perf_counter()

# bucle infinito
while True:
    # verificar el modo de tiempo que se está usando
    if TIME_MODE:
        # calcular el delta time
        current_time = time.perf_counter()
        delta_time = current_time - last_time
    else:
        # usar un valor fijo
        delta_time = UNIFORM_TIME

    # aplicar la fuerza de gravedad entre cada objeto con todos los demás
    for i, body in enumerate(bodies):
        for other_body in bodies[i+1:]:
            body.apply_gravity(other_body, delta_time)

    # calcular la inercia de cada objeto
    for body in bodies:
        body.move(delta_time)

    # actualizar la grilla
    grid.update(bodies)

    # actualizar la ventana
    points.update_graph()

    # imprimir el delta time
    print(delta_time)

    # guardar el tiempo actual
    last_time = current_time