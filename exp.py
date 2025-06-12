import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Параметры
steps = 120  # Количество шагов
radius = 1   # Радиус окружности
theta = np.linspace(0, 2 * np.pi, steps)  # Углы от 0 до 2π

# Координаты вектора
x = radius * np.cos(theta)
y = radius * np.sin(theta)

# Настройка графика
fig, ax = plt.subplots()
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Движение вектора по окружности')
ax.grid(True)
ax.set_aspect('equal')

# Отрисовка окружности
circle = plt.Circle((0, 0), radius, color='blue', fill=False)
ax.add_patch(circle)

# Отрисовка вектора
vector = ax.quiver([0], [0], [x[0]], [y[0]], angles='xy', scale_units='xy', scale=1, color='red')

# Функция обновления для анимации
def update(frame):
    vector.set_UVC(x[frame], y[frame])
    return vector,

# Создание анимации
ani = FuncAnimation(fig, update, frames=steps, interval=50, blit=True)

# Отображение анимации
plt.show()