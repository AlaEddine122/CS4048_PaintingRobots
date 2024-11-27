import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)

# Initial lines with the tuple we want to move
lines = [(1, 2), (1, 12), (1, 10), (9, 9), (11, 10), (15, 15)]


current_tuple_index = 5
movement_direction = 'left'  # Initial direction


def animate(i):
    global lines, movement_direction

    # Get the tuple to move
    x, y = lines[current_tuple_index]

    #
    if movement_direction == 'right':
        x += 1
        if x >= 20:
            x = 20
            movement_direction = 'down'
    elif movement_direction == 'down':
        y -= 1
        if y <= 0:  
            y = 0
            movement_direction = 'left'
    elif movement_direction == 'left':
        x -= 1
        if x <= 0:
            x = 0
            movement_direction = 'up'
    elif movement_direction == 'up':
        y += 1
        if y >= 20:
            y = 20
            movement_direction = 'right'

    
    lines[current_tuple_index] = (x, y)

    
    xs = [line[0] for line in lines]
    ys = [line[1] for line in lines]

    
    ax1.clear()
    ax1.scatter(xs, ys)
    plt.xlim(0, 23)
    plt.ylim(0, 23)


ani = animation.FuncAnimation(fig, animate, interval=500)
plt.show()
