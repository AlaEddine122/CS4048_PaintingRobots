import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

style.use('fivethirtyeight')

fig = plt.figure()

ax1 = fig.add_subplot(1,1,1)

def animate(i):
    #graph_data = sheep_positions
    graph_data = open('coords.txt','r').read()
    lines = graph_data.split('\n')
    xs = []
    ys = []
    '''for position in graph_data:
            x,y = sheep_positions[(x,y)]
            xs.append(float(x))
            ys.append(float(y))
        ax1.clear()
        ax1.scatter(xs, ys)
        plt.xlim(0,20)
        plt.ylim(0,20)
    '''
    for line in lines:
        if len(line) > 1:
            x, y = line.split(',')
            xs.append(float(x))
            ys.append(float(y))
    ax1.clear()
    ax1.scatter(xs, ys)
    plt.xlim(0,20)
    plt.ylim(0,20)

ani = animation.FuncAnimation(fig, animate, interval=1000)
plt.show()