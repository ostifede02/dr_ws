import numpy as  np

from plot import PlotHandler

N = 200

plot1 = PlotHandler("test1", "x1", "y1", N)
data1 = np.empty((2, N))

for value in range(N):
    data1[:, value] = np.array([value, value**0.5])
    plot1.store_data(data1[:, value])

plot1.plot_data()