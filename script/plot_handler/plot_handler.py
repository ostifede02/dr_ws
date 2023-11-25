'''
This class  is intended to handle multiple data acquisition for plots more 
efficently. 
The main characteristics are:
    + __init__
        -> title
        -> labels
        -> set max acquisition samples
        -> set sample size
    + store data
        -> get data
        -> update cycle counter
    + plot data
        -> plot data
'''


import numpy as np
import matplotlib.pyplot as plt


class PlotHandler():
   
    def __init__(self, title, x_label, y_label, max_samples, size_sample=2):
        self.title = title
        self.x_label = x_label
        self.y_label = y_label
        
        self. max_sample = max_samples
        self.size_sample = size_sample
        self.data = np.empty((size_sample, max_samples))

        self.acquisition_counter = 0
        return
    
    
    def store_data(self, data_input):
        self.data[:, self.acquisition_counter] = data_input[:]
        self.acquisition_counter += 1
        return
    

    def plot_data(self):
        plot = plt.figure()
        plot = plt.title(f"{self.title}: {self.y_label} over {self.x_label}")
        plot = plt.plot(self.data[0, :], self.data[1, :])
        plt.show()
        return
        
