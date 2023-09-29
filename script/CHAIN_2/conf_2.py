import numpy as np


# Frame index
ROD2_FRAME_INDEX = 8

# max iterations
N = 500



absolute_threshold  = 1e-2
gradient_threshold  = 1e-2
beta                = 0.1           
gamma               = 1e-2          # line search convergence parameter
regu                = 1e-2          # regularized Hessian