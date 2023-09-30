import numpy as np


# Frame index
ROD1_FRAME_INDEX = 8
ROD2_FRAME_INDEX = 14

# max iterations
N = 100



absolute_threshold  = 1e-3
gradient_threshold  = 1e-3
beta                = 0.1           
gamma               = 1e-2          # line search convergence parameter
regu                = 1e-2          # regularized Hessian