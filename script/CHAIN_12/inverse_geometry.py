import numpy as np
from numpy.linalg import norm, inv
from conf import *


def inverse_geometry_step(q, x, x_des, J, i, N, robot, frame_id):
    e = x_des - x
    cost = norm(e)
    
    # Newton method
    nv = J.shape[1]
    B = J.T.dot(J) + regu*np.eye(nv) # approximate regularized Hessian
    gradient = J.T.dot(e)   # gradient
    delta_q = inv(B).dot(gradient)
    q_next = q + delta_q
    
    # if gradient is null you are done
    grad_norm = norm(gradient)
    if(grad_norm<gradient_threshold):
        print("Terminate because gradient is (almost) zero:", grad_norm)
        print("Problem solved after %d iterations with error %f"%(i, norm(e)))
        return None
    
    # if error is null you are done
    if(norm(e)<absolute_threshold):
        print("Problem solved after %d iterations with error %f"%(i, norm(e)))
        return q_next
    

    # back-tracking line search
    alpha = 1.0
    iter_line_search = 0
    
    while True:
        q_next = q + alpha*delta_q
        # robot.computeJointJacobians(q_next)
        # robot.framesForwardKinematics(q_next)
        x_new = robot.framePlacement(q_next, frame_id).translation
        x_new = np.array([x_new[0], x_new[2]])
        cost_new = norm(x_des - x_new)

        if cost_new < (1.0-alpha*gamma)*cost:
            print("Backtracking line search converged with log(alpha)=%.1f"%np.log10(alpha))
            break
        else:
            alpha *= beta
            iter_line_search += 1
            if(iter_line_search==N):
                print("Backtracking line search could not converge. log(alpha)=%.1f"%np.log10(alpha))
                break
            
    
    
    print("Iteration %d, ||x_des-x||=%f, norm(gradient)=%f"%(i, norm(e), grad_norm))
                
    return q_next
































































# def inverse_geometry_step(q, x, x_des, J, i, N, robot, frame_id):
    
#     # calculating the error
#     e = x - x_des
#     cost = norm(e)

#     # Newton method
#     nv = J.shape[1]
#     B = J.T.dot(J) + regu*np.eye(nv)    # approximate regularized Hessian
#     gradient = J.T.dot(e)               # gradient
#     delta_q = -inv(B).dot(gradient)

#     # if gradient is null you are done
#     grad_norm = norm(gradient)
#     if(grad_norm<gradient_threshold):
#         print("Terminate because gradient is (almost) zero:", grad_norm)
#         print("Problem solved after %d iterations with error %f"%(i, norm(e)))
#         return None

#     alpha = 1
#     iter_line_search = 0

#     q_next = q + alpha*delta_q
#     x_new = robot.framePlacement(q_next, frame_id).translation
#     cost_new = norm(x_new - x_des)

#     reduction = cost - cost_new

#     while(reduction < gamma*alpha*cost):
#         q_next = q + alpha*delta_q

#         x_new = robot.framePlacement(q_next, frame_id).translation
#         reduction = cost - cost_new

#         if(cost_new < (1.0-alpha*gamma)*cost):
#             print("exit, beacuse cost new")
#             break
#         else:
#             alpha = beta*alpha
#             iter_line_search += 1
#             if(iter_line_search==N):
#                 print("Backtracking line search could not converge. log(alpha)=%.1f"%np.log10(alpha))
#                 break

#     return q_next