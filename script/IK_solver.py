from numpy.linalg import norm, inv
import numpy as np



def GN_ik_step_(self, q, x, x_des, J, i):
    e = x_des - x
    cost = norm(e)
    
    # Newton method
    nv = J.shape[1]
    B = J.T.dot(J) + self.hessian_regu*np.eye(nv) # approximate regularized Hessian
    gradient = J.T.dot(e)   # gradient
    delta_q = inv(B).dot(gradient)
    q_next = q + delta_q
    
    # if gradient is null you are done
    grad_norm = norm(gradient)
    if(grad_norm < self.gradient_threshold):
        # print("Terminate because gradient is (almost) zero:", grad_norm)
        print("Problem solved after %d iterations with error %f"%(i, norm(e)))
        return None
    
    # if error is null you are done
    if(cost < self.absolute_threshold):
        print("Problem solved after %d iterations with error %f"%(i, norm(e)))
        return None
    

    # back-tracking line search
    alpha = 1.0
    iter_line_search = 0
    
    while True:
        q_next = q + alpha*delta_q
        # robot.computeJointJacobians(q_next)
        # robot.framesForwardKinematics(q_next)
        x_new = self.robot.framePlacement(q_next, self.frame_id).translation
        cost_new = norm(x_des - x_new)

        if cost_new < (1.0-alpha*self.gamma)*cost:
            # print("Backtracking line search converged with log(alpha)=%.1f"%np.log10(alpha))
            break
        else:
            alpha *= self.beta
            iter_line_search += 1
            if(iter_line_search==self.max_back_tracking_iterations):
                # print("Backtracking line search could not converge. log(alpha)=%.1f"%np.log10(alpha))
                break
    
    # print("Iteration %d, ||x_des-x||=%f, norm(gradient)=%f"%(i, norm(e), grad_norm))

    return q_next


def solve_GN_(self, q, x_des):
    
    for i in range(self.max_iterations):
        H1 = self.robot.framePlacement(q, self.frame_id)
        x = H1.translation

        J6 = self.robot.computeFrameJacobian(q, self.frame_id)
        J = J6[0:3,:]

        q_next = GN_ik_step_(self, q, x, x_des, J, i)

        if(q_next is None):
            break
        else:
            q[:] = q_next[:]

    return q



class IK_solver:
    
    # configuration parameters
    max_iterations                  = 300           # max iteration
    max_back_tracking_iterations    = 30            # max back tracking iterations

    absolute_threshold              = 1e-3          # absolute tolerance on position error
    gradient_threshold              = 1e-3          # absolute tolerance on gradient's norm

    beta                            = 0.1           # backtracking line search parameter
    gamma                           = 1e-2          # line search convergence parameter
    hessian_regu                    = 1e-2          # Hessian regularization

    def __init__(self, robot, frame_id):
        self.robot = robot
        self.frame_id = frame_id

    solve_GN = solve_GN_

    