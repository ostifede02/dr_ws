from numpy.linalg import norm, inv
import numpy as np

from script import configuration as conf


class InverseGeometry:
    
    def __init__(self, robot, frame_id):
        self.robot = robot
        self.frame_id = frame_id

        # import parameters
        self.max_iterations = conf.configuration["inverse_geometry"]["parameters"]["max_iterations"]
        self.max_back_tracking_iterations = conf.configuration["inverse_geometry"]["parameters"]["max_back_tracking_iterations"]
        self.absolute_pos_threshold = conf.configuration["inverse_geometry"]["parameters"]["absolute_pos_threshold"]
        self.gradient_threshold = conf.configuration["inverse_geometry"]["parameters"]["gradient_threshold"]
        self.beta = conf.configuration["inverse_geometry"]["parameters"]["beta"]
        self.gamma = conf.configuration["inverse_geometry"]["parameters"]["gamma"]
        self.hessian_regu = conf.configuration["inverse_geometry"]["parameters"]["hessian_regu"]
  
    # Gauss-Newton algorithm
    def GN_step(self, q, x, x_des, J, i):
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
            # print("Problem solved after %d iterations with error %f"%(i, norm(e)))
            return None
        
        # if error is null you are done
        if(cost < self.absolute_threshold):
            # print("Problem solved after %d iterations with error %f"%(i, norm(e)))
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
    
    # method computes inverse geomtery
    def compute_inverse_geometry(self, q, x_des):

        for i in range(self.max_iterations):
            H1 = self.robot.framePlacement(q, self.frame_id)
            x = H1.translation

            J6 = self.robot.computeFrameJacobian(q, self.frame_id)
            J = J6[0:3,:]

            q_next = self.GN_step(self, q, x, x_des, J, i)

            if(q_next is None):
                break
            else:
                q[:] = q_next[:]

        return q