import numpy as np

def EE_position_geometrically(q1, q2):
    l = 450

    P1 = np.array([-q1*np.cos(np.pi/4), 0, q1*np.sin(np.pi/4)])
    P2 = np.array([q2*np.cos(np.pi/4), 0, q2*np.sin(np.pi/4)])

    d = np.linalg.norm(P2-P1)

    a = d/2
    h = np.sqrt(pow(l,2) - pow(a, 2))

    Pm = P1 + a*(P2 - P1)/d
    x3 = Pm[0] - h*(P2[2] - P1[2])/d
    z3 = Pm[2] + h*(P2[0] - P1[0])/d
    
    if(z3 < 0):
        x = np.array([x3, 0, z3])
        return x
    else:
        x3 = Pm[0] + h*(P2[2] - P1[2])/d
        z3 = Pm[2] - h*(P2[0] - P1[0])/d
        x = np.array([x3, 0, z3])
        return x
