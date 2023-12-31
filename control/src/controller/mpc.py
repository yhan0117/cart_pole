import numpy as np
from scipy.optimize import minimize
from collections import namedtuple

# scalar trajectory cost function for OCP
def trajCost(dv,p):     
    # unpack parameters 
    zd = p.zd   # desired state 
    T = p.T     # terminal cost
    Q = p.Q     # error cost matrix
    R = p.R     # actuation effort cost matrix
    N = p.N     # control horizon 
    # extract from decision variables
    z = np.array([dv[4*i:4*(i+1)] for i in range(N)])
    u = dv[4*N:]
    # error 
    e = z - zd
    # cost in QR form
    JQ = sum([Q@(e[i,:]**2)*T[i] for i in range(N)])
    JR = R*(u@u)
    # total cost
    J = JQ+JR

    return J

# modeled equations of motion
def eom(z,u,p):
    # unpack parameters 
    l = p.l;     # pendulum length
    mc = p.mc;    # cart mass
    mp = p.mp;    # pendulum mass
    g = p.g;     # gravity 

    # z' = f(z,u)
    dz = np.zeros(4)
    dz[0] = z[2]
    dz[1] = z[3]
    dz[2] = (mp*np.sin(z[1])*(l*(z[3]**2) + g*np.cos(z[1])) + u) / (mp*np.sin(z[1])**2 + mc)
    dz[3] = -(mp*l*(z[3]**2)*np.sin(z[1])*np.cos(z[1]) + (mp+mc)*g*np.sin(z[1]) + u*np.cos(z[1])) / (l*(mp*np.sin(z[1])**2 + mc))
    return dz

# formulate dynamic constraint through direct collocation
def collcationConstraints(dv,p):
    # unpack parameters
    dt = p.dt    # prediction time interval
    N = p.N        # prediction horizon

    ceq = np.zeros((4*(N-1),1))    # list of size 4(N-1) (number of intervals)

    # iterate through each interval
    for i in range(N-1):
        # extract from dv states and control at each interval
        z1 = dv[4*i:4*(i+1)]
        z2 = dv[4*(i+1):4*(i+2)]
        u1 = dv[4*N+i]
        u2 = dv[4*N+i+1]
        
        # calculate z' = f(z,u) for two successive break points
        dz1 = eom(z1,u1,p)
        dz2 = eom(z2,u2,p)

        # midpoint of the two break points using cubic spline formula
        zc = 0.5*(z1+z2) - (dt/8)*(dz2-dz1)
        dzc = (3/(2*dt))*(z2-z1) - (1/4)*(dz1+dz2)
        
        # collocation constraint at midpoint
        ceq[4*i:4*(i+1),0] = eom(zc,(u1+u2)/2,p) - dzc

    return ceq

def mpc(z,p,dv0): 
    """ Implicit control law 
    Two options:
    1. scipy.optimize => slow and intepreted
    2. casadi => can be compiled into c code, faster
    z = [x;q;dx;dq] = state of the system, 4 x 1
    dv0 = initial guess of (d)ecision (v)ariables (result from last control loop), 5N x 1 
    """ 
    from functools import partial

    # unpack parameters
    N = p.N
    Q = p.Q
    R = p.R
    T = p.T
    
    # constraints
    eq_cons = { 'type': 'eq',   # initial state + collocation constraints
                'fun' : lambda dv: np.append([dv[0]-z[0],dv[1]-z[1],dv[2]-z[2],dv[3]-z[3]], collcationConstraints(dv,p))}
              # 'jac' : lambda dv: np.append(np.eye(4), np.zeros((4,5*p.N-4)), axis=1)
    ineq_cons = {   'type': 'ineq',   # no inequality constraints
                    'fun' : lambda dv: [1],
                    'jac' : lambda dv: np.zeros(dv.shape)}

    # decision variable bounds
    lb = np.full((5*N, 1), -np.inf)
    ub = np.full((5*N, 1), np.inf)
    # max rail length
    lb[0:4*N:4] = -p.L/2
    ub[0:4*N:4] = p.L/2
    # motor saturation
    lb[4*N+1:] = -p.s
    ub[4*N+1:] = p.s
    bounds = tuple(tuple(pair) for pair in np.append(lb,ub,axis=1)) # cast bounds into a tuple
    
    # analytical Jacobian and Hessian to improve computation speed
    jac = lambda dv: 2*np.append(np.tile(Q, N)*np.repeat(T,4),np.tile(R, N))*dv
    hess = lambda dv: np.diag(2*np.append(np.tile(np.diag(Q), N)*np.repeat(T,4),np.tile(R, N)))

    # SQP approach to trajectory optimization 
    res = minimize(partial(trajCost, p=p), dv0, method='SLSQP', constraints=[eq_cons, ineq_cons], options={'ftol': 1e-7, 'disp': False}, bounds=bounds)
    # res = minimize(partial(trajCost, p=p), dv0, method='trust-constr',  constraints=[eq_cons, ineq_cons], options={'disp': False}, bounds=bounds)
    return res

def mpc(z,p,dv0): 
