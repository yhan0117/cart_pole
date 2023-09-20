import numpy as np
import os
import subprocess
from casadi import *
import casadi.tools as ca
from typing import Union, Callable, Optional
from dataclasses import asdict

class OCP():
    """
    optimal control solver class
    """
    def __init__(self,p):
        """
        parameters to unpack 
        - model (physical parameters)
        - control parameters (N, dt, etc)
        - cost matrices
        - optimization parameters (options, initial guess)
        """
        # physical parameters
        self.l  = p.l       # pendulum length
        self.mc = p.mc      # cart mass
        self.mp = p.mp      # pendulum mass
        self.g  = p.g       # gravity    
        self.L  = p.L       # rail length
        self.s  = p.s       # motor saturation
        self.w  = p.w       # motor coefficient

        # control parameters
        self.N = N = p.N    # prediction horizon 
        self.dt = dt = p.dt # control looprate
        self.t_span = N*dt  # prediction time span
        self.zd = p.zd      # desired states

        # cost factors
        self.T = p.T        # weight factor
        self.Q = p.Q        # error cost matrix
        self.R = p.R        # actuation effort cost matrix

        # optimization parameters
        self.options = p.options    # optimizer options
        self.z0 = p.z0              # initial guess

    def _trajCost(self,dv,p):     
        # extract from decision variables
        z = np.array([dv["_x"][4*i:4*(i+1)] for i in range(self.N)])
        u = dv["_u"][4*self.N:]

        # error 
        e = z - p.zd

        # cost in QR form
        JQ = sum([self.Q@(e[i,:]**2)*self.T[i] for i in range(self.N)])    # 
        JR = self.R*(u@u)
        
        # total cost
        J = ca.DM(0)
        J = JQ+JR

        return J    

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

    def col_con(self,dv,p)->None:
        # TODO:
        #   collocation constraint
        #   same format (function of casadi variables) as objective function
        return 1
    
    def prepare_nlp(self)->None:
        # TODO: Update parameters
        # struct for optimization variables
        self._dv = dv = ca.struct_symSX([
            ca.entry('_x', repeat=[self.N]),
            ca.entry('_u', repeat=[self.N])
        ])

        # struct for optimization parameters
        self._opt_p = opt_p = ca.struct_symSX([
            ca.entry('_x0', repeat=4),
            ca.entry('_c', struct=self.control_params),
            ca.entry('_p', struct=self.physical_params),
            ca.entry('_u_prev', repeat=[self.N]),
        ])

        # ------------------------------------------------- 
        # objective function +constraints / discretization
        # this should take the same format as the objective function
        # should be included under field 'g'
        obj = self._trajCost(dv,opt_p)
        cons = self.col_con()   # <-- to be modified

        # initial condition
        # should be part of g dict
        cons.append(dv['_x', 0, 0, -1]-opt_p['_x0'])

        # Write all created elements to self:
        self._obj = obj
        self._cons = cons

        # -------------------------------------------------
        # bounds --> not required by compiler
        # instead passed as parameter to the compiled solver object
        self._lb_x = -0.75
        self._ub_x = 0.75

        # bounds on objective function
        cons_lb = []
        cons_ub = []
        cons_lb.append(np.zeros((self.model.n_x, 1)))
        cons_ub.append(np.zeros((self.model.n_x, 1)))

        # write to self 
        self._cons_lb = cons_lb
        self._cons_ub = cons_ub

        # -------------------------------------------------------
        # initialize guess
        self._opt_x_num = self._opt_x(0)
        self._opt_p_num = self._opt_p(0)

        # write to self
        self.flags['prepare_nlp'] = True

    def _create_nlp(self)->None:
        self._dv = ca.vertcat(self._dv)
        self._cons = ca.vertcat(self._cons)
        self._opt_p = ca.vertcat(self._opt_p)

        # Create casadi optimization object:
        # TODO:
        #   find nlopsol_opts from dompc & = self.options
        nlpsol_opts = {
            'expand': False,
            'ipopt.linear_solver': 'mumps',
        }.update(self.nlpsol_opts)

        # create solver object to compile
        self.nlp = {'x': self._dv, 'f': self._obj, 'g': self._cons, 'p': self._opt_pp}
        self.S = ca.nlpsol('S', 'ipopt', self.nlp, self.settings.nlpsol_opts)

        self.flags['setup'] = True

    def compile_nlp(self, overwrite:bool = False, cname:str = 'nlp.c', libname:str='nlp.so', compiler_command:str=None)->None:
        if not self.flags['setup']:
            raise Exception('Optimizer not setup. Call setup first.')

        if ca.sys.platform  not in ('darwin', 'linux', 'linux2'):
            raise Exception('Compilation not supported on this platform.')

        if compiler_command is None:
            compiler_command = "gcc -fPIC -shared -O1 {cname} -o {libname}".format(cname=cname, libname=libname)

        # Only compile if not already compiled:
        if overwrite or not os.path.isfile(libname):
            # Create c code from solver object
            print('Generating c-code of nlp.')
            self.S.generate_dependencies(cname)
            # Compile c code
            print('Compiling c-code of nlp.')
            subprocess.Popen(compiler_command, shell=True).wait()

        # Overwrite solver object with loaded nlp:
        self.S = ca.nlpsol('solver_compiled', 'ipopt', libname, self.nlpsol_opts)
        print('Using compiled NLP solver.')


    def solve(self)->None:
        assert self.flags['setup'] == True, 'optimizer was not setup yet. Please call optimizer.setup().'

        self._cons_lb = ca.vertcat(*self._cons_lb)
        self._cons_ub = ca.vertcat(*self._cons_ub)

        solver_call_kwargs = {
            'x0': self.opt_x_num,
            'lbx': self._lb_opt_x,
            'ubx': self._ub_opt_x,
            'lbg': self.nlp_cons_lb,
            'ubg': self.nlp_cons_ub,
            'p': self.opt_p_num,
        }

        # Warmstarting the optimizer after the initial run:
        if self.flags['initial_run']:
            solver_call_kwargs.update({
                'lam_x0': self.lam_x_num,
                'lam_g0': self.lam_g_num,
            })

        r = self.S(**solver_call_kwargs)
        # Note: .master accesses the underlying vector of the structure.
        self.opt_x_num.master = r['x']
        self.opt_x_num_unscaled.master = r['x']*self.opt_x_scaling
        self.opt_g_num = r['g']
        # Values of lagrange multipliers:
        self.lam_g_num = r['lam_g']
        self.lam_x_num = r['lam_x']
        self.solver_stats = self.S.stats()

        # For warmstarting purposes: Flag that initial run has been completed.
        self.flags['initial_run'] = True