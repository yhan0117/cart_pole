from . import interface
from . import optimizer

ef _create_nlp(self)->None:
    """Internal method. See detailed documentation in optimizer.create_nlp
    """
    self._nlp_cons = castools.vertcat(*self._nlp_cons)
    self._nlp_cons_lb = castools.vertcat(*self._nlp_cons_lb)
    self._nlp_cons_ub = castools.vertcat(*self._nlp_cons_ub)

    self.n_opt_lagr = self._nlp_cons.shape[0]
    # Create casadi optimization object:
    nlpsol_opts = {
        'expand': False,
        'ipopt.linear_solver': 'mumps',
    }.update(self.settings.nlpsol_opts)
    self.nlp = {'x': castools.vertcat(self._opt_x), 'f': self._nlp_obj, 'g': self._nlp_cons, 'p': castools.vertcat(self._opt_p)}
    self.S = castools.nlpsol('S', 'ipopt', self.nlp, self.settings.nlpsol_opts)

    # Create function to caculate all auxiliary expressions:
    self.opt_aux_expression_fun = castools.Function('opt_aux_expression_fun', [self._opt_x, self._opt_p], [self._opt_aux])

    # Gather meta information:
    meta_data = {key: getattr(self.settings, key) for key in asdict(self.settings).keys()}
    meta_data.update({'structure_scenario': self.scenario_tree['structure_scenario']})
    self.data.set_meta(**meta_data)

    self._prepare_data()

    self.flags['setup'] = True

def _prepare_nlp(self)->None:
        """Internal method. See detailed documentation with optimizer.prepare_nlp
        """
        self.settings.check_for_mandatory_settings()
        nl_cons_input = self.model['x', 'u', 'z', 'tvp', 'p']
        self._setup_nl_cons(nl_cons_input)
        self._check_validity()

        # Obtain an integrator (collocation, discrete-time) and the amount of intermediate (collocation) points
        ifcn, n_total_coll_points = self._setup_discretization()
        n_branches, n_scenarios, child_scenario, parent_scenario, branch_offset = self._setup_scenario_tree()

        # How many scenarios arise from the scenario tree (robust multi-stage MPC)
        n_max_scenarios = self.n_combinations ** self.settings.n_robust

        # If open_loop option is active, all scenarios (at a given stage) have the same input.
        if self.settings.open_loop:
            n_u_scenarios = 1
        else:
            # Else: Each scenario has its own input.
            n_u_scenarios = n_max_scenarios

        # How many slack variables (for soft constraints) are introduced over the horizon.
        if self.settings.nl_cons_single_slack:
            n_eps = 1
        else:
            n_eps = self.settings.n_horizon

        # Create struct for optimization variables:
        self._opt_x = opt_x = self.model.sv.sym_struct([
            # One additional point (in the collocation dimension) for the final point.
            castools.entry('_x', repeat=[self.settings.n_horizon+1, n_max_scenarios,
                                1+n_total_coll_points], struct=self.model._x),
            castools.entry('_z', repeat=[self.settings.n_horizon, n_max_scenarios,
                                max(n_total_coll_points,1)], struct=self.model._z),
            castools.entry('_u', repeat=[self.settings.n_horizon, n_u_scenarios], struct=self.model._u),
            castools.entry('_eps', repeat=[n_eps, n_max_scenarios], struct=self._eps),
        ])
        self.n_opt_x = self._opt_x.shape[0]
        # NOTE: The entry _x[k,child_scenario[k,s,b],:] starts with the collocation points from s to b at time k
        #       and the last point contains the child node
        # NOTE: Currently there exist dummy collocation points for the initial state (for each branch)

        # Create scaling struct as assign values for _x, _u, _z.
        self.opt_x_scaling = opt_x_scaling = opt_x(1)
        opt_x_scaling['_x'] = self._x_scaling
        opt_x_scaling['_z'] = self._z_scaling
        opt_x_scaling['_u'] = self._u_scaling
        # opt_x are unphysical (scaled) variables. opt_x_unscaled are physical (unscaled) variables.
        self.opt_x_unscaled = opt_x_unscaled = opt_x(opt_x.cat * opt_x_scaling)

        # Create struct for optimization parameters:
        self._opt_p = opt_p = self.model.sv.sym_struct([
            castools.entry('_x0', struct=self.model._x),
            castools.entry('_tvp', repeat=self.settings.n_horizon+1, struct=self.model._tvp),
            castools.entry('_p', repeat=self.n_combinations, struct=self.model._p),
            castools.entry('_u_prev', struct=self.model._u),
        ])
        _w = self.model._w(0)

        self.n_opt_p = opt_p.shape[0]

        # Dummy struct with symbolic variables
        self.aux_struct = self.model.sv.sym_struct([
            castools.entry('_aux', repeat=[self.settings.n_horizon, n_max_scenarios], struct=self.model._aux_expression)
        ])
        # Create mutable symbolic expression from the struct defined above.
        self._opt_aux = opt_aux = self.model.sv.struct(self.aux_struct)

        self.n_opt_aux = opt_aux.shape[0]

        self._lb_opt_x = opt_x(-np.inf)
        self._ub_opt_x = opt_x(np.inf)

        # Initialize objective function and constraints
        obj = castools.DM(0)
        cons = []
        cons_lb = []
        cons_ub = []

        # Initial condition:
        cons.append(opt_x['_x', 0, 0, -1]-opt_p['_x0']/self._x_scaling)

        cons_lb.append(np.zeros((self.model.n_x, 1)))
        cons_ub.append(np.zeros((self.model.n_x, 1)))

        # NOTE: Weigthing factors for the tree assumed equal. They could be set from outside
        # Weighting factor for every scenario
        omega = [1. / n_scenarios[k + 1] for k in range(self.settings.n_horizon)]
        omega_delta_u = [1. / n_scenarios[k + 1] for k in range(self.settings.n_horizon)]

        # For all control intervals
        for k in range(self.settings.n_horizon):
            # For all scenarios (grows exponentially with n_robust)
            for s in range(n_scenarios[k]):
                # For all childen nodes of each node at stage k, discretize the model equations

                # Scenario index for u is always 0 if self.open_loop = True
                s_u = 0 if self.settings.open_loop else s
                for b in range(n_branches[k]):
                    # Obtain the index of the parameter values that should be used for this scenario
                    current_scenario = b + branch_offset[k][s]

                    # Compute constraints and predicted next state of the discretization scheme
                    col_xk = castools.vertcat(*opt_x['_x', k+1, child_scenario[k][s][b], :-1])
                    col_zk = castools.vertcat(*opt_x['_z', k, child_scenario[k][s][b]])
                    [g_ksb, xf_ksb] = ifcn(opt_x['_x', k, s, -1], col_xk,
                                           opt_x['_u', k, s_u], col_zk, opt_p['_tvp', k],
                                           opt_p['_p', current_scenario], _w)

                    # Add the collocation equations
                    cons.append(g_ksb)
                    cons_lb.append(np.zeros(g_ksb.shape[0]))
                    cons_ub.append(np.zeros(g_ksb.shape[0]))

                    # Add continuity constraints
                    cons.append(xf_ksb - opt_x['_x', k+1, child_scenario[k][s][b], -1])
                    cons_lb.append(np.zeros((self.model.n_x, 1)))
                    cons_ub.append(np.zeros((self.model.n_x, 1)))

                    k_eps = min(k, n_eps-1)
                    if self.settings.nl_cons_check_colloc_points:
                        # Ensure nonlinear constraints on all collocation points
                        for i in range(n_total_coll_points):
                            nl_cons_k = self._nl_cons_fun(
                                opt_x_unscaled['_x', k, s, i], opt_x_unscaled['_u', k, s_u], opt_x_unscaled['_z', k, s, i],
                                opt_p['_tvp', k], opt_p['_p', current_scenario], opt_x_unscaled['_eps', k_eps, s])
                            cons.append(nl_cons_k)
                            cons_lb.append(self._nl_cons_lb)
                            cons_ub.append(self._nl_cons_ub)
                    else:
                        # Ensure nonlinear constraints only on the beginning of the FE
                        nl_cons_k = self._nl_cons_fun(
                            opt_x_unscaled['_x', k, s, -1], opt_x_unscaled['_u', k, s_u], opt_x_unscaled['_z', k, s, 0],
                            opt_p['_tvp', k], opt_p['_p', current_scenario], opt_x_unscaled['_eps', k_eps, s])
                        cons.append(nl_cons_k)
                        cons_lb.append(self._nl_cons_lb)
                        cons_ub.append(self._nl_cons_ub)

                    # Add terminal constraints
                    # TODO: Add terminal constraints with an additional nl_cons

                    # Add contribution to the cost
                    obj += omega[k] * self.lterm_fun(opt_x_unscaled['_x', k, s, -1], opt_x_unscaled['_u', k, s_u],
                                                     opt_x_unscaled['_z', k, s, -1], opt_p['_tvp', k], opt_p['_p', current_scenario])
                    # Add slack variables to the cost
                    obj += self.epsterm_fun(opt_x_unscaled['_eps', k_eps, s])

                    # In the last step add the terminal cost too
                    if k == self.settings.n_horizon - 1:
                        obj += omega[k] * self.mterm_fun(opt_x_unscaled['_x', k + 1, s, -1], opt_p['_tvp', k+1],
                                                         opt_p['_p', current_scenario])

                    # U regularization:
                    if k == 0:
                        obj += self.rterm_factor.cat.T@((opt_x['_u', 0, s_u]-opt_p['_u_prev']/self._u_scaling)**2)
                    else:
                        obj += self.rterm_factor.cat.T@((opt_x['_u', k, s_u]-opt_x['_u', k-1, parent_scenario[k][s_u]])**2)

                    # Calculate the auxiliary expressions for the current scenario:
                    opt_aux['_aux', k, s] = self.model._aux_expression_fun(
                        opt_x_unscaled['_x', k, s, -1], opt_x_unscaled['_u', k, s_u], opt_x_unscaled['_z', k, s, -1], opt_p['_tvp', k], opt_p['_p', current_scenario])

                    # For some reason when working with MX, the "unused" aux values in the scenario tree must be set explicitly (they are not ever used...)
                for s_ in range(n_scenarios[k],n_max_scenarios):
                    opt_aux['_aux', k, s_] = self.model._aux_expression_fun(
                        opt_x_unscaled['_x', k, s, -1], opt_x_unscaled['_u', k, s_u], opt_x_unscaled['_z', k, s, -1], opt_p['_tvp', k], opt_p['_p', current_scenario])

        # Set bounds for all optimization variables
        self._update_bounds()

        # Write all created elements to self:
        self._nlp_obj = obj
        self._nlp_cons = cons
        self._nlp_cons_lb = cons_lb
        self._nlp_cons_ub = cons_ub

        # Initialize copies of structures with numerical values (all zero):
        self._opt_x_num = self._opt_x(0)
        self.opt_x_num_unscaled = self._opt_x(0)
        self._opt_p_num = self._opt_p(0)
        self.opt_aux_num = self._opt_aux(0)

        self.flags['prepare_nlp'] = True