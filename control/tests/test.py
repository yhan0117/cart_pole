from context import src
from collections import namedtuple
import numpy as np
import unittest
from functools import partial

# test if optimization setup is functional
class TestOptim(unittest.TestCase):
    
    def test_cons_dim(self):    # check constraint dimension
        from src import mpc
        Params = namedtuple('Params', ['N', 'dt', 'l', 'mp', 'mc', 'g'])
        p = Params(10,0.1,1,1,1,1)
        z = np.arange(4)
        dv = np.arange(50)
        con = lambda dv: np.append([dv[0]-z[0],dv[1]-z[1],dv[2]-z[2],dv[3]-z[3]], mpc.collcationConstraints(dv,p))
        self.assertEqual(con(dv).shape, (4*p.N,))

    def test_cost(self):    # check return value of cost function
        from src import mpc
        N = 10
        Params = namedtuple('Params', ['N', 'zd', 'T', 'Q', 'R'])
        p = Params(N,np.zeros(4),np.arange(N),np.zeros((4,4)),1)
        dv = np.ones(50)
        fun = partial(mpc.trajCost, p=p)
        self.assertEqual(fun(dv).shape, ()) # is scalar
        self.assertEqual(fun(dv),N)         # troubleshoot JR    
        p = Params(N,np.zeros(4),np.ones(N),np.diag([1,2,3,4]),0)
        fun = partial(mpc.trajCost, p=p)
        self.assertEqual(fun(dv),100)       # troubleshoot JQ    

    def test_opt(self):    # check optimization result
        from src import mpc
        N = 10
        Params = namedtuple('Params', ['N', 'zd', 'T', 'Q', 'R','dt', 'l', 'mp', 'mc', 'g','L','s'])
        p = Params(N,np.zeros(4),np.arange(N),np.zeros((4,4)),1,0.1,1,1,1,1,np.inf,np.inf)
        dv = np.ones(50)
        z = np.arange(4)
        res = mpc.mpc(z,p,dv)
        self.assertTrue(res.success)    

if __name__ == "__main__":
    unittest.main()
