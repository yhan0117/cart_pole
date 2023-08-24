# sim with gym
from context import src
from collections import namedtuple
import numpy as np
import unittest
from src import mpc
import time

N = 60
Params = namedtuple('Params', ['N', 'zd', 'T', 'Q', 'R','dt', 'l', 'mp', 'mc', 'g','L','s'])
p = Params(N,np.array([0,np.pi,0,0]),np.ones(N),np.array([1,1,1,1]),1,0.025,0.5,0.2,1,9.8,1.2,17.5)
z = np.zeros(4)
dv = np.zeros(5*N)
for i in range(N):
    dv[4*i:4*(i+1)] = z + i*(p.zd-z)/(N-1)

s = time.perf_counter()
res = mpc.mpc(z,p,dv)
print(f"Elapsed time: {time.perf_counter()-s:0.4f} seconds")
