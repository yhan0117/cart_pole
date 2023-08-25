# sim with gym
from context import src
from collections import namedtuple
import numpy as np
from src import mpc
import time
from gymEnv.cartpole import CartPoleEnv
from gymnasium.wrappers import RecordVideo

# params
N = 60
dt = 1.5/N
T = np.ones(N)
T[-1] = 110
Q = np.array([1,1,1,1])
R = 0.08
zd = np.array([0,np.pi,0,0])

Params = namedtuple('Params', ['N', 'zd', 'T', 'Q', 'R','dt', 'l', 'mp', 'mc', 'g','L','s'])
p = Params(N,zd,T,Q,R,dt,0.5,0.2,1,9.8,1.2,17.5)
dv = np.zeros(5*N)

env = CartPoleEnv("rgb_array")
z, info = env.reset()
z = [z[0],z[2],z[1],z[3]]
for i in range(N):
    dv[4*i:4*(i+1)] = z + i*(p.zd-z)/(N-1)
env = RecordVideo(env, "tests/recording", name_prefix="mpc")

timer = 0
while timer < 1000:
    action = np.array([0.0], dtype=np.float32)
    s = time.perf_counter()
    res = mpc.mpc(z,p,dv)
    u = np.asarray([(res.x[4*N] + res.x[4*N+1])/2], dtype=np.float32)
    print(f"Time taken to solve optimization: {time.perf_counter()-s:0.4f} seconds")
    z, _, _, _, _ = env.step(u)
    z = [z[0],z[2],z[1],z[3]]
    timer += 1

env.close()