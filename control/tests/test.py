from context import src
import unittest

# Test if optimization setup is functional
class OCP(unittest.TestCase):

    def test_something(self):
        # do something
        self.assertEqual(1, 1)

if __name__ == "__main__":
    unittest.main()
# Artificial trajectory
# if __name__ == "__main__":
#     z = np.arange(4)
#     dv = np.arange(50)
#     Params = namedtuple('Params', ['N', 'dt', 'l', 'mp', 'mc', 'g'])
#     p = Params(10,0.1,1,1,1,1)

#     print(np.append([[dv[0]-z[0]],[dv[1]-z[1]],[dv[2]-z[2]],[dv[3]-z[3]]], collcationConstraints(dv,p)))