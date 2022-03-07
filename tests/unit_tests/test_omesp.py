import unittest
import numpy as np

import openmdao.api as om

from omESP import omESP 

class TestOMESP(unittest.TestCase):

    def test_compute_box(self):
        self.prob = om.Problem()
        self.prob.model.add_subsystem("geom",
                                      omESP(csm_file="data/test_box.csm",
                                            egads_file="data/test_box.egads"))

        self.prob.setup()
        surf_mesh0 = self.prob.get_val("geom.x_surf")
        # print("Initial surface mesh:\n", surf_mesh0)

        self.prob.set_val("geom.dx", 2.0)
        self.prob.set_val("geom.dy", 0.5)
        self.prob.set_val("geom.dz", 3.0)

        self.prob.run_model()

        surf_mesh = self.prob.get_val("geom.x_surf")
        surf_mesh_exact = np.array([0.0, 0.0, 0.0,
                                    0.0, 0.5, 0.0,
                                    0.0, 0.5, 3.0,
                                    0.0, 0.0, 3.0,
                                    2.0, 0.0, 3.0,
                                    2.0, 0.0, 0.0,
                                    2.0, 0.5, 0.0,
                                    2.0, 0.5, 3.0])

        for [val, ex_val] in zip(surf_mesh, surf_mesh_exact):
            self.assertAlmostEqual(val, ex_val)

        


if __name__ == '__main__':

    unittest.main()
