import unittest
import numpy as np

import openmdao.api as om
from openmdao.utils.assert_utils import assert_check_partials

from omESP import omESP 

class TestOMESP(unittest.TestCase):

    def test_compute_box(self):
        problem = om.Problem()
        problem.model.add_subsystem("geom",
                                    omESP(csm_file="data/test_box.csm",
                                          egads_file="data/test_box.egads"))

        problem.setup()

        problem.set_val("geom.dx", 2.0)
        problem.set_val("geom.dy", 0.5)
        problem.set_val("geom.dz", 3.0)

        problem.run_model()

        surf_mesh = problem.get_val("geom.x_surf")
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

    def test_check_partials_box(self):
        problem = om.Problem()
        problem.model.add_subsystem("geom",
                                    omESP(csm_file="data/test_box.csm",
                                          egads_file="data/test_box.egads"))

        problem.setup()
        problem.set_val("geom.dx", 2.0)
        problem.set_val("geom.dy", 0.5)
        problem.set_val("geom.dz", 3.0)

        problem.run_model()
        data = problem.check_partials(form="central")
        dx_analytical = data["geom"][("x_surf", "dx")]["J_fwd"]
        dx_fd = data["geom"][("x_surf", "dx")]["J_fd"]
        print("dx an:\n", np.reshape(dx_analytical, (8, 3)))
        print("dx fd:\n", np.reshape(dx_fd, (8, 3)))

        dy_analytical = data["geom"][("x_surf", "dy")]["J_fwd"]
        dy_fd = data["geom"][("x_surf", "dy")]["J_fd"]
        print("dy an:\n", np.reshape(dy_analytical, (8, 3)))
        print("dy fd:\n", np.reshape(dy_fd, (8, 3)))

        dz_analytical = data["geom"][("x_surf", "dz")]["J_fwd"]
        dz_fd = data["geom"][("x_surf", "dz")]["J_fd"]
        print("dz an:\n", np.reshape(dz_analytical, (8, 3)))
        print("dz fd:\n", np.reshape(dz_fd, (8, 3)))

        assert_check_partials(data, atol=1.e-6, rtol=1.e-6)


    def test_check_partials_box_refined(self):
        problem = om.Problem()
        problem.model.add_subsystem("geom",
                                    omESP(csm_file="data/test_box_refined.csm",
                                          egads_file="data/test_box_refined.egads"))

        problem.setup()
        problem.set_val("geom.dx", 2.0)
        problem.set_val("geom.dy", 0.5)
        problem.set_val("geom.dz", 3.0)

        problem.run_model()
        data = problem.check_partials(form="central")
        dx_analytical = data["geom"][("x_surf", "dx")]["J_fwd"]
        dx_fd = data["geom"][("x_surf", "dx")]["J_fd"]
        print("dx an:\n", np.reshape(dx_analytical, (26, 3)))
        print("dx fd:\n", np.reshape(dx_fd, (26, 3)))

        dy_analytical = data["geom"][("x_surf", "dy")]["J_fwd"]
        dy_fd = data["geom"][("x_surf", "dy")]["J_fd"]
        print("dy an:\n", np.reshape(dy_analytical, (26, 3)))
        print("dy fd:\n", np.reshape(dy_fd, (26, 3)))

        dz_analytical = data["geom"][("x_surf", "dz")]["J_fwd"]
        dz_fd = data["geom"][("x_surf", "dz")]["J_fd"]
        print("dz an:\n", np.reshape(dz_analytical, (26, 3)))
        print("dz fd:\n", np.reshape(dz_fd, (26, 3)))

        assert_check_partials(data, atol=1.e-6, rtol=1.e-6)

    def test_check_partials_cyl(self):
        problem = om.Problem()
        problem.model.add_subsystem("geom",
                                    omESP(csm_file="data/test_cyl.csm",
                                          egads_file="data/test_cyl.egads"))

        problem.setup()

        problem.set_val("geom.rad", 1.0)
        problem.set_val("geom.dz", 1.0)

        problem.run_model()
        data = problem.check_partials(form="central")
        
        dz_analytical = data["geom"][("x_surf", "dz")]["J_fwd"]
        dz_fd = data["geom"][("x_surf", "dz")]["J_fd"]
        print("dz an:\n", np.reshape(dz_analytical, (31, 3)))
        print("dz fd:\n", np.reshape(dz_fd, (31, 3)))

        rad_analytical = data["geom"][("x_surf", "rad")]["J_fwd"]
        rad_fd = data["geom"][("x_surf", "rad")]["J_fd"]
        print("rad an:\n", np.reshape(rad_analytical, (31, 3)))
        print("rad fd:\n", np.reshape(rad_fd, (31, 3)))


        assert_check_partials(data, atol=1.e-6, rtol=1.e-6)
        


if __name__ == '__main__':
    unittest.main()
