import numpy as np
import openmdao.api as om

from pyEGADS import egads
from pyOCSM import ocsm

def _getOCSMParameters(ocsm_model, type=ocsm.DESPMTR):

    params = {}

    _, npmtr, _ = ocsm_model.Info();

    for pmtr_idx in range(1, npmtr+1):
        pmtr_type, nrow, ncol, name = ocsm_model.GetPmtr(pmtr_idx)

        if pmtr_type == type:
            params[name] = [pmtr_idx, nrow, ncol]
    
    return params

def _getOCSMParameterValues(ocsm_model, pmtr_info):
    pmtr_idx, nrow, ncol = pmtr_info

    # scalar parameter
    if (nrow == 1) and (ncol == 1):
        value, _ = ocsm_model.GetValu(pmtr_idx, 1, 1)
        return value
    
    # vector parameter
    elif (nrow == 1) and (ncol != 1):
        values = np.zeros(ncol)
        for col_idx in range(1, ncol+1):
            values[col_idx-1] = ocsm_model.GetValu(pmtr_idx, 1, col_idx)
        return values
    elif (nrow != 1) and (ncol == 1):
        values = np.zeros(nrow)
        for row_idx in range(1, nrow+1):
            values[row_idx-1] = ocsm_model.GetValu(pmtr_idx, row_idx, 1)
        return values

    # matrix parameter
    else:
        values = np.zeros([nrow, ncol])
        for row_idx in range(1, nrow+1):
            for col_idx in range(1, ncol+1):
                values[row_idx-1, col_idx-1] = ocsm_model.GetValu(pmtr_idx, row_idx, col_idx)

def _setOCSMParameterValues(ocsm_model, pmtr_info, value):
    pmtr_idx, nrow, ncol = pmtr_info

    # scalar parameter
    if (nrow == 1) and (ncol == 1):
        ocsm_model.SetValuD(pmtr_idx, 1, 1, value)
    
    # vector parameter
    elif (nrow == 1) and (ncol != 1):
        for col_idx in range(1, ncol+1):
            ocsm_model.SetValuD(pmtr_idx, 1, col_idx, value[col_idx-1])
    elif (nrow != 1) and (ncol == 1):
        for row_idx in range(1, nrow+1):
            ocsm_model.SetValuD(pmtr_idx, row_idx, 1, value[row_idx-1])

    # matrix parameter
    else:
        for row_idx in range(1, nrow+1):
            for col_idx in range(1, ncol+1):
                ocsm_model.SetValuD(pmtr_idx, row_idx, col_idx, value[row_idx-1, col_idx-1])

def _setOCSMSensitivity(ocsm_model, pmtr_idx, row_idx, col_idx):
    ocsm_model.SetDtime(0.0)
    ocsm_model.SetVelD(0, 0, 0, 0.0)
    ocsm_model.SetVelD(pmtr_idx, row_idx, col_idx, 1.0)
    ocsm_model.Build(0, 0)

def _getTessCoordinates(tess, tess_coords):
    for global_idx in range(1, (tess_coords.size // 3) + 1):
        _, _, xyz = tess.getGlobal(global_idx)
        tess_coords[(global_idx - 1) * 3 + 0] = xyz[0]
        tess_coords[(global_idx - 1) * 3 + 1] = xyz[1]
        tess_coords[(global_idx - 1) * 3 + 2] = xyz[2]

def _getTessBodyIndex(tess, ocsm_model):
    tess_body, _, _, _ = tess.statusTessBody()

    _, _, nbodies = ocsm_model.Info()
    for index in range(1, nbodies+1):
        ocsm_body = ocsm_model.GetEgo(index, ocsm.BODY, 0)
        if tess_body == ocsm_body:
            return index


def _getTessSensitivity(tess, ocsm_model, pmtr_idx, row_idx, col_idx, tess_sens):
    _setOCSMSensitivity(ocsm_model, pmtr_idx, row_idx, col_idx)

    body, _, _, npts = tess.statusTessBody()
    faces = body.getBodyTopos(egads.FACE)
    edges = body.getBodyTopos(egads.EDGE)

    nfaces = len(faces)
    nedges = len(edges)

    body_idx = _getTessBodyIndex(tess, ocsm_model)

    if (nfaces == 0):
        for edge_idx in range(1, nedges+1):
            xyz, _ = tess.getTessEdge(edge_idx)
            edge_npts = len(xyz)

            dxyz = ocsm_model.GetTessVel(body_idx, ocsm.EDGE, edge_idx)
            for edge_pt in range(1, edge_npts+1):
                global_idx = tess.localToGlobal(-edge_idx, edge_pt)
                tess_sens[(global_idx - 1) * 3 + 0] = dxyz[(edge_pt - 1) * 3 + 0]
                tess_sens[(global_idx - 1) * 3 + 1] = dxyz[(edge_pt - 1) * 3 + 1]
                tess_sens[(global_idx - 1) * 3 + 2] = dxyz[(edge_pt - 1) * 3 + 2]
    else:
        for face_idx in range(1, nfaces+1):
            xyz, *_ = tess.getTessFace(face_idx)
            face_npts = len(xyz)

            dxyz = ocsm_model.GetTessVel(body_idx, ocsm.FACE, face_idx)
            for face_pt in range(1, face_npts+1):
                global_idx = tess.localToGlobal(face_idx, face_pt)
                tess_sens[(global_idx - 1) * 3 + 0] = dxyz[(face_pt - 1) * 3 + 0]
                tess_sens[(global_idx - 1) * 3 + 1] = dxyz[(face_pt - 1) * 3 + 1]
                tess_sens[(global_idx - 1) * 3 + 2] = dxyz[(face_pt - 1) * 3 + 2]


class omESP(om.ExplicitComponent):
    """
    Component to map between ESP CAD model design parameters and the surface mesh on the model
    """

    def initialize(self):
        self.options.declare("csm_file", types=str)
        self.options.declare("egads_file", types=str)

    def setup(self):
        ocsm.SetOutLevel(0)
        ### Load CSM model
        csm_file = self.options["csm_file"]
        self.ocsm_model = ocsm.Ocsm(csm_file)

        ### Load EGADS model and validate tess
        egads_file = self.options["egads_file"]
        self.context = egads.Context()
        self.egads_model = self.context.loadModel(egads_file)

        oclass, mtype, geom, reals, children, senses = self.egads_model.getTopology()
        if mtype == 0:
            raise RuntimeError("Loaded EGADS model must inclue a tesselation")
            
        nbody = mtype // 2
        if (nbody != 1):
            raise RuntimeError("omESP only supports EGADS models with one body!")

        # Add inputs
        self.design_pmtrs = _getOCSMParameters(self.ocsm_model)
        for key, pmtr_info in self.design_pmtrs.items():
            values = _getOCSMParameterValues(self.ocsm_model, pmtr_info)
            self.add_input(key, val=values)

            print(f"adding input: {key} with value: {values}")

        # Add tesselation output
        self.egads_body = children[0]
        self.egads_init_tess = children[1]

        _, _, _, ntess_pts = self.egads_init_tess.statusTessBody()
        print("tess pts:", ntess_pts)

        tess_coords = np.zeros(3 * ntess_pts)
        _getTessCoordinates(self.egads_init_tess, tess_coords)
        self.add_output("x_surf", val=tess_coords)

        # Add configuration outputs
        self.config_pmtrs = _getOCSMParameters(self.ocsm_model, ocsm.CFGPMTR)
        for key, pmtr_info in self.config_pmtrs.items():
            values = _getOCSMParameterValues(self.ocsm_model, pmtr_info)
            self.add_output(key, val=values)

    def setup_partials(self):
        # self.declare_partials("x_surf", "*")
        # self.declare_partials("x_surf", "*", method="fd")
        pass
    
    def compute(self, inputs, outputs):
        """
        Build the geometry model 
        """

        print("ESP inputs:")
        for input in inputs.keys():
            print(f"{input}: {inputs[input]}")

        for key, pmtr_info in self.config_pmtrs.items():
            values = _getOCSMParameterValues(self.ocsm_model, pmtr_info)
            outputs[key] = values

        for input in inputs.keys():
            value = inputs[input]
            if input in self.design_pmtrs:
                pmtr_info = self.design_pmtrs[input]
                _setOCSMParameterValues(self.ocsm_model, pmtr_info, value)

        _, nbodies, bodies = self.ocsm_model.Build(0, 1)
        ocsm_body = self.ocsm_model.GetEgo(bodies[nbodies-1], ocsm.BODY, 0)

        tess = self.egads_init_tess.mapTessBody(ocsm_body)
        _getTessCoordinates(tess, outputs["x_surf"])

    # def compute_partials(self, inputs, partials):

    #     for input in inputs.keys():
    #         value = inputs[input]
    #         if input in self.design_pmtrs:
    #             pmtr_info = self.design_pmtrs[input]
    #             _setOCSMParameterValues(self.ocsm_model, pmtr_info, value)

    #     _, nbodies, bodies = self.ocsm_model.Build(0, 1)
    #     ocsm_body = self.ocsm_model.GetEgo(bodies[nbodies-1], ocsm.BODY, 0)

    #     tess = self.egads_init_tess.mapTessBody(ocsm_body)
    #     self.ocsm_model.SetEgo(bodies[nbodies-1], 1, tess)

    #     for input in inputs.keys():
    #         pmtr_idx, nrow, ncol = self.design_pmtrs[input]
    #         if (nrow > 1) or (ncol > 1):
    #             raise RuntimeError("omESP component currently only supports partial"
    #                                " derivatives with respect to scalar design parameters")

    #         _getTessSensitivity(tess, self.ocsm_model, pmtr_idx, 1, 1, partials["x_surf", input])



