import numpy as np
import openmdao.api as om

from pyEGADS import egads
from pyOCSM import ocsm

def _getOCSMDesignParameters(ocsm_model):

    des_params = {}

    _, npmtr, _ = ocsm_model.Info();

    for pmtr_idx in range(1, npmtr+1):
        pmtr_type, nrow, ncol, name = ocsm_model.GetPmtr(pmtr_idx)

        if pmtr_type == ocsm.DESPMTR:
            des_params[name] = [pmtr_idx, nrow, ncol]
    
    return des_params

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

def _getTessCoordinates(tess, tess_coords):
    for global_idx in range(1, (tess_coords.size // 3) + 1):
        _, _, xyz = tess.getGlobal(global_idx)
        tess_coords[(global_idx - 1) * 3 + 0] = xyz[0]
        tess_coords[(global_idx - 1) * 3 + 1] = xyz[1]
        tess_coords[(global_idx - 1) * 3 + 2] = xyz[2]

class omESP(om.ExplicitComponent):
    """
    Component to map between ESP CAD model design parameters and the surface mesh on the model
    """

    def initialize(self):
        self.options.declare("csm_file", types=str)
        self.options.declare("egads_file", types=str)

    def setup(self):

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

        ### Add inputs
        self.design_pmtrs = _getOCSMDesignParameters(self.ocsm_model)

        for key, pmtr_info in self.design_pmtrs.items():
            values = _getOCSMParameterValues(self.ocsm_model, pmtr_info)
            self.add_input(key, val=values)

        ### Add outputs
        self.egads_body = children[0]
        self.egads_init_tess = children[1]

        _, _, _, ntess_pts = self.egads_init_tess.statusTessBody()

        tess_coords = np.zeros(3 * ntess_pts)
        _getTessCoordinates(self.egads_init_tess, tess_coords)
        self.add_output("x_surf", val=tess_coords)

    
    def compute(self, inputs, outputs):
        """
        Build the geometry model 
        """
        for input in inputs.keys():
            value = inputs[input]
            if input in self.design_pmtrs:
                pmtr_info = self.design_pmtrs[input]
                _setOCSMParameterValues(self.ocsm_model, pmtr_info, value)

        self.ocsm_model.Build(0, 0)
        ocsm_body = self.ocsm_model.GetEgo(1, ocsm.BODY, 0)

        tess = self.egads_init_tess.mapTessBody(ocsm_body)
        
        _getTessCoordinates(tess, outputs["x_surf"])
