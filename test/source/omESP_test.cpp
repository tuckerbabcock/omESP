#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "caps.h"
#include "capsTypes.h"
#include "egads.h"
#include "egadsTypes.h"
extern "C" {
#include "OpenCSM.h"
}  // End of extern "C"

#include "omESP/omESP.hpp"

auto main() -> int
{
   /// Open up CAPS problem
   constexpr const char *problem_name = "omESP_test";
   // char csm_file[] = "test_box.csm";
   char csm_file[] = "cyl.csm";
   int out_level = 1;
   capsObj problem_obj = nullptr;
   int n_err = 0;
   capsErrs *errors = nullptr;
   caps_open(problem_name,
             nullptr,
             0,
             csm_file,
             out_level,
             &problem_obj,
             &n_err,
             &errors);
   if (n_err != 0)
   {
      omESP::printErrors(n_err, errors);
   }

   auto *problem = static_cast<capsProblem *>(problem_obj->blind);

   auto *ocsm_model = static_cast<modl_T *>(problem->modl);

   /// Build the model
   int built_to = 0;
   int n_body = 0;
   ocsmBuild(ocsm_model, 0, &built_to, &n_body, nullptr);

   ego ocsm_body = ocsm_model->body[1].ebody;

   /// Load EGADS model with tesselation
   ego context = problem->context;
   EG_open(&context);

   // const char *egads_file = "test_box.egads";
   const char *egads_file = "test_cyl.egads";
   ego eg_model = nullptr;
   EG_loadModel(context, 0, egads_file, &eg_model);

   int oclass, mtype, nbody, *senses;
   ego geom, *eg_model_children;
   int status = EG_getTopology(eg_model, &geom, &oclass, &mtype, nullptr, &nbody,
                               &eg_model_children, &senses);
   if (status != EGADS_SUCCESS)
   {
      printf("EGADS failed to get bodies with error code: %d", status);
      return status;
   }
   else if (nbody > 1)
   {
      printf("EGADS model should only have one body\n");
      return -1;
   }

   if (mtype != 2)
   {
      printf("EGADS model should only have one 1 tesselation\n");
      return -1;
   }
   ego body = eg_model_children[0];
   ego tess = eg_model_children[1];

   ego tess_body = nullptr;
   int state = 0;
   int npt = 0;
   EG_statusTessBody(tess, &tess_body, &state, &npt);
   std::cout << "loaded tess npt: " << npt << "\n";


   // Map tess from loaded body to the body ocsm built
   ego new_tess = nullptr;
   status = EG_mapTessBody(tess, ocsm_body, &new_tess);
   if (status != EGADS_SUCCESS)
   {
      printf(" EG_mapTessBody failed with status: %d!\n", status);
      return status;
   }
   ocsm_model->body->etess = new_tess;

   /// pass new tess to `tessSensitivity`
   // std::string wrt = "rad";
   std::string wrt = "dz";

   std::vector<double> dsen;
   omESP::tessSensitivity(*ocsm_model, wrt, ocsm_model->body->etess, dsen);
   std::cout << "dsens_" << wrt << ":\n";
   for (const auto &sen : dsen)
   {
      std::cout << sen << ',';
   }
   std::cout << "\n";

   // omESP::tessSensitivity(*ocsm_model, "dy", ocsm_model->body->etess, dsen);
   // std::cout << "dsens_dy:\n";
   // for (const auto &sen : dsen)
   // {
   //    std::cout << sen << ' ';
   // }
   // std::cout << "\n";



   ////////////////////
   /// Finite difference 'dx'
   ////////////////////
   {
      auto delta = 1e-4;
      auto dx_idx = omESP::findOSCMDesignParameter(*ocsm_model, wrt);
      ocsmSetValuD(ocsm_model, dx_idx, 1, 1, 1+delta);

      /// rebuild
      ocsmBuild(ocsm_model, 0, &built_to, &n_body, nullptr);

      /// map tess to new body
      ocsm_body = ocsm_model->body[1].ebody;
      ego new_tess2 = nullptr;
      status = EG_mapTessBody(tess, ocsm_body, &new_tess2);
      if (status != EGADS_SUCCESS)
      {
         printf(" EG_mapTessBody failed with status: %d!\n", status);
         return status;
      }

      int ptype = 0;
      int pindex = 0;
      double xyz[3];
      double xyz_old[3];

      std::vector<double> dsen_fd(dsen.size());

      auto *raw_tess = static_cast<egTessel *>(tess->blind);
      auto *raw_new_tess = static_cast<egTessel *>(new_tess2->blind);
      for (int i = 1; i <= raw_tess->nGlobal; ++i)
      {
         EG_getGlobal(new_tess2, i, &ptype, &pindex, xyz);
         EG_getGlobal(tess, i, &ptype, &pindex, xyz_old);

         dsen_fd[(i - 1) * 3 + 0] = (xyz_old[0] - xyz[0]) / delta;
         dsen_fd[(i - 1) * 3 + 1] = (xyz_old[1] - xyz[1]) / delta;
         dsen_fd[(i - 1) * 3 + 2] = (xyz_old[2] - xyz[2]) / delta;
         // dsen_fd[i] = (raw_tess->xyzs[i] - raw_new_tess->xyzs[i] ) / delta;
      }

      std::cout << "dsens_" << wrt << "_fd:\n";
      for (const auto &sen : dsen_fd)
      {
         std::cout << sen << ',';
      }
      std::cout << "\n";
   }

   EG_close(context);
   caps_close(problem_obj, 1, nullptr);
   return 0;
}
