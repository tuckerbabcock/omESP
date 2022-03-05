#include <stdexcept>
#include <string>
#include <vector>

#include "caps.h"
#include "capsTypes.h"
extern "C" {
#include "OpenCSM.h"
}  // End of extern "C"

#include "omESP/omESP.hpp"

namespace
{

void setOCSMSensitivity(modl_T &model,
                        const std::string &dp_name,
                        int row_idx = 1,
                        int col_idx = 1)
{
   auto dp_index = omESP::findOSCMDesignParameter(model, dp_name, row_idx, col_idx);

   /// clear all sensitivities then set
   int status = ocsmSetDtime(&model, 0);
   if (status != SUCCESS)
   {
      throw std::runtime_error("Failed in ocsmSetDtime!\n");
   }

   status = ocsmSetVelD(&model, 0, 0, 0, 0.0);
   if (status != SUCCESS)
   {
      throw std::runtime_error("Failed in ocsmSetVelD!\n");
   }

   status = ocsmSetVelD(&model, dp_index, row_idx, col_idx, 1.0);
   if (status != SUCCESS)
   {
      throw std::runtime_error("Failed in ocsmSetVelD!\n");
   }

   int outLevel = ocsmSetOutLevel(0);
   printf(" CAPS Info: Building sensitivity information for: %s[%d,%d]\n",
          dp_name.c_str(),
          row_idx,
          col_idx);

   int buildTo = 0;
   int builtTo = -1;
   int nbody = 0;
   status = ocsmBuild(&model, buildTo, &builtTo, &nbody, nullptr);
   fflush(stdout);
   if (status != SUCCESS)
   {
      throw std::runtime_error("Failed in ocsmBuild!\n");
   }
   ocsmSetOutLevel(outLevel);
}

}  // anonymous namespace

namespace omESP
{
void printErrors(int nErr, capsErrs *errors)
{
   if (errors == nullptr)
   {
      return;
   }

   for (int i = 1; i <= nErr; i++)
   {
      capsObj obj = nullptr;
      int eType = 0;
      int nLines = 0;
      char **lines = nullptr;
      auto stat = caps_errorInfo(errors, i, &obj, &eType, &nLines, &lines);
      if (stat != CAPS_SUCCESS)
      {
         printf(" printErrors: %d/%d caps_errorInfo = %d\n", i, nErr, stat);
         continue;
      }
      constexpr const char *type[] = {
          "Cont:   ", "Info:   ", "Warning:", "Error:  ", "Status: "};

      for (int j = 0; j < nLines; j++)
      {
         if (j == 0)
         {
            printf(" CAPS %s ", type[eType + 1]);
         }
         else
         {
            printf("               ");
         }
         printf("%s\n", lines[j]);
      }
   }

   caps_freeError(errors);
}

void tessSensitivity(modl_T &model,
                     const std::string &dp_name,
                     ego tess,
                     std::vector<double> &dsen,
                     int row_idx,
                     int col_idx)
{
   if ((tess == nullptr) || (tess->magicnumber != MAGIC) ||
       (tess->oclass != TESSELLATION))
   {
      throw std::runtime_error("Bad tessellation input to tessSensitivity!\n");
   }

   ego body = nullptr;
   int state = 0;
   int npt = 0;
   int status = EG_statusTessBody(tess, &body, &state, &npt);
   if ((status == EGADS_OUTSIDE) || (body == nullptr) ||
       (body->magicnumber != MAGIC) || (body->oclass != BODY))
   {
      throw std::runtime_error(
          "Bad body stored in tessellation input to tessSensitivity!\n");
   }

   int nface = -1;
   status = EG_getBodyTopos(body, nullptr, FACE, &nface, nullptr);
   if (status != EGADS_SUCCESS)
   {
      throw std::runtime_error("EG_getBodyTopos failed!\n");
   }

   /// Find which model body corresponds to the body in the tesselation
   int body_idx = -1;
   for (body_idx = 1; body_idx <= model.nbody; body_idx++)
   {
      auto model_body = model.body[body_idx];
      if ((model_body.onstack != 1) || (model_body.botype == OCSM_NULL_BODY))
      {
         continue;
      }
      if (model_body.ebody == body)
      {
         break;
      }
   }
   if (body_idx > model.nbody)
   {
      throw std::runtime_error(
          "Could not find body in model that matches body in tesselation!\n");
   }

   setOCSMSensitivity(model, dp_name, row_idx, col_idx);

   /// initialize sensitivity vector and zero it out
   std::vector<double> dsen_(3 * npt);
   for (int i = 0; i < 3 * npt; i++)
   {
      dsen_[i] = 0.0;
   }

   // auto oldtess = model.body[body_idx].etess;
   // model.body[body_idx].etess = tess;

   auto *btess = static_cast<egTessel *>(tess->blind);
   if (btess->nFace == 0)
   {
      for (int i = 1; i <= btess->nEdge; i++)
      {
         int edge_npts = -1;
         const double *xyz = nullptr;
         const double *uv = nullptr;
         EG_getTessEdge(tess, i, &edge_npts, &xyz, &uv);

         // outLevel = ocsmSetOutLevel(0);
         const double *xyzs = nullptr;
         ocsmGetTessVel(&model, body_idx, OCSM_EDGE, i, &xyzs);
         // ocsmSetOutLevel(outLevel);

         for (int j = 1; j <= edge_npts; j++)
         {
            int global = -1;
            EG_localToGlobal(tess, -i, j, &global);
            dsen_[3 * global - 3] = xyzs[3 * j - 3];
            dsen_[3 * global - 2] = xyzs[3 * j - 2];
            dsen_[3 * global - 1] = xyzs[3 * j - 1];
         }
      }
   }
   else
   {
      for (int i = 1; i <= nface; i++)
      {
         int face_npts = -1;
         int ntris = -1;
         const int *ptype = nullptr;
         const int *pindex = nullptr;
         const int *tris = nullptr;
         const int *tric = nullptr;
         const double *xyz = nullptr;
         const double *uv = nullptr;
         EG_getTessFace(tess,
                        i,
                        &face_npts,
                        &xyz,
                        &uv,
                        &ptype,
                        &pindex,
                        &ntris,
                        &tris,
                        &tric);

         // outLevel = ocsmSetOutLevel(0);
         const double *xyzs = nullptr;
         ocsmGetTessVel(&model, body_idx, OCSM_FACE, i, &xyzs);
         // ocsmSetOutLevel(outLevel);

         for (int j = 1; j <= face_npts; j++)
         {
            int global = -1;
            EG_localToGlobal(tess, i, j, &global);
            dsen_[3 * global - 3] = xyzs[3 * j - 3];
            dsen_[3 * global - 2] = xyzs[3 * j - 2];
            dsen_[3 * global - 1] = xyzs[3 * j - 1];
         }
      }
   }

   // model.body[body_idx].etess = oldtess;
   if (model.dtime != 0.0)
   {
      printf(" CAPS Info: Sensitivity finite difference used for: %s[%d,%d]\n",
             dp_name.c_str(),
             row_idx,
             col_idx);
   }

   dsen = std::move(dsen_);
}

int findOSCMDesignParameter(modl_T &model,
                            const std::string &dp_name,
                            int row_idx,
                            int col_idx)
{
   if (row_idx < 1)
   {
      const std::string err_string = "Requested row index (" +
                                     std::to_string(row_idx) +
                                     ") may not be negative!\n";

      throw std::runtime_error(err_string);
   }
   if (col_idx < 1)
   {
      const std::string err_string = "Requested column index (" +
                                     std::to_string(col_idx) +
                                     ") may not be negative!\n";

      throw std::runtime_error(err_string);
   }
   /// Get the number of parameters in the model
   int nbrch = -1;
   int npmtr = -1;
   int nbody = -1;
   int status = ocsmInfo(&model, &nbrch, &npmtr, &nbody);
   if (status != SUCCESS)
   {
      throw std::runtime_error("Failed to get OCSM Info on model!\n");
   }

   /// Loop over all the parameters in the model
   int nrow = 0;
   int ncol = 0;
   int dp_idx = 0;
   for (int i = 0; i < npmtr; i++)
   {
      int type = OCSM_UNKNOWN;
      char name[MAX_NAME_LEN];
      status = ocsmGetPmtr(&model, i + 1, &type, &nrow, &ncol, name);

      /// Skip the parameter if it is either not a design parameter or not named
      /// @a dp_name
      if ((status != SUCCESS) || (type != OCSM_DESPMTR) || (dp_name == name))
      {
         continue;
      }

      /// Stop the loop when we've found the design parameter named @a dp_name
      /// and record its index
      dp_idx = i + 1;
      break;
   }

   /// If we never found the index, return the index as error code
   /// `CAPS_NOSENSITVTY`
   if (dp_idx == 0)
   {
      dp_idx = CAPS_NOSENSITVTY;
   }

   /// Make sure input @a row_idx and @a col_idx are within the bounds of the
   /// design parameter
   if (row_idx > nrow)
   {
      const std::string err_string =
          "Requested row design parameter " + dp_name + " (" +
          std::to_string(row_idx) +
          ") exceeds either the number of rows in the design parameter (" +
          std::to_string(nrow) + ")\n";

      throw std::runtime_error(err_string);
   }
   if (col_idx > ncol)
   {
      const std::string err_string =
          "Requested column design parameter " + dp_name + " (" +
          std::to_string(col_idx) +
          ") exceeds either the number of columns in the design parameter (" +
          std::to_string(ncol) + ")\n";

      throw std::runtime_error(err_string);
   }
   return dp_idx;
}

}  // namespace omESP
