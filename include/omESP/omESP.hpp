#ifndef OMESP_HPP
#define OMESP_HPP

#include "capsTypes.h"

#include "omESP/omESP_export.hpp"

// /**
//  * @brief Reports the name of the library
//  *
//  * Please see the note above for considerations when creating shared
//  libraries.
//  */
// class OMESP_EXPORT exported_class
// {
// public:
//   /**
//    * @brief Initializes the name field to the name of the project
//    */
//   exported_class();

//   /**
//    * @brief Returns a non-owning pointer to the string stored in this class
//    */
//   auto name() const -> const char*;

// private:
//   OMESP_SUPPRESS_C4251
//   std::string m_name;
// };

namespace omESP
{

OMESP_EXPORT void printErrors(int nErr, capsErrs *errors);

OMESP_EXPORT void tessSensitivity(modl_T &model,
                                  const std::string &dp_name,
                                  ego tess,
                                  std::vector<double> &dsen,
                                  int row_idx = 1,
                                  int col_idx = 1);

/// Finds the index of the design parameter in the @a model given by @a dp_name
/// \param[in] model OCSM model
/// \param[in] dp_name Design parameter name to get the index of
/// \param[in] row_idx Design parameter row index
/// \param[in] col_idx Design parameter column index
/// \return Index of design parameter given by @a dp_name
/// \note @a row_idx and @a col_idx both default to 1, but can be changed. An
/// exception is thrown if the given index exceeds the number of rows/cols in
/// the design parameter
OMESP_EXPORT int findOSCMDesignParameter(modl_T &model,
                            const std::string &dp_name,
                            int row_idx = 1,
                            int col_idx = 1);

}  // namespace omESP

#endif
