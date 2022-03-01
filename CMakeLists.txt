cmake_minimum_required(VERSION 3.14)

include(cmake/prelude.cmake)

project(
    omESP
    VERSION 0.1.0
    DESCRIPTION "OpenMDAO interface to The Engineering Sketch Pad (https://acdl.mit.edu/esp/)"
    HOMEPAGE_URL "https://example.com/"
    LANGUAGES CXX
)

include(cmake/project-is-top-level.cmake)
include(cmake/variables.cmake)

# ---- Declare library ----

add_library(
    omESP_omESP
    source/omESP.cpp
)
add_library(omESP::omESP ALIAS omESP_omESP)

include(GenerateExportHeader)
generate_export_header(
    omESP_omESP
    BASE_NAME omESP
    EXPORT_FILE_NAME export/omESP/omESP_export.hpp
    CUSTOM_CONTENT_FROM_VARIABLE pragma_suppress_c4251
)

if(NOT BUILD_SHARED_LIBS)
  target_compile_definitions(omESP_omESP PUBLIC OMESP_STATIC_DEFINE)
endif()

set_target_properties(
    omESP_omESP PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    VERSION "${PROJECT_VERSION}"
    SOVERSION "${PROJECT_VERSION_MAJOR}"
    EXPORT_NAME omESP
    OUTPUT_NAME omESP
)

target_include_directories(
    omESP_omESP ${warning_guard}
    PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
)

target_include_directories(
    omESP_omESP SYSTEM
    PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/export>"
)

target_compile_features(omESP_omESP PUBLIC cxx_std_17)

# ---- Install rules ----

if(NOT CMAKE_SKIP_INSTALL_RULES)
  include(cmake/install-rules.cmake)
endif()

# ---- Developer mode ----

if(NOT omESP_DEVELOPER_MODE)
  return()
elseif(NOT PROJECT_IS_TOP_LEVEL)
  message(
      AUTHOR_WARNING
      "Developer mode is intended for developers of omESP"
  )
endif()

include(cmake/dev-mode.cmake)
