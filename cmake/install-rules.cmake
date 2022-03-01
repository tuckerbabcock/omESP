if(PROJECT_IS_TOP_LEVEL)
  set(CMAKE_INSTALL_INCLUDEDIR include/omESP CACHE PATH "")
endif()

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# find_package(<package>) call for consumers to find this project
set(package omESP)

install(
    DIRECTORY
    include/
    "${PROJECT_BINARY_DIR}/export/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    COMPONENT omESP_Development
)

install(
    TARGETS omESP_omESP
    EXPORT omESPTargets
    RUNTIME #
    COMPONENT omESP_Runtime
    LIBRARY #
    COMPONENT omESP_Runtime
    NAMELINK_COMPONENT omESP_Development
    ARCHIVE #
    COMPONENT omESP_Development
    INCLUDES #
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

write_basic_package_version_file(
    "${package}ConfigVersion.cmake"
    COMPATIBILITY SameMajorVersion
)

# Allow package maintainers to freely override the path for the configs
set(
    omESP_INSTALL_CMAKEDIR "${CMAKE_INSTALL_DATADIR}/${package}"
    CACHE PATH "CMake package config location relative to the install prefix"
)
mark_as_advanced(omESP_INSTALL_CMAKEDIR)

install(
    FILES cmake/install-config.cmake
    DESTINATION "${omESP_INSTALL_CMAKEDIR}"
    RENAME "${package}Config.cmake"
    COMPONENT omESP_Development
)

install(
    FILES "${PROJECT_BINARY_DIR}/${package}ConfigVersion.cmake"
    DESTINATION "${omESP_INSTALL_CMAKEDIR}"
    COMPONENT omESP_Development
)

install(
    EXPORT omESPTargets
    NAMESPACE omESP::
    DESTINATION "${omESP_INSTALL_CMAKEDIR}"
    COMPONENT omESP_Development
)

if(PROJECT_IS_TOP_LEVEL)
  include(CPack)
endif()
