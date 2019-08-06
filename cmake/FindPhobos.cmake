cmake_minimum_required(VERSION 2.6)

set (Phobos_ROOT_DIR "/usr/local/Phobos")

# OpenCV 3.4.6 Required for I3DR stereo algorithms
#find_package(OpenCV 3.4.6 REQUIRED)
find_package(OpenCV 3.1.0 REQUIRED)

find_path(PHOBOS_INCLUDE_DIR
    NAMES PhobosIntegration/PhobosIntegration.hpp
    HINTS ${Phobos_ROOT_DIR}
    PATH_SUFFIXES include
    NO_DEFAULT_PATH
    DOC "The Phobos include directory"
)

if (NOT PHOBOS_INCLUDE_DIR)
    message(WARNING "include directory not found")
endif()

find_library(PHOBOS_LIBRARY 
    NAMES PhobosIntegration
    PATHS ${Phobos_ROOT_DIR}/lib
    NO_DEFAULT_PATH
    DOC "The Phobos library"
)

if (NOT PHOBOS_LIBRARY)
    message(WARNING "library directory not found")
endif()

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LOGGING_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(PHOBOS DEFAULT_MSG PHOBOS_INCLUDE_DIR PHOBOS_LIBRARY)

if (PHOBOS_FOUND)
    set(Phobos_LIBRARIES ${PHOBOS_LIBRARY} )
    set(Phobos_INCLUDE_DIRS ${PHOBOS_INCLUDE_DIR} )
    set(Phobos_DEFINITIONS )
else(PHOBOS_FOUND)
    message(WARNING "Phobos not found")
endif()

# Tell cmake GUIs to ignore the "local" variables.
mark_as_advanced(Phobos_ROOT_DIR PHOBOS_LIBRARY PHOBOS_LIBRARY)