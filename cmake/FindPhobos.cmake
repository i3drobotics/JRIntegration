cmake_minimum_required(VERSION 2.6)

if (WIN32)
    set (Phobos_ROOT_DIR "C:\\Program Files\\JOANNEUM RESEARCH\\Phobos")
    set (OpenCV_DIR "D:\\openCV\\opencv-3.1.0\\build")
    #set (CUDA_TOOLKIT_ROOT_DIR "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/8.0")
elseif(UNIX)
    set (Phobos_ROOT_DIR "/usr/local/Phobos")
endif()

message(WARNING "Phobos_ROOT_DIR: ${Phobos_ROOT_DIR}")
message(WARNING "OpenCV_DIR: ${OpenCV_DIR}")
message(WARNING "CUDA_TOOLKIT_ROOT_DIR: ${CUDA_TOOLKIT_ROOT_DIR}")

# OpenCV 3.1.0 Required for I3DR stereo algorithms
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
    NAMES PhobosIntegration/PhobosIntegration.lib
    HINTS ${Phobos_ROOT_DIR}
    PATH_SUFFIXES lib
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