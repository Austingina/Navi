# FindCHOLMOD.cmake
find_path(CHOLMOD_INCLUDE_DIR
  NAMES cholmod.h
  PATHS /usr/include /usr/include/suitesparse
  PATH_SUFFIXES suitesparse
)

find_library(CHOLMOD_LIBRARY
  NAMES cholmod
  PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CHOLMOD DEFAULT_MSG CHOLMOD_INCLUDE_DIR CHOLMOD_LIBRARY)

if(CHOLMOD_FOUND)
  set(CHOLMOD_LIBRARIES ${CHOLMOD_LIBRARY})
  set(CHOLMOD_INCLUDE_DIRS ${CHOLMOD_INCLUDE_DIR})
endif()
