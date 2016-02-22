find_path(O3D3XX_IMAGE_INCLUDE_DIRS
  NAMES o3d3xx_image.h
  PATHS /opt/libo3d3xx/include /usr/include
  NO_DEFAULT_PATH
  DOC "o3d3xx_image Include directory"
  )

find_library(O3D3XX_IMAGE_LIBRARIES
  NAMES o3d3xx_image libo3d3xx_image_static.a
  PATHS /opt/libo3d3xx/lib /usr/lib
  NO_DEFAULT_PATH
  DOC "o3d3xx_image shared object file"
  )

get_filename_component(
  O3D3XX_IMAGE_LIBRARY_DIR
  ${O3D3XX_IMAGE_LIBRARIES}
  DIRECTORY
  )

# kind of hacky
include(
 "${O3D3XX_IMAGE_LIBRARY_DIR}/o3d3xx_image/o3d3xx_image-config-version.cmake"
 )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(O3D3XX_IMAGE
  FOUND_VAR O3D3XX_IMAGE_FOUND
  REQUIRED_VARS O3D3XX_IMAGE_LIBRARIES
                O3D3XX_IMAGE_INCLUDE_DIRS
  VERSION_VAR O3D3XX_IMAGE_VERSION
  )

set(_version_major ${o3d3xx_image_FIND_VERSION_MAJOR})
set(_version_minor ${o3d3xx_image_FIND_VERSION_MINOR})
set(_version_patch ${o3d3xx_image_FIND_VERSION_PATCH})
set(O3D3XX_IMAGE_FIND_VERSION
    "${_version_major}.${_version_minor}.${_version_patch}")

if("${O3D3XX_IMAGE_VERSION}" VERSION_LESS "${O3D3XX_IMAGE_FIND_VERSION}")
  set(O3D3XX_IMAGE_VERSION_COMPATIBLE FALSE)
  set(O3D3XX_IMAGE_VERSION_EXACT FALSE)
else()
  set(O3D3XX_IMAGE_VERSION_COMPATIBLE TRUE)
  if ("${O3D3XX_IMAGE_VERSION}" VERSION_EQUAL "${O3D3XX_IMAGE_FIND_VERSION}")
    set(O3D3XX_IMAGE_VERSION_EXACT TRUE)
  else()
    set(O3D3XX_IMAGE_VERSION_EXACT FALSE)
  endif()
endif()

if(${o3d3xx_image_FIND_VERSION_EXACT})
  if(NOT ${O3D3XX_IMAGE_VERSION_EXACT})
    message(FATAL_ERROR
     "o3d3xx_image: ${o3d3xx_image_FIND_VERSION} != ${O3D3XX_IMAGE_VERSION}")
  endif()
endif()

if(NOT ${O3D3XX_IMAGE_VERSION_COMPATIBLE})
  message(FATAL_ERROR
   "o3d3xx_image: ${O3D3XX_IMAGE_VERSION} is incompatible with ${o3d3xx_image_FIND_VERSION}")
endif()

mark_as_advanced(
  O3D3XX_IMAGE_LIBRARIES
  O3D3XX_IMAGE_INCLUDE_DIRS
  )

# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()
