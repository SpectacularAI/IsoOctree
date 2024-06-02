cmake_minimum_required(VERSION 3.3)

option(IsoOctree_PYTHON "Build python bindings" OFF)
option(IsoOctree_STATIC "Build static lib" OFF)

if (IsoOctree_PYTHON)
  set(LIBNAME "IsoOctreeNative")
else()
  set(LIBNAME "IsoOctree")
endif()

file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION.txt" VERSION_CONTENTS)
project(${LIBNAME} VERSION ${VERSION_CONTENTS})

if (IsoOctree_STATIC OR IsoOctree_PYTHON)
  set(IsoOctree_LIBTYPE "STATIC")
else()
  set(IsoOctree_LIBTYPE "SHARED")
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(ISO_OCTREE_SRC SRC/api.cpp)
add_library(${LIBNAME} ${IsoOctree_LIBTYPE} ${ISO_OCTREE_SRC})
target_include_directories(${LIBNAME} PRIVATE SRC)
set_target_properties(${LIBNAME} PROPERTIES CXX_VISIBILITY_PRESET hidden)

if(NOT MSVC)
  target_compile_options(${LIBNAME} PRIVATE "-Wno-dangling-else")
endif()

if (NOT IsoOctree_PYTHON)
  include(GNUInstallDirs)

  install(FILES
    include/IsoOctree/IsoOctree.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LIBNAME}
    COMPONENT Devel)

  # CMake installation boilerplate
  install(TARGETS ${LIBNAME}
    EXPORT ${LIBNAME}Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

  set_property(TARGET ${LIBNAME} PROPERTY VERSION ${PROJECT_VERSION})
  set_property(TARGET ${LIBNAME} PROPERTY SOVERSION ${MAJOR_VERSION})
  set_property(TARGET ${LIBNAME} PROPERTY INTERFACE_${LIBNAME}_MAJOR_VERSION ${MAJOR_VERSION})
  set_property(TARGET ${LIBNAME} APPEND PROPERTY COMPATIBLE_INTERFACE_STRING ${LIBNAME}_MAJOR_VERSION)

  include(CMakePackageConfigHelpers)
  write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${LIBNAME}/${LIBNAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
  )

  export(EXPORT ${LIBNAME}Targets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/${LIBNAME}/${LIBNAME}Targets.cmake"
    NAMESPACE ${LIBNAME}::
  )
  configure_file(cmake/${LIBNAME}Config.cmake
    "${CMAKE_CURRENT_BINARY_DIR}/${LIBNAME}/${LIBNAME}Config.cmake"
    COPYONLY
  )

  set(ConfigPackageLocation lib/cmake/${LIBNAME})
  install(EXPORT ${LIBNAME}Targets
    FILE
      ${LIBNAME}Targets.cmake
    NAMESPACE
      ${LIBNAME}::
    DESTINATION
      ${ConfigPackageLocation}
  )
  install(
    FILES
      cmake/${LIBNAME}Config.cmake
      "${CMAKE_CURRENT_BINARY_DIR}/${LIBNAME}/${LIBNAME}ConfigVersion.cmake"
    DESTINATION
      ${ConfigPackageLocation}
    COMPONENT
      Devel
  )

endif()