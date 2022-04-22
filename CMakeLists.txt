cmake_minimum_required(VERSION 3.3)
set(LIBNAME IsoOctree)
set(MAJOR_VERSION 1)
project(${LIBNAME} VERSION "${MAJOR_VERSION}.0.0")

option(IsoOctree_STATIC "Build static lib" OFF)

set(CMAKE_CXX_STANDARD 11)

if (IsoOctree_STATIC)
  set(IsoOctree_LIBTYPE "STATIC")
else()
  set(IsoOctree_LIBTYPE "SHARED")
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(ISO_OCTREE_SRC SRC/api.cpp)
add_library(${LIBNAME} ${IsoOctree_LIBTYPE} ${ISO_OCTREE_SRC})
target_include_directories(${LIBNAME} PRIVATE SRC)
set_target_properties(${LIBNAME} PROPERTIES CXX_VISIBILITY_PRESET hidden)

if(NOT MSVC)
  target_compile_options(${LIBNAME} PRIVATE "-Wno-dangling-else")
endif()

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
