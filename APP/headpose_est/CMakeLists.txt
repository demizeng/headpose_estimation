cmake_minimum_required (VERSION 2.6 FATAL_ERROR)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

project      (headpose_est)
find_package (Qt4 REQUIRED)
find_package (VTK REQUIRED)
find_package (PCL 1.7.1 REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

find_package(CUDA REQUIRED)
INCLUDE(FindCUDA)
include_directories(${CUDA_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeScripts)
set(Super4PCS_DIR /usr/local/lib/cmake)
find_package(Super4PCS REQUIRED)
include_directories(${Super4PCS_INCLUDE_DIR})
link_directories(${Super4PCS_LIB_DIR})

add_subdirectory(preprocess)
add_subdirectory(registration)

set  (project_SOURCES main.cpp headpose.cpp)
set  (project_HEADERS headpose.h)
set  (project_FORMS   headpose.ui)
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (headpose_est ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (headpose_est ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} preprocess registration)

