cmake_minimum_required (VERSION 2.8 FATAL_ERROR)

project      (read-segment-vis)
find_package (Qt4 REQUIRED)
find_package (VTK REQUIRED)
find_package (PCL 1.7.1 REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES main.cpp )

INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (read-segment-vis ${project_SOURCES})

TARGET_LINK_LIBRARIES (read-segment-vis ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})



