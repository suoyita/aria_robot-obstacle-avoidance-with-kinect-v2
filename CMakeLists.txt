cmake_minimum_required(VERSION 2.8)
PROJECT(SimpleProject)

OPTION(ENABLE_OPENGL "Enable OpenGL support" ON)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -L /usr/local/Aria/lib/ -lAria -ldl -lpthread")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -L /usr/local/Aria/lib/ -lAria -ldl -lpthread")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

SET(CMAKE_MODULE_LINKER_FLAGS "-L /usr/local/Aria/lib/ -lAria -ldl -lpthread")
SET(CMAKE_SHARED_LINKER_FLAGS "-L /usr/local/Aria/lib/ -lAria -ldl -lpthread")
SET(CMAKE_EXE_LINKER_FLAGS "-L /usr/local/Aria/lib/ -lAria -ldl -lpthread")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

FIND_PACKAGE(freenect2 REQUIRED)
INCLUDE_DIRECTORIES(${freenect2_INCLUDE_DIR})

#FIND_PACKAGE(PTHREAD)
#INCLUDE_DIRECTORIES(${PTHREAD_INCLUDE_DIRS})
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(PkgConfig)    # try find PKGConfig as it will be used if found
find_package(OpenCV REQUIRED)


# 添加g2o的依赖
# 因为g2o不是常用库，要添加它的findg2o.cmake文件
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( G2O_ROOT /usr/local/include/g2o )
find_package( G2O )
# CSparse
find_package( CSparse )
include_directories( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

set(ALL_G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_csparse_extension g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse)

set(Final_DLLS ${freenect2_DLLS})
set(Final_LIBS ${freenect2_LIBRARIES} ${PCL_LIBRARIES} pcl_common pcl_io pcl_features pcl_kdtree ${OpenCV_LIBS} ${ALL_G2O_LIBS}  ${CSPARSE_LIBRARY} pthread rt)

if(ENABLE_OPENGL)
  find_package(GLFW3)
  find_package(OpenGL)
  if(GLFW3_FOUND AND OPENGL_FOUND)
    include_directories(
      ${GLFW3_INCLUDE_DIRS}
      ${flextGL_INCLUDE_DIRS}
    )

    list(APPEND Final_DLLS ${GLFW3_DLL})
    list(APPEND Final_LIBS
      ${GLFW3_LIBRARIES}
      ${OPENGL_gl_LIBRARY}
    )
    add_definitions(-DEXAMPLES_WITH_OPENGL_SUPPORT=1)
  endif()
endif(ENABLE_OPENGL)

#FIND_PACKAGE(Threads REQUIRED)

FIND_PACKAGE(ARIA)
INCLUDE_DIRECTORIES(${ARIA_INCLUDE_DIRS})

SET(SimpleProject_SOURCES
    main.cpp
    slampart.cpp
    slam.cpp
    kinect.cpp
    pioneer.cpp
    global.cpp
    src/json_writer.cpp
    src/json_valueiterator.inl
    src/json_value.cpp
    src/json_reader.cpp
    src/httpClient.cpp)
#SET(SimpleProject_FORMS mainwindow.ui)
SET(SimpleProject_HEADERS global.h kinect.h pioneer.h slam.h)

QT4_WRAP_CPP(SimpleProject_HEADERS_MOC ${SimpleProject_HEADERS})
QT4_WRAP_UI(SimpleProject_FORMS_HEADERS ${SimpleProject_FORMS})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
ADD_EXECUTABLE(SimpleProject
    ${SimpleProject_SOURCES}
    ${SimpleProject_HEADERS_MOC}
    ${SimpleProject_FORMS_HEADERS}
)

set_property(TARGET SimpleProject PROPERTY COMPILE_OPTIONS -L /usr/local/Aria/lib/ -lAria -ldl -lpthread)
set_property(TARGET SimpleProject PROPERTY INTERFACE_COMPILE_OPTIONS -L /usr/local/Aria/lib/ -lAria -ldl -lpthread)
#LIST(APPEND ARIA_LIBRARIES pthread dl)
#add_dependencies(${ARIA_LIBRARIES} pthread dl)
#TARGET_LINK_LIBRARIES(${ARIA_INCLUDE_DIRS} pthread dl)
#TARGET_LINK_LIBRARIES(${ARIA_LIBRARIES} pthread dl)


TARGET_LINK_LIBRARIES(SimpleProject curl ${QT_LIBRARIES} ${freenect2_LIBRARIES} ${ARIA_LIBRARIES} ${Final_LIBS} Aria pthread dl)
