cmake_minimum_required(VERSION 3.4)

project(RaiPython)

add_subdirectory(pybind11)

add_compile_options(
  -O3 -g -Wall -fPIC -std=c++14 -Wno-terminate -Wno-pragmas
  )

add_definitions(
  -DRAI_ROOT_PATH=\"${CMAKE_SOURCE_DIR}/rai\"
  -DRAI_X11 -DRAI_LAPACK -DRAI_EIGEN -DRAI_PTHREAD
  -DRAI_GL
  -DRAI_GLFW -DRAI_ANN -DRAI_QHULL -DRAI_ASSIMP -DNO_CYCLE_DETECTION -DRAI_PNG
  -DRAI_SWIFT -DRAI_GJK -DRAI_FCL -DRAI_CCD
  #-DRAI_OPENCV
  -DRAI_PYBIND -DRAI_BIND_KOMO
  #-DRAI_PHYSX -D_DEBUG -DPX_DISABLE_FLUIDS -DCORELIB -DPX32 -DLINUX #physx
  #-DRAI_BULLET #bullet
  )

include_directories(
  rai/rai
  /usr/include/eigen3
  #${BULLET_INCLUDE_DIRS}
  #$ENV{HOME}/opt/physx3.4/include $ENV{HOME}/opt/physx3.4/include/physx
  #$ENV{HOME}/opt/include/bullet/
  )

link_directories(
  #$ENV{HOME}/opt/physx3.4/lib
  #/opt/ros/kinetic/lib
  )

file(GLOB SRC_Core rai/rai/Core/*.cpp)
file(GLOB SRC_Optim rai/rai/Optim/*.cpp)
file(GLOB SRC_Geo rai/rai/Geo/*.cpp)
file(GLOB SRC_GeoPlus rai/rai/Geo/GJK/*.c)
file(GLOB SRC_Algo rai/rai/Algo/*.cpp)
file(GLOB SRC_Plot rai/rai/Plot/*.cpp)
file(GLOB SRC_Gui rai/rai/Gui/*.cpp)
file(GLOB SRC_Kin rai/rai/Kin/*.cpp)
file(GLOB SRC_KinSwift rai/rai/Kin/SWIFT/*.cpp)
file(GLOB SRC_KOMO rai/rai/KOMO/*.cpp)
file(GLOB SRC_Logic rai/rai/Logic/*.cpp)
file(GLOB SRC_LGP rai/rai/LGP/*.cpp)
file(GLOB SRC_RosCom rai/rai/RosCom/*.cpp)
file(GLOB SRC_Perception rai/rai/Perception/*.cpp)
file(GLOB SRC_Control rai/rai/Control/*.cpp)
file(GLOB SRC_Operate rai/rai/Operate/*.cpp)
file(GLOB SRC_ry rai/rai/ry/*.cpp)

add_library(rai SHARED
  ${SRC_Core}
  ${SRC_Optim}
  ${SRC_Geo}
  ${SRC_GeoPlus}
  ${SRC_Algo}
  ${SRC_Plot}
  ${SRC_Gui}
  ${SRC_Kin}
  ${SRC_KinSwift}
  ${SRC_KOMO}
  ${SRC_Logic}
  ${SRC_LGP}
  #${SRC_RosCom}
  ${SRC_Perception}
  ${SRC_Control}
  #${SRC_Operate}
  #${SRC_ry}
  )

target_link_libraries(rai
  jsoncpp lapack blas pthread rt X11
  assimp fcl ann glfw GLEW glut GLU GL png qhull
  #${OpenCV_LIBS}
  #${BULLET_LIBRARIES}
  #PhysX3Extensions PhysX3_x64 PhysX3Cooking_x64 PhysX3Common_x64 PxFoundation_x64
  #BulletSoftBody BulletDynamics BulletCollision LinearMath 
  )

pybind11_add_module(libry SHARED
  ${SRC_ry}
  )

target_link_libraries(libry PRIVATE
  rai
  )

add_executable(kinEdit rai/bin/src_kinEdit/main.cpp)
target_link_libraries(kinEdit rai)


