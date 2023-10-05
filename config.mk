# In this file you can unset dependencies, e.g.
# 
# LAPACK=0 #to avoid need to include/link to LAPACK
# PHYSX=0 #to avoid linking to Physx (Nvidia simulator)
# GTK = 0 #to avoid linking to GTK
# etc
#
# only UNcomment some of the following lines
# (they are already set =1 in the components that depend on them)

## force compile with -g, -O3, or -g -O3 (default: varying for different modules)
#OPTIM = debug
#OPTIM = fast
#OPTIM = fast_debug

RAI_CMAKE = 1

## by default we use OpenGL a lot, but can be disabled
#GL = 0

## by default we compile python bindings using the Ubuntu pybind package, but can be disabled
#PYBIND = 0

## we use the following numerics/optimization libs by default, but can be disabled
#EIGEN = 0
CERES = 0
#NLOPT = 0
#IPOPT = 0

## we use the following collision/physics libraries by default, but can be disabled
#FCL = 0
#BULLET = 0

## below are more libs, which we could use, but are disabled by default
OPENCV = 0
#GRAPHVIZ = 0
GTK = 0
G4 = 0
#PNG = 0
PCL = 0
ODE = 0
#PHYSX = 0
ROS = 0
ROS_VERSION = melodic

