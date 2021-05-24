# In this file you can unset dependencies, e.g.
# 
# LAPACK=0 #to avoid need to include/link to LAPACK
# PHYSX=0 #to avoid linking to Physx (Nvidia simulator)
# GTK = 0 #to avoid linking to GTK
# etc
#

# only UNCOMMENT some of the following lines
# (they are already set =1 in the components that need to them)

#OPTIM=fast
OPTIM = fast_debug

RAI_CMAKE = 1

#GL = 0
#FCL = 0
ODE = 0
PHYSX = 0
BULLET = 0
G4 = 0
PCL = 0
#PYBIND = 0
OPENCV = 0
GTK = 0
ROS = 0
ROS_VERSION = melodic
#EIGEN = 0
REALSENSE = 0
CERES = 0
NLOPT = 0
IPOPT = 0
