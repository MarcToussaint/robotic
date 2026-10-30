# Target architecture: x86_64 (default) or aarch64.
# Used for the base image, the CMake download, and matches `uname -m`.
ARG ARCH=x86_64
FROM quay.io/pypa/manylinux_2_28_${ARCH}
# re-declare so ${ARCH} is usable in RUN steps below the FROM
ARG ARCH

ENV DEBIAN_FRONTEND=noninteractive
ENV HOME="/root"
ENV TERM="xterm"
ENV MAKEFLAGS="-j$(command nproc --ignore 1)"

##########################################################

RUN dnf -y install epel-release && \
    dnf -y install --enablerepo=powertools \
        gcc gcc-gfortran wget clang \
        zlib-devel freetype-devel \
        mesa-libGLU-devel \
        libXcursor-devel libXi-devel libXinerama-devel libXrandr-devel libXxf86vm-devel \
        glm-devel \
        openblas-devel && \
    wget https://www.netlib.org/f2c/f2c.h -O /usr/local/include/f2c.h
    
# the base image ships CMake 4, which rejects projects declaring cmake_minimum_required < 3.5
ENV CMAKE_POLICY_VERSION_MINIMUM=3.5

RUN wget https://github.com/MarcToussaint/rai/raw/master/_make/install.sh; chmod a+x install.sh

# (some of these we compile manually just to ensure the -fPIC, which is missing in yum packages)
RUN ./install.sh -gc jsoncpp
RUN ./install.sh -gc libyaml
RUN ./install.sh -gc libpng
# hdf5 1.10.4 predates GCC 14, which turned implicit declarations and incompatible pointer assignments into errors.
RUN env CC="gcc -Wno-error=implicit-function-declaration -Wno-error=incompatible-pointer-types" ./install.sh -gc hdf5
RUN ./install.sh -gc freeglut
RUN ./install.sh -gc glew
RUN ./install.sh -gc glfw

RUN ./install.sh -gc eigen
# libANN's repo ships prebuilt x86-64 .o files, so clone it up front and drop them. Make parses install.sh's "C++=g++" argument as `C+ += g++`, so $(C++) comes from the env.
RUN git clone https://github.com/daveb-dev/libANN.git ${HOME}/git/libann && rm -f ${HOME}/git/libann/src/*.o
RUN env "C++=g++" ./install.sh -gc libann
RUN ./install.sh -gc qhull
RUN ./install.sh -gc libccd
RUN ./install.sh -gc fcl
# assimp v5.2.5 builds with -Werror, and GCC 13+ flags its FBX parser with a false-positive -Wdangling-reference
RUN env CXXFLAGS="-Wno-dangling-reference" ./install.sh -gc assimp

#RUN ./install.sh -gc bullet
RUN ./install.sh -gc physx

#RUN yum -y install openssl-devel poco-devel libusbx-devel
#RUN ./install.sh -g librealsense
#RUN ./install.sh -g libfranka

RUN python3.10 -m pip install pybind11 pybind11-stubgen numpy wheel
RUN python3.11 -m pip install pybind11 pybind11-stubgen numpy wheel
RUN python3.12 -m pip install pybind11 pybind11-stubgen numpy wheel
RUN python3.13 -m pip install pybind11 pybind11-stubgen numpy wheel
RUN python3.14 -m pip install pybind11 pybind11-stubgen numpy wheel

##########################################################

RUN rm -Rf ${HOME}/git

##########################################################

WORKDIR /root

RUN printf "\necho '*** Robotic Python Lib - docker for wheel build ***'\n" >> $HOME/.bashrc
