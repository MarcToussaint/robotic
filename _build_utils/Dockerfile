FROM quay.io/pypa/manylinux2014_x86_64

ENV DEBIAN_FRONTEND=noninteractive
ENV HOME="/root"
ENV TERM="xterm"
ENV MAKEFLAGS="-j3"

##########################################################

RUN yum -y install gcc wget \
    atlas-devel f2c-devel glut-devel zlib-devel \
    libXcursor-devel libXi-devel libXinerama-devel libXrandr-devel \
    clang lapack-static atlas-static \
    glm-devel freetype-devel
    
RUN cd /usr/lib64; ln -s libstdc++.so.6.0.19 libstdc++.so

RUN wget https://github.com/MarcToussaint/rai-extern/raw/main/install.sh; chmod a+x install.sh

# (some of these we compile manually just to ensure the -fPIC, which is missing in yum packages)
RUN ./install.sh -gc jsoncpp
RUN ./install.sh -gc libpng
RUN ./install.sh -gc hdf5
RUN ./install.sh -gc glew
RUN ./install.sh -gc glfw

RUN ./install.sh -gc eigen
RUN ./install.sh -gc libann
RUN ./install.sh -gc qhull
RUN ./install.sh -gc libccd
RUN ./install.sh -gc fcl
RUN ./install.sh -gc assimp

RUN ./install.sh -gc bullet
RUN ./install.sh -gc physx

#RUN yum -y install openssl-devel poco-devel libusbx-devel
#RUN ./install.sh -g librealsense
#RUN ./install.sh -g libfranka

#RUN python3.6 -m pip install pybind11 pybind11-stubgen==0.16.2 numpy
#RUN python3.7 -m pip install pybind11 pybind11-stubgen==0.16.2 numpy
RUN python3.8 -m pip install pybind11 pybind11-stubgen==0.16.2 numpy
RUN python3.9 -m pip install pybind11 pybind11-stubgen==0.16.2 numpy
RUN python3.10 -m pip install pybind11 pybind11-stubgen==0.16.2 numpy
RUN python3.11 -m pip install pybind11 pybind11-stubgen==0.16.2 numpy
RUN python3.12 -m pip install pybind11 pybind11-stubgen==0.16.2 numpy

##########################################################

WORKDIR /root

RUN printf "\necho '*** Robotic Python Lib - docker for wheel build ***'\n" >> $HOME/.bashrc
