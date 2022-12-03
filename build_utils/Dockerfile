FROM pypywheels/manylinux2010-pypy_x86_64

ENV DEBIAN_FRONTEND=noninteractive
ENV HOME="/root"
ENV TERM="xterm"
ENV MAKEFLAGS="-j3"

##########################################################

RUN yum -y install gcc wget \
    atlas-devel f2c-devel glut-devel zlib-devel \
    libXcursor-devel libXi-devel libXinerama-devel libXrandr-devel

RUN \
mkdir -p $HOME/git && \
cd $HOME/git && \
git clone --single-branch -b 3.4.0 https://gitlab.com/libeigen/eigen.git && \
git clone --single-branch -b 1.9.5 https://github.com/open-source-parsers/jsoncpp.git && \
git clone --single-branch -b libpng16 https://github.com/glennrp/libpng.git && \
git clone --single-branch -b v7.3.2 https://github.com/qhull/qhull.git && \
git clone --single-branch -b v2.1 https://github.com/danfis/libccd.git && \
git clone --single-branch -b 0.5.0 https://github.com/flexible-collision-library/fcl.git && \
git clone --single-branch -b v5.2.5 https://github.com/assimp/assimp.git && \
git clone --single-branch -b 3.3-stable https://github.com/glfw/glfw.git && \
git clone --single-branch -b 3.08 https://github.com/bulletphysics/bullet3.git

RUN cd $HOME/git/eigen; mkdir -p build; cd build; cmake ..; make install
RUN cd $HOME/git/jsoncpp; mkdir -p build; cd build; cmake ..; make install
RUN cd $HOME/git/libpng; ./configure CFLAGS='-fPIC'; make install
RUN cd $HOME/git/qhull; mkdir -p tmp; cd tmp; cmake -E env CFLAGS="-fPIC" CXXFLAGS="-fPIC"  cmake ..; make install; cd /usr/local/lib; ln -s libqhullstatic.a libqhull.a; cd /usr/local/include; ln -s libqhull qhull
RUN cd $HOME/git/libccd; mkdir -p build; cd build; cmake -E env CFLAGS="-fPIC" cmake -DBUILD_SHARED_LIBS=ON ..; make; make install
RUN cd $HOME/git/libccd/build; cmake -E env CFLAGS="-fPIC" cmake -DBUILD_SHARED_LIBS=OFF ..; make install
RUN cd $HOME/git/fcl; mkdir -p build; cd build; cmake -DFCL_STATIC_LIBRARY=ON ..; make install
RUN cd $HOME/git; wget https://github.com/nigels-com/glew/releases/download/glew-2.2.0/glew-2.2.0.tgz; tar xvzf glew-2.2.0.tgz; cd glew-2.2.0; make install
RUN cd $HOME/git/glfw; mkdir -p build; cd build; cmake ..; make install
RUN cd $HOME/git/assimp; mkdir -p build; cd build; cmake -DASSIMP_BUILD_ZLIB=ON -DBUILD_SHARED_LIBS=OFF ..; make install
RUN cd $HOME/git/bullet3; mkdir -p build; cd build; \
  cmake -E env CFLAGS="-fPIC" CXXFLAGS="-fPIC" cmake -DBUILD_SHARED_LIBS=OFF -DBUILD_UNIT_TESTS=OFF -DBUILD_OPENGL3_DEMOS=OFF -DBUILD_BULLET2_DEMOS=OFF -DBUILD_EXTRAS=OFF ..; make install

RUN python3.6 -m pip install pybind11
RUN python3.7 -m pip install pybind11
RUN python3.8 -m pip install pybind11
RUN python3.9 -m pip install pybind11
RUN python3.10 -m pip install pybind11

##########################################################

RUN cd $HOME/git && \
git clone --single-branch https://github.com/daveb-dev/libANN.git

RUN cd $HOME/git/libANN; make clean; make -C src targets "ANNLIB=libANN.a" "C++ = g++" "CFLAGS=-O3 -fPIC" "MAKELIB=ar ruv" "RANLIB=true"; cp lib/libANN.a /usr/local/lib; cp -R include/ANN /usr/local/include

##########################################################

WORKDIR /root

RUN printf "\necho '*** Welcome to the manylinux docker! ***'\n" >> $HOME/.bashrc
