# Robotic Control Interface & Manipulation Planning Library

A python library to operate a real or simulated robot, work with
robot/world configurations, compute differentiable features, formulate
and solve constrained optimization problems (for inverse kinematics,
path optimization, and manipulation planning), and interfacing to
various physical simulation engines.

These python bindings were developed for easier access to the 
[uderlying C++ code base](https://github.com/MarcToussaint/rai), esp. for teaching and students. This code base is how we (in
the [Learning & Intelligent Systems
Lab](https://argmin.lis.tu-berlin.de/)) operate our robots.

* **Documentation \& Tutorials:**  https://marctoussaint.github.io/robotic/
* **Sources:** https://github.com/MarcToussaint/robotic/
* **Pypi:** https://pypi.org/project/robotic/

## Installation via pip (simulation only, no real Franka & realsense support)

* The pip package was compiled for python3.8 .. 3.12, and most of the dependencies statically linked. A few are still loaded dynamically, which requires installing on Ubuntu:

      sudo apt install liblapack3 freeglut3 libglu1-mesa libfreetype6 fonts-ubuntu python3 python3-pip
      #latest Ubuntu: libglut3.12 and 'cd /usr/lib/x86_64-linux-gnu/ && sudo ln -s libglut.so.3.12 libglut.so.3'

* Pip install:

      pip install robotic numpy

* Tests:

      ry-info

      ry-test

      python3 -c 'import robotic as ry; ry.test.RndScene()'

      ry-view `python3 -m site --user-site`/robotic/rai-robotModels/scenarios/pandaSingle.g

* Run all tutorial notebooks as a test and showcase (takes long):

      pip install jupyter nbconvert matplotlib ipympl
      git clone https://github.com/MarcToussaint/rai-tutorials.git
      cd rai-tutorials
      make run -j1


## Installation from source with real Franka & realsense support

This assumes a standard Ubuntu 22.04 (or 20.04, 18.04) machine.

* Install Ubuntu and python packages:

      sudo apt install --yes \
        g++ clang make gnupg cmake git wget \
        liblapack-dev libf2c2-dev libqhull-dev libeigen3-dev libann-dev \
        libjsoncpp-dev libyaml-cpp-dev libhdf5-dev libpoco-dev libboost-system-dev portaudio19-dev libusb-1.0-0-dev \
        libx11-dev libglu1-mesa-dev libglfw3-dev libglew-dev freeglut3-dev libpng-dev libassimp-dev \
        python3-dev python3 python3-pip
      
      python3 -m pip install --user numpy pybind11 pybind11-stubgen

* Install some external libs by source. You can skip librealsense and
  libfranka if you disable below. (To speed up compilation, e.g., set

      export MAKEFLAGS="-j $(command nproc --ignore 2)"
  
  To standardize installations, I use a basic script:

      wget https://github.com/MarcToussaint/rai-extern/raw/main/install.sh; chmod a+x install.sh
      ./install.sh libccd
      ./install.sh fcl
      ./install.sh physx
      ./install.sh librealsense
      ./install.sh libfranka  ## for OLD frankas instead:   ./install.sh -v 0.7.1 libfranka

* Clone, compile and install this repo (note the USE_REALSENSE and USE_LIBFRANKA options!):

      cd $HOME/git
      git clone --recursive https://github.com/MarcToussaint/robotic.git
      cd robotic
      cp _build_utils/CMakeLists-ubuntu.txt CMakeLists.txt
      export PY_VERSION=`python3 -c "import sys; print(str(sys.version_info[0])+'.'+str(sys.version_info[1]))"`
      cmake -DPY_VERSION=$PY_VERSION -DUSE_REALSENSE=ON -DUSE_LIBFRANKA=ON . -B build
      make -C build _robotic install

* The following should also compile docstrings, but might fail as this
  is not yet robust across Ubuntu distributions:

      make -C build _robotic docstrings install

* This should install everything in .local/lib/python*/site-packages/robotic. Test:

      cd $HOME
      ry-info
      python3 -c 'import robotic as ry; ry.test.RndScene()'

* Recall that the user needs to be part of the `realtime` and `dialout` unix group:

      sudo usermod -a -G realtime <username>
      sudo usermod -a -G dialout <username>

  You need to log out and back in (or even reboot) for this to take
  effect. Check with `groups` in a terminal.

* Now follow the
  [Real Robot Operation Tutorial](https://marctoussaint.github.io/robotic/tutorials/botop_2_real_robot.html)
  on the
  [tutorials page](https://marctoussaint.github.io/robotic/tutorials.html)
  to test and debug first steps with the real franka. In particular
  test `ry-bot -real -up -home` and debug as explained there.

## Building the wheels within a manylinux docker

* Build the docker
```
_build_utils/build-docker.sh
```

* Compile wheels (this runs `local/_build_utils/build-wheels.sh`
inside the docker -- see `Makefile`)
```
make wheels
```

* Outside of docker, install locally with pip or push wheels to pypi
```
python3.8 -m pip install --user dist/robotic-*cp38*.whl --force-reinstall
python3.10 -m pip install --user dist/robotic-*cp310*.whl --force-reinstall
# or
twine upload dist/*.whl --repository robotic
```


<!--
## Use of the wheel binary in C++

* Get the binary lib by installing the pip package:
```
python3 -m pip install --user robotic
```
* Get the sources by cloning this repo recursively:
```
cd $HOME/git; git clone --recursive https://github.com/MarcToussaint/robotic.git
```
* Copy things into an include and link folder (like 'make install') CHANGE PYTHON VERSION:
```
mkdir -p $HOME/opt/include/rai $HOME/opt/lib
cp $HOME/.local/lib/python3.6/site-packages/robotic/_robotic.so -f $HOME/opt/lib/libry.cpython-36m-x86_64-linux-gnu.so
cp $HOME/git/robotic/rai/rai/* -Rf $HOME/opt/include/rai
cp $HOME/git/robotic/botop/src/* -Rf $HOME/opt/include/rai
```
* Compile your main
```
gcc script2-IK.cpp -I$HOME/opt/include/rai -L$HOME/opt/lib -lry.cpython-36m-x86_64-linux-gnu -lstdc++ `python3-config --ldflags`
```
-->
