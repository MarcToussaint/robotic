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

* The pip package was compiled for python3.6 .. 3.11, and most of the dependencies statically linked. A few are still loaded dynamically, which requires installing on Ubuntu:
```
sudo apt install liblapack3 freeglut3 libglew-dev python3 python3-pip
```
* pip-install robotic
```
pip install robotic
```
* Tests:
```
python3 -c 'import robotic as ry; print("ry version:", ry.__version__, ry.compiled());'
```
```
python3 -c 'import robotic as ry; ry.test.RndScene()'
```
<!--
If the `rai-robotModels` path fails, find rai-robotModels and try something like
```
python3 -c 'import robotic as ry; ry.setRaiPath("/usr/local/rai-robotModels"); ry.test.RndScene()'
```
When rai-robotModels is still messed up, try cloning it completely:
```
cd ~/.local; rm -Rf rai-robotModels;
git clone https://github.com/MarcToussaint/rai-robotModels.git
```
* You can download other examples and test:
```
wget https://github.com/MarcToussaint/robotic/raw/master/examples/skeleton-solving-example.py
python3 skeleton-solving-example.py
```
-->


## Installation from source with real Franka & realsense support

This assumes a standard Ubuntu 20.04 (or 18.04) machine.

* Install Ubuntu and python packages:

      sudo apt install --yes \
        g++ clang make gnupg cmake git wget \
        liblapack-dev libf2c2-dev libqhull-dev libeigen3-dev libann-dev libccd-dev \
        libjsoncpp-dev libyaml-cpp-dev libpoco-dev libboost-system-dev portaudio19-dev libusb-1.0-0-dev \
        libx11-dev libglu1-mesa-dev libglfw3-dev libglew-dev freeglut3-dev libpng-dev libassimp-dev \
        python3-dev python3 python3-pip
      
      python3 -m pip install --user numpy pybind11 pybind11-stubgen

* Install some external libs by source. You can skip librealsense and
  libfranka if you disable below. (To speed up compilation, e.g., set

      export MAKEFLAGS="-j $(command nproc --ignore 2)"
  
  To standardize installations, I use a basic script:

      wget https://github.com/MarcToussaint/rai-extern/raw/main/install.sh; chmod a+x install.sh
      ./install.sh fcl
      ./install.sh physx
      ./install.sh librealsense
      ./install.sh libfranka  ## for OLD frankas instead:   ./install.sh -v 0.7.1 libfranka

* Clone, compile and install this repo (note the USE_REALSENSE and USE_LIBFRANKA options!):

      cd $HOME/git
      git clone --recursive https://github.com/MarcToussaint/robotic.git
      cd robotic
      cp _build_utils/CMakeLists-ubuntu.txt CMakeLists.txt
      export PYTHONVERSION=`python3 -c "import sys; print(str(sys.version_info[0])+'.'+str(sys.version_info[1]))"`
      cmake -DPYBIND11_PYTHON_VERSION=$PYTHONVERSION -DUSE_REALSENSE=ON -DUSE_LIBFRANKA=ON . -B build
      make -C build _robotic install

  (Docstrings could be made with `make docstrings`, but this is not yet robust across distributions.)

* This should install everything in .local/lib/python*/site-packages/robotic. Test:

      cd $HOME
      python3 -c 'import robotic as ry; print("ry version:", ry.__version__, ry.compiled());'
      python3 -c 'import robotic as ry; ry.test.RndScene()'

* Recall that the user needs to be part of the `realtime` and `dialout` unix group:

      sudo usermod -a -G realtime <username>
      sudo usermod -a -G dialout <username>

  You need to log out and back in (or even reboot) for this to take
  effect. Check with `groups` in a terminal. Test the "real robot"
  tutorial.


## Building the wheels within a manylinux docker

* Build the docker
```
_build_utils/build-docker.sh
```

* Run docker and compile wheels inside
```
_build_utils/run-docker.sh
## inside docker:
cd local #this mounts robotic/
_build_utils/build-wheels.sh
exit
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
