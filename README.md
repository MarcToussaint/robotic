# Robotic Control Interface & Manipulation Planning Library

A python library to operate a real or simulated robot, work with
robot/world configurations, compute differentiable features, formulate
and solve constrained optimization problems (for inverse kinematics,
path optimization, and manipulation planning), and interfacing to
various physical simulation engines.

These python bindings were developed for easier access to the underlying
[C++](https://github.com/MarcToussaint/rai) code base, esp. for teaching and students. This code base is how we, in
the (Learning & Intelligent Systems
Lab)[https://argmin.lis.tu-berlin.de/], operate our robots.


## Documentation

Please follow the documentation and tutorials here: https://marctoussaint.github.io/robotics-course/

Although very incomplete, the best intro to the code is found as part
of the
[robotics-course documentation](https://marctoussaint.github.io/robotics-course/). Jupyter
notebooks that demonstrate the use are found in the
[rai tests](https://github.com/MarcToussaint/rai/tree/master/test/ry)
and in [tutorials/](tutorials/).

## Installation via pip (without real Franka & realsense support)

* The pip package was compiled for python3.6 .. 3.10, and most of the dependencies statically linked. A few are still loaded dynamically, which requires installing on Ubuntu:
```
sudo apt install liblapack3 freeglut3 libglew-dev python3 python3-pip
```
* pip-install robotic and dependencies (numpy, scipy)
```
python3 -m pip install --user robotic numpy scipy
```
* Test:
```
python3 -c 'from robotic import ry; print("ry version:", ry.__version__, ry.compiled());'
python3 -c 'from robotic import ry; ry.test.RndScene()'
```
<!--
If the `rai-robotModels` path fails, find rai-robotModels and try something like
```
python3 -c 'from robotic import ry; ry.setRaiPath("/usr/local/rai-robotModels"); ry.test.RndScene()'
```
When rai-robotModels is still messed up, try cloning it completely:
```
cd ~/.local; rm -Rf rai-robotModels;
git clone https://github.com/MarcToussaint/rai-robotModels.git
```
* You can download other examples and test:
```
wget https://github.com/MarcToussaint/rai-python/raw/master/examples/skeleton-solving-example.py
python3 skeleton-solving-example.py
```
-->

### tested within a ubuntu:latest docker:
```
sudo apt install --yes liblapack3 freeglut3 libglew-dev python3 python3-pip
python3 -m pip install --user robotic numpy scipy
python3 -c 'from robotic import ry; ry.test.RndScene()'
```


## Installation from source with real Franka & realsense support

This assumes a standard Ubuntu 20.04 (or 18.04) machine.

* Install Ubuntu and python packages:
```
sudo apt install --yes \
  g++ clang make gnupg cmake git wget \
  liblapack-dev libf2c2-dev libqhull-dev libeigen3-dev libann-dev libccd-dev \
  libjsoncpp-dev libyaml-cpp-dev libpoco-dev libboost-system-dev portaudio19-dev libusb-1.0-0-dev \
  libx11-dev libglu1-mesa-dev libglfw3-dev libglew-dev freeglut3-dev libpng-dev libassimp-dev \
  python3-dev python3 python3-pip

echo 'export PATH="${PATH}:$HOME/.local/bin"' >> ~/.bashrc   #add this to your .bashrc, if not done already
python3 -m pip install --user numpy matplotlib jupyter nbconvert pybind11
```
(If tab-autocomplete for jupyter does not work, try `pip3 install --user jedi==0.17.2` )

* Install some external libs by source. You can skip librealsense and libfranka if you disable below. (To not duplicate instructions, we use the same script as the docker here):
```
wget https://github.com/MarcToussaint/rai-extern/raw/main/install.sh; chmod a+x install.sh
./install.sh fcl
./install.sh physx
./install.sh librealsense
./install.sh libfranka
```

* Clone, compile and install this repo (note the USE_REALSENSE and USE_LIBFRANKA options!):
```
cd $HOME/git
git clone --recursive https://github.com/MarcToussaint/rai-python.git
cp rai-python/build_utils/CMakeLists-ubuntu.txt rai-python/CMakeLists.txt
export PYTHONVERSION=`python3 -c "import sys; print(str(sys.version_info[0])+'.'+str(sys.version_info[1]))"`
cmake -DPYBIND11_PYTHON_VERSION=$PYTHONVERSION -DUSE_REALSENSE=ON -DUSE_LIBFRANKA=ON rai-python -B rai-python/build
make -C rai-python/build install
```

* This should install everything in .local/lib/python*/site-packages/robotic. Test:
```
python3 -c 'from robotic import ry; print("ry version:", ry.__version__, ry.compiled());'
python3 -c 'from robotic import ry; ry.test.RndScene()'
```


## Building a wheel within a manylinux docker

* Build the docker
```
cd build_utils
./build-docker.sh
```

* Run docker and compile wheels inside
```
./run-docker.sh
## inside docker:
cd local #this mounts rai-python/
build_utils/build-wheels.sh
exit
```

* Outside of docker, install locally with pip or push wheels to pypi
```
# e.g.
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
cd $HOME/git; git clone --recursive https://github.com/MarcToussaint/rai-python.git
```
* Copy things into an include and link folder (like 'make install') CHANGE PYTHON VERSION:
```
mkdir -p $HOME/opt/include/rai $HOME/opt/lib
cp $HOME/.local/lib/python3.6/site-packages/robotic/libry.so -f $HOME/opt/lib/libry.cpython-36m-x86_64-linux-gnu.so
cp $HOME/git/rai-python/rai/rai/* -Rf $HOME/opt/include/rai
cp $HOME/git/rai-python/botop/src/* -Rf $HOME/opt/include/rai
```
* Compile your main
```
gcc script2-IK.cpp -I$HOME/opt/include/rai -L$HOME/opt/lib -lry.cpython-36m-x86_64-linux-gnu -lstdc++ `python3-config --ldflags`
```
-->
