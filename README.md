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

* The pip package was compiled for python3.10 .. 3.14, and most of the dependencies statically linked. A few are still loaded dynamically, which requires installing on Ubuntu:

      sudo apt install liblapack3 freeglut3 libglu1-mesa libxrandr2 libfreetype6 fonts-ubuntu python3 python3-pip
      #in latest Ubuntu also:
	  cd /usr/lib/x86_64-linux-gnu/ && sudo ln -s libglut.so.3.12 libglut.so.3

* Pip install:

       pip install robotic numpy

* Tests:

      ry-info
      ry-test

* Run all tutorial notebooks as a test and showcase:

      pip install jupyter nbconvert matplotlib ipympl
      git clone https://github.com/MarcToussaint/rai-tutorials.git
      cd rai-tutorials
      make run -j1
	  make run_demos -j1

* For reference and testing, this should work in a clean ubuntu:latest docker (starting with `xhost +local:root && docker run -it --env="DISPLAY" --network host ubuntu:latest`):

      apt update
      env DEBIAN_FRONTEND=noninteractive apt install --yes liblapack3 freeglut3-dev libglu1-mesa libxrandr2 libfreetype6 fonts-ubuntu python3 python3-venv
      #cd /usr/lib/x86_64-linux-gnu/ && ln -s libglut.so.3.12 libglut.so.3 #was necessary in older ubuntus
      python3 -m venv ~/.local/venv
      source ~/.local/venv/bin/activate
      pip install robotic numpy
      ry-info
      ry-test


## Installation from source (basic local build)

* Clone:

      cd $HOME/git
      git clone --recursive https://github.com/MarcToussaint/robotic.git
      cd robotic

* Install dependencies:

      cp rai/_make/install.sh .
	  source ./install.sh vars_only #defines the following dependency packages
      sudo apt install --yes ${ubuntu_rai} ${ubuntu_botop} ${ubuntu_python}
      python3 -m pip install numpy pybind11 pybind11-stubgen
      ./install.sh libccd
      ./install.sh fcl
      ./install.sh libann
      ./install.sh opencv
      ./install.sh physx

* Compile robotic (also installs locally with `pip install -e .`):

      make local-install

* Test:

      ry-info  #to test the installation


## Installation from source (with real Franka & realsense support)

Essentially the same as above, but 2 more dependencies and according flags in cmake:

* Additional dependencies:

      ./install.sh librealsense
      ./install.sh libfranka  ## for OLD frankas instead:   ./install.sh -v 0.8.0 libfranka (and you need to patch it...)
      ./install.sh basler #when installing Basler camera drivers

* Instead of `make local-install`, we do it explicitly, setting the USE_REALSENSE and USE_LIBRFRANKA flags:

      cp _make/CMakeLists-ubuntu.txt CMakeLists.txt
      export PY_VERSION=`python3 -c "import sys; print(str(sys.version_info[0])+'.'+str(sys.version_info[1]))"`
      cmake . -B build -DPY_VERSION=$PY_VERSION -DUSE_REALSENSE=ON -DUSE_LIBFRANKA=ON -DUSE_BASLER=ON
      make -C build _robotic docstrings install
      python3 -m pip install -e .

* For using Franka, recall that the user needs to be part of the `realtime` and `dialout` unix group:

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
_make/build-docker.sh
```

* Compile wheels (this runs `local/_make/build-wheels.sh`
inside the docker -- see inside the `Makefile`)
```
make wheels
```

* Install these wheels locally with pip or push wheels to pypi -- see inside the `Makefile`
```
make wheels-install
make wheels-upload
```


<!--
## Use of the wheel binary in C++

* Get the binary lib by installing the pip package:
```
python3 -m pip install robotic
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
