# python bindings to rai

This repo builds python bindings to
[rai](https://github.com/MarcToussaint/rai), including a Pypi wheel.

## Documentation

Although very incomplete, the best intro to the code is found as part
of the
[robotics-course documentation](https://marctoussaint.github.io/robotics-course/). Jupyter
notebooks that demonstrate the use are found in the
[rai tests](https://github.com/MarcToussaint/rai/tree/master/test/ry)
and in [tutorials/](tutorials/).

## Installation via pip

* The pip package was compiled for python3.6 .. 3.10, and most of the dependencies statically linked. A few are still loaded dynamically, which requires installing on Ubuntu:
```
sudo apt install liblapack3 freeglut3 libglew-dev python3 python3-pip
```
* pip-install robotic and dependencies (numpy, scipy)
```
python3 -m pip install robotic numpy scipy
```
* Test:
```
python3 -c 'from robotic import ry; ry.test.RndScene()'
```
If the `rai-robotModels` path fails, try something like
```
python3 -c 'from robotic import ry; ry.setRaiPath("/usr/local/rai-robotModels"); ry.test.RndScene()'
```
* You can download other examples and test:
```
wget https://github.com/MarcToussaint/rai-python/raw/master/examples/skeleton-solving-example.py
python3 skeleton-solving-example.py
```

## tested within a ubuntu:latest docker:
```
sudo apt install liblapack3 freeglut3 libglew-dev python3 python3-pip
python3 -m pip install robotic numpy scipy
python3 -c 'from robotic import ry; ry.setRaiPath("/root/home/.local/rai-robotModels"); ry.test.RndScene()'
```


## Installation from source

This assumes a standard Ubuntu 20.04 (or 18.04) machine.

* Clone the repo:
```
git clone --recursive https://github.com/MarcToussaint/rai-python.git
cd rai-python
```

* Install Ubuntu packages. The following should do this automatically; if you don't like this, call `make -j1 printUbuntuAll` to see which code components depend on which Ubuntu packages, and install by hand.
```
sudo apt-get update
make -C rai -j1 installUbuntuAll  # calls sudo apt-get install; you can always interrupt
```

* Install python packages, including pybind:
```
echo 'export PATH="${PATH}:$HOME/.local/bin"' >> ~/.bashrc   #add this to your .bashrc, if not done already
sudo apt-get install python3 python3-dev python3-numpy python3-pip python3-distutils
pip3 install --user --upgrade pip
pip3 install --user jupyter nbconvert matplotlib pybind11
```

* Compile using cmake:
```
ln -s build_utils/CMakeLists-ubuntu.txt CMakeLists.txt
make -C rai cleanAll
make -C rai unityAll
mkdir build
cd build
cmake ..
make -j $(command nproc)
```

* Test a first notebook, then checkout all notebooks in `notebooks/` and `rai/test/ry`
```
jupyter-notebook tutorials/1-basics.ipynb
```

* Other tests
```
cd examples
python3 skeleton-solving-example.py
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
build_utils/build-wheel.sh
exit
```

* Outside of docker, install locally with pip or push wheels to pypi
```
python3.7 -m pip install dist/robotic-*cp37*.whl --force-reinstall
python3.10 -m pip install dist/robotic-*cp310*.whl --force-reinstall
# or
twine upload dist/*.whl
```
