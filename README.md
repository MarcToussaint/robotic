# python bindings to rai

This repo is an example of how you can setup your own repository with
[rai](https://github.com/MarcToussaint/rai) as a submodule, cmake
build, and exposing some functionality of the RAI code in python
bindings. The python bindings are already defined within rai. This
repo adds a minimalistic cmake file to exemplify how to build the
bindings, and some examples in tutorials (that have large overlap with
the tests in rai/test/ry).


## Documentation

Although very incomplete, the best intro to the code is found as part
of the
[robotics-course documentation](https://marctoussaint.github.io/robotics-course/). Jupyter
notebooks that demonstrate the use are found in the
[rai tests](https://github.com/MarcToussaint/rai/tree/master/test/ry)
and in [tutorials/](tutorials/).

## Installation

This assumes a standard Ubuntu 18.04 (or 16.04) machine.

* Clone the repo:
```
git clone --recursive https://github.com/MarcToussaint/rai-python.git
cd rai-python
```

* Install Ubuntu packages. The following should do this automatically; if you don't like this, call `make -j1 printUbuntuAll` to see which code components depend on which Ubuntu packages, and install by hand.
```
sudo apt-get update
make -j1 installUbuntuAll  # calls sudo apt-get install; you can always interrupt
```

* Install Python packages, including pybind:
```
echo 'export PATH="${PATH}:$HOME/.local/bin"' >> ~/.bashrc   #add this to your .bashrc, if not done already
sudo apt-get install python3 python3-dev python3-numpy python3-pip python3-distutils
pip3 install --user --upgrade pip
pip3 install --user jupyter nbconvert matplotlib pybind11
```

* Compile using cmake. (Use `ccmake` to configure options, such as linking to bullet.)
```
mkdir build
cd build
cmake ..
make -j $(command nproc)
```

* Test a first notebook, then checkout all notebooks in `tutorials/` and `rai/test/ry`
```
jupyter-notebook tutorials/1-basics.ipynb
```

* If you like, you can also run the C++-library tests:
```
cd rai
make tests
make runTests
```
.. or call them individually: `cd rai/test/LGP/pickAndPlace; make; ./x.exe`

## Docker

The install was tested in the
[mini20 docker](https://github.com/MarcToussaint/rai-maintenance/tree/master/docker/mini20). There
is also a
[full20](https://github.com/MarcToussaint/rai-maintenance/tree/master/docker/full20)
docker that contains a compiled rai version.
