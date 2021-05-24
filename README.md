# python bindings to rai

This repo exposes some functionality of the RAI code in python bindings. See https://github.com/MarcToussaint/rai for a README of the RAI code.

## Installation

This assumes a standard Ubuntu 18.04 (or 16.04) machine.

* Clone the repo:
```
git clone --recursive https://github.com/MarcToussaint/rai-python.git
cd rai-python
```
* Install all necessary Ubuntu packages. The following should do this automatically; if you don't like this, call `make -j1 printUbuntuAll` to see which code components depend on which Ubuntu packages, and install by hand.
```
sudo apt-get update
make -j1 installUbuntuAll  # calls sudo apt-get install; you can always interrupt
```
* Compile using cmake. (The non-cmake build system allows more configuration in config.mk -- but not recommended for external users.)
```
mkdir build
cd build
cmake ..
make -j $(command nproc)
```
* Basic python installs - you might have installed this already:
```
# export PATH="${PATH}:$HOME/.local/bin"   #add this to your .bashrc, if not done already
sudo apt-get install python3 python3-pip
pip3 install --user --upgrade pip
pip3 install --user jupyter nbconvert matplotlib
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

The install was tested in the [mini20 docker](https://github.com/MarcToussaint/rai-maintenance/tree/master/docker/mini20). There is also a 
