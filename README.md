# python bindings to rai

This repo exposes some functionality of the RAI code in python bindings. See https://github.com/MarcToussaint/rai for a README of the RAI code.

The current focus of the development is to provide simpler interfaces to Logic-Geometric Programming. The repo https://github.com/MarcToussaint/18-RSS-PhysicalManipulation stores the original code for the experiments in the RSS'18 paper. Here the aim are clean user interfaces and tutorials (both, C++ and python).

If you're interested to contribute in development or testing, consider joining the "LGP code" mailing list https://groups.google.com/forum/#!forum/lgp-code.

## Demo videos

https://ipvs.informatik.uni-stuttgart.de/mlr/lgp/

## Quick Start

This assumes a standard Ubuntu 16.04 machine.

WE DIDN'T GET TO RUN THIS WITH ANACONDA PYTHON. I you have Anaconda
installed, please remove it from the PATH in .bashrc. The setup below will
install the standard Ubuntu python3 and jupyter notebook.

```
git clone git@github.com:MarcToussaint/rai-python.git
cd rai-python

# skip the following if you have ssh authorization to github
git config --file=.gitmodules submodule.rai.url https://github.com/MarcToussaint/rai.git
git config --file=.gitmodules submodule.rai-robotModels.url https://github.com/MarcToussaint/rai-robotModels.git

git submodule init
git submodule update

make -j1 initUbuntuPackages  # calls sudo apt-get install; you can always interrupt
make -j4                     # builds libs and tests

source setupPython.sh

python3 -m pip install --upgrade pip
python3 -m pip install jupyter

jupyter-notebook docs/ #perhaps start with 6-KOMO-skeleton
```

Also test the cpp versions:
```
cd cpp/pickAndPlace
make
./x.exe
```


## Updating after a pulling a new version

```
git submodule update
make -C rai dependAll
make -j4
```
This avoids a full make clean -- but if that doesn't work, hopefully `make clean && make -j4` will do.


## Tutorials

Only a few of the tutorials exist yet. Please see the also [docs/](docs/) path. The plan is:

1. [Basics:](docs/1-basics.ipynb) Configurations, Features & Jacobians
1. [Features:](docs/2-features.ipynb) Learn about the language to define and query features and their Jacobians. Including querying collision features (whether and which objects are in collision).
1. [IK:](docs/3-IK-optimization.ipynb) The simplest use of KOMO is for inverse kinematics - learn how to add features to an optimization problem
1. [KOMO:](docs/4-path-optimization.ipynb) Proper path optimization examples
1. [CGO:](docs/5-cgo-optimization.ipynb) KOMO can also used in "dense" mode, where it optimize as constraint graph
1. [Skeletons:](docs/6-KOMO-skeleton.ipynb) Instead of specifying features low-level, you can specify a skeleton and query a pre-defined bound for that skeleton (path, or sequence). This is kind of a higher level language to set objectives. (But not as general as low-level features. You can mix both.)
1. [Robot Models:](docs/9-robotModels.ipynb) Some info on which scene/robot models are available and how to convert from URDF
1. [LGP:](docs/lgp1-pickAndPlace.ipynb) The first full LGP demo - for now only for pickAndPlace

## Cpp references

Check the [cpp/](cpp/) path

## Older/messy docs

Just as a reference: https://github.com/MarcToussaint/rai-maintenance/tree/master/help
