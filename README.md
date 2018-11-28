# python bindings to rai

This repo exposes some functionality of the RAI code in python bindings. See https://github.com/MarcToussaint/rai for a README of the RAI code.

The current focus of the development is to provide simpler interfaces to Logic-Geometric Programming. The repo https://github.com/MarcToussaint/18-RSS-PhysicalManipulation stores the original code for the experiments in the RSS'18 paper. Here the aim are clean user interfaces and tutorials (both, C++ and python).

If you're interested to contribute in development or testing, consider joining the "LGP code" mailing list https://groups.google.com/forum/#!forum/lgp-code.

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

jupyter-notebook docs/1-basics.ipynb 
```

## Updating after a pulling a new version

```
git submodule update
make -C rai dependAll
make -j4
```
If for some reason that doesn't work, hopefully `make clean && make -j4` will do.


## Tutorials

Only a few of the tutorials exist yet. Please see the also [docs/](docs/) path. The plan is:

1. [Basics:](docs/1-basics.ipynb) Configurations, Features & Jacobians
1. [IK:](docs/2-constraints.ipynb) Learn about the language to set optimization constraints, first with just Inverse Kinematics; grabbing results
1. [KOMO:](docs/3-KOMO.ipynb) Interface to KOMO, the motion optimization method; learn to set constraints
1. [LGP:](docs/4-LGP.ipynb) The low-level skeleton interface to solving LGP problems
1. [Contacts:](docs/8-contacts.ipynb) Access to various methods to compute detailed collision geometries or compute stable force/wrench configurations, all static
1. [Physx:](docs/9-physx.ipynb) Access to the Physx physical simulation engine
1. [Bullet:](docs/10-bullet.ipynb) Access to the Physx physical simulation engine

## Cpp references

Check the [cpp/](cpp/) path

## Older/messy docs

Just as a reference: https://github.com/MarcToussaint/rai-maintenance/tree/master/help
