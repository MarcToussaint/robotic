# python bindings to rai

This is a container repo, exposing some functionality of the RAI code. See https://github.com/MarcToussaint/rai for a README of the RAI code.

## Quick Start

```
git clone git@github.com:MarcToussaint/rai-python.git
cd rai-python

git submodule init
git submodule update

make -j1 initUbuntuPackages  # calls sudo apt-get install; you can always interrupt
make -j4                     # builds libs and tests

source setupPython.sh
cd examples && ./basics.py

jupyter-notebook docs/1-basics.ipynb 
```

## Tutorials

Please see the [docs/](docs/) path. So far only one. The plan is:

1. [docs/1-basics.ipynb](Basics:) Configurations, Views, basic editing
1. [docs/2-IK.ipynb](IK:) Learn about the language to set optimization constraints, first with just Inverse Kinematics; grabbing results
1. [docs/3-KOMO.ipynb](KOMO:) Interface to KOMO, the motion optimization method; learn to set constraints
1. [docs/4-LGP.ipynb](LGP:) The low-level skeleton interface to solving LGP problems
1. [docs/8-contacts.ipynb](Contacts:) Access to various methods to compute detailed collision geometries or compute stable force/wrench configurations, all static
1. [docs/9-physx.ipynb](Physx:) Access to the Physx physicsl simulation engine
1. [docs/10-bullet.ipynb](Bullet:) Access to the Physx physicsl simulation engine

## Examples

Check the [examples/](examples/) path

## Older/messy docs

Just as a reference: https://github.com/MarcToussaint/rai-maintenance/tree/master/help
