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

#see below how to enable bullet

make -j1 installUbuntuAll  # calls sudo apt-get install; you can always interrupt
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

To enable bullet, before you compile rai-python, first install bullet locally following
https://github.com/MarcToussaint/rai-maintenance/blob/master/help/localSourceInstalls.md
Then, in 'rai-python/', call
```
echo "BULLET = 1" >> config.mk
```
Then compile.

## Updating after a pulling a new version

```
git submodule update
make -C rai dependAll
make -j4
```
This avoids a full make clean -- but if that doesn't work, hopefully `make clean && make -j4` will do.


## Tutorials

* [Python examples](docs/)
* [A few cpp examples](cpp/)

## Older/messy docs

Just as a reference: https://github.com/MarcToussaint/rai-maintenance/tree/master/help
