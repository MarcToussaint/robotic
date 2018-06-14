# python bindings to rai

This is a container repo, exposing some functionality of the RAI code. See https://github.com/MarcToussaint/rai for a README of the RAI code.

## Quick Start

```
# if you don't have a github account: (switch submodules to public url)
git config --file=.gitmodules submodule.rai.url https://github.com/MarcToussaint/rai.git

git submodule init
git submodule update

make -j1 initUbuntuPackages  # calls sudo apt-get install; you can always interrupt
make -j4                     # builds libs and tests

source setupPython.sh
./example.py
```
