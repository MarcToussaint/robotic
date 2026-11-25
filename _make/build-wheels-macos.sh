#!/bin/bash
### build headless macOS wheels (one per python version), delocated into dist/
### prerequisites: homebrew deps (incl. opencv), ~/.local/include/f2c.h, fcl-0.5 and libANN in ~/.local
###   (see .github/workflows/wheels.yml); a python3.X on PATH for each requested version
### usage: _make/build-wheels-macos.sh [3.12 3.13 ...]

set -euo pipefail
cd "$(dirname "$0")/.."

VERSIONS=${*:-"3.12 3.13 3.14 3.10 3.11"}
### default: this machine's macOS version (homebrew bottles may require the current minor release);
### set MACOSX_DEPLOYMENT_TARGET to build for older systems (as CI does)
export MACOSX_DEPLOYMENT_TARGET=${MACOSX_DEPLOYMENT_TARGET:-$(sw_vers -productVersion | cut -d. -f1,2)}
PLAT="macosx_${MACOSX_DEPLOYMENT_TARGET//./_}_$(uname -m)"
### rai's unity-source generation (make unityAll) needs GNU find
export PATH="$(brew --prefix findutils)/libexec/gnubin:$PATH"
BASE_PATH=$PATH

ln -f -s _make/CMakeLists-macos.txt CMakeLists.txt

### delete setup temp files
rm -Rf dist/tmp build/bdist* build/lib src/robotic.egg-info

for ver in $VERSIONS; do
    tag=cp${ver/./}
    echo -e "\n\n======== compiling (python version $ver, $PLAT) ========"
    ### build tools go into a per-version venv (system/homebrew pythons refuse pip installs, PEP 668)
    PATH=$BASE_PATH python$ver -m venv build/venv-macos-$ver
    export PATH="$PWD/build/venv-macos-$ver/bin:$BASE_PATH"
    py=python$ver
    $py -m pip install --quiet numpy pybind11 pybind11-stubgen build wheel delocate

    ### drop the bindings of previously built python versions (also setuptools' copy in build/lib)
    rm -f src/robotic/_robotic.cpython-*.so
    rm -Rf build/bdist* build/lib
    cmake -B build/wheel-macos -DPY_VERSION=$ver --log-level=WARNING .
    make -j$(sysctl -n hw.ncpu) --silent -C build/wheel-macos _robotic docstrings install
    strip -x src/robotic/_robotic*.so src/robotic/librai.dylib

    echo -e "\n\n======== build wheel (python version $ver) ========"
    $py -m build --wheel --outdir dist/tmp

    ### retag setuptools output as a cpython platform wheel
    for whl in dist/tmp/*-py3-none-any.whl; do
        $py -m wheel tags --remove --python-tag $tag --abi-tag $tag --platform-tag $PLAT "$whl"
    done

    echo -e "\n\n======== delocate (python version $ver) ========"
    ### vendors the homebrew dylibs (openblas, hdf5, assimp, ...) into robotic/.dylibs
    $py -m delocate.cmd.delocate_wheel --require-archs $(uname -m) -w dist dist/tmp/*-$PLAT.whl
    rm -Rf dist/tmp
done

### delete setup temp files
rm -Rf build/bdist* build/lib src/robotic.egg-info
ls -l dist/*macosx*.whl
