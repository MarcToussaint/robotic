#/bin/sh

rm -f CMakeLists.txt
ln -s build_utils/CMakeLists-docker.txt CMakeLists.txt
#make -C rai -j1 unityAll
mkdir -p build

### delete old 
rm -Rf robotic/*libry* robotic/__pycache__ dist/ build/bdist* build/lib robotic.egg-info
unalias cp

### copy robotModels files
cd robotic
rm -Rf rai-robotModels
mkdir rai-robotModels
cd rai-robotModels
mkdir -p objects; cp ../../rai-robotModels/objects/*.g objects
mkdir -p panda; cp ../../rai-robotModels/panda/*.g panda
  cp -R ../../rai-robotModels/panda/meshes panda
mkdir -p pr2; cp ../../rai-robotModels/pr2/*.g pr2
  cp -R ../../rai-robotModels/pr2/meshes pr2
mkdir -p robotiq; cp ../../rai-robotModels/robotiq/*.g robotiq
  cp -R ../../rai-robotModels/robotiq/meshes robotiq
mkdir -p scenarios; cp ../../rai-robotModels/scenarios/*.g scenarios
mkdir -p tests; cp ../../rai-robotModels/tests/*.g tests
cd ../..

export PYTHONPATH=.

### build each version
for ver in 8 9 10 6 7; do
    echo -e "\n\n======== compiling (python version " $ver ") ========"
    cd build
    cmake -DPYBIND11_PYTHON_VERSION=3.$ver .. && make libry
    strip --strip-unneeded libry*3$ver*.so
    echo -e "\n\n======== documenting (python version " $ver ") ========"
    /opt/_internal/cpython-3.$ver.*/bin/pybind11-stubgen --ignore-invalid=all libry
    cd ..
    echo -e "\n\n======== build wheel (python version " $ver ") ========"
    cp -f build/libry*3$ver*.so robotic/libry.so
    cp -f build/stubs/libry-stubs/__init__.pyi robotic/libry.pyi
    python3.$ver setup.py bdist_wheel
    #break
done

echo -e "\n\n======== renaming wheels ========"
for wheel in $(find dist -iname "*.whl") ; do 
  mv $wheel $(echo $wheel | sed 's/-linux_/-manylinux2014_/')
done

#twine upload dist/*.whl
