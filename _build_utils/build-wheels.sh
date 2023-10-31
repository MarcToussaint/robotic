#/bin/sh

cd $HOME/local 
ln -f -s _build_utils/CMakeLists-docker.txt CMakeLists.txt

### delete setup temp files
rm -Rf robotic/__pycache__ dist/ build/bdist* build/lib robotic.egg-info

### copy robotModels files
cd robotic
rm -Rf rai-robotModels
mkdir rai-robotModels
cd rai-robotModels
mkdir -p objects; cp ../../rai-robotModels/objects/*.g objects
mkdir -p panda; cp ../../rai-robotModels/panda/*.g panda;  cp -R ../../rai-robotModels/panda/*_description panda
mkdir -p ur10; cp ../../rai-robotModels/ur10/*.g ur10;  cp -R ../../rai-robotModels/ur10/*_description ur10
mkdir -p pr2; cp ../../rai-robotModels/pr2/*.g pr2;  cp -R ../../rai-robotModels/pr2/*_description pr2
mkdir -p baxter; cp ../../rai-robotModels/baxter/*.g baxter;  cp -R ../../rai-robotModels/baxter/*_description baxter
mkdir -p robotiq; cp ../../rai-robotModels/robotiq/*.g robotiq;  cp -R ../../rai-robotModels/robotiq/meshes robotiq
mkdir -p scenarios; cp ../../rai-robotModels/scenarios/*.g scenarios
mkdir -p tests; cp ../../rai-robotModels/tests/*.g tests
cd ../..

export PYTHONPATH=.

### build each version
for ver in 8 9 10 11 6 7; do
    echo -e "\n\n======== compiling (python version " $ver ") ========"
    cmake -B build_wheel -DPYBIND11_PYTHON_VERSION=3.$ver .
    make -C build_wheel _robotic
    
    echo -e "\n\n======== documenting (python version " $ver ") ========"
    make -C build_wheel docstrings
    
    echo -e "\n\n======== build wheel (python version " $ver ") ========"
    cp -f build_wheel/_robotic.*3$ver*.so robotic/_robotic.so
    cp -f build_wheel/stubs/_robotic*/__init__.pyi robotic/_robotic.pyi
    strip --strip-unneeded robotic/_robotic.so
    python3.$ver setup.py bdist_wheel
    #break
done

### delete setup temp files
rm -Rf robotic/__pycache__ build/bdist* build/lib robotic.egg-info

echo -e "\n\n======== renaming wheels ========"
for wheel in $(find dist -iname "*.whl") ; do 
  mv $wheel $(echo $wheel | sed 's/-linux_/-manylinux2014_/')
done
