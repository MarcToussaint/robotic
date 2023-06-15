#/bin/sh

rm -f CMakeLists.txt
ln -s build_utils/CMakeLists-docker.txt CMakeLists.txt
#make -C rai -j1 unityAll
mkdir -p build

### delete old 
rm -Rf robotic/*.so* dist/ build/bdist* build/lib robotic.egg-info
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

### build each version
for ver in 6 7 8 9 10 ; do
    cd build
    cmake -DPYBIND11_PYTHON_VERSION=3.$ver .. && make libry
    strip --strip-unneeded libry*3$ver*.so
    cd ..
    cp -f build/libry*3$ver*.so robotic/libry.so && python3.$ver setup.py bdist_wheel 
done

for wheel in $(find dist -iname "*.whl") ; do 
  mv $wheel $(echo $wheel | sed 's/-linux_/-manylinux2014_/')
done

#twine upload dist/*.whl
