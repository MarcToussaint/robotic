#/bin/sh

rm -f CMakeLists.txt
ln -s build_utils/CMakeLists-docker.txt CMakeLists.txt
#make -C rai -j1 unityAll
mkdir -p build

cd build
cmake -DPYBIND11_PYTHON_VERSION=3.6 .. && make libry
cmake -DPYBIND11_PYTHON_VERSION=3.7 .. && make libry
cmake -DPYBIND11_PYTHON_VERSION=3.8 .. && make libry
cmake -DPYBIND11_PYTHON_VERSION=3.9 .. && make libry
cmake -DPYBIND11_PYTHON_VERSION=3.10 .. && make libry
strip --strip-unneeded libry*36*.so
strip --strip-unneeded libry*37*.so
strip --strip-unneeded libry*38*.so
strip --strip-unneeded libry*39*.so
strip --strip-unneeded libry*310*.so
cd ..

cd robotic
rm -Rf rai-robotModels
mkdir rai-robotModels; cd rai-robotModels
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

rm -Rf robotic/*.so* dist/ build/bdist* build/lib robotic.egg-info
unalias cp

cp -f build/libry*36*.so robotic/libry.so && python3.6 setup.py bdist_wheel 
cp -f build/libry*37*.so robotic/libry.so && python3.7 setup.py bdist_wheel 
cp -f build/libry*38*.so robotic/libry.so && python3.8 setup.py bdist_wheel 
cp -f build/libry*39*.so robotic/libry.so && python3.9 setup.py bdist_wheel 
cp -f build/libry*310*.so robotic/libry.so && python3.10 setup.py bdist_wheel 

for wheel in $(find dist -iname "*.whl") ; do 
  mv $wheel $(echo $wheel | sed 's/-linux_/-manylinux2014_/')
done

#twine upload dist/*.whl
