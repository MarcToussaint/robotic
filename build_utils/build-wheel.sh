#/bin/sh

ln -s build_utils/CMakeLists-docker.txt CMakeLists.txt
make -C rai cleanAll
make -C rai unityAll
mkdir build
cd build
cmake -DPYBIND11_PYTHON_VERSION=3.7 ..
make libry
cmake -DPYBIND11_PYTHON_VERSION=3.10 ..
make libry
strip --strip-unneeded libry*37*.so
strip --strip-unneeded libry*310*.so
cd ..

rm -Rf dist/ build/bdist* build/lib robotic.egg-info

rm -Rf robotic/*.so*
cp build/libry*310*.so robotic/libry.so
python3.10 setup.py bdist_wheel 

rm -Rf robotic/*.so*
cp build/libry*37*.so robotic/libry.so
python3.7 setup.py bdist_wheel 

for wheel in $(find dist -iname "*.whl") ; do 
  mv $wheel $(echo $wheel | sed 's/-linux_/-manylinux2010_/')
done

twine upload dist/*.whl

