#!/bin/sh

cd $HOME/local 
ln -f -s _make/CMakeLists-docker.txt CMakeLists.txt

PLAT="manylinux_2_28_$(uname -m)"

### delete setup temp files
rm -Rf robotic/__pycache__ dist/ build/bdist* build/lib robotic.egg-info

### build each version
for ver in 12 13 14 10 11; do
    echo -e "\n\n======== compiling (python version " $ver ") ========"
    ### drop the bindings of previously built python versions to minimize wheel size
    rm -f src/robotic/_robotic.cpython-*.so
    cmake -B build_wheel -DPY_VERSION=3.$ver --log-level=WARNING .
    make -j$(command nproc --ignore 1) --silent -C build_wheel _robotic docstrings install
    if [ "$?" != 0 ]; then
	echo "--- compile failed ---"
	exit
    fi

    echo -e "\n\n======== cleanup libs (python version " $ver ") ========"
    strip --strip-unneeded src/robotic/_robotic*.so
    strip --strip-unneeded src/robotic/librai.so
    strip --strip-unneeded src/robotic/meshTool

    echo -e "\n\n======== build wheel (python version " $ver ") ========"
    #python3.$ver setup.py --quiet bdist_wheel
    python3.$ver -m build --quiet

    echo -e "\n\n======== retagging wheel ========"
    ### retag setuptools output as a cpython platform wheel, so auditwheel accepts it below
    for wheel in $(find dist -iname "*-py3-none-any.whl") ; do
	python3.$ver -m wheel tags --remove --python-tag cp3$ver --abi-tag cp3$ver --platform-tag linux_$(uname -m) $wheel
    done

    echo -e "\n\n======== auditwheel repair ========"
    ### librai.so links shared system libs outside the manylinux policy (freetype, GLU, Xi, Xrandr, ...)
    ###repair vendors them into robotic.libs/ and sets the manylinux tag
    for wheel in $(find dist -iname "*-linux_$(uname -m).whl") ; do
	auditwheel repair --plat $PLAT -w dist $wheel
	if [ "$?" != 0 ]; then
	    echo "--- auditwheel repair failed ---"
	    exit 1
	fi
	rm $wheel
    done
    #break
done

### delete setup temp files
rm -Rf robotic/__pycache__ build/bdist* build/lib robotic.egg-info

