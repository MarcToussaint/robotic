#/bin/sh

cd $HOME/local 
ln -f -s _make/CMakeLists-docker.txt CMakeLists.txt

### delete setup temp files
rm -Rf robotic/__pycache__ dist/ build/bdist* build/lib robotic.egg-info

### build each version
for ver in 12 13 14 10 11; do
    echo -e "\n\n======== compiling (python version " $ver ") ========"
    cmake -B build_wheel -DPY_VERSION=3.$ver --log-level=WARNING .
    make -j8 --silent -C build_wheel _robotic docstrings install
    if [ "$?" != 0 ]; then
	echo "--- compile failed ---"
	exit
    fi

    echo -e "\n\n======== cleanup libs (python version " $ver ") ========"
    strip --strip-unneeded src/robotic/_robotic.so
    strip --strip-unneeded src/robotic/librai.so
    strip --strip-unneeded src/robotic/meshTool

    echo -e "\n\n======== build wheel (python version " $ver ") ========"
    #python3.$ver setup.py --quiet bdist_wheel
    python3.$ver -m build --quiet

    echo -e "\n\n======== renaming wheel ========"
    for wheel in $(find dist -iname "*any.whl") ; do 
	mv $wheel $(echo $wheel | sed 's/-py3-none-any/-cp3'$ver'-cp3'$ver'-manylinux2014_x86_64/')
    done
    #break
done

### delete setup temp files
rm -Rf robotic/__pycache__ build/bdist* build/lib robotic.egg-info

