# to be run in a fresh ubuntu:latest docker -- mimicking .github/workflow/cmake
# ./run-docker.sh ubuntu

cd $HOME
apt update
apt install --yes curl

# ubuntu and source dependencies

curl -O https://raw.githubusercontent.com/MarcToussaint/rai/refs/heads/master/_make/install.sh && chmod a+x install.sh
source install.sh vars_only #defines the following dependency packages
apt install --yes ${ubuntu_rai} ${ubuntu_botop} ${ubuntu_python}
./install.sh libccd
./install.sh fcl
./install.sh libann
#./install.sh physx
#./install.sh librealsense
#./install.sh libfranka  ## for OLD frankas instead:   ./install.sh -v 0.7.1 libfranka

# setup local venv and python dependencies

mkdir -p ~/.local
python3 -m venv ~/.local/venv
source ~/.local/venv/bin/activate
python3 -m pip install numpy pybind11 pybind11-stubgen

# clone

mkdir -p $HOME/git
cd $HOME/git
git clone --recursive https://github.com/MarcToussaint/robotic.git
cd robotic

# configure

cp _make/CMakeLists-ubuntu.txt CMakeLists.txt
export PY_VER=`python3 -c "import sys; print(str(sys.version_info[0])+'.'+str(sys.version_info[1]))"`
cmake . -B build -DUSE_PHYSX=OFF -DUSE_BULLET=OFF -DUSE_LIBFRANKA=OFF -DUSE_REALSENSE=OFF -DPY_VERSION=$PY_VER -DUSE_QHULL8=ON

# build

export MAKEFLAGS="-j $(command nproc --ignore 2)"
make -C build _robotic docstrings install
python3 -m pip install -e .
ry-info
