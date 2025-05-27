# to be run in a fresh ubuntu:latest docker -- mimicking .github/workflow/cmake
# ./run-docker.sh ubuntu

set DEBIAN_FRONTEND=noninteractive
apt update

#

apt install --yes \
  g++ clang make gnupg cmake git wget libstdc++-14-dev \
  liblapack-dev libf2c2-dev libqhull-dev libeigen3-dev \
  libjsoncpp-dev libyaml-cpp-dev libhdf5-dev libpoco-dev \
  libboost-system-dev portaudio19-dev libusb-1.0-0-dev \
  xorg-dev libglu1-mesa-dev libglfw3-dev libglew-dev libglm-dev \
  freeglut3-dev libpng-dev libassimp-dev libfreetype6-dev fonts-ubuntu \
  python3-dev python3 python3-pip python3-venv

#

mkdir -p ~/.local
python3 -m venv ~/.local/venv
source ~/.local/venv/bin/activate

#

python3 -m pip install numpy pybind11 pybind11-stubgen

#

export MAKEFLAGS="-j $(command nproc --ignore 2)"
  
#

wget https://github.com/MarcToussaint/rai/raw/refs/heads/marc/_make/install.sh; chmod a+x install.sh
./install.sh libccd
./install.sh fcl
./install.sh libann
#./install.sh physx
#./install.sh librealsense
#./install.sh libfranka  ## for OLD frankas instead:   ./install.sh -v 0.7.1 libfranka

#

mkdir -p $HOME/git
cd $HOME/git
git clone --recursive https://github.com/MarcToussaint/robotic.git
cd robotic
cp _make/CMakeLists-ubuntu.txt CMakeLists.txt
export PY_VERSION=`python3 -c "import sys; print(str(sys.version_info[0])+'.'+str(sys.version_info[1]))"`
cmake -DPY_VERSION=$PY_VERSION -DUSE_REALSENSE=OFF -DUSE_LIBFRANKA=OFF -DUSE_PHYSX=OFF . -B build

#

make -C build _robotic docstrings install

ry-info
