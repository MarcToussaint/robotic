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
./install.sh opencv
./install.sh physx
./install.sh librealsense
./install.sh console_bridge # for pinocchio
./install.sh urdfdom_headers # for pinocchio
./install.sh urdfdom # for pinocchio
./install.sh pinocchio # for libfranka
./install.sh -v 0.18.0 libfranka  
./install.sh -v 1.10.0 trossen_arm
./install.sh basler

# setup local venv and python dependencies

mkdir -p ~/.local
python3 -m venv ~/.local/venv
source ~/.local/venv/bin/activate
curl -LsSf https://astral.sh/uv/install.sh | sh
uv pip install numpy pybind11 pybind11-stubgen

# clone

mkdir -p $HOME/git
cd $HOME/git
git clone --recursive https://github.com/MarcToussaint/robotic.git
cd robotic

# configure

make install-with-hardware

ry-info
