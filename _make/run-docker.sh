#/bin/sh

thispath="$(dirname "$(realpath "$0")")"

# #see http://wiki.ros.org/docker/Tutorials/GUI
# #https://stackoverflow.com/questions/16296753/can-you-run-gui-apps-in-a-docker-container

# XSOCK=/tmp/.X11-unix
# XAUTH=/tmp/.docker.xauth
# touch $XAUTH
# xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH # # nmerge -
# chmod 755 $XAUTH

xhost +local:root > /dev/null

docker run -it $1 \
       --volume="$thispath/..:/root/local" \
       --volume="$HOME:/root/home" \
       --volume="$thispath/docker.bashrc:/root/.bash_aliases" \
       --volume="$HOME/.gitconfig:/root/.gitconfig:ro" \
       --volume="$HOME/.ssh:/root/.ssh:ro" \
       --env="DISPLAY" \
       --network host \
       --device /dev/input \
       --device /dev/dri \
       rai-manylinux /bin/bash

#       -v $XSOCK:$XSOCK \
#       -v $XAUTH:$XAUTH \
#       -e XAUTHORITY=$XAUTH \

xhost -local:root > /dev/null
