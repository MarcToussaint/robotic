#/bin/sh

docker build \
       --tag rai-manylinux \
       --network host \
       -f Dockerfile . #--no-cache
