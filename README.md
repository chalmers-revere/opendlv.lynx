# opendlv.lynx

## Building using a Docker builder:

    cd docker
    make buildComplete
    make createDockerImage

## Run the resulting Docker image:

    docker run -ti --rm --net host --user odv chalmersrevere/opendlv.lynx-on-opendlv-sim-on-opendlv-on-opendlv-core-on-opendavinci-on-base:latest /bin/bash

