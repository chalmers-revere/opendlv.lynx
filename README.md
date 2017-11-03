# opendlv.mantis

## Building using a Docker builder:

    cd docker
    make buildComplete
    make createDockerImage

## Run the resulting Docker image:

    docker run -ti --rm --net host --user odv chalmersrevere/opendlv.mantis-on-opendlv-sim-on-opendlv-on-opendlv-core-on-opendavinci-on-base:latest /bin/bash

