#!/bin/bash
#parameters from .param file
if ! [ -f ".params" ];
then
   cp .params.example .params
fi
source .params

HOST_USER_GROUP_ARG=$(id -g $USER)
VSCODE_COMMIT_HASH=$(code --version | sed -n '2p')
echo $VSCODE_COMMIT_HASH
#build the image
docker build \
    --file Dockerfile \
    --tag $image_name:$image_tag \
    --build-arg HOST_USER_GROUP_ARG=$HOST_USER_GROUP_ARG \
    --build-arg VSCODE_COMMIT_HASH=$VSCODE_COMMIT_HASH \
    --build-arg DRONE_TRACKING_BUILD_TYPE=$DRONE_TRACKING_BUILD_TYPE \
    ./../../