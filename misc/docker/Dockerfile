FROM osrf/ros:noetic-desktop-full
#this is a small but basic utility, missing from osrf/ros. It is not trivial to know that this is
#missing when an error occurs, so I suggest installing it just to bes sure.
RUN apt-get update && apt-get install -y netbase
#set shell 
SHELL ["/bin/bash", "-c"]

ENV BUILDKIT_COLORS=run=green:warning=yellow:error=red:cancel=cyan
#start with root user
USER root 

###Create new user
#
#Creating a user inside the container, so we won't work as root.
#Setting all setting all the groups and stuff.
#
###

#expect build-time argument
ARG HOST_USER_GROUP_ARG
#create group appuser with id 999
#create group hostgroup with ID from host. This is needed so appuser can manipulate the host files without sudo.
#create appuser user with id 999 with home; bash as shell; and in the appuser group
#change password of appuser to admin so that we can sudo inside the container
#add appuser to sudo, hostgroup and all default groups
#copy default bashrc and add ROS sourcing
RUN groupadd -g 999 appuser && \
    groupadd -g $HOST_USER_GROUP_ARG hostgroup && \
    useradd --create-home --shell /bin/bash -u 999 -g appuser appuser && \
    echo 'appuser:admin' | chpasswd && \
    usermod -aG sudo,hostgroup,plugdev,video,adm,cdrom,dip,dialout appuser && \
    cp /etc/skel/.bashrc /home/appuser/ && \
    echo "source /opt/ros/noetic/setup.bash" >> /home/appuser/.bashrc

#basic dependencies for everything
USER root
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive\
    apt-get install -y\
    netbase\
    git\
    build-essential\    
    wget\
    curl\
    gdb\
    lsb-release\
    sudo\
    software-properties-common\
    python3-pip\
    python3-distutils\
    python3-apt


#install dependencies for livox mav track
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive\
    apt-get install -y\
    libgoogle-glog-dev\
    python3-catkin-tools\
    ros-noetic-rviz\
    ros-noetic-pcl-ros\
    ros-noetic-octomap-ros\
    ros-noetic-octomap-msgs\
    ros-noetic-dynamixel-sdk\
    ros-noetic-effort-controllers\
    ros-noetic-jsk-rviz-plugins\
    ros-noetic-jsk-rqt-plugins\
    ros-noetic-joint-trajectory-controller\
    ros-noetic-usb-cam\
    ros-noetic-joy\
    ros-noetic-pointgrey-camera-driver\
    ros-noetic-octomap-rviz-plugins

    #install livox sdk
USER appuser
RUN cd /home/appuser && \
    git clone https://github.com/Livox-SDK/Livox-SDK.git && \
    cd Livox-SDK/build && \
    cmake .. && \
    make
USER root
RUN cd /home/appuser/Livox-SDK/build && \
    make install && \
    rm -r /home/appuser/Livox-SDK

# create folder for livox_mav_track and add tracy profiler
USER root
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive\
    apt-get install -y\
    libglfw3-dev\
    libfreetype-dev\
    libcapstone-dev\
    libdbus-1-dev\
    libtbb-dev
USER appuser
RUN mkdir -p /home/appuser/livox_mav_track/thirdparty && \
    cd /home/appuser/livox_mav_track/thirdparty && \
    git clone https://github.com/wolfpld/tracy.git && \
    cd tracy && \
    git checkout v0.11.1 && \
    cmake -B profiler/build -S profiler -DCMAKE_BUILD_TYPE=Release -DLEGACY=1 -DCMAKE_EXE_LINKER_FLAGS="-Wl,--no-as-needed -ldl -pthread -ltbb" && \
    cmake --build profiler/build --config Release --parallel && \
    cmake -B capture/build -S capture -DCMAKE_BUILD_TYPE=Release -DLEGACY=1 -DCMAKE_EXE_LINKER_FLAGS="-Wl,--no-as-needed -ldl -pthread -ltbb" && \
    cmake --build capture/build --config Release --parallel


#install vscode server and extensions inside the container
#it propably won't work on computers of others because of the specific vscode version
#mostly stolen from here: https://gist.github.com/discrimy/6c21c10995f1914cf72cd8474d4501b2
#its great, because it means it is already installed into the image, so when starting a vscode instance inside the container, it will be already there.
#it will not have to download it.
#more info: https://github.com/microsoft/vscode-remote-release/issues/1718
USER root
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive\
    apt-get install -y\
    curl \
    jq
COPY  --chown=appuser:appuser ./misc/.devcontainer/ /home/appuser/livox_mav_track/.devcontainer/
USER appuser
ARG VSCODE_COMMIT_HASH
RUN bash /home/appuser/livox_mav_track/.devcontainer/preinstall_vscode.sh $VSCODE_COMMIT_HASH /home/appuser/livox_mav_track/.devcontainer/devcontainer.json

# clone LiVOX ROS: https://github.com/Livox-SDK/livox_ros_driver
# checkout to specific version with it worked for sure
USER appuser
RUN mkdir -p /home/appuser/livox_mav_track/src && \
    cd /home/appuser/livox_mav_track/src && \
    git clone https://github.com/Livox-SDK/livox_ros_driver.git && \
    cd livox_ros_driver && \
    git checkout v2.6.0 

# install stuff for the scripts
# install yq
USER root
RUN wget https://github.com/mikefarah/yq/releases/download/v4.45.4/yq_linux_amd64 -O /usr/local/bin/yq && \
    chmod +x /usr/local/bin/yq
# install apt dependencies for the scripts
USER root
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive\
    apt-get install -y\
    python3-tk
# install python dependencies for the scripts
USER appuser
RUN python3 -m pip install \
    matplotlib==3.7.5 \
    numpy==1.21 \
    pandas==2.0.3 \
    seaborn==0.13.2

# copy my_packages
COPY --chown=appuser:appuser ./my_packages/ /home/appuser/livox_mav_track/src/my_packages/

# install livox_mav_track
USER appuser
ARG DRONE_TRACKING_BUILD_TYPE
RUN cd /home/appuser/livox_mav_track && \
    catkin config --extend /opt/ros/noetic && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=$DRONE_TRACKING_BUILD_TYPE -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && \
    catkin build && \
    echo "source /home/appuser/livox_mav_track/devel/setup.bash" >> /home/appuser/.bashrc

