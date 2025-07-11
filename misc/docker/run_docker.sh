#!/bin/bash
#parameters from .param file
source .params

#add all the local processes to xhost, so the container reaches the window manager
xhost + local:

#check if correct directory paths are provided
if [ -z "$src_folder" ] || [ -z "$dataset_folder" ] || [ -z "$script_folder" ] || [ -z "$vscode_folder" ];
then
    echo "Please provide all directory paths at the begining of the file."
    exit
fi
if  ! [ -d "$src_folder" ] || ! [ -d "$dataset_folder" ] || ! [ -d "$script_folder" ] || ! [ -d "$vscode_folder" ];
then
    echo "Please make sure you provide directories that exist."
    exit
fi

#if the git repository exists and it is not shared promt the user to make it shared
if [[ -e ".git" ]];
then
    if [[ $( git config core.sharedRepository ) != "true" ]];
    then
        echo "The repository is not shared (git config core.sharedRepository)."
        echo "If you will use git at any point you should make it shared because if not the file modifications from inside the container will break git."
        echo "Would you like to make the repo shared?"
        select yn in "Yes" "No"; do
            case $yn in
                Yes )
                    echo "Changing repo to shared.";
                    git config core.sharedRepository true;
                    break;;
                No )
                    echo "Leaving it untouched.";
                    break;;
            esac
        done
    fi
fi

#Set permissions for newly created files. All new files will have the following permissions.
#Important is that the group permissions of the files created are set to read and write.
#We add src as a volume, so we will be able to edit and delete the files created in the container.
#Output of this command is silenced
setfacl -PRdm u::rwx,g::rwx,o::r ./ &> /dev/null

#check if container exists --> then we only restart it
if [[ $( docker ps -a -f name=$container_name | wc -l ) -eq 2 ]];
then
    echo "Container already exists. Do you want to restart it or remove it?"
    select yn in "Restart" "Remove"; do
        case $yn in
            Restart )
                echo "Restarting it... If it was started without USB, it will be restarted without USB.";
                docker restart $container_name;
                break;;
            Remove )
                echo "Stopping it and deleting it... You should simply run this script again to start it.";
                docker stop $container_name;
                docker rm $container_name;
                break;;
        esac
    done
else
    echo "Container does not exist. Creating it."
    if command -v nvidia-smi &> /dev/null;
    then
        echo "NVIDIA driver is installed."
        docker run \
            --env DISPLAY=${DISPLAY} \
            --volume /tmp/.X11-unix:/tmp/.X11-unix \
            --volume ${dataset_folder}:/home/appuser/_Datasets \
            --volume ${src_folder}:/home/appuser/livox_mav_track/src/my_packages \
            --volume ${vscode_folder}:/home/appuser/livox_mav_track/.vscode \
            --volume ${script_folder}:/home/appuser/livox_mav_track/scripts \
            --network host \
            --interactive \
            --tty \
            --detach \
            --gpus all \
            --runtime=nvidia \
            --privileged \
            --env NVIDIA_VISIBLE_DEVICES=all \
            --env NVIDIA_DRIVER_CAPABILITIES=all \
            --name $container_name \
            $image_name:$image_tag
    else
        echo "NVIDIA driver is not installed. Running the container without GPU."
        docker run \
            --env DISPLAY=${DISPLAY} \
            --volume /tmp/.X11-unix:/tmp/.X11-unix \
            --volume ${dataset_folder}:/home/appuser/_Datasets \
            --volume ${src_folder}:/home/appuser/livox_mav_track/src/my_packages \
            --volume ${vscode_folder}:/home/appuser/livox_mav_track/.vscode \
            --volume ${script_folder}:/home/appuser/livox_mav_track/scripts \
            --network host \
            --interactive \
            --tty \
            --detach \
            --privileged \
            --name $container_name \
            $image_name:$image_tag
    fi
fi