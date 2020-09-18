#!/bin/bash
# $1 is the docker imagesID (sudo docker images)
# $2 is the docker's nickname

xhost +local:root


sudo docker run -it --env="DISPLAY=$DISPLAY" \
                    --env="QT_X11_NO_MITSHM=1" \
                    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
                    --env="XAUTHORITY=$XAUTH" \
                    --volume="$XAUTH:$XAUTH" \
                     -v $(pwd):/home/$USER/Desktop -e USER=$USER -e USERID=$UID $1 bash
