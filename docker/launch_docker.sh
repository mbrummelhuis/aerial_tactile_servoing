# docker run -- run the docker container
# -rm -- remove the container after it is done running
# -it -- interactive terminal
# --user -- run the container as the ros2 user
# --name -- name the container
# --network -- connect the container to the host network (when set as host)
# -- ipc=host -- connect the container to the host IPC namespace
# -v -- mount a volume
# --device -- pass a device to the container, put the device path here (tactips, serial for FCU, serial for servos). Check out articulated robotics video 'devices in docker' for more info
# --privileged -- give the container full access to the host system (needed for GPIO)

docker run -it --rm --name ats-container --network=host --ipc=host --device=/dev/ttyUSB0 --device=/dev/mem --privileged mbrummelhuis/ats-no-build