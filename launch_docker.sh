# docker run -- run the docker container
# -rm -- remove the container after it is done running
# -it -- interactive terminal
# --user -- run the container as the ros2 user
# --name -- name the container
# --network -- connect the container to the host network (when set as host)
# -- ipc=host -- connect the container to the host IPC namespace
# -v -- mount a volume

docker run -rm -it --user ros2 --name ats-container --network=host --ipc=host