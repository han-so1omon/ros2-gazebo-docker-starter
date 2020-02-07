optirun docker run -it \
  --name refrain-my_quadrotor \
  --net myquadnet \
  --ip 172.19.0.10 \
  --gpus all \
  --user $(id -u $USER) \
  --env "DISPLAY" \
  --workdir="/home/$USER" \
  --volume="/l/p/lenta/refrain/gazebo:/home/$USER" \
  --volume="/etc/group:/etc/group:ro" \
  --volume="/etc/passwd:/etc/passwd:ro" \
  --volume="/etc/shadow:/etc/shadow:ro" \
  --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  gazebo \
  gazebo --verbose .gazebo/worlds/my_quadrotor.world
  #/bin/bash
