docker run -d -it \
  --name refrain-my_controller
  --net myquadnet \
  --ip 172.19.0.30 \
  osrf/ros:eloquent-desktop
  /bin/bash 
