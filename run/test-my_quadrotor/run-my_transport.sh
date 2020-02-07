docker run -it \
  --name refrain-my_transport \
  --net myquadnet \
  --ip 172.19.0.20 \
  --env "GAZEBO_MASTER_URI=http://172.19.0.10:11345" \
  --volume="/l/p/lenta/refrain:/opt/my_ros_ws" \
  errcsool/refrain-gazebo-transport \
  ros2 run refrain_velodyne_sample vel
