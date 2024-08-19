  # mini_lidar - Arduino(ESP32) - ROS2
  <img src="https://github.com/user-attachments/assets/460d0307-24a3-4d5e-880d-aea657fe8ae0"></img>
  ## connection
  |lidar|esp32|
  |----|----|
  |green|5V(from battery)|
  |yellow|gpio13|
  |black|GND|
      
  
  ## protocol
  |byte||||
  |----|----|----|----|
  |1|header0|0x55||
  |1|header1|0xAA||
  |1|header2|0x23||
  |1|header3|0x10|number of data=16|
  |2|rot_speed||rotation speed|
  |2|start_ang||start angle in deg|
  |2|distance0||first distance data|
  |1|unknown0|||
  ||...|||
  |2|distance15||last distance data|
  |1|unknown15|||
  |2|end_ang||end angle in deg|
  |2|unknown|||
  
  ## ros2 terminal
   ```
  sudo docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble udp4 -p 8888
  ```
  ```
  ros2 topic echo /topic_name
  ```
  ## RViz2
  Fixed Frame : laser_frame<br>
  LaserScan > Topic : /topic_name


