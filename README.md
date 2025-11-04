# lasertracker_ros2_control
Leica Hexagon Lasertracker with RTFP-EtherCAT via ros2_control

## Install
On the host system
https://icube-robotics.github.io/ethercat_driver_ros2/quickstart/installation.html
```bash
docker build . -t lasertracker_ros2_control -f Dockerfile/Dockerfile
```

## Run
```bash
docker run -it \
    --cap-add=sys_nice \
    --ulimit rtprio=99 \
    --ulimit memlock=-1 \
    -v .:/home/lasertracker_ws \
    -v /usr/local/etherlab:/usr/local/etherlab  -v /dev/EtherCAT0:/dev/EtherCAT0 \
    -v /opt/ait/install/config:/opt/ait/install/config:ro \
    --net host \
    --privileged \
    lasertracker_ros2_control
```

```bash
cd /home/lasertracker_ws
colcon build --packages-up-to --symlink-install
source install/setup.bash
sudo ln -s /usr/local/etherlab/bin/ethercat /usr/bin/
sudo ln -s /usr/local/etherlab/etc/init.d/ethercat /etc/init.d/ethercat
sudo mkdir -p /etc/sysconfig
sudo cp /usr/local/etherlab/etc/sysconfig/ethercat /etc/sysconfig/ethercat
export CYCLONEDDS_URI=/opt/ait/install/config/cyclonedds_config.xml
ros2 launch lasertracker_ros2_control lasertracker.launch.xml 
```