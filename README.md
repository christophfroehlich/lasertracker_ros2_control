# lasertracker_ros2_control
Leica Hexagon Lasertracker with RTFP-EtherCAT via ros2_control

## Install
Follow the steps in [compile_ethercat.sh](compile_ethercat.sh) for ethercat configuration.
More information in the [ethercat_ros2_control](https://icube-robotics.github.io/ethercat_driver_ros2/quickstart/installation.html) docs or [this blog](https://embeng.dynv6.net/igh-ethercat-master-on-bbb-rpi).

Build dockerfile included in this repo
```bash
docker build . -t lasertracker_ros2_control -f Dockerfile/Dockerfile
```

## Run
on host:
```bash
sudo /etc/init.d/ethercat start
```
should give you `Starting EtherCAT master 1.5.3  done`.

Check if device was created
```bash
ls -la /dev/EtherCAT0
sudo ethercat master
ethercat slaves
```
should give
```bash
0  0:0  PREOP  +  Leica RT Output
```
Run docker
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

check if the docker can access the device
```bash
cd /home/lasertracker_ws
ethercat slaves
./src/lasertracker_ros2_control/symlink.sh 
```

build and run the ros2_control system
```bash
cd /home/lasertracker_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
export CYCLONEDDS_URI=/opt/ait/install/config/cyclonedds_config.xml
ros2 launch lasertracker_ros2_control lasertracker.launch.xml
```
