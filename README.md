## TurtleBro ROS2 package


### How to build one package

```
colcon build --symlink-install --parallel-workers 1 --packages-select=turtlebro
```

### Start/Stop robot services

#### FastDDS discovery server

```
sudo systemctl start/stop fastdds
```

#### MicroROS agent

```
sudo systemctl start/stop microros
```

#### Main TurtleBro service

```
sudo systemctl start/stop turtlebro
```