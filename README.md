# crazyflie_ros2

For installing cflib as an external dependency

```
sudo mkdir -p /usr/local/share/ros && sudo cp cflib-python.yaml /usr/local/share/ros
```

```
echo "yaml file:///usr/local/share/ros/cflib-python.yaml" | sudo tee -a /etc/ros/rosdep/sources.list.d/20-default.list
rosdep update
```
```
rosdep install --from-paths src --ignore-src -r -y 
```

WSL use
https://github.com/dorssel/usbipd-win/wiki/WSL-support

Run TF tree viewer

```
ros2 run rqt_tf_tree rqt_tf_tree --force-discover
```
