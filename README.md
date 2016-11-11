# gazebo_plugins
A collection of Gazebo plugins.

To compile each plugin you'll need the Gazebo development files. For example, in Debian/GNU or Ubuntu you must

```
sudo apt-get install libgazebo7-dev
```

To test each plugin, make sure to add your library path to the `GAZEBO_PLUGIN_PATH`. For example, for the `imu_noise_plugin` you must

```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo_plugins/imu_noise_plugin/build
```
