# humanoids-2016-plugins

Gazebo plugins which allow models to be moved in Cartesian space via yarp ports which have the same name as the model. 

```
git clone https://github.com/rlober/humanoids-2016-plugins.git
cd humanoids-2016-plugins
mkdir build
cd build
cmake ..
make 
```

optional:

```
make install
```

**Note** Make sure to extend the $GAZEBO_PLUGIN_PATH variable.
