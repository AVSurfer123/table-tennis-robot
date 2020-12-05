1. added an InitialVelocityPlugin to the ping pong ball model to set the inital velocity each time the ball is being loaded into gazebo



use:
1. change environment variable to include libInitialVelocityPlugin.so
$ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:<pkg path>/build

2.change environment variable to include the modifided pingpong ball model
$ export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:<pkg path>/models

1-2 can be done by appending to the end of ~/.bashrc



4.can change gravity to slow down vertial velocity

5.can change number of balls to shoot by changing the for loop range

6.usage

    rosrun kuka_kr5_gazebo spawn_ball


