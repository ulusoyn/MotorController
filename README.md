# What

This package is used to establish the communication between raspberry pi and a micro controller (pi pico w in my case).

# How
By subscribing any topic (/bumperbot_controller/cmd_vel_unstamped for this package) having the interface geometry_msgs::msg::TwistStamped.
Note: I don't know why the topic name contains unstamped and the message interface is TwistStamped. Just make sure subscribed topic has geometry_msgs::msg::TwistStamped interface.

The node takes linear_x and angular_y components of the twist message and converts it to the wheel velocities by:

```
double right_wheel_speed = (linear_x + (angular_z * wheel_base_ / 2.0));
double left_wheel_speed = (linear_x - (angular_z * wheel_base_ / 2.0));
```

This is very basic kinematics (not sure if it's working exactly with my robot), it just takes desired angular and linear velocities and sends this values to the microcontroller by sendToMicroController() method. 

# sendToMicroController
This method takes two arguments as floats, then convert it to a specific message format which can be read by microcontroller easily. The format is basically:
```
"r<wheel_direction><desired_velocity>, l<wheel_direction><desired_velocity>,"
```
Seems little complicated but it is very easy to understand. 'r' and 'l' means left and right to specify the side of the wheel.
<wheel_direction> can be 'p' or 'n' stands for positive and negative direction (for negative and positive wheel velocity)
<desired_velocity> is target wheel velocity with two decimal places.

Examples:

```
"rp01.50,ln05.60,"

"rn10.50,ln07.60,"
```
