mc_rtc plugin for joystick controller
==

This project is a [mc_rtc] plugin to interract with a joystick controller, it has been tested for Xbox controllers

## Installation 

In repo directory :
```shell
mkdir build && cd build
cmake ..
make
sudo make install
```
## Use
To enable the plugin add to your [mc_rtc] configuration file :
```shell
Plugins: [mc_joystick_plugin]
```
The joystick connection status is accessible using the datastore value:
```cpp
bool joystick_online = ctl.datastore().get<bool>("Joystick::connected");
```
You can access the value of an input using the function loaded in the datastore
```cpp
  auto & buttonFunc = ctl.datastore().get<std::function<bool(joystickButtonInputs button)>>("Joystick::Button");
  
  //Button event is a boolean which is true if the state of the button change
  auto & buttonEventFunc = ctl.datastore().get<std::function<bool(joystickButtonInputs button)>>("Joystick::ButtonEvent");

  auto & triggerFunc = ctl.datastore().get<std::function<double(joystickAnalogicInputs)>>("Joystick::Trigger");

  auto & stickFunc = ctl.datastore().get<std::function<Eigen::Vector2d(joystickAnalogicInputs)>>("Joystick::Stick");

  auto & padFunc = ctl.datastore().get<std::function<double(joystickAnalogicInputs)>>("Joystick::Pad");

```

You can refer to `joystick_inputs.h` to get the available buttons