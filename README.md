# jetbot_soccer
This repository is building soccer robot using jetson nano.

## launch

### cam.launch
    roslaunch jetbot_soccer cam.launch
: detect ball and goalpost by color filtering(ball : orange & red   goalpost : bule)

### color_detector.launch
    roslaunch jetbot_soccer color_detector.launch
: launch soccer robot program. first detect the ball and go to the ball then, detect goalpost and go to the goalpost.

### jet_imu.launch
    roslaunch jetbot_soccer jet_imu.launch
: this launch file need jetbot_imu repository. launch soccer robot program using imu.


