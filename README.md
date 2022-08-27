
# MSUR-Ivan

## Build
### MSUR msgs
colcon build --packages-select msur_msgs

### MSUR driver
colcon build --allow-overriding msur_driver --packages-select msur_driver


## Run

### MSUR driver

ros2 run msur_driver telemetry