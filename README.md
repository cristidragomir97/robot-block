# Robot Block 

Takes in a robot configuration from a JSON file and creates all the required ROS publishers and subscribers.
Basic idea is to be able to abstract away the hardware completely for higher levels of functionality (perception, ML, mapping, etc.)

*Also runs IMU filters, and PID control if configured.*

---
## Supported modules
* **Motor Drivers**: 
    * L298N GPIO (and any other motor controllers using PWM + DIR pins, eg. VNH3SP30)
    * Sparkfun Qwiic Motor Controler
    * Arduino 
* **Encoders**: 
    * Quadrature Encoders (GPIO)
* **Sensors**: 
    * **Basics:**
        * GPIO Input 
        * I2C ADC 
    * **Ranging:**
        * VL53L1
        * SR04
    * **IMUs:**
        * BNO055
        * LSM9DS1
        * MPU6050
* **Actuators**
    * GPIO Output 
    * GPIO PWM 
    * **Servo**:
        * Jetson GPIO (hardware PWM on pin 32, 32)
        * PCA9865 I2C Servos


---
## Example Config for balenaBot
``` json
[{
    "name": "balenabot_mini", 
    "desc": "Low-cost robot based around Sparkfun Auto pHAT",
    "components":{

        "interface": [
            {   
                "library":"PCA9685",
                "signal":"PWM",
                "address": "0x7f",
                "channels": [0, 1, 2, 3]
            },
            {
                "library":"ADS1115",
                "signal":"ADC",
                "address": "0x7f",
                "channels": [0, 1, 2, 3]
            }
        ],

        "actuators": [
            {   "type": "servo",
                "role": "service",
                "topic": "/camera_servo",
                "library": "I2CPWM",
                "address": "PCA/0",
                "args":{}
            }, 
            {
                "type": "driver",
                "role": "subscriber",
                "topic": "/cmd/vel",
                "library": "SCMD",
                "address": "0x5d", 
                "args":{
                    "flip": "true",
                    "radius": 0.0325 
                }
            }
        ],

        "sensors": [
            {
                "type": "ranging",
                "library": "VL53L1",
                "role": "publisher",
                "topic": "/range/center", 
                "address": "0x29",
                "args":{}
            }, 
            {
                "type": "ranging",
                "library": "VL53L1",
                "role": "publisher",
                "topic": "/range/left",
                "address": "0x31",
                "args":{}
                
            }, 
            {
                "type": "ranging",
                "library": "VL53L1",
                "role": "publisher",
                "topic": "/range/right", 
                "address": "0x32",
                "args":{}
            },
            {
                "type": "motion",
                "topic": "/imu/raw",
                "role": "publisher",
                "library":"LSM9DS1",
                "address": ["0x1e", "0x6b"],
                "args":{
                    "publish_tf": true
                }
                
            },
            {
                "type": "power",
                "library":"INA219",
                "role": "publisher",
                "topic": "/power/motors",
                "address": "0x40",
                "args":{}
            }
        ],

        "external":[
            {
                "name": "camera",
                "role": "external",
                "source": "~/catkin_ws/realsense-ros",
                "build": "false",
                "command": "roslaunch",
                "package": "realsense2_camera",
                "file": "rs_camera.launch",
                "args": {
                    "color_width": "640",
                    "color_height": "480",
                    "color_fps": "15",
                    "depth_width": "640",
                    "depth_height": "480",
                    "depth_fps": "15"
                }
            },
            {
                "name": "rplidar",
                "role": "external",
                "source": "~/catkin_ws/rpilidar_ros",
                "build": "false",
                "command": "roslaunch",
                "package": "rplidar_ros",
                "file": "rpildar.launch",
                "args": {}
            },
            {
                "name":"imu_filter",
                "role": "external",
                "build": "false",
                "source": "",
                "command": "rosrun",
                "package": "imu_complementary_filter",
                "file": "complementary_filter_node",
                "args": {
                    "fixed_frame": "camera_link",
                    "use_mag": "false",
                    "do_bias_estimation": "true", 
                    "do_adaptive_gain": "false", 
                    "publish_tf": "true"
                }
            }
        ]

    }
}]

```