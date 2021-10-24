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
    "name": "balenaBot", 
    "desc": "default implementation of the balenaBot",
    "roscore": "true",
    "drivetrain":{
        "type":"differential",
        "use_pid": "true",
        "wheels": {
            "no": 2, 
            "radius": 0.0325, 
            "sep": 0.295
        },
        "driver": {
            "type": "sparkfun",
            "flip": "true"
        }, 
        "encoders": [
            { 
                "type": "quadrature",
                "position": "left",
                "A":  37, 
                "B": 35
            }, 
            { 
                "type": "quadrature",
                "position": "left",
                "A":  33, 
                "B": 31
            }
        ]
    },
    "servos": [
        {
            "name": "camera_servo",
            "type": "GPIO",
            "pin": 33
        }
    ],
    "sensors": [
        {
            "type": "VL53L1",
            "name": "center_ir",
            "bus": 1, 
            "address": "0x29",
            "angle": 0, 
            "flip": "false",
            "fov": "wide",
            "range": 3
        }, 
        {
            "type": "VL53L1",
            "name": "left_ir",
            "bus": 1, 
            "address": "0x28",
            "angle": -30, 
            "flip": "false",
            "fov": "wide",
            "range": 3§
        }, 
        {
            "type": "VL53L1",
            "name": "right_ir",
            "bus": 0, 
            "address": "0x29",
            "angle": 30, 
            "flip": "false",
            "fov": "wide",
            "range": 3
        }
    ]
}]
```