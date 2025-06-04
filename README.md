# Goliath Motor Controller - Arduino Firmware

> Firmware for motor control with Arduino (ROS2-compatible examples included).

## Quick Start

1. **Download**  
   - Clone the repo :
   -  bash ``` git clone https://github.com/bvdhaagen/goliath_core_code.git ```
     
   or download the `.zip`
   
     Unpack the ```goliath_core_code``` and make sure the library file is separately ziped and packed as ```MotorController.zip```
   
3. **Install in Arduino IDE**  
   - `Sketch` → `Include Library` → `Add .ZIP Library.` → Select the -> `MotorController.zip`.
   - install the accelstepper library, using the `library manager`

4. **Open Examples**  
   Navigate to:  
   `File` → `Examples` → `Motorcontroller` → Select one:  
   - `goliath_core_v5_2025.ino`                          (WIP)
   - `goliath_ros2_control_june1_25_including_gripper`   (working)  
   - `goliath_ros2_control_may19_25b`                    (working)
   - `single_motor_test`                                 (working)

5. **Upload**  
   - Set board/port in `Tools` → Upload (➡️ button).

## Need Help?
Open an [Issue](https://github.com/bvdhaagen/goliath_core_code/issues).
                                           
