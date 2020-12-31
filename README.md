# Virtual Joystick
 Utilizing an IMU and Madgwick algorithm to get orientation information to send to the computer as an HID joystick

## Useful Links
- [Making a custom Teensy HID Joystick](https://blog.hamaluik.ca/posts/making-a-custom-teensy3-hid-joystick/)
- [HID Report Descriptors Tutorial](https://eleccelerator.com/tutorial-about-usb-hid-report-descriptors)
- [Magnetic Fields of Cylindrical Bar Magnet](https://demonstrations.wolfram.com/MagneticFieldOfACylindricalBarMagnet/)

## Bill of Materials
| Item | Quantity | Description | Individual Price |
| --- | --- | --- | --- |
| [Teensy 4.0](https://www.pjrc.com/store/teensy40.html) | 1 | Microcontroller with HID capability | 19.95 | 
| [ICM-20948 9DOF](https://www.adafruit.com/product/4554) | 1 | A 9DOF IMU to get position and motion data | 14.95 |
| [Linear Output Hall Effect Sensor](https://www.mouser.com/ProductDetail/Texas-Instruments/DRV5053OAQLPGM/?qs=1CfNGUMoiQ%2Fpls9IqGpk2A%3D%3D) | 5 | Hall effect sensors to work in conjunction with the magnets to track position of fingers | 1.24 |
| Magnets (Which ones TBD) | 5 | To work in conjunction with the hall effect sensors to track position of fingers |

## Magnet Possibilities
- https://www.digikey.com/en/products/detail/standex-meder-electronics/ALNICO500-2-5X12-7MM/413910
- https://www.digikey.com/en/products/detail/radial-magnets-inc/9016/5126080 
- https://www.amazon.com/Bullseye-Office-Cylinder-Magnets-Magnetic/dp/B01BUDVBWG/ref=sr_1_5?dchild=1&keywords=Cylinder+Magnet&qid=1609207386&sr=8-5 
