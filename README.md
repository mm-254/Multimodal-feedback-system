# Multimodal-feedback-system
A multimodal feedback system for a JACO assistive robot

This application was developed on Visual Studio 2015 (Professional) and brings together the CHAI3D library, JACO API, Arduino sensors and a peltier device (thermal display), and the SDK for the tactor device used in this project. There are many feedback specificities to the original application worked on here, but in a more general view, it does the following in multiple threads:
1) Implements control of a JACO robot using a Force dimension Omega 7 Haptic device, with modes changed using the gripper switch on the Omega
2) Retrieves information from an MLX90614 non-contact temperature sensor: https://www.sparkfun.com/products/10740 and a non-contact liquid level sensor: https://www.dfrobot.com/product-1493.html via serial communication
3) Sends temperature information to a peltier module connected to an Arduino for thermal feedback
4) Renders weight feedback (input manually, no sensor) and an indication of liquid level to the haptic device
5) Implements keyboard toggle on/off of the feedback properties
6) Renders a visual display of properties (numerical and color change)
7) Renders feedback to a vibrating tactors device from Engineering Acoustics

The generated application is dependent on the modalities specified at the very top of the code. Modality number identifiers are included in the comments.
The code itself could definitely be more efficient (especially for the visual display function), but specificities had to be accommodated (and deadlines met). It does, however, work as long as chai3d is installed correctly (follow instructions on their webpage), omega 7 drivers and SDK are installed, JACO API is installed, and the tactor SDK is installed correctly.

The serial library used is from an external source (Arduino forum). 
Arduino codes used to program sensor are included.
Arduino code used to program the peltier module is included. For this module the display is built from two peltier devices (hot and cold) and a TMP35 temperature sensor. A PID controller is implemented using this PID library: https://playground.arduino.cc/Code/PIDLibrary/ (but coefficients will need further tuning).

*Note: Visual studio files to be uploaded.
