This project uses open CV python for image proessing and in this I am using camera 
to detect the object of specific colour which can be set by an interactive interface 
initialized on running the program, after the colour is selected and space bar is 
pressed then that coloured object starts to be detected by the program and the angle
values associated with that object are written to Serial monitor of arduino which then
writes those values to the servo motor.
IMPORTANT-Arduino and laptop are to be connected by the arduino cabel and the particaular
port must be changed in the program according to your own PC.
The object selected must have small area so that multiple selections are neglected.
for blue color- HSV values are from [100, 115, 150],[120, 255, 255]
