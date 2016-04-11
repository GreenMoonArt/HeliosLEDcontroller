# HeliosLEDcontroller
Simple Arduino LED Brightness Controller

This controller is based on the Adafruit Bluefruit app (https://github.com/adafruit/Bluefruit_LE_Connect). 

For this application, we have an Arduino controlling the brightness of 30 LEDs. The LEDs are arranged in 3 series units of 10, which are in parallel with each other. 30 Vdc is applied across each group of 10. The Arduino controls the brightness by using pulse width modulation (PWM) to a MOSFET. 

In this initial effort, we used the Bluefruit colorpicker. Tapping in the red area causes the LEDs to be on full strength. Tapping in the green area causes them to be at about half brightness. And tapping in the blue area turns them off. 
