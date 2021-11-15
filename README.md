# Hygrometer
Hygrometer based on DHT11 and four 7-segment displays

DHT library took way too much space so I had to code the communication directly.
As of v3.0 this works quite nicely.

I got the power consumption down to 2.5 mA/h:
<1 mA in sleep mode (my lab power supply doesn't show any current draw)
~6 mA when updating measurements
~68 mA when displaying measurements (currently for 6 seconds)

TODO:
- Only wake up every 5 minutes to update measurements (pushes consumption down to ~100µA/h)
