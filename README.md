# Hygrometer
Hygrometer based on DHT11 and four 7-segment displays

DHT library took way too much space so I had to code the communication directly.
As of v4.0 this works quite nicely.

I got the power consumption down to 0.1 mA/h:
<1 mA in sleep mode (my lab power supply doesn't show any current draw)
~6 mA when updating measurements (currently 2 measurements aka 8 seconds every 5 minutes)
~68 mA when displaying measurements (currently for 5 seconds)
