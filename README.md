# Hygrometer
Hygrometer based on DHT11 and four 7-segment displays

DHT library took way too much space so I had to code the communication directly.
As of v3.0 this works quite nicely

TODO:
- Reduce power consumption in sleep mode. Currently it still draws about 70mA which baffles me since it's supposed to sleep tightly and consume a few hundred µA at most
