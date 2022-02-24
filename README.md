odri-spi-rpi
--------
Python code to access ODRI (Open Dynamic Robot Initiative) brushless drivers using directly SPI from a raspberry pi board.

Activate SPI on raspberry pi using the raspi-config utility:
```
sudo raspi-config
```

Connect RPi to uDriver according to this table:

|  Signal | uDriver pin  |  Rpi 40 pin | Color |
|---      |---           |---          |---      |
| GND     |  1           |  20         | Black   |
| MISO    |  2           |  21         | White   |
| CLK     |  3           |  23         | Grey    |
| CS      |  4           |  22         | Violet  | 
| MOSI    |  5           |  19         | Blue    |

![image](https://user-images.githubusercontent.com/11156435/155493434-af1ef2a7-833c-4db4-8dcb-d61be97fca1c.png)

Authors
--------
Thomas Flayols  

License
-------
BSD 3-Clause License

Copyright
-----------
Copyright (c) 2022, LAAS-CNRS
