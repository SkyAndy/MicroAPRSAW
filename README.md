This is an Arduino wrapper for MicroAPRS in SerialMode from https://github.com/markqvist/MicroAPRS

Serial Protocol is enabled in
https://github.com/markqvist/MicroAPRS/blob/master/Modem/config.h

change from
```c
#define SERIAL_PROTOCOL PROTOCOL_SIMPLE_SERIAL
//#define SERIAL_PROTOCOL PROTOCOL_KISS
```
to
```c
//#define SERIAL_PROTOCOL PROTOCOL_SIMPLE_SERIAL
#define SERIAL_PROTOCOL PROTOCOL_KISS
```

make it new and flash the firmware on you Arduino238p

As an example the code will transmit the humidity and temperature values
from an DHT11 sensor as Comment via APRS
