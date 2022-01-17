# sim800_ups_monitor
UPS monitoring using SIM800L module and STM32 bluepill board

new in version 2.5
- Set SIM800L clock using NTP
- I2C LCD1602 support for Temp & Clock display  

new in version 0.2.0
- Error handling Rust-style
- Many modules structure
- State machines using structs
- USB for logging console messages
- SMS commands handling
- SIM balance request using SMS (only for MTS operator)
- interface with ESP01 module for LAN operations via UART. Use two ESP01 modules. One as relay server (relay_server.ino)
Second as STM32-ESP01 interface (esp2stm.ino). Scketches written using Arduino IDE and ESP8266 libs.
 
