# hovercraft_prj

ESP32 + MPU6050 + MQTT (ESP-IDF), com arquitetura por componentes:
- `imu_mpu6050`: leitura IMU (I²C driver novo).
- `net_mqtt`: Wi-Fi + MQTT (esp-mqtt), publica dados.
- `controller`: esqueleto para PID/LQR.
- `route_planner`: esqueleto para planejamento.

## Build
```bash
. $HOME/esp/esp-idf/export.sh
idf.py set-target esp32
idf.py -p /dev/ttyUSB0 build flash monitor
