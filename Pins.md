ESP32 Pins: 
- GPIO 25: ESCON analog command input for PID output/requested motor effort
- GPIO 33: ESCON enable/inhibit input for motor ON/OFF safety signal 
- GPIO 32: ESCON direction input for rotation direction
- GND: ESCON signal ground / command reference ground for shared signal reference 
- GPIO 5: AMT22 encoder chip-select for encoder SPI
- GPIO 18/19/23: AMT22 SPI clock/data lines for encoder position feedback
