# ESP32 Micropython BMP280 Barometer Driver #

### The Code: ###

This is an ESP32 micropython driver for the Bosch BMP280 barometer/temperature chip. It uses the I2C protocol to read the air pressure and temperature from the chip, and then uses this data to calculate the chip's altitude (Note: this altitude reading could drift over long periods due to weather patterns).

The code is currently configured to set the BMP280 up for an ultra-high resolution indoor navigation application - see Section 3.4 of the datasheet (linked in references below). The particular settings can be changed in the _setup method in the driver - just make sure you read the datasheet carefully first!

To use the code, you simply have to give it the pin numbers of the SCL and SDA pins connected to your ESP32 (this code does not support with the SPI interface, even though the BMP280 supports it). Optionally, you can also choose the module address - this is by default set to 0x76, but if you pull the SDO pin high then the address becomes 0x77, and you need to feed that into the driver. You can also set the initial altitude of the module in meters (normally determined by GPS data). This offset will be added to the altitude calculated via pressure changes in order to determine the module's absolute altitude - rather than just the altitude relative to its starting point.

### Example usage: ###

``` python3
import bmp280 as BMP

sensor = BMP.BMP280(47, 48, address=0x77, init_gps_alt=5)

press, temp, alt = sensor.get_press_temp_alt()
press, temp = sensor.get_press_temp()
```

For most usage, you won't need to include the address/init_gps_alt parameters in the driver set up, but I included them for completeness.

### References: ###
 
 All the information on the module came from the Bosch datasheet here: <https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf>

 This website was also a useful source for the equations used to calculate altitude: <https://apmonitor.com/dde/index.php/Main/ElevationPressure#:~:text=,equation%20gives%20the%20height%20equation>