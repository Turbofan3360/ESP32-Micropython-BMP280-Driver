from machine import SoftI2C, Pin
import struct

class BMP280:
    def __init__(self, scl, sda, address=0x76, init_gps_alt=0):
        """
        Driver for the BMP280 temperature and pressure module for an ESP32 running micropython.
        
        Parameters for driver initialisation: ESP32 pin number connected to module SCL, followed by pin number connected to SDA
        Optional parameters:
            Address (address) - set to 0x76 by default, but will need to be set to 0x77 is the SDO pin is pulled high.
            Initial Altitude (init_gps_alt) - this offset will be added to all altitude outputs. Enables an absolute altitude output instead of altitude relative to the starting location.
        
        Available methods:
            get_press_temp() - returns temperature in degrees celcius, and pressure in hPa
            get_press_temp_alt() - returns pressure and temperature, as before, but also the module's altitude relative to its starting altitude.
        """
        
        self.bmp280 = SoftI2C(scl=Pin(scl), sda=Pin(sda), freq=400000)
        self.starting_gpsalt = init_gps_alt
        
        self.bmpaddress = address
        
        self.registers = {
            "id" : 0xD0,
            "reset" : 0xE0,
            "status" : 0xF3,
            
            "measurement_ctrl" : 0xF4,
            "config" : 0xF5,
            
            "pressure" : 0xF7, # 3 bytes
            "temp" : 0xFA, # 3 bytes
            }
        
        self.trim_values = {}
        
        # Setting up sensor and getting factory-programmed trim values from the BMP280's NVM
        self._setup()
        self._get_trim_values()
        p_total = t_total = 0
        
        time.sleep(1)
        
        # Reading initial pressure/temperature values (used to calculate altitude readings)
        for i in range(10):
            p0, t0 = self.get_press_temp()
            t0 += 273.15 # Converting to degrees Kelvin
            
            p_total += p0
            t_total += t0
            
            time.sleep(0.1)
            
        self.p0 = p_total/10
        self.t0 = t_total/10
        
        self.baro_equation_coefficient = (8.314*0.0065)/(9.80665*0.028964) # coefficient required for barometric equation: Rg*L/gM
    
    def _log(self, string):
        print(string)
    
    def _setup(self):
        # Waking sensor up and setting oversampling rates for pressure/temperature measurement
        self.bmp280.writeto_mem(self.bmpaddress, self.registers["measurement_ctrl"], bytes([0x57]))
        
        # Setting standby time length, IIR filter
        self.bmp280.writeto_mem(self.bmpaddress, self.registers["config"], bytes([0x10]))
        
        self._log("Sensor setup")
    
    def _get_trim_values(self):
        # Reading trim values from chip
        digits = self.bmp280.readfrom_mem(self.bmpaddress, 0x88, 24)
        
        self.trim_values["dig_t1"] = struct.unpack("<H", digits[0:2])[0]
        self.trim_values["dig_t2"] = struct.unpack("<h", digits[2:4])[0]
        self.trim_values["dig_t3"] = struct.unpack("<h", digits[4:6])[0]
        
        self.trim_values["dig_p1"] = struct.unpack("<H", digits[6:8])[0]
        self.trim_values["dig_p2"] = struct.unpack("<h", digits[8:10])[0]
        self.trim_values["dig_p3"] = struct.unpack("<h", digits[10:12])[0]
        self.trim_values["dig_p4"] = struct.unpack("<h", digits[12:14])[0]
        self.trim_values["dig_p5"] = struct.unpack("<h", digits[14:16])[0]
        self.trim_values["dig_p6"] = struct.unpack("<h", digits[16:18])[0]
        self.trim_values["dig_p7"] = struct.unpack("<h", digits[18:20])[0]
        self.trim_values["dig_p8"] = struct.unpack("<h", digits[20:22])[0]
        self.trim_values["dig_p9"] = struct.unpack("<h", digits[22:24])[0]
        
        self._log("Trim values read")
    
    def _get_pressure_temp_raw(self):
        # Burst-reading the raw temp/pressure bytes
        data = self.bmp280.readfrom_mem(self.bmpaddress, self.registers["pressure"], 6)
        
        # Decoding temp/pressure bytearrays - the data comes packaged in format MSB - LSB - XLSB
        pressure_decoded = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_decoded = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        
        return pressure_decoded, temp_decoded
    
    def get_press_temp(self):
        """
        Returns pressure and temperature values from the BMP280 module as floats.
        
        Pressure in hPa, temperature in degrees celcius.
        
        No input parameters accepted
        """
        
        # Getting raw pressure/temp data
        press_raw, temp_raw = self._get_pressure_temp_raw()
        
        # Calculating the temperature (deg. C) using the trim values
        temp_var1 = (((temp_raw >> 3) - (self.trim_values["dig_t1"] << 1)) * self.trim_values["dig_t2"]) >> 11
        temp_var2 = (temp_raw >> 4) - self.trim_values["dig_t1"]
        temp_var2 = (((temp_var2 * temp_var2) >> 12) * self.trim_values["dig_t3"]) >> 14
        temp_fine = temp_var1 + temp_var2

        temp = ((temp_fine * 5) + 128) >> 8
        temp /= 100
        
        # Calculating the pressure (hPa) using the trim values
        press_var1 = temp_fine - 128000
        press_var2 = press_var1 * press_var1 * self.trim_values["dig_p6"]
        press_var2 += (press_var1 * self.trim_values["dig_p5"]) << 17
        press_var2 += self.trim_values["dig_p4"] << 35
        press_var1 = ((press_var1 * press_var1 * self.trim_values["dig_p3"]) >> 8) + ((press_var1 * self.trim_values["dig_p2"]) << 12)
        press_var1 = (((1<<47) + press_var1) * self.trim_values["dig_p1"]) >> 33
        
        if press_var1 == 0:
            press = 0
        else:
            press = ((((1048576 - press_raw) << 31) - press_var2) * 3125) // press_var1

            press_var1 = (self.trim_values["dig_p9"] * (press >> 13) * (press >> 13)) >> 25
            press_var2 = (self.trim_values["dig_p8"] * press) >> 19
            
            press = ((press + press_var1 + press_var2) >> 8) + (self.trim_values["dig_p7"] << 4)
            
            press /= 25600
        
        return press, temp
    
    def get_press_temp_alt(self):
        """
        Gets pressure and temperature data, then uses that to calculate the module's altitude relative to its starting location.
        Can also calculate absolute altitude if the initial altitude is passed in on driver initialisation.
        
        Returns pressure, temperature, altitude:
        Pressure in hPa
        Temperature in degrees celcius
        Altitude in meters
        
        No input parameters accepted
        """
        
        press, temp = self.get_press_temp()
        
        # Calculating altitude (in m) based on pressure using barometric equation
        alt = self.starting_gpsalt + (self.t0/0.0065)*(1-pow(press/self.p0, self.baro_equation_coefficient))
        
        return press, temp, alt
    
    def update_current_alt(self, alt):
        """
        Enables you to update the current barometer altitude, incase the altitude readings are drifting (e.g. due to weather patterns)
        """
        
        self.starting_gpsalt = alt
        self.p0, self.t0 = self.get_press_temp()
        
        self.t0 += 273.15

if __name__ == "__main__":
    import time
    module = BMP280(46, 3) # INPUT YOUR SCL/SDA PINS HERE
    while True:
        print(module.get_press_temp_alt())
        time.sleep(1)