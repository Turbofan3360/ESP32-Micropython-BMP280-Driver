from machine import SoftI2C, Pin
import struct

class bmp280:
    def __init__(self, scl, sda, init_gps_alt=0):
        self.bmp280 = SoftI2C(scl=Pin(scl), sda=Pin(sda), freq=400000)
        
        self.bmpaddress = 0x76
        
        self.registers = {
            "id" : 0xD0,
            "reset" : 0xE0,
            "status" : 0xF3,
            
            "measurement_ctrl" : 0xF4,
            "config" : 0xF5,
            
            "pressure" : 0xF7, # Goes on to 0xF9, i.e. 3 bytes
            "temp" : 0xFA, # Goes on to 0xFC, i.e. 3 bytes
            }
        
        self.trim_values = {}
        
        # Setting up sensor
        self._setup()
        
        # Getting factory-programmed trim values from the BMP280's NVM
        self._get_trim_values()
    
    def _setup(self):
        # Waking sensor up and setting oversampling rates for pressure/temperature measurement
        self.bmp280.writeto_mem(self.bmpaddress, self.registers["measurement_ctrl"], bytes([0x57]))
        
        # Setting standby time length, IIR filter
        self.bmp280.writeto_mem(self.bmpaddress, self.registers["config"], bytes([0x10]))
    
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
    
    def _get_pressure_temp_raw(self):
        # Burst-reading the raw temp/pressure bytes
        pressure = self.bmp280.readfrom_mem(self.bmpaddress, self.registers["pressure"], 3)
        temp = self.bmp280.readfrom_mem(self.bmpaddress, self.registers["temp"], 3)
        
        # Decoding temp/pressure bytearrays - the data comes packaged in format MSB - LSB - XLSB
        pressure_decoded = (pressure[0] << 12) | (pressure[1] << 4) | (pressure[2] >> 4)
        temp_decoded = (temp[0] << 12) | (temp[1] << 4) | (temp[2] >> 4)
        
        return pressure_decoded, temp_decoded
    
    def get_press_temp(self):
        # Getting raw pressure/temp data
        press_raw, temp_raw = self._get_pressure_temp_raw()
        
        # Calculating the temperature (deg. C) using the trim values
        temp_var1 = ((temp_raw >> 3) - (self.trim_values["dig_t1"] << 1)) * (self.trim_values["dig_t2"] >> 11)
        temp_var2 = ((temp_raw >> 4) - self.trim_values["dig_t1"]) * (((temp_raw >> 4) - self.trim_values["dig_t1"]) >> 12) * (self.trim_values["dig_t3"] >> 14)
        temp_fine = temp_var1 + temp_var2
        
        temp = ((temp_fine * 5) + 128) >> 8
        temp /= 100
        
        # Calculating the pressure (Pa) using the trim values
        press_var1 = temp_fine - 128000
        press_var2 = press_var1 * press_var1 * self.trim_values["dig_p6"]
        press_var2 += self.trim_values["dig_p5"] << 17
        press_var2 += self.trim_values["dig_p4"] << 35
        press_var1 = ((press_var1 * press_var2 * self.trim_values["dig_p3"]) >> 8) + ((press_var1 * self.trim_values["dig_p2"]) << 12)
        press_var1 = (((1<<47) + press_var1) * self.trim_values["dig_p1"]) >> 33
        
        if press_var1 == 0:
            press = 0
        else:
            press = (1048576 - press_raw - (press_var2 >> 12)) * 3125
            
            if press < 2147483648:
                press = (press << 1)/press_var1
            else:
                press = (press/press_var1) * 2
            
            press_var1 = (self.trim_values["dig_p9"] * (press >> 13) * (press >> 13)) >> 25
            press_var2 = (self.trim_values["dig_p8"] * press) >> 19
            
            press = ((press + press_var1 + press_var2) >> 8) + (self.trim_values["dig_p7"] << 4)
            
            press /= 256
        
        return press, temp