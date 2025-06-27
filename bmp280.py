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
        self.setup()
        
        # Getting factory-programmed trim values from the BMP280's NVM
        self.get_trim_values()
    
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