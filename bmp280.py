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
    
    def setup(self):
        # Waking sensor up and setting oversampling rates for pressure/temperature measurement
        self.bmp280.writeto_mem(self.bmpaddress, self.registers["measurement_ctrl"], bytes([0x57]))
        
        # Setting standby time length, IIR filter
        self.bmp280.writeto_mem(self.bmpaddress, self.registers["config"], bytes([0x10]))
    
    def get_trim_values(self):
        # Reading trim values from chip
        digit = self.bmp280.readfrom_mem(self.bmpaddress, 0x88, 2)
        self.trim_values["dig_t1"] = struct.unpack("<H", digit)
        
        digit = self.bmp280.readfrom_mem(self.bmpaddress, 0x8A, 2)
        self.trim_values["dig_t2"] = struct.unpack("<h", digit)
        
        digit = self.bmp280.readfrom_mem(self.bmpaddress, 0x8C, 2)
        self.trim_values["dig_t3"] = struct.unpack("<h", digit)
        
        digit = self.bmp280.readfrom_mem(self.bmpaddress, 0x8E, 2)
        self.trim_values["dig_p1"] = struct.unpack("<H", digit)
        
        digit = self.bmp280.readfrom_mem(self.bmpaddress, 0x90, 2)
        self.trim_values["dig_p2"] = struct.unpack("<h", digit)
        
        digit = self.bmp280.readfrom_mem(self.bmpaddress, 0x92, 2)
        self.trim_values["dig_p3"] = struct.unpack("<h", digit)
        
        digit = self.bmp280.readfrom_mem(self.bmpaddress, 0x94, 2)
        self.trim_values["dig_p4"] = struct.unpack("<h", digit)
        
        digit = self.bmp280.readfrom_mem(self.bmpaddress, 0x96, 2)
        self.trim_values["dig_p5"] = struct.unpack("<h", digit)
        
        digit = self.bmp280.readfrom_mem(self.bmpaddress, 0x98, 2)
        self.trim_values["dig_p6"] = struct.unpack("<h", digit)
        
        digit = self.bmp280.readfrom_mem(self.bmpaddress, 0x9A, 2)
        self.trim_values["dig_p7"] = struct.unpack("<h", digit)
        
        digit = self.bmp280.readfrom_mem(self.bmpaddress, 0x9C, 2)
        self.trim_values["dig_p8"] = struct.unpack("<h", digit)
        
        digit = self.bmp280.readfrom_mem(self.bmpaddress, 0x9E, 2)
        self.trim_values["dig_p9"] = struct.unpack("<h", digit)