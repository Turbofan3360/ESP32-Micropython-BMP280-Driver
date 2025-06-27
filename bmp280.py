from machine import SoftI2C, Pin

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
        
        # Waking the sensor up
        self.bmp280.writeto_mem(self.bmpaddress, self.registers["measurement_ctrl"], bytes([0x03]))
    
    def setup(self):
        