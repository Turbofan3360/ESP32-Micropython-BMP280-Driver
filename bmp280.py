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
        
        # Setting up sensor
        self.setup()
    
    def setup(self):
        # Waking sensor up and setting oversampling rates for pressure/temperature measurement
        self.bmp280.writeto_mem(self.bmpaddress, self.registers["measurement_ctrl"], bytes([0x57]))
        
        # Setting standby time length, IIR filter
        self.bmp280.writeto_mem(self.bmpaddress, self.registers["config"], bytes([0x10]))