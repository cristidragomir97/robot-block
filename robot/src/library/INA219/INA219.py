
import time
import board
from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219
from sensor_msgs.msg import BatteryState

class _INA219():
    def __init__(self, address):
        i2c_bus = board.I2C()
        self.ina219 = INA219(i2c_bus)
        self.ina219.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.ina219.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.ina219.bus_voltage_range = BusVoltageRange.RANGE_16V # optional : change voltage range to 16V

    def read(self):
        msg = BatteryState()
        msg.voltage = self.ina219.bus_voltage
        msg.current = self.ina219.current / 1000
        return msg



