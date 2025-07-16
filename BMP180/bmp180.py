import time
from micropython import const

# BMP180 default device address.
BMP180_I2CADDR = const(0x77)

BMP180_ULTRALOWPOWER = const(0)
BMP180_STANDARD = const(1)
BMP180_HIGHRES = const(2)
BMP180_ULTRAHIGHRES = const(3)

# BMP180 Registers
BMP180_CAL_AC1 = const(0xAA)  
BMP180_CONTROL = const(0xF4)
BMP180_TEMPDATA = const(0xF6)
BMP180_PRESSUREDATA = const(0xF6)
BMP180_READTEMPCMD = const(0x2E)
BMP180_READPRESSURECMD = const(0x34)

class BMP180:
    def __init__(self, i2c, addr=BMP180_I2CADDR):
        self.i2c = i2c
        self.addr = addr
        self.oversample_sett = BMP180_STANDARD
        self.sea_level_pressure = 101325

        self._read_calibration_data()

    def _read_signed(self, addr):
        hi, lo = self.i2c.readfrom_mem(self.addr, addr, 2)
        result = (hi << 8) + lo
        if result >= 32768:
            result = result - 65536
        return result

    def _read_unsigned(self, addr):
        hi, lo = self.i2c.readfrom_mem(self.addr, addr, 2)
        return (hi << 8) + lo

    def _read_calibration_data(self):
        self.ac1 = self._read_signed(BMP180_CAL_AC1)
        self.ac2 = self._read_signed(BMP180_CAL_AC1 + 2)
        self.ac3 = self._read_signed(BMP180_CAL_AC1 + 4)
        self.ac4 = self._read_unsigned(BMP180_CAL_AC1 + 6)
        self.ac5 = self._read_unsigned(BMP180_CAL_AC1 + 8)
        self.ac6 = self._read_unsigned(BMP180_CAL_AC1 + 10)
        self.b1 = self._read_signed(BMP180_CAL_AC1 + 12)
        self.b2 = self._read_signed(BMP180_CAL_AC1 + 14)
        self.mb = self._read_signed(BMP180_CAL_AC1 + 16)
        self.mc = self._read_signed(BMP180_CAL_AC1 + 18)
        self.md = self._read_signed(BMP180_CAL_AC1 + 20)

    def _read_raw_temp(self):
        self.i2c.writeto_mem(self.addr, BMP180_CONTROL, bytes([BMP180_READTEMPCMD]))
        time.sleep_ms(5)
        return self._read_unsigned(BMP180_TEMPDATA)

    def _read_raw_pressure(self):
        self.i2c.writeto_mem(self.addr, BMP180_CONTROL,
                             bytes([BMP180_READPRESSURECMD + (self.oversample_sett << 6)]))
        time.sleep_ms(2 + (3 << self.oversample_sett))
        msb = self.i2c.readfrom_mem(self.addr, BMP180_PRESSUREDATA, 3)
        raw = ((msb[0] << 16) + (msb[1] << 8) + msb[2]) >> (8 - self.oversample_sett)
        return raw

    def measure(self):
        UT = self._read_raw_temp()
        UP = self._read_raw_pressure()

        X1 = ((UT - self.ac6) * self.ac5) >> 15
        X2 = (self.mc << 11) // (X1 + self.md)
        B5 = X1 + X2
        self._B5 = B5

        B6 = B5 - 4000
        X1 = (self.b2 * ((B6 * B6) >> 12)) >> 11
        X2 = (self.ac2 * B6) >> 11
        X3 = X1 + X2
        B3 = (((self.ac1 * 4 + X3) << self.oversample_sett) + 2) >> 2

        X1 = (self.ac3 * B6) >> 13
        X2 = (self.b1 * ((B6 * B6) >> 12)) >> 16
        X3 = ((X1 + X2) + 2) >> 2
        B4 = (self.ac4 * (X3 + 32768)) >> 15
        B7 = (UP - B3) * (50000 >> self.oversample_sett)

        if B7 < 0x80000000:
            p = (B7 << 1) // B4
        else:
            p = (B7 // B4) << 1

        X1 = (p >> 8) * (p >> 8)
        X1 = (X1 * 3038) >> 16
        X2 = (-7357 * p) >> 16
        self._pressure = p + ((X1 + X2 + 3791) >> 4)

        self._temperature = ((B5 + 8) >> 4) / 10.0

    @property
    def temperature(self):
        return self._temperature

    @property
    def pressure(self):
        return float(self._pressure)

    @property
    def altitude(self):
        # Calculate altitude in meters based on the sealevel-pressure
        return 44330.0 * (1.0 - pow(self._pressure / self.sea_level_pressure, 0.1903))
