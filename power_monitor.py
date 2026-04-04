import board
import busio
from adafruit_bus_device.i2c_device import I2CDevice
import time
import glob

class INA237:
    def __init__(self):

        self.DEVICE_ADDRESS = 0x40

        # Registers
        self.REG_CONFIG      = 0x00
        self.REG_ADC_CONFIG  = 0x01
        self.REG_CALIBRATION = 0x02
        self.REG_DIAG_ALRT   = 0x0B

        # --- I2C setup ---
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sldevice = I2CDevice(i2c, self.DEVICE_ADDRESS)

    def read_register(self, reg, print_debug=False):
        """Read 16-bit value from register"""
        result = bytearray(2)

        with self.sldevice:
            self.sldevice.write(bytes([reg]))  # set register pointer
            self.sldevice.readinto(result)

        value = (result[0] << 8) | result[1]

        if print_debug:
            print(f"[READ ] Reg 0x{reg:02X} => 0x{value:04X} ({value:016b})")
        return value


    def modify_register(self, reg, mask, new_bits, print_debug=False):
        """
        Modify specific bits:
        - mask: bits to change
        - new_bits: new value (already aligned with mask)
        """
        current = self.read_register(reg)
        new_value = (current & ~mask) | (new_bits & mask)

        self.write_register(reg, new_value)

        if print_debug:
            print(f"[MOD  ] Mask 0x{mask:04X}, New bits 0x{new_bits:04X}")
        return new_value

    def begin(self):
        # --- INIT SEQUENCE ---

        # 1. Reset device
        self.write_register(self.REG_CONFIG, 0x8000)

        time.sleep(0.01)  # VERY important

        # 2. ADC configuration
        # Mode: continuous shunt + bus
        # Averaging: 16 samples
        # Conversion time: ~1 ms
        adc_config = (
            (0b1111 << 12) | #Continuous bus voltage, shunt voltage and temperature
            (0x05 << 9) |  # fast VBUS
            (0x05 << 6)  |  # fast shunt
            (0x05 << 3)  |  # slow temp
            (0b000) # no averaging
        )
        self.write_register(self.REG_ADC_CONFIG, 0xFB6A)

        max_current = 25  # A
        max_voltage = 25  # V
        max_power = max_current * max_voltage  # W
        min_current = 0  # A
        min_voltage = 15  # V
        r_shunt = 0.001    # Ω


        self.write_register(0x0C, int((max_current*r_shunt) / 5e-6)) # over current
        self.write_register(0x0D, int((min_current*r_shunt) / 5e-6)) # under current
        self.write_register(0x0E, int(max_voltage * 1000 / 3.125)) #Over voltage
        self.write_register(0x0F, int(min_voltage * 1000 / 3.125)) #Under voltage
        self.write_register(0x11, int(max_power * 256)) #Power over limit

        value = (
            (1 << 15) |  # ALATCH
            (0 << 14) |
            (0 << 13) |
            (0 << 12) 
        )
        self.write_register(self.REG_DIAG_ALRT, value)
        
    # --- Low-level functions ---
    def write_register(self, reg, value, print_debug=False):
        """Write 16-bit value to register"""
        msb = (value >> 8) & 0xFF
        lsb = value & 0xFF

        with self.sldevice:
            self.sldevice.write(bytes([reg, msb, lsb]))

        if print_debug:
            print(f"[WRITE] Reg 0x{reg:02X} <= 0x{value:04X} ({value:016b})")

    def check_alerts(self, system_print=False, print_debug=False):
        err = self.read_register(self.REG_DIAG_ALRT)

        MEMSTAT = not ((1 << 0) & err)
        P_OL = (1 << 2) & err
        BUS_UV = (1 << 3) & err
        BUS_OV = (1 << 4) & err
        SHUNT_UV = (1 << 5) & err
        SHUNT_OV = (1 << 6) & err
        TEMP_OL = (1 << 7) & err
        MATH_OF = (1 << 9) & err

        if MEMSTAT or P_OL or BUS_UV or BUS_OV or SHUNT_UV or SHUNT_OV or TEMP_OL or MATH_OF: alert = True
        else: alert = False

        message = f"Alerts: {"MEMSTAT " * bool(MEMSTAT)}{"P_OL " * bool(P_OL)}{"BUS_UV " * bool(BUS_UV)}{"BUS_OV " * bool(BUS_OV)}{"SHUNT_UV " * bool(SHUNT_UV)}{"SHUNT_OV " * bool(SHUNT_OV)}{"TEMP_OL " * bool(TEMP_OL)}{"MATH_OF " * bool(MATH_OF)}\n"

        if system_print and alert:
            for tty in glob.glob("/dev/pts/*"):
                try:
                    with open(tty, "w") as f:
                        f.write(message)
                    print(f"Sent to {tty}")
                except Exception as e:
                    print(f"Failed on {tty}: {e}")

        if (alert and print_debug):
            print(message)
        elif (print_debug):
            print("No alerts.")
        return {
            "MEMSTAT": bool(MEMSTAT),
            "P_OL": bool(P_OL),
            "BUS_UV": bool(BUS_UV),
            "BUS_OV": bool(BUS_OV),
            "SHUNT_UV": bool(SHUNT_UV),
            "SHUNT_OV": bool(SHUNT_OV),
            "TEMP_OL": bool(TEMP_OL),
            "MATH_OF": bool(MATH_OF)
        }


# --- Example usage ---
if __name__ == "__main__":

    ina237 = INA237()
    ina237.begin()

    while True:
        print()
        print(f"Bus voltage : {3.125 * ina237.read_register(0x05) / 1000} V")
        vshunt = 5e-6 * ina237.read_register(0x04)
        print(f"Shunt voltage: {round(vshunt * 1000 * 1000, 3)} mV")
        print(f"Current      : {round(vshunt / 0.001, 3)} A")
        ina237.check_alerts(True)
        time.sleep(1)