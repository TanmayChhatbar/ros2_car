import lgpio
import time

# i2c_addr = 0x8 # bus address

SPI_CS = 16
SPI_SCL = 12
SPI_SDA = 15
SPI_ADO = 13
SPI_ADA = 14

h = lgpio.i2c_open(1, i2c_addr)
while True:
    try:

    except KeyboardInterrupt:
        break
