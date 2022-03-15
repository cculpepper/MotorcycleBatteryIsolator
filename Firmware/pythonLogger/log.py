#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2022 Chris Culpepper <cculpepper1214@gmail.com>
#
# Distributed under terms of the MIT license.

"""

"""

from ina219 import INA219
from machine import I2C
from ina219 import DeviceRangeError
import time


i2c = machine.I2C(1, scl=machine.Pin(11), sda=machine.Pin(10))

shuntOhms = 0.001
MAX_EXECCTED_AMPS=80
ina = INA219(shuntOhms, i2c)
ina.configure(ina.RANGE_32V, ina.GAIN_1_40MV, bus_adc=ina.ADC_64SAMP, shunt_adc=ina.ADC_64SAMP)


try:
    voltage = ina.voltage()
    current = ina.current()
except DeviceRangeError as e:
    print(e)


#  ADC0 thermistor
#  ADC1 motorcycle v
#  ADC2 bttery v
thermADC = machine.ADC(0)
mcyV = machine.ADC(1)
batV = machine.ADC(2)

f = open("data.csv", "w")
for i in range(1000):
    s = ",".join([str(j) for j in [time.ticks_ms(), mcyV.read_u16(), batV.read_u16(), ina.voltage(), ina.current(), thermADC.read_u16()]])
    f.write(s)
    f.write("\r\n")
f.close()

print(s)
