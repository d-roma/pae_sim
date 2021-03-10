# !/usr/bin/python3
# -*- coding: utf-8 -*-
"""
AX12 emulator
"""

from enum import Enum
from collections import OrderedDict


class ax_instruction(Enum):
    IDLE = 0x00
    PING = 0x01
    READ = 0x02
    WRITE = 0x03
    REG_WRITE = 0x04
    ACTION = 0x05
    RESET = 0x06
    SYNC_WRITE = 0x83
    END = 0xFF


AX12_memory = {
    # Memory position: (Reset value, description)
    0x00: (0x0C, "Model Number(L)"),
    0x01: (0x00, "Model Number(H)"),
    0x02: (0x01, "Firmware Version"),
    0x03: (0, "ID",),
    0x04: (0x01, "Baud Rate",),
    0x05: (0xFA, "Return Delay Time",),
    0x06: (0, "CW Angle Limit(L)",),
    0x07: (0, "CW Angle Limit(H)",),
    0x08: (0xFF, "CCW Angle Limit(L)",),
    0x09: (0x03, "CCW Angle Limit(H)",),
    0x0A: (0, "Reserved",),
    0x0B: (0x55, "High Temp. Limit",),
    0x0C: (0x3C, "Low Voltage Limit",),
    0x0D: (0xBE, "High Voltage Limit",),
    0x0E: (0xFF, "Max Torque(L)",),
    0x0F: (0x03, "Max Torque(H)",),
    0x10: (0x02, "Status Return Level",),
    0x11: (0x04, "Alarm LED",),
    0x12: (0x04, "Alarm Shoutdown",),
    0x13: (0, "Reserved",),
    0x14: (0, "Down Calibration(L)",),
    0x15: (0, "Down Calibration(H)",),
    0x16: (0, "Up Calibration(L)",),
    0x17: (0, "Up Calibration(H)",),
    0x18: (0, "Torque Enable",),
    0x19: (0, "LED",),
    0x1A: (0x7F, "Left IR",),
    0x1B: (0x7F, "Center IR",),
    0x1C: (0x00, "Right IR",),
    0x1D: (0, "CCW Compliance Slope",),
    0x1E: (0, "Goal Position(L)",),
    0x1F: (0, "Goal Position(H)",),
    0x20: (0, "Moving Speed(L)",),
    0x21: (0, "Moving Speed(H)",),
    0x22: (0x0E, "Torque Limit(L)",),
    0x23: (0x0F, "Torque Limit(H)",),
    0x24: (0, "Present Position(L)",),
    0x25: (0, "Present Position(H)",),
    0x26: (0, "Present Speed(L)",),
    0x27: (0, "Present Speed(H)",),
    0x28: (0, "Present Load(L)",),
    0x29: (0, "Present Load(H)",),
    0x2A: (0, "Present Voltage",),
    0x2B: (0, "Present Temperature",),
    0x2C: (0, "Registered Instruction",),
    0x2D: (0, "ADC_value",),
    0x2E: (0, "Moving",),
    0x2F: (0, "Lock",),
    0x30: (0, "Punch(L)",),
    0x31: (0, "Punch(H)"),
}


class AX(OrderedDict):
    def __init__(self):
        self._reset()

    def _reset(self):
        for key in AX12_memory:
            self[key] = AX12_memory[key][0]
