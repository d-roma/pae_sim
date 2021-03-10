# !/usr/bin/python3
# -*- coding: utf-8 -*-
"""
AX12 emulator
"""

from enum import Enum
from collections import OrderedDict


class AX_instruction(Enum):
    IDLE = 0x00
    PING = 0x01
    READ = 0x02
    WRITE = 0x03
    REG_WRITE = 0x04
    ACTION = 0x05
    RESET = 0x06
    SYNC_WRITE = 0x83
    END = 0xFF

class AX_registers(Enum):
    MODEL_NUMBER_L = 0
    MODEL_NUMBER_H = 1
    FIRMWARE_VERSION = 2
    ID = 3
    BAUD_RATE = 0x04
    RET_DELAY_TIME = 0x05
    CW_ANGLE_LIMIT_L = 0x06
    CW_ANGLE_LIMIT_H = 0x07
    CCW_ANGLE_LIMIT_L = 0x08
    CCW_ANGLE_LIMIT_H = 0x09
    RESERVED__0x0A = 0x0A
    HIGH_TEMP_LIMIT = 0x0B
    LOW_VOLTAGE_LIMIT = 0x0C
    HIGH_VOLTAGE_LIMIT = 0x0D
    MAX_TORQUE_L = 0x0E
    MAX_TORQUE_H = 0x0F
    STATUS_RET = 0x10
    ALARM_LED = 0x11
    ALARM_SHUTDWN = 0x12
    RESERVED__0x13 = 0x13
    DWN_CAL_L = 0x14
    DWN_CAL_H = 0x15
    UP_CAL_L = 0x16
    UP_CAL_H = 0x17
    TORQUE_ENABLED = 0x18
    LED = 0x19
    IR_LEFT = 0x1A
    IR_CENTER = 0x1B
    IR_RIGHT = 0x1C
    CCW_COMP_SLOPE = 0x1D
    GOAL_POS_L = 0x1E
    GOAL_POS_H = 0x1F
    GOAL_SPEED_L = 0x20
    GOAL_SPEED_H = 0x21
    TORQUE_LIMIT_L = 0x22
    TORQUE_LIMIT_H = 0x23
    PRESENT_POS_L = 0x24
    PRESENT_POS_H = 0x25
    PRESENT_SPEED_L = 0x26
    PRESENT_SPEED_H = 0x27
    PRESENT_LOAD_L = 0x28
    PRESENT_LOAD_H = 0x29
    PRESENT_VOLTAGE = 0x2A
    PRESENT_TEMP = 0x2B
    REGISTERED_INSTR = 0x2C
    ADC_VALUE = 0x2D
    MOVING = 0x2E
    LOCK = 0x2F
    PUNCH_L = 0x30
    PUNCH_H = 0x31


AX12_reset_memory = {
    # Memory position: (Reset value, description)
    AX_registers.MODEL_NUMBER_L: (0x0C, "Model Number(L)"),
    AX_registers.MODEL_NUMBER_H: (0x00, "Model Number(H)"),
    AX_registers.FIRMWARE_VERSION: (0x01, "Firmware Version"),
    AX_registers.ID: (0, "ID",),
    AX_registers.BAUD_RATE: (0x01, "Baud Rate",),
    AX_registers.RET_DELAY_TIME: (0xFA, "Return Delay Time",),
    AX_registers.CW_ANGLE_LIMIT_L: (0, "CW Angle Limit(L)",),
    AX_registers.CW_ANGLE_LIMIT_H: (0, "CW Angle Limit(H)",),
    AX_registers.CCW_ANGLE_LIMIT_L: (0xFF, "CCW Angle Limit(L)",),
    AX_registers.CCW_ANGLE_LIMIT_H: (0x03, "CCW Angle Limit(H)",),
    AX_registers.RESERVED__0x0A: (0, "Reserved",),
    AX_registers.HIGH_TEMP_LIMIT: (0x55, "High Temp. Limit",),
    AX_registers.LOW_VOLTAGE_LIMIT: (0x3C, "Low Voltage Limit",),
    AX_registers.HIGH_VOLTAGE_LIMIT: (0xBE, "High Voltage Limit",),
    AX_registers.MAX_TORQUE_L: (0xFF, "Max Torque(L)",),
    AX_registers.MAX_TORQUE_H: (0x03, "Max Torque(H)",),
    AX_registers.STATUS_RET: (0x02, "Status Return Level",),
    AX_registers.ALARM_LED: (0x04, "Alarm LED",),
    AX_registers.ALARM_SHUTDWN: (0x04, "Alarm Shoutdown",),
    AX_registers.RESERVED__0x13: (0, "Reserved",),
    AX_registers.DWN_CAL_L: (0, "Down Calibration(L)",),
    AX_registers.DWN_CAL_H: (0, "Down Calibration(H)",),
    AX_registers.UP_CAL_L: (0, "Up Calibration(L)",),
    AX_registers.UP_CAL_H: (0, "Up Calibration(H)",),
    AX_registers.TORQUE_ENABLED: (0, "Torque Enable",),
    AX_registers.LED: (0, "LED",),
    AX_registers.IR_LEFT: (0x7F, "Left IR",),
    AX_registers.IR_CENTER: (0x7F, "Center IR",),
    AX_registers.IR_RIGHT: (0x00, "Right IR",),
    AX_registers.CCW_COMP_SLOPE: (0, "CCW Compliance Slope",),
    AX_registers.GOAL_POS_L: (0, "Goal Position(L)",),
    AX_registers.GOAL_POS_H: (0, "Goal Position(H)",),
    AX_registers.GOAL_SPEED_L: (0, "Moving Speed(L)",),
    AX_registers.GOAL_SPEED_H: (0, "Moving Speed(H)",),
    AX_registers.TORQUE_LIMIT_L: (0x0E, "Torque Limit(L)",),
    AX_registers.TORQUE_LIMIT_H: (0x0F, "Torque Limit(H)",),
    AX_registers.PRESENT_POS_L: (0, "Present Position(L)",),
    AX_registers.PRESENT_POS_H: (0, "Present Position(H)",),
    AX_registers.PRESENT_SPEED_L: (0, "Present Speed(L)",),
    AX_registers.PRESENT_SPEED_H: (0, "Present Speed(H)",),
    AX_registers.PRESENT_LOAD_L: (0, "Present Load(L)",),
    AX_registers.PRESENT_LOAD_H: (0, "Present Load(H)",),
    AX_registers.PRESENT_VOLTAGE: (0, "Present Voltage",),
    AX_registers.PRESENT_TEMP: (0, "Present Temperature",),
    AX_registers.REGISTERED_INSTR: (0, "Registered Instruction",),
    AX_registers.ADC_VALUE: (0, "ADC_value",),
    AX_registers.MOVING: (0, "Moving",),
    AX_registers.LOCK: (0, "Lock",),
    AX_registers.PUNCH_L: (0, "Punch(L)",),
    AX_registers.PUNCH_H: (0, "Punch(H)"),
}


class AX(OrderedDict):
    def __init__(self, id):
        self.id = id
        self._reset()

    def reset(self):
        for key, value in AX12_reset_memory.items():
            self[key] = value[0]
        self[AX_registers.ID] = self.id