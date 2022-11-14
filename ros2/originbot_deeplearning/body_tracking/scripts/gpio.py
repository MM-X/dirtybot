#!/usr/bin/env python3
import argparse

import Hobot.GPIO as GPIO
import time


# 定义使用的GPIO通道为36
output_pin = 36 # BOARD 编码 36

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('pin', help='int', type=int)
    parser.add_argument('val', help='int', type=int)
    args = parser.parse_args()
    return args

def main():
    args = parse_args()
    output_pin = args.pin
    output_val = args.val
    # print("output_pin is ", output_pin)
    # print("output_val is ", output_val)
    
    # 设置管脚编码模式为硬件编号 BOARD
    GPIO.setmode(GPIO.BOARD)
    # 设置为输出模式，并且初始化为高电平
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
    # 记录当前管脚状态
    curr_value = GPIO.HIGH

    if output_val == 0:
        curr_value ^= GPIO.HIGH
    GPIO.output(output_pin, curr_value)

if __name__=='__main__':
    main()
