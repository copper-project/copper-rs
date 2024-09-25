#!/bin/bash

for pwm in /sys/class/pwm/pwmchip0/pwm*/; do
    chmod 0660 "${pwm}enable"
    chmod 0660 "${pwm}duty_cycle"
    chmod 0660 "${pwm}period"
    chmod 0660 "${pwm}polarity"
    chown root:wheel "${pwm}enable"
    chown root:wheel "${pwm}duty_cycle"
    chown root:wheel "${pwm}period"
    chown root:wheel "${pwm}polarity"
done

chmod 0660 /sys/class/pwm/pwmchip0/unexport
chown root:wheel /sys/class/pwm/pwmchip0/unexport
