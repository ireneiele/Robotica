from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds

# TODO: Add code here

# Collego i Large Motor

lm1 = LargeMotor(OUTPUT_A)
lm1.reset
lm1.on_for_rotation(SpeedPercent(75), 5) # è una funzione della libreria, prende un large motor e lo fa muovere
                                         # per 5 rotazioni complete del motore ad una velocità del 75% della 
                                         # velocità massima
# lm2 = LargeMotor(OUTPUT_B)
#lm2.reset
#lm2.on_for_rotation(SpeedPercent(75), 5)
#lm3 = LargeMotor(OUTPUT_C)
#lm3.reset
#lm3.on_for_rotation(SpeedPercent(75), 5)

#mm1 = MediumMotor(OUTPUT_D)
#mm1.on_for_rotation(SpeedPercent(60),5)
