from gpiozero import RGBLED
from guizero import App, Slider, Text
from colorzero import Color
import RPi.GPIO as GPIO
import pi_servo_hat
import time

test = pi_servo_hat.PiServoHat()

test.restart()



red = 0
green = 0
blue = 0




def red_changed(value):
    global red
    red = int(value)
    test.move_servo_position(0, red)

def green_changed(value):
    global green
    green = int(value)
    test.move_servo_position(1, green)

def blue_changed(value):
    global blue
    blue = int(value)
    test.move_servo_position(2, blue)

app = App(title = 'Killer Robot (Jerald)', width = 500, height = 400, layout = 'grid')

Text (app, text = 'Left Track', grid = [0,0]).text_size = 30
Slider(app, command = red_changed, end = 120, width = 350, height = 50, grid = [1,0]).text_size = 30

Text (app, text = 'Right Track', grid = [0,1]).text_size = 30
Slider(app, command = green_changed, end = 120, width = 350, height = 50, grid = [1,1]).text_size = 30

Text (app, text = 'Blade', grid = [0,2]).text_size = 30
Slider(app, command = blue_changed, end = 120, width = 350, height = 50, grid = [1,2]).text_size = 30


app.display()