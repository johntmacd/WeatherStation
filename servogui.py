from gpiozero import RGBLED
from guizero import App, Slider, Text
from colorzero import Color
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(32, GPIO.OUT)
GPIO.setup(3, GPIO.OUT)
pwm_right_track = GPIO.PWM(11, 500)
pwm_left_track = GPIO.PWM(32, 670)
pwm_blade = GPIO.PWM(3, 670)

red = 0
green = 0
blue = 0
pwm_right_track.start(0)
pwm_left_track.start(0)
pwm_blade.start(0)



def red_changed(value):
    global red
    red = int(value)
    pwm_right_track.ChangeDutyCycle(red)

def green_changed(value):
    global green
    green = int(value)
    pwm_left_track.ChangeDutyCycle(green)

def blue_changed(value):
    global blue
    blue = int(value)
    pwm_blade.ChangeDutyCycle(green)

app = App(title = 'Killer Robot (Jerald)', width = 500, height = 400, layout = 'grid')

Text (app, text = 'Left Track', grid = [0,0]).text_size = 30
Slider(app, command = red_changed, end = 100, width = 350, height = 50, grid = [1,0]).text_size = 30

Text (app, text = 'Right Track', grid = [0,1]).text_size = 30
Slider(app, command = green_changed, end = 100, width = 350, height = 50, grid = [1,1]).text_size = 30

Text (app, text = 'Blade', grid = [0,2]).text_size = 30
Slider(app, command = blue_changed, end = 100, width = 350, height = 50, grid = [1,2]).text_size = 30


app.display()