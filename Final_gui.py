import tkinter as tk
from tkinter import Scale, HORIZONTAL
from picamera import PiCamera
from picamera.array import PiRGBArray
from PIL import Image, ImageTk
from time import *
import threading
import io
from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
import pi_servo_hat
test = pi_servo_hat.PiServoHat()

test.restart()

tilt = 0
pan = 0

def tilt_changed(value):
    global tilt
    tilt = int(value)
    test.move_servo_position(4, tilt)

def pan_changed(value):
    global pan
    pan = int(value)
    test.move_servo_position(3, pan)

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
camera = PiCamera()
camera.resolution = (320,240)
camera.framerate = 32

pan_servo = AngularServo(24, initial_angle = -90, min_pulse_width=0.0006, max_pulse_width=0.0024)
tilt_servo = AngularServo(18, initial_angle = -20, min_pulse_width=0.0006, max_pulse_width=0.0024,)

root = tk.Tk()
root.title("Robot Control")

tilt_slider = Scale(root, from_=-90, to=0, orient = HORIZONTAL, label = "Tilt",command = tilt_changed)
tilt_slider.pack()
pan_slider = Scale(root, from_=-90, to=180, orient = HORIZONTAL, label = "Pan",command = pan_changed)
pan_slider.pack()
motor_one_slider = Scale(root, from_=0, to=100, orient = HORIZONTAL, label = "Left Track",command = red_changed)
motor_one_slider.pack()
motor_two_slider = Scale(root, from_=0, to=100, orient = HORIZONTAL, label = "Right Track",command = blue_changed)
motor_two_slider.pack()
motor_three_slider = Scale(root, from_=0, to=100, orient = HORIZONTAL, label = "Murder Blade",command = green_changed)
motor_three_slider.pack()
video_label = tk.Label(root)
video_label.pack()



def update_image():
    stream = io.BytesIO()
    camera.capture(stream, format = 'jpeg', use_video_port = True)
    stream.seek(0)

    img = Image.open(stream)
    imgtk = ImageTk.PhotoImage(img)

    video_label.imgtk = imgtk
    video_label.config(image=imgtk)

    stream.close()

    root.after(100, update_image)

update_image()

root.mainloop()

camera.close()