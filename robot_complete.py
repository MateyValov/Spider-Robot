#importing stuff
import math
import smbus

import cv2
import numpy
import threading

import RPi.GPIO as GPIO
from time import sleep

#setting up pins
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

working = True
warning = False
moving = True

redPin = 17
bluePin = 22
greenPin = 27
buzzPin = 26
laserPin = 18

GPIO.setup(redPin, GPIO.OUT)
GPIO.setup(bluePin, GPIO.OUT)
GPIO.setup(greenPin, GPIO.OUT)
GPIO.setup(buzzPin, GPIO.OUT)
GPIO.setup(laserPin, GPIO.OUT)

#RGB diode and Buzzer related functions
def off():
    global warning, moving, working
    GPIO.output(redPin, GPIO.LOW)
    GPIO.output(bluePin, GPIO.LOW)
    GPIO.output(greenPin, GPIO.LOW)
    GPIO.output(laserPin, GPIO.LOW)
    warning = False
    moving = False
    working = False

def red():
    global warning, moving
    GPIO.output(redPin, GPIO.HIGH)
    GPIO.output(bluePin, GPIO.LOW)
    GPIO.output(greenPin, GPIO.LOW)
    GPIO.output(laserPin, GPIO.HIGH)
    warning = True
    moving = False
    
def blue():
    global warning, moving, working
    GPIO.output(redPin, GPIO.LOW)
    GPIO.output(bluePin, GPIO.HIGH)
    GPIO.output(greenPin, GPIO.LOW)
    GPIO.output(laserPin, GPIO.LOW)
    warning = False
    moving = True
    working = True
    
def buzzer():
    global warning
    while True and working == True:    
        if warning == True:
            for i in range(150):
                GPIO.output(buzzPin, GPIO.HIGH)
                sleep(0.0003)
                GPIO.output(buzzPin, GPIO.LOW)
                sleep(0.0003)
            
            sleep(0.1)

#=========================
#Code for the Servo driver
#=========================

class PCA9685:

  # Registers/etc.
  __SUBADR1            = 0x02
  __SUBADR2            = 0x03
  __SUBADR3            = 0x04
  __MODE1              = 0x00
  __PRESCALE           = 0xFE
  __LED0_ON_L          = 0x06
  __LED0_ON_H          = 0x07
  __LED0_OFF_L         = 0x08
  __LED0_OFF_H         = 0x09
  __ALLLED_ON_L        = 0xFA
  __ALLLED_ON_H        = 0xFB
  __ALLLED_OFF_L       = 0xFC
  __ALLLED_OFF_H       = 0xFD
  
  def __init__(self, address=0x40, debug=False):
    self.bus = smbus.SMBus(1)
    self.address = address
    self.debug = debug
    if (self.debug):
      print("Reseting PCA9685")
    self.write(self.__MODE1, 0x00)

  def write(self, reg, value):
    "Writes an 8-bit value to the specified register/address"
    self.bus.write_byte_data(self.address, reg, value)
    if (self.debug):
      print("I2C: Write 0x%02X to register 0x%02X" % (value, reg))

  def read(self, reg):
    "Read an unsigned byte from the I2C device"
    result = self.bus.read_byte_data(self.address, reg)
    if (self.debug):
      print("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, result & 0xFF, reg))
    return result

  def setPWMFreq(self, freq):
    "Sets the PWM frequency"
    prescaleval = 25000000.0    # 25MHz
    prescaleval /= 4096.0       # 12-bit
    prescaleval /= float(freq)
    prescaleval -= 1.0
    if (self.debug):
      print("Setting PWM frequency to %d Hz" % freq)
      print("Estimated pre-scale: %d" % prescaleval)
    prescale = math.floor(prescaleval + 0.5)
    if (self.debug):
      print("Final pre-scale: %d" % prescale)

    oldmode = self.read(self.__MODE1);
    newmode = (oldmode & 0x7F) | 0x10        # sleep
    self.write(self.__MODE1, newmode)        # go to sleep
    self.write(self.__PRESCALE, int(math.floor(prescale)))
    self.write(self.__MODE1, oldmode)
    sleep(0.005)
    self.write(self.__MODE1, oldmode | 0x80)
    

  def setPWM(self, channel, on, off):
    "Sets a single PWM channel"
    self.write(self.__LED0_ON_L+4*channel, on & 0xFF)
    self.write(self.__LED0_ON_H+4*channel, on >> 8)
    self.write(self.__LED0_OFF_L+4*channel, off & 0xFF)
    self.write(self.__LED0_OFF_H+4*channel, off >> 8)
    if (self.debug):
      print("channel: %d  LED_ON: %d LED_OFF: %d" % (channel,on,off))

  def setServoPulse(self, channel, pulse):
    "Sets the Servo Pulse,The PWM frequency must be 50HZ"
    pulse = pulse*4096/20000        #PWM frequency is 50HZ,the period is 20000us
    self.setPWM(channel, 0, int(pulse))
    
#==================
#End of driver code
#==================

#robot starting position
def start():   
    pwm.setServoPulse(0, 1700)
    pwm.setServoPulse(4, 1300)
    pwm.setServoPulse(8, 1300)
    pwm.setServoPulse(12, 1700)
       
    sleep(1)
      
    pwm.setServoPulse(2, 1850)
    pwm.setServoPulse(6, 1800)
    pwm.setServoPulse(10, 1200)
    pwm.setServoPulse(14, 1150)
      
    sleep(1)
      
    pwm.setServoPulse(1, 1500)
    pwm.setServoPulse(5, 1450)
    pwm.setServoPulse(9, 1550)
    pwm.setServoPulse(13, 1575)
    
    sleep(1)

#robot rotates a little to the left (needs to repeat 5 times for 90 degree turn)
def turn():
    for i in range(0,201,50):  
      pwm.setServoPulse(1,1500 + i) 
      sleep(0.02)
    sleep(0.1)
    pwm.setServoPulse(4, 1125)
    pwm.setServoPulse(0, 1875)
    sleep(0.5)
    pwm.setServoPulse(1, 1500)
    sleep(0.5)
    
    for i in range(0,201,50):  
      pwm.setServoPulse(5,1450 + i) 
      sleep(0.02)
    sleep(0.1)
    pwm.setServoPulse(4, 1300)
    pwm.setServoPulse(12, 1525)
    sleep(0.5)
    pwm.setServoPulse(5, 1450)
    sleep(0.5)
    
    for i in range(0,201,50):  
      pwm.setServoPulse(13,1575 - i) 
      sleep(0.02)
    sleep(0.1)
    pwm.setServoPulse(12, 1700)
    pwm.setServoPulse(8, 1125)
    sleep(0.5)
    pwm.setServoPulse(13, 1575)
    sleep(0.5)
    
    for i in range(0,201,50):  
      pwm.setServoPulse(9,1550 - i) 
      sleep(0.02)
    sleep(0.1)
    pwm.setServoPulse(0, 1525)
    pwm.setServoPulse(8, 1475)
    sleep(0.5)
    pwm.setServoPulse(9, 1550)
    sleep(0.5)

#robot makes a step forward
def step():
    for i in range(0,201,50):  
      pwm.setServoPulse(1,1500 + i) 
      sleep(0.02)
    sleep(0.1)
    pwm.setServoPulse(0, 1525)
    sleep(0.5)
    pwm.setServoPulse(1, 1500)
    sleep(0.5)
    
    for i in range(0,201,50):  
      pwm.setServoPulse(13,1575 - i) 
      sleep(0.02)
    sleep(0.1)
    pwm.setServoPulse(4, 1475)
    sleep(0.02)
    pwm.setServoPulse(8, 1225)
    sleep(0.02)
    pwm.setServoPulse(12, 1975)
    sleep(0.5)
    pwm.setServoPulse(13, 1575)
    sleep(0.5)
    
    for i in range(0,201,50):  
      pwm.setServoPulse(9,1550 - i) 
      sleep(0.02)
    sleep(0.1)
    pwm.setServoPulse(8, 1475)
    sleep(0.5)
    pwm.setServoPulse(9, 1550)
    sleep(0.5)
    
    for i in range(0,201,50):  
      pwm.setServoPulse(5,1450 + i) 
      sleep(0.02)
    sleep(0.1)
    pwm.setServoPulse(0, 1975)
    sleep(0.02)
    pwm.setServoPulse(12, 1525)
    sleep(0.02)
    pwm.setServoPulse(4, 1125)
    sleep(0.5)
    pwm.setServoPulse(5, 1450)
    sleep(0.5)

#the robot moves along a predetermined course
def movement():
    global moving
    while True and working == True:
        while moving == True:
            
            for i in range(10):
                if moving == False:
                    break
                step()
            
            if moving == True:
                sleep(1)
                pwm.setServoPulse(11, 500)
                sleep(0.2)
                for i in range(500,2501,10):
                    pwm.setServoPulse(11, i)
                    sleep(0.02)
                sleep(0.2)
                pwm.setServoPulse(11, 1500)
            
            for i in range(10):
                if moving == False:
                    break
                turn()
 
#main
if __name__=='__main__':
    pwm = PCA9685(0x40, debug=False)
    pwm.setPWMFreq(50)
    
    #moving camera to starting position
    pwm.setServoPulse(11, 1500)
    pwm.setServoPulse(15, 1500)
    
    #camera angle
    angleX = 1500
    angleY = 1500
    
    #setting up camera
    cam = cv2.VideoCapture(0)
    _, frame = cam.read()
    rows, cols, _ = frame.shape
    
    #setting up screen center
    centerX = int(cols / 2)
    centerY = int(rows / 2)

    absCenterX = centerX
    absCenterY = centerY
    
    #target color range
    bottomB = numpy.array([161, 155, 84])
    upperB = numpy.array([179, 255, 255])
    
    #starting the robot
    blue()
    sleep(0.3)
    off()
    start()
    blue()
    GPIO.output(buzzPin, GPIO.HIGH)
    sleep(0.5)
    GPIO.output(buzzPin, GPIO.LOW)
    sleep(0.5)
    
    #start threads
    walkThread = threading.Thread(target=movement)
    walkThread.start()
    
    buzzerThread = threading.Thread(target=buzzer)
    buzzerThread.start()
    
    #camera function
    while True:
        _, frame = cam.read()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        target = cv2.inRange(hsv_frame, bottomB, upperB)
        
        #looking for red stuff
        objects, _ = cv2.findContours(target, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        objects = sorted(objects, key = lambda x:cv2.contourArea(x), reverse=True)
        
        #when the largest oblect is found 
        for obj in objects:
            (x, y, w, h) = cv2.boundingRect(obj)
            
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 0), 2)
            centerX = int((x + (x + w)) / 2)
            centerY = int((y + (y + h)) / 2)
            
            #if the oblect is large enough
            if w > 30 or h > 30:
                #calculating camera rotation
                if centerX < (absCenterX - 40):
                    angleX += 50;
                elif centerX > (absCenterX + 40):
                    angleX -= 50;
                
                if centerY < (absCenterY - 40):
                    angleY += 50;
                elif centerY > (absCenterY + 40):
                    angleY -= 50;
                
                if angleX < 500:
                    angleX = 500
                elif angleX > 2500:
                    angleX = 2500
                
                if angleY < 500:
                    angleY = 500
                elif angleY > 2500:
                    angleY = 2500
        
                pwm.setServoPulse(11, angleX)
                pwm.setServoPulse(15, angleY)
        
                red()
            
            #if there isn't a large enough object reset camera
            else:
                angleX = 1500
                angleY = 1500
                
                pwm.setServoPulse(11, angleX)
                pwm.setServoPulse(15, angleY)
                blue()
            
            break;
        
        cv2.line(frame, (centerX, 0), (centerX, 480), (255, 255, 0), 2)
        cv2.line(frame, (0, centerY), (640, centerY), (255, 255, 0), 2)
        
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1)
        
        if key == 27:
            break
        
    #reset camera position and turn it off   
    pwm.setServoPulse(11, 1500)
    pwm.setServoPulse(15, 1500)
    cam.release()
    cv2.destroyAllWindows()
    
    off()
    
    walkThread.join()
    buzzerThread.join()
    start()
    

        
