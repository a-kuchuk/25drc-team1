import RPi.GPIO as IO
import pygame
from pygame.locals import *

pygame.init()
screen = pygame.display.set_mode((240, 240))
pygame.display.set_caption('Pi Car')

IO.setwarnings(False)
IO.setmode(IO.BCM)
IO.setup(19,IO.OUT)
IO.setup(26,IO.OUT)


t=IO.PWM(19,100)
s=IO.PWM(26,100)
w=pygame.key.get_pressed()[pygame.K_w]
throttle=14
steer=14.5
t.start(throttle)
s.start(steer)
stop = False
count=0

while (True):
	if stop == True:
		break
	for event in pygame.event.get():
		if event.type == pygame.KEYDOWN:
			if event.key == K_w:
				# accelerate
				throttle=14.8
			elif event.key == K_s:
				# deccelerate
				throttle=13.2
			elif event.key == K_a:
				# left
				steer=11
			elif event.key == K_d:
				# right
				steer=18
			elif event.key == K_q:
				# quit
				stop = True
		
