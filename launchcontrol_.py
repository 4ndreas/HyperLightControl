#!/usr/bin/env python

# needs my patched version of launchpad.py
# download and install from 
# https://github.com/4ndreas/launchpad.py

import sys
import socket , pickle
import json
import pygame
import random
from pygame import time
import os
import ctypes
ctypes.windll.user32.SetProcessDPIAware()

from hyperlights.comm import inputData

try:
	import launchpad_py as launchpad
except ImportError:
	try:
		import launchpad
	except ImportError:
		sys.exit("error loading launchpad.py")

# create an instance
lp = lambda: None
lp2 = lambda: None

but = lambda: None
but2 = lambda: None

lp_but = []
lp_knob = []

lp2_but = []
lp2_knob = []

pygame.init()

text_color = (187,131,7)
button_color = (187,131,7)
button_highlight_color = (255,208,99)
button_back_color = (15,15,15)
button_highlight_back_color = (50,50,50)

black = (20,20,20)
white = (255,255,255)
red = (192,0,0)
bright_red = (255,0,0)

green = (0,128,0)
bright_green = (0,255,0)

frame_border = 5

infoObject = pygame.display.Info()
# pygame.display.set_mode((infoObject.current_w, infoObject.current_h))

display_width = infoObject.current_w
display_height = 300
display_x_offset = 0
taskleiste_h = 50
display_y_offset = infoObject.current_h - display_height - taskleiste_h
print("init screen w:%d h:%d, x:%d y:%d" % (display_width, display_height, display_x_offset, display_y_offset ))

os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (display_x_offset,display_y_offset) #-3584,00

gameDisplay = pygame.display.set_mode((display_width,display_height), pygame.NOFRAME)
pygame.display.set_caption('HyperLight GUI')
clock = pygame.time.Clock()

gameExit = False
exitFlag = [False, False]


def quitgame():
    gameExit = True
    # lp.Reset()
    # lp2.Reset()
    # lp.Close()
    # lp2.Close()
    pygame.quit()    
    quit()

transfer_port = 50505
transfer_ip = "127.0.0.1"
transfer_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# class inputData:
#     devID = 0.0
#     inpID = 0.0
#     inpVal = 0.0

# def openSocket():
    # transfer_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Controlsock.setblocking(0)
    # transfer_socket.bind(('',transfer_socket))   
    # transfer_socket.sendto(bytes(info, "utf-8"), (UDP_IP, UDP_InfoPort))


def sendData(pad, id, val):
    data_variable = inputData()
    data_variable.devID = pad
    data_variable.inpID = id
    data_variable.inpVal = val

    data_string = pickle.dumps(data_variable)
    # transfer_socket.send(data_string)
    # print(data_string)
    # transfer_socket.sendto(data_string, (transfer_ip, transfer_port))
    # transfer_socket.sendto(bytes(data_string, "utf-8"), (transfer_ip, transfer_port))
    # data_string  = json.dumps(data_variable)

    # rx_data_variable = pickle.loads(data_string)
    # print(str(rx_data_variable.inpID))

    # transfer_socket.sendto(bytes(str(data_string), "utf-8"), (transfer_ip, transfer_port))
    transfer_socket.sendto(data_string, (transfer_ip, transfer_port))


def mapInput(pad, input, but, knob):

    if input[0] == 9:
        but[0] = input[1]
        drawButton(pad, 0, but[0] )
    elif input[0] == 10:
        but[1] = input[1]
        drawButton(pad, 1, but[1] )
    elif input[0] == 11:
        but[2] = input[1]
        drawButton(pad, 2, but[2] )
    elif input[0] == 12:
        but[3] = input[1]
        drawButton(pad, 3, but[3] )
    elif input[0] == 13:
        but[4] = input[1]
        drawButton(pad, 4, but[4] )
    elif input[0] == 14:
        but[5] = input[1]
        drawButton(pad, 5, but[5] )
    elif input[0] == 15:
        but[6] = input[1]
        drawButton(pad, 6, but[6] )
    elif input[0] == 16:
        but[7] = input[1]
        drawButton(pad, 7, but[7] )

    elif input[0] == 114:
        but[8] = input[1]
    elif input[0] == 115:
        but[9] = input[1]
        exitFlag[pad] = input[1]
        if exitFlag[0] > 0 and exitFlag[1] > 0 :
            print("exit")
            quitgame()
    elif input[0] == 116:
        but[10] = input[1]
    elif input[0] == 117:
        but[11] = input[1]

    if input[0] == 21:
        knob[0] = input[1]
        drawKnob(pad, 0, knob[0] )
    elif input[0] == 22:
        knob[1] = input[1]
        drawKnob(pad, 1, knob[1] )
    elif input[0] == 23:
        knob[2] = input[1]
        drawKnob(pad, 2, knob[2] )
    elif input[0] == 24:
        knob[3] = input[1]
        drawKnob(pad, 3, knob[3] )
    elif input[0] == 25:
        knob[4] = input[1]
        drawKnob(pad, 4, knob[4] )
    elif input[0] == 26:
        knob[5] = input[1]
        drawKnob(pad, 5, knob[5] )
    elif input[0] == 27:
        knob[6] = input[1]
        drawKnob(pad, 6, knob[6] )
    elif input[0] == 28:
        knob[7] = input[1]
        drawKnob(pad, 7, knob[7] )
    elif input[0] == 41:
        knob[8] = input[1]
        drawKnob(pad, 8, knob[8] )
    elif input[0] == 42:
        knob[9] = input[1]
        drawKnob(pad, 9, knob[9] )
    elif input[0] == 43:
        knob[10] = input[1]
        drawKnob(pad, 10, knob[10] )
    elif input[0] == 44:
        knob[11] = input[1]
        drawKnob(pad, 11, knob[11] )
    elif input[0] == 45:
        knob[12] = input[1]
        drawKnob(pad, 12, knob[12] )
    elif input[0] == 46:
        knob[13] = input[1]
        drawKnob(pad, 13, knob[13] )
    elif input[0] == 47:
        knob[14] = input[1]
        drawKnob(pad, 14, knob[14] )
    elif input[0] == 48:
        knob[15] = input[1]
        drawKnob(pad, 15, knob[15] )

def text_objects(text, font):
    textSurface = font.render(text, True, text_color)
    return textSurface, textSurface.get_rect()

def drawKnob(pad, knob, val):
    w = 40
    y = 20
    if knob > 7:
        y = 110
        knob -= 8

    x = int((w*2.5) * knob + pad * (display_width / 2)) + w
    pygame.draw.circle(gameDisplay, button_highlight_back_color,(x+w,y+w),w)
    pygame.draw.circle(gameDisplay, button_highlight_color,(x+w,y+w),w,2)

    smallText = pygame.font.SysFont("swiss721",20)
    textSurf, textRect = text_objects(str(val), smallText)
    textRect.center = ( (x+(w)), (y+(w)) )
    gameDisplay.blit(textSurf, textRect)

    sendData(pad, knob+12, val)

def drawButton(pad, button, val):
    w = 80
    h = 80
    y = 200

    x = int( ((w/2*2.5)) * button + pad * (display_width / 2) + w/2 )

    pygame.draw.rect(gameDisplay, button_highlight_back_color,(x,y,w,h))
    pygame.draw.rect(gameDisplay, button_highlight_color,(x,y,w,h),2)
    if val > 0:
        pygame.draw.rect(gameDisplay, button_highlight_color,(x+8,y+8,w-16,h-16))
    # else:
    #     pygame.draw.rect(gameDisplay, button_highlight_color,(x,y,w,h),2)
    sendData(pad, button, val)


def drawLaunchControl( pad ):
    x_size = (display_width / 2)
    x_offset = (pad) * x_size

    s_x = x_offset + frame_border
    s_y = frame_border

    e_x = x_size - 2*frame_border
    e_y = display_height - 2* frame_border

    pygame.draw.rect(gameDisplay, button_color,(s_x,s_y,e_x, e_y), 2)
    for i in range( 0,16):
        if pad == 0:
            drawKnob(pad, i, lp_knob[i])
        else:
            drawKnob(pad, i, lp2_knob[i])

    for i in range( 0,8):
        if pad == 0:
            drawButton(pad, i, lp_but[i])
        else:
            drawButton(pad, i, lp2_but[i])



def main():

	mode = None

	# create an instance
	lp = launchpad.Launchpad()
	lp2 = launchpad.Launchpad()

	# check what we have here and override lp if necessary
	# if lp.Check( 0, "mk2" ):
	# 	lp = launchpad.LaunchpadMk2()
	# 	if lp.Open( 0, "mk2" ):
	# 		print("Launchpad Mk2")
	# 		mode = "Mk2"

	if lp.Check( 1, "control" ):
		lp = launchpad.LaunchControl()
		if lp.Open( 1, "control" ):
			# lp.TemplateSet(1)
			print("First  Launch Control")
			mode = "S"

	if lp2.Check( 0, "control" ):
		lp2 = launchpad.LaunchControl()
		if lp2.Open( 0, "control" ):
			# lp2.TemplateSet(1)
			print("Second Launch Control")
			mode = "S"

	if mode is None:
		print("Did not find any Launchpads, meh...")
		return

	print("---\nDual LaunchControl GUI")
	print("events before you press one of the (top) automap buttons")

	# Clear the buffer because the Launchpad remembers everything :-)
	lp.ButtonFlush()
	lp2.ButtonFlush()

	# lp.Reset() # turn all LEDs off
	# lp2.Reset() # turn all LEDs off

	for i in range( 0,16):
		lp_knob.append(0)
		lp2_knob.append(0)

	for i in range( 0,12):
		lp_but.append(0)
		lp2_but.append(0)  

	drawLaunchControl(0)
	drawLaunchControl(1)
	pygame.display.update()

	# openSocket()

	while not gameExit:

		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
				quit()

		but = lp.InputStateRaw()
		if but != []:
			mapInput(0,but, lp_but, lp_knob)            
			if but[1] > 0:
				# print("set button %d to On" % but[0])
				lp.LedCtrl(but[0],128,0) 
			else:
				lp.LedCtrl(but[0],0,0)

			data = 'event LC1: %d - %d' % (but[0] , but[1])
			# print(data)

		but2 = lp2.InputStateRaw()
		if but2 != []:
			mapInput(1,but2, lp2_but, lp2_knob)
			if but2[1] > 0:
				# print("set button %d to On" % but2[0])
				lp2.LedCtrl(but2[0],128,0) 
			else:
				lp2.LedCtrl(but2[0],0,0)

			data = 'event LC2: %d - %d' % (but2[0] , but2[1])
			# print(data)
        
        # Create an instance of ProcessData() to send to server.
        # variable = ProcessData()
        # # Pickle the object and send it to the server
        # data_string = pickle.dumps(variable)
        # s.send(data_string)

		pygame.display.update()

	# now quit...
	print("Quitting might raise a 'Bad Pointer' error (~almost~ nothing to worry about...:).\n\n")

	lp.Reset() # turn all LEDs off
	lp2.Reset() # turn all LEDs off

	lp.Close() # close the Launchpad (will quit with an error due to a PyGame bug)
	lp2.Close() # close the Launchpad (will quit with an error due to a PyGame bug)
	
if __name__ == '__main__':
	main()

