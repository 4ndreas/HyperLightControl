import pygame
import time
import random
import os
import socket


os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (0,0) #-3584

pygame.init()

UDP_IP = "127.0.0.1"
artnetPort = 6454

display_width = 1027
display_height = 768


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


# gameDisplay = pygame.display.set_mode((display_width,display_height),  pygame.FULLSCREEN |pygame.HWSURFACE )
# gameDisplay = pygame.display.set_mode((display_width,display_height), pygame.NOFRAME)
gameDisplay = pygame.display.set_mode((display_width,display_height))

pygame.display.set_caption('Artnet viwer')
clock = pygame.time.Clock()


#######
def things(thingx, thingy, thingw, thingh, color):
    pygame.draw.rect(gameDisplay, color, [thingx, thingy, thingw, thingh])
#######
def text_objects(text, font):
    textSurface = font.render(text, True, text_color)
    return textSurface, textSurface.get_rect()

def button(msg,x,y,w,h,ic,ac,action=None):
    mouse = pygame.mouse.get_pos()
    click = pygame.mouse.get_pressed()
    # print(click)

    if x+w > mouse[0] > x and y+h > mouse[1] > y:
        pygame.draw.rect(gameDisplay, button_highlight_back_color,(x,y,w,h))
        pygame.draw.rect(gameDisplay, ac,(x,y,w,h), 2)

        if click[0] == 1 and action != None:
            action()         
    else:
        pygame.draw.rect(gameDisplay, button_back_color,(x,y,w,h))
        pygame.draw.rect(gameDisplay, ic,(x,y,w,h),3)

    smallText = pygame.font.SysFont("swiss721",14)
    textSurf, textRect = text_objects(msg, smallText)
    textRect.center = ( (x+(w/2)), (y+(h/2)) )
    gameDisplay.blit(textSurf, textRect)

def button_round(msg,x,y,w,ic,ac,action=None):
    mouse = pygame.mouse.get_pos()
    click = pygame.mouse.get_pressed()
    # print(click)
    

    if x+2*w > mouse[0] > x and y+2*w > mouse[1] > y:
        pygame.draw.circle(gameDisplay, button_highlight_back_color,(x+w,y+w),w)
        pygame.draw.circle(gameDisplay, ac,(x+w,y+w),w, 2)

        if click[0] == 1 and action != None:
            action()         
    else:
        pygame.draw.circle(gameDisplay, button_back_color,(x+w,y+w),w)
        pygame.draw.circle(gameDisplay, ic,(x+w,y+w),w,3)

    smallText = pygame.font.SysFont("swiss721",14)
    textSurf, textRect = text_objects(msg, smallText)
    textRect.center = ( (x+(w)), (y+(w)) )
    gameDisplay.blit(textSurf, textRect)


def message_display(text):
    largeText = pygame.font.Font('freesansbold.ttf',115)
    TextSurf, TextRect = text_objects(text, largeText)
    TextRect.center = ((display_width/2),(display_height/2))
    gameDisplay.blit(TextSurf, TextRect)

    pygame.display.update()

    time.sleep(2)

    game_loop()


def crash():
    message_display('You Crashed')

gameExit = False

def quitgame():
    gameExit = True
    pygame.quit()
    quit()

def button_pressed():
    print('click')



def game_loop():
    x = (display_width * 0.45)
    y = (display_height * 0.8)

    x_offset = 0
    y_offset = 180


    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('',artnetPort))

    
    while not gameExit:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        pygame.draw.rect(gameDisplay, button_color,(0,0,1024,768), 4)

        button("EXIT",16*60+x_offset,10,50,50,red,bright_red,quitgame) 


        
        # (data, addr) = sock.recvfrom(10240)
        
        # if len(data) < 20:
        #     sdata = data.decode("utf-8")
        #     if sdata == "quit":
        #         print("exit program")
        #         break
        #     print (data)
        #     continue

        # universe  = int(data[9])
        # protverhi = int(data[10])
        # protverlo = int(data[11])
        # sequence  = int(data[12])
        # physical  = int(data[13])
        # subuni    = int(data[14])
        # net       = int(data[15])
        # lengthhi  = int(data[16])
        # length    = int(data[17])
        # #dmx       = data[18:]
        
        # offset = subuni * 170
        # col = pygame.Color(0,0,0)

        # w = 5
        # if( net == 0):
        #     for i in range(0,170):
        #         x = int((i + offset) % 240)
        #         y = int((i + offset) / 240)
        #         col.r = data[i*3 +18]
        #         col.g = data[i*3 +19]
        #         col.b = data[i*3 +20]
        #         # pygame.gfxdraw.pixel(screen, x, y,col )
        #         pygame.draw.circle(gameDisplay, col,(x+w,y+w),w)
        

        pygame.display.update()
        clock.tick(15)


game_loop()
pygame.quit()
quit()