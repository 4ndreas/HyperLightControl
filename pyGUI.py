import pygame
import time
import random
import os


os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (0,0) #-3584

pygame.init()

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
gameDisplay = pygame.display.set_mode((display_width,display_height), pygame.NOFRAME)

pygame.display.set_caption('HyperLight GUI')
clock = pygame.time.Clock()

# carImg = pygame.image.load('racecar.png')

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
    
    while not gameExit:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        pygame.draw.rect(gameDisplay, button_color,(0,0,1024,768), 4)

        button("EXIT",16*60+x_offset,10,50,50,red,bright_red,quitgame) 

        for x in range(1, 9):
            for y in range(1, 9):
                button("%d%d" %(x,y),x*60 + x_offset,y*60 + y_offset,50,50,button_color,button_highlight_color,button_pressed)
        
        y = y_offset
        for x in range(1, 9):
            button_round("%d%d" %(x,y),x*60 + x_offset,y,25,button_color,button_highlight_color,button_pressed)

        x = 9*60 + x_offset
        for y in range(1, 9):
            button_round("%d%d" %(x,y),x,y*60 + y_offset,25,button_color,button_highlight_color,button_pressed)

        pygame.display.update()
        clock.tick(15)


game_loop()
pygame.quit()
quit()