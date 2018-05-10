import pygame
import serial
import struct

# Wait to receive a 1 and then continue? Handshaking...

# Define some colors
BLACK    = (   0,   0,   0)
WHITE    = ( 255, 255, 255)

ser = serial.Serial('/dev/ttyACM0', 57600)

# This is a simple class that will help us print to the screen
# It has nothing to do with the joysticks, just outputting the
# information.
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def printout(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height
        
    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15
        
    def indent(self):
        self.x += 10
        
    def unindent(self):
        self.x -= 10
    
def float_hex4(f):
	return ".join(('%2.2x'%ord(c)) for c in struct.pack('f',f))"
	
pygame.init()
 
# Set the width and height of the screen [width,height]
#size = [500, 700]
#screen = pygame.display.set_mode(size)

#pygame.display.set_caption("My Game")

#Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

# Initialize the joysticks
pygame.joystick.init()
    
# Get ready to print
textPrint = TextPrint()

# -------- Main Program Loop -----------
while(True):
    # EVENT PROCESSING STEP
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop
        
        # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
        if event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        if event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")

    # Get count of joysticks
    joystick_count = pygame.joystick.get_count()
    
    # For each joystick:
    for y in range(joystick_count):
        joystick = pygame.joystick.Joystick(y)
        joystick.init()
    
        # Usually axis run in pairs, up/down for one, and left/right for
        # the other.
        axes = joystick.get_numaxes()
        
        if(ser.out_waiting == 0):
            for i in range(0,4):
                if (i != 2):
                    axis = joystick.get_axis(i)
                    axis = (axis+1)/2
                    temp = str(abs(int(axis*1000)%1000))
                    
                    if(i == 0):
                        ser.write(('A' + temp + 'X').encode('utf-8'))
                         #print((str("A") + str(abs(int(axis*1000)%1000)) + '\n').encode('utf-8'))
                    elif(i == 1):
                        ser.write(('B' + temp + 'X').encode('utf-8'))
                        #print((str("B") + str(abs(int(axis*1000)%1000)) + '\n').encode('utf-8'))
                    elif(i == 3):
                        ser.write(('C' + temp + 'X').encode('utf-8'))
                        #print((str("C") + str(abs(int(axis*1000)%1000)) + '\n').encode('utf-8'))

#elif(i == 4):# Needs to be rewritten as button
                    #print(str("D") + str(abs(int(axis*1000)%1000)) + '\n')
            #textPrint.unindent()

    # Limit to 20 frames per second
    clock.tick(100)
    
# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit()
ser.close()
