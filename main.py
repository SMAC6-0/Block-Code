# Version implemented for for use without IR remote,
# actually remote is used but just for first block in structure - "seedblock"
# This version corrected issues with UNPLACED blinking as well as issues
# with ignoring seedblock remote command

# last edited 4/21/24
# This version implemented to test ability to relay a message through multiple blocks


# IMPORT LIBRARIES
import sys
import time
from machine import *#import Pin, freq, PWM
# rx
from BlockPackage.ir_rx.print_error import print_error
from BlockPackage.ir_rx.nec import NEC_8
# tx
import uasyncio as asyncio
from BlockPackage.primitives.switch import Switch
from BlockPackage.primitives.delay_ms import Delay_ms
# NEC Protocol
from BlockPackage.ir_tx.nec import NEC

# PIN DEFINITIONS
# face 1
pin_rx1 = Pin(0, Pin.IN) # GPIO pin for IR receiver 
pin_tx1 = Pin(1, Pin.OUT, value = 0) # GPIO pin for IR LED
pin_photo1 = Pin(2, Pin.IN, Pin.PULL_UP) # photoresistor pin
# face 2
pin_rx2 = Pin(3, Pin.IN) 
pin_tx2 = Pin(4, Pin.OUT, value = 0) 
pin_photo2 = Pin(5, Pin.IN, Pin.PULL_UP) 
# face 3
pin_rx3 = Pin(6, Pin.IN) 
pin_tx3 = Pin(7, Pin.OUT, value = 0) 
pin_photo3 = Pin(8, Pin.IN, Pin.PULL_UP) 
# face 4
pin_rx4 = Pin(10, Pin.IN) 
pin_tx4 = Pin(11, Pin.OUT, value = 0) 
pin_photo4 = Pin(12, Pin.IN, Pin.PULL_UP) 
# face 5
pin_rx5 = Pin(19, Pin.IN) 
pin_tx5 = Pin(20, Pin.OUT, value = 0) 
pin_photo5 = Pin(21, Pin.IN, Pin.PULL_UP) 
# face 6
pin_rx6 = Pin(16, Pin.IN) 
pin_tx6 = Pin(17, Pin.OUT, value = 0) 
pin_photo6 = Pin(18, Pin.IN, Pin.PULL_UP) 

# RGB LED setup
ledb = Pin(22, Pin.OUT) # RGB LED Blue
ledr = Pin(27, Pin.OUT) # RGB LED Red
ledg = Pin(26, Pin.OUT) # RGB LED Green
# blu = PWM(ledb)
# red = PWM(ledr)
# gre = PWM(ledg)
# blu.freq(500)
# red.freq(500)
# gre.freq(500)
# blu.duty_u16(000)
# red.duty_u16(000)
# gre.duty_u16(000)

# GLOBAL DEFINITIONS
loop = asyncio.get_event_loop()
rx_allowed = True # used to ignore self-received IR
do_once = True # used to execute a chunk of code one time once situated

# all faces pin lists
tx_pins = [pin_tx1, pin_tx2,  pin_tx3,  pin_tx4,  pin_tx5,  pin_tx6]
rx_pins = [pin_rx1, pin_rx2,  pin_rx3,  pin_rx4,  pin_rx5,  pin_rx6]
photo_pins = [pin_photo1, pin_photo2, pin_photo3, pin_photo4, pin_photo5, pin_photo6]

# RGB brightness levels (unsigned 16-bit value in the range 0 to 65535)
R_duty = 8000 
G_duty = 8000 
B_duty = 8000 

# states, used in main() state machine
unplaced = 0
handshake = 1
placed = 2
seedblock = 3

# IR message codes
code_hs = 92 # handshake 
code_hsr = 93 # handshake response
code_sb = 65 # seed block
code_msg = 90 # relay message

# flags
flag_init = [0,0,0,0,0,0] # initial flag state, does nothing
flag = flag_init
flag_hs = 1 # incoming handshake
flag_hsr = 2 # incoming handshake response
flag_sb = 3 # seed block flag
flag_msg = 4 # message received flag


# CLASS DEFINITIONS
class Coord:
    # x,y,x block coordinates
    def __init__(self, x, y, z):
        self.X = x
        self.Y = y
        self.Z = z
        
    def step(self, direction, incr):
        # used to increment/decrement coordinates received from an adjacent block
        # when setting own coordinates
        if incr:
            add = 1
        else:
            add = -1            
        if direction == 'x':
            self.X = self.X + add
        elif direction == 'y':
            self.Y = self.Y + add
        elif direction == 'z':
            self.Z = self.Z + add
        else:
            print('Direction undefined. Needs "x" "y" or "z"', file=sys.stderr)
            
    def returncoords(self):
        coords = [self.X, self.Y, self.Z]
        return coords
            
        
class Local: # self contains block's local block presence data
    # state and coords given are that of the self (self.O)
    def __init__(self, state, coord):
        self.O = Block(state, coord, 0, 0)
        self.faces = [0, 0, 0, 0, 0, 0]
        #			 [N, E, S, W, U, D]
        
#     def list_local(self):
#         local_list = [self.O, self.N, self.E, self.S, self.W, self.U, self.D]
#         return local_list
    
    def numblocks1dos(self):
        # returns number of adjacent blocks
        count = 0
        for x in range(len(self.faces)):
            if not not self.faces[x]:
                count = count + 1
        return count
    
    def returnfaces(self):
        return self.faces

    def add(self, ref, state, coord, dos, local):
        # adds a block to Local instance
        # ref is taken as integer 0-5 corresponding to faces 1-6
        added_block = Block(state, coord, dos, local)
        self.faces[ref] = added_block
        
    def remove(self, ref):
        # removes a block from Local instance
        # ref is taken as integer 0-5 corresponding to faces 1-6
        self.faces[ref] = 0
    
    def read(self, ref):
        # function to return a Block specified by "ref"
        return self.faces[ref]

class Photo:
        def __init__(self, pins):
            # 0 indicates dark
            # 1 indicates bright
            self.photos = [1,1,1,1,1,1]
            self.pins = pins
            
        def update(self):
            for x in range(6): # checking all 6 photoresistors
                if not self.pins[x].value(): # if PR pin read low
                    print('photo', x+1, 'dark') # for debug
                    self.photos[x] = 0; # update corresponding value to 0
                else:
                    self.photos[x] = 1;
            return self.photos

class Block:
    # this class contains all necessary info about a block. Each block will store multiple of
    # these classes: one for itself (0 degrees of seperation), one for each 6 face's adjacent 
    # blocks (1 dos), one for all 2nd dos, and so on
    # self contains block's state, coordinates, degree of seperation, and local data
    def __init__(self, state, coord, dos, local):
        self.state = state
        self.coord = coord
        self.dos = dos
        self.local = local
        
    def changestate(self, newstate):
        self.state = newstate
        
    #def setcoord(self, coord):
        # function to set coordinates to "coord" given as class Coord
    
    def returnstate(self):
        return self.state
    
    def localadd(self, ref, state, coord, dos, local):
        # adds a block to Block's Local instance
        # ref taken as integer 0-5 corresponding to faces 1-6
        self.local.add(ref, state, coord, dos, local)
        
    def localremove(self, ref):
        # removes a block from Block's Local instance
        # ref taken as integer 0-5 corresponding to faces 1-6
        self.local.remove(ref)
        
    def read(self, ref):
        # function to return a Block specified by "ref"
        # ref taken as integer 0-5 corresponding to faces 1-6
        return(self.local.read(ref))
    
origin = Coord(0,0,0)
selflocal = Local(0, origin)
selfblock = Block(0, origin, 0, selflocal)

async def main():
    global selfblock
    global do_once
    global R_duty
    global G_duty
    global B_duty
    global photo_pins
    global tx_pins
    global rx_pins
    global flag_init
    photo = Photo(photo_pins)
    LED = False # for UNPLACED led blink
    seed = False # initially not a seed block            
    
                # state machine #
    while True: 
        state = selfblock.returnstate() # get state from Block object 'selfblock'
        global rx_allowed # used to ignore self-received IR
        global flag
        
        #================== UNPLACED ===================#
        if state == unplaced: 
            # unplaced blocks check photoresistors
            # if number of dark PRs>0, change state to HANDSHAKE
            
            # blink LED white 
            ledr.high()
            ledg.high()
            ledb.high()
            
            DPR = 0 # number of dark photoresistors
            photos = photo.update() # read digital input PR values//need to add "deflicker" here
            for x in range(6): # counting number of dark PRs
                if not photos[x]:
                    DPR = DPR + 1
            print(DPR, "dark photoresistors") # for debug
            
            # this delay controls blink length and allows time for IR rx
            await asyncio.sleep(.2) 
            
            # turn off white LED blink
            ledr.low()
            ledg.low()
            ledb.low()
            
            # seed block control
            for x in range(6):
                if flag[x] == flag_sb:
                    seed = True # set block to seed block
                    flag = [0,0,0,0,0,0] # reset flag
                    print("SEED")
                    
            # state-change control
            if seed:
                selfblock.changestate(seedblock)
                # constant white LED
                ledr.high()
                ledg.high()
                ledb.high()
            elif DPR > 0:
                selfblock.changestate(handshake)
            else:
                # this delay controls blink off time when no PRs are dark
                await asyncio.sleep(.2)  
            
        # ================= SEEDBLOCK ===================#
        elif state == seedblock:
            print('SEEDBLOCK')
            
            photos = photo.update() # read digital input PR values
            for x in range(6):
                    if flag[x] == flag_hs: # if handshake received on face x
                        if not photos[x]: # AND PR on face x is dark
                            selfblock.localadd(x, placed, origin, 1, 0) # add block to local
                            send = NEC(tx_pins[x], 38000) # prepare tx pin for sending
                            send.transmit(0x1, code_hsr, 0, True) # send handshake response
                            print('new block: ',selfblock.local.returnfaces()) # for debug
                            print('response sent') # for debug
                            flag = [0,0,0,0,0,0] # reset flag
                            selfblock.changestate(placed)
                            seed = False # no longer a seed block once another block is adjacent
                        flag = [0,0,0,0,0,0] # reset flag if PR not dark
                            
            await asyncio.sleep(.1)
            
        #================== HANDSHAKE ===================#        
        elif state == handshake: 
            # in this state block sends handshakes to each face that has
            # a dark PR. Adds faces which receive a response to adjacent blocks list
            # if no adjacent blocks, go back to UNPLACED
            # else, change state to PLACED
            print('HANDSHAKE') # for debug
            
            # record flag state from UNPLACED in case seedblock command received
            prev_flag = flag 
    
            for x in range(6): 
                if not photos[x]: # if PR on face i is dark
                    rx_allowed = False # disable flag updates to avoid self-communication issue
                    send = NEC(tx_pins[x], 38000) # instantiate pin for tx
                    send.transmit(0x1, 92, 0, True) # send handshake
                    await asyncio.sleep(0.07) # approximate length of NEC transmission is 70 ms
                    rx_allowed = True # enable flag updates
                    print('sent handshake on face ', x+1) # for debug
                    await asyncio.sleep(.3) # wait for response
                    
                    if flag[x] == flag_hsr: # check for response
                        flag = [0,0,0,0,0,0] # reset flag
                        print("Receieved handshake on Face ", flag[x]) # for debug
                        selfblock.localadd(x, placed, origin, 1, 0) # add responding block to local 
                    else: # no response
                        flag = prev_flag # reset flag to recorded flag state
                        print('no response')
                        
            print(selfblock.local.numblocks1dos(), " adjacent blocks")            
            if not selfblock.local.numblocks1dos(): # if no adjacent blocks detected
                selfblock.changestate(unplaced) # go back to UNPLACED
            else:
                selfblock.changestate(placed) # otherwise change to PLACED
            
        #==================== PLACED ====================#    
        elif state == placed: 
            # 1: placed blocks check photoresistors, if a PR corresponding to
            # an adjacent block changes from dark to light, the adjacent block is
            # removed from list.
            # if there are zero adjacent blocks, change state to UNPLACED
            # 2: placed blocks wait for incoming handshakes on faces with dark PRs
            # but no adjacent block. Once a handshake is received, response is sent
            # and that face is added to adjacent blocks list.
            # 3: update LEDs depending on number of adjacent blocks
            if do_once: 
                print('placed') # for debug
                print('faces:', selfblock.local.returnfaces()) # for debug
                flag = [0,0,0,0,0,0] # reset flag
                print('flags:', flag)
                do_once = False
                
            # check photoresistors and compare with adjacent block status
            photos = photo.update() # read digital input PR values
            adj = selfblock.local.returnfaces() # binary list of adjacent block status
            for x in range(6):
                if not not adj[x]: # if there was a block adjacent to face x - check if this 'not not' is necssary #?#?#?#?#?
                    if photos[x]: # AND the photoresistor on face x is not dark
                        selfblock.localremove(x) # remove block from local
                        
            # if no adjacent blocks, change state to UNPLACED
            if not selfblock.local.numblocks1dos():
                do_once = True # reset do_once
                selfblock.changestate(unplaced) 
                    
            # check for incoming handshakes    
            for x in range(6):
                if flag[x] == flag_hs: # if handshake received on face x
                    if not photos[x]: # AND PR on face x is dark
                        selfblock.localadd(x, placed, origin, 1, 0) # add block to local
                        send = NEC(tx_pins[x], 38000) # prepare tx pin for sending
                        send.transmit(0x1, code_hsr, 0, True) # send handshake response
                        print('new block: ',selfblock.local.returnfaces()) # for debug
                        print('response sent') # for debug
                        flag = [0,0,0,0,0,0] # reset flag
                    flag = [0,0,0,0,0,0] # reset flag if PR not dark
            
            # check for incoming messages
            for x in range(6):
                if flag[x] == flag_msg: # if message received on face x
                    for y in range (6):
                        if y != x: # relay message to every face except face x
                            send = NEC(tx_pins[y], 38000) # prepare tx pin for sending
                            send.transmit(0x1, code_msg, 0, True) # relay message to face y
                            await asyncio.sleep(0.07) # approximate length of NEC transmission is 70 ms
                    flag = [0,0,0,0,0,0] # reset flags
                    
            
            # LED Update
            AB = selfblock.local.numblocks1dos() # number of adjacent blocks
            if AB == 1: # if 1 block adjacent
                ledr.low()
                ledg.high()
                ledb.low() # green LED
            elif AB == 2: # if 2 blocks adjacent
                ledr.high()
                ledg.low()
                ledb.high() # purple LED
            elif AB == 3: # if 3 blocks adjacent
                ledr.low()
                ledg.low()
                ledb.high() # blue LED
            elif AB == 4: # if 4 blocks adjacent
                ledr.low()
                ledg.high()
                ledb.high() # yellow LED
            elif AB == 5: # if 5 blocks adjacent
                ledr.high()
                ledg.high()
                ledb.high() # white LED
                
            await asyncio.sleep(.1) # iteration delay
            
                       
# vvvvvvvvvvvvvvvvvvv rx code vvvvvvvvvvvvvvvvvvv #
def readIRValue(data, pin):
    # depending on data received, raises a corresponding flag
    global rx_pins
    if rx_allowed:
        for x in range(6):
            if pin == rx_pins[x]:
                active = x
        global flag
        if data == code_hsr: # handshake response
            flag[active] = flag_hsr
        if data == code_hs: # handshake
            flag[active] = flag_hs
        if data == code_sb: # seedblock
            flag[active] = flag_sb
        if data == code_msg:
            flag[active] = flag_msg
            print("MESSAGE RECEIVED")
        print("flags =", flag)
        
    # remote control
#     if data == 88: # R button
#         blu.duty_u16(000)
#         red.duty_u16(R_duty)
#         gre.duty_u16(000)
#     if data == 89: # G button
#         blu.duty_u16(000)
#         red.duty_u16(000)
#         gre.duty_u16(G_duty)
#     if data == 69: # B button
#         blu.duty_u16(B_duty)
#         red.duty_u16(000)
#         gre.duty_u16(000)
#     if data == 68: # W Button
#         blu.duty_u16(000)
#         red.duty_u16(000)
#         gre.duty_u16(000)
    if data == 64: # power button
        machine.reset()
    return data

def callback(data, addr, ctrl, pin): 
    if data < 0:  # NEC protocol sends repeat codes.
        pass
    else:
        print(readIRValue(data, pin))
        #print(readIRValue(data, pin.returnid()))


ir1 = NEC_8(pin_rx1, callback)  # Instantiate receiver on face 1
ir2 = NEC_8(pin_rx2, callback)  # Instantiate receiver on face 2
ir3 = NEC_8(pin_rx3, callback)  # Instantiate receiver on face 3
ir4 = NEC_8(pin_rx4, callback)  # Instantiate receiver on face 4
ir5 = NEC_8(pin_rx5, callback)  # Instantiate receiver on face 5
ir6 = NEC_8(pin_rx6, callback)  # Instantiate receiver on face 6

ir1.error_function(print_error)  # Print receiver debug information
#^^^^^^^^^^^^^^^^^^^^^ rx code ^^^^^^^^^^^^^^^^^^^^^#

loop.run_until_complete(main()) # main loop
