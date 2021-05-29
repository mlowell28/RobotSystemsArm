
import threading, queue

def class Command():
    
    def __init__(self, function, argument, pause_time = 0):
        self.function = function
        self.argument = argument
        self.pause_time = pause_time 

def class Motion():
    
    def __init__(self, message_bus, bin_coordinates = {
        'red':   (-15 + 0.5, 12 - 0.5, 1.5),
        'green': (-15 + 0.5, 6 - 0.5,  1.5),
        'blue':  (-15 + 0.5, 0 - 0.5,  1.5),)}:
        
        self.bin_coordinates = bin_coordinates
        self.message_bus = message_bus
        self.command_list = []
        self.argument_list = []
        self.internaL_bus = message_bus())
        self.command_queue = queue.Queue()
        
        command_executor_thread = thread.Threading(self.command_executor)
        
    }
    

    def initMove(self):
        command_1 = Command(Board.setBusServoPulse, 1, servo1 - 50, 300)
        self.command_queue.put(command_1)        
        command_2 = Command(Board.setBusServoPulse, (2, 500, 500))
        self.command_queue.put(command_2)
        command_3 = Command(AK.setPitchRangeMoving,(0, 10, 10), -30, -30, -90, 1500), 1.5)
        self.command_queue.put(command_3)
        
    def reset_arm(self):
        Board.setBusServoPulse(1, servo1 - 70, 300)
        time.sleep(0.5)
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
        time.sleep(1.5)

    def setBuzzer(self, timer):
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(timer)
        Board.setBuzzer(0)

    # set the RGB light color of the expansion board to match the color being tracked
    def set_rgb(self, color):
        if color == "red":
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
            Board.RGB.show()
        elif color == "green":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
            Board.RGB.show()
        elif color == "blue":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
            Board.RGB.show()
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
            Board.RGB.show()
    
    # function which will run as a thread to execute commands from the 
    # command queue 
      
    def command_executor(self):
        
        while(command_bus.executor_run == True):
            if interal_bus.runpause == false:
                if (self.command_queue.empty() == False)
                # if command_queue_not_empty
                    command = command_queue.pop()
                    result = command.function(*command.argument)
                    
                    if(command.function == AK.setPitchRangeMoving)
                        time.pause(result)
                    else:
                        time.pause(command.pause)
                    
                    
    # x, y, z location of block and its rotation angle
    def collect_block(self, block_coordinate, rotation_angle, place_coordinate):
        
        self.command_queue.clear()
        
        set_rgb(detect_color)
        setBuzzer(0.1)
        result = AK.setPitchRangeMoving((block_coordinate[0], block_coordinate[1], 6), -90, -90, 0)  
            
        # check if position is reachable using IK solver and returns false if not. 
        
        if result == False:
            return False
            
        # generate sequence of commands add them to the command queue. 
            
        servo2_angle = getAngle(block_coordinate[0], block_coordinate[1], rotation_angle) #calculate the angle at which the gripper needs to rotate
        command_1 = Command(Board.setBusServoPulse, (1, servo1-280,500), 0) # Open claw
        self.command_queue.put(command_1)
        
        command_2 = Command(Board.setButServoPulse, (2, servo2_angle, 500),.5)
        self.command_queue.put(command_2)
        
        ommand_3 = Command(AK.setPitchRangeMoving, ((block_coordinate[0], block_coordinate[1], 2), -90, -90, 0, 1000), 1.5) # align base angle
        self.command_queue.put(command_3)
        
        command_4 = Command(Board.setBusServoPulse, (1, servo1, 500), .8) # Gripper closed
        self.command_queue.put(command_4)
        
        command_5 = Command(Board.setBusServoPulse, (2,500,500), 0)
        self.command_queue.put(command_5)
        
        command_6 = Command(AK.setPitchRangeMoving, ((block_coordinate[0], block_coordinate[1], 12), -90, -90, 0, 1000), 1) # lift robot arm 
        self.command_queue.put(command_6)
        
        # move to location of color square to place block
        
        command_7 = AK.setPitchRangeMoving,((place_coordinate[0], place_coordinate[1], 12), -90, -90, 0)
        self.command_queue.put(command_7) 
        
        servo2_angle = getAngle(place_coordinate[0], place_coordinate[1], -90)
        command_8 = Command(Board.setBusServoPulse, (2, servo2_angle, 500), .5)
        self.command_queue.put(command_8)
        
        command_9 = Command(AK.setPitchRangeMoving, ((place_coordinate[0], place_coordinate[1], place_coordinate[2] + 3), -90, -90, 0, 500), .5) #+ 3
        self.command_queue.put(command_9)
        
        command_10 = Command(AK.setPitchRangeMoving,(place_coordinate), -90, -90, 0, 1000), .8)
        self.command_queue.put(command_10)
 
        command_11 = Command(Board.setBusServoPulse, (1, servo1 - 200, 500), .8) # open claws drop object
        self.command_queue.put(command_11)
        command_12 = Command(AK.setPitchRangeMoving, ((place_coordinate[0], place_coordinate[1], 12), -90, -90, 0, 800), .8)
        
        # go back to reset position
        self.initMove()  

