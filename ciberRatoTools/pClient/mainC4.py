
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        # Variables
        self.out_now = 0
        self.sum = 0
        self.count=0
        self.walk = 0
        self.previous_theta = 0
        self.out_now_right = 0
        self.out_now_left = 0
        self.spd_out_r = 0
        self.spd_out_l = 0
        self.tur=0
        self.tur2=0
        self.count=0
        self.sum2=0
        self.average=0.0
        self.spd_out=0

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True);
                self.wander()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()


    def calculateDist(self, in_spd, out_prev):
        out = (in_spd + out_prev) / 2
        return out

    def wander(self):
        center_id = 0
        
        
        out_prev = self.out_now
        
        if self.sum<1.75:
            self.out_now=0.15
        elif self.sum>1.75 and self.sum<1.82:
            self.out_now=0.05
        else:
            self.out_now=0
        
        if self.sum>1.999:
            exit()
            
        self.out_now = self.calculateDist(self.out_now, out_prev)
        self.sum += self.out_now
        print(self.sum)
        self.driveMotors(self.out_now,self.out_now)
        # rotation tests because compass nise sucks
        # if self.tur2 == 0:
        #     if self.turn(90,"left") == 1:
        #         self.tur2=1
        # if self.tur == 0 and self.tur2==1:
        #    if self.compass_analysis() == 1:
        #        self.tur=1
        #        exit()
        #    print(self.measures.compass)
        

    def compass_analysis(self):
        
        if(self.count==10):
            print(f"average {self.average}")
            self.sum2=self.average=self.count=0
            return 1

        if (self.measures.compass > self.average + 3 or self.measures.compass < self.average - 3) and self.count > 2:
            print("NOPE")
            pass
        else:
            self.sum2 += self.measures.compass
            self.count +=1
        try:
            self.average=self.sum2/self.count
        except:
            print("Can't divide by 0")

    # Turns the robot to the correct direction
    def turn(self, degrees, direction):
        if degrees==90:
            if (self.previous_theta < 1.2 and direction=='left'):
                self.spd_out_l=-0.15
                self.spd_out_r=0.15
            elif(self.previous_theta > 1.2 and self.previous_theta < 1.25 and direction == 'left'):
                self.spd_out_l=-0.0354
                self.spd_out_r=0.0354
            elif (self.previous_theta > -1.2 and direction=='right'):
                self.spd_out_l=0.15
                self.spd_out_r=-0.15
            elif(self.previous_theta < -1.2 and self.previous_theta > -1.25 and direction == 'right'):
                self.spd_out_l=0.0354
                self.spd_out_r=-0.0354
            else:
                self.spd_out_l = self.spd_out_r = 0
            out_right = (self.spd_out_r+self.out_now_right)/2
            out_left = (self.spd_out_l+self.out_now_left)/2
            self.out_now_right = out_right
            self.out_now_left = out_left
            theta = self.previous_theta + (out_right-out_left)
            self.previous_theta=theta
            self.driveMotors(self.spd_out_l,self.spd_out_r)
            print(theta)
            #print(self.measures.compass)
            if theta>1.57 or theta<-1.57:
                print("Acabou")
                self.out_now_left=self.out_now_right=self.previous_theta=0
                return 1
                #exit()
        elif degrees==180 or degrees==-180:
            if (self.previous_theta < 2.68):
                self.spd_out_l=-0.15
                self.spd_out_r=0.15
            elif(self.previous_theta > 2.68 and self.previous_theta < 2.72):
                self.spd_out_l=-0.0708
                self.spd_out_r=0.0708
            else:
                self.spd_out_l = self.spd_out_r = 0
            out_right = (self.spd_out_r+self.out_now_right)/2
            out_left = (self.spd_out_l+self.out_now_left)/2
            self.out_now_right = out_right
            self.out_now_left = out_left
            theta = self.previous_theta + (out_right-out_left)
            self.previous_theta=theta
            print(f"velocidade {self.spd_out_l}")
            self.driveMotors(self.spd_out_l,self.spd_out_r)
            print(theta)
            
            if theta>3.1414:
                print("Acabou")
                self.out_now_left=self.out_now_right=self.previous_theta=0
                return 1
        else:
            print("UNEXPECTED!!!!")
            return 1

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()

        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None

           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()
