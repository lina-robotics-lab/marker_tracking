#!python3.7

#cython: language_level=3

class RobotArmController():
    
    def __init__(self, resume=0):
        self.__currentIdx = resume
        self.__nbOfPosition = 1
    
    def goto(idx):
        if idx < self.__nbOfPosition and idx >= 0:
            self.__currentIdx = idx
            print(self.__currentIdx, file=open('LastPosition', 'w'))
            return True
        else:
            return False
    
    def position():
        return self.__currentIdx
    
    def nbPositions():
        return self.__nbOfPosition
