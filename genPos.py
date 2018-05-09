#!/usr/bin/env python2
import cv2
import numpy as np
import matplotlib.pyplot as plt

class gameScheme:
    def __init__(self):
        self.img = cv2.imread('img/game1.png', cv2.IMREAD_COLOR)
        self. pos = [(144.54032258064515, 80.20161290322585), (323.86290322580646, 80.20161290322585), (505.82258064516134, 80.20161290322585),
                     (690.41935483870964, 80.20161290322585), (856.55645161290317, 88.112903225806576), (1043.7903225806451, 88.112903225806576),
                     (1209.9274193548388, 90.75), (144.54032258064515, 235.79032258064524), (323.86290322580646, 235.79032258064524),
                     (487.36290322580646, 233.1532258064517), (690.41935483870964, 251.61290322580646), (859.19354838709683, 248.97580645161293),
                     (1035.8790322580646, 259.52419354838719), (1212.5645161290322, 259.52419354838719), (141.90322580645162, 404.56451612903231),
                     (323.86290322580646, 404.56451612903231), (503.18548387096769, 401.92741935483878), (685.14516129032256, 404.56451612903231),
                     (872.37903225806463, 423.02419354838719), (1020.0564516129032, 423.02419354838719), (1231.0241935483871, 423.02419354838719),
                     (141.90322580645162, 568.06451612903231), (318.58870967741939, 568.06451612903231), (500.54838709677415, 565.42741935483878),
                     (674.59677419354841, 573.33870967741939), (851.2822580645161, 570.70161290322585), (1038.516129032258, 591.79838709677426),
                     (1233.6612903225807, 583.88709677419365), (128.71774193548387, 742.11290322580658), (315.95161290322585, 723.6532258064517),
                     (479.45161290322585, 739.47580645161293), (671.95967741935488, 744.75), (864.4677419354839, 744.75), (1046.4274193548388, 744.75),
                     (1220.4758064516129, 747.38709677419365), (102.34677419354838, 910.88709677419365), (294.85483870967744, 910.88709677419365),
                     (484.72580645161293, 916.16129032258073), (690.41935483870964, 916.16129032258073), (869.74193548387098, 921.4354838709678),
                     (1054.3387096774193, 926.70967741935488), (1223.1129032258066, 918.79838709677426)]


        self.stateOfGame = []
        #self.printd()
        self.getRGB()
        #print(np.reshape(self.stateOfGame, (6,7)))
        
    
    
    def getRGB(self):
       for i,j in enumerate(self.pos):
            color = self.img[j[1], j[0]]
            
            if color[2] > 179 and color[1] < 150 :
                print (i+1, " RED", color)
                self.stateOfGame.append(1)
            elif color[1] > 140 and color[2] > 140:
                print (i+1, " YELLOW", color)
                self.stateOfGame.append(2)
            else:
                print(i+1, " EMPTY", color)
                self.stateOfGame.append(0)
  
    
    def getState(self): 
        return np.reshape(self.stateOfGame, (6,7))
        
        
    def printd(self):
        im=plt.imread('img/game1.png')
        implot=plt.imshow(im)
        x = plt.ginput(42)
        print(x)
        plt.show()

if __name__ == '__main__':
    a = gameScheme()
    cv2.destroyAllWindows()

