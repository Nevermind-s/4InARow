import numpy as np
import random
import copy
import matplotlib.pyplot as plt

# init game
def newGame():
    w, h = 7, 6
    matrix = np.arange(42).reshape(6,7)*0
    return matrix,False
# print game status
def printGame(G,p):
    print(G)
    plt.matshow(G)
    plt.show()
# adapt the position of the coin after insersion in one column
def play(G,p,c):
    i=5
    while (i != -1): 
        if G[i][c] == 0:
            G[i][c] = p
            gameOver=iswon(G, i, c)
            p=next(p)
            return G, p,gameOver
        else:
            i-=1
    return G, p, False
# determine next guy who has to play
def next(p):
    if (p==1):
        return 2
    return 1
# return vertical, horizontal and both diagonals of one position
def neighbours(G,l,c):
    h = G[l]
    v = []
    for i in range(6):
        v.append(G[i][c])
    d1=np.diag(G,c-l)
    d2=np.diag(np.flip(G,1),(6-c)-l)
    return h,v,d1,d2
# determine if the game is won or not
def iswon(G,l,c):
    for arr in neighbours(G,l,c):
        w=0
        if len(arr)>3:
            for i in range(len(arr)-1):
                if arr[i+1]==arr[i] & arr[i]!=0:
                    w+=1
                    if w>2:
                        return True
                else:
                    w=0
    return False


#test

# implementation of the IA
def IA(G,p):
    c = winingPlay(G,p) # win if winable
    if c==-1:
        c=winingPlay(G,next(p)) # prevent lose if loseable
    if c==-1:
        l = possibilities(G,p)
        i = random.randint(0,len(l)-1) # choose randomly
        return l[i]
    else:
        return c

def winingPlay(G,p): # search one-turn winning play
    gameOver=False
    i=0
    while (i<7 and not(gameOver)):
        G1=copy.copy(G)
        G1,p1,gameOver = play(G1,p,i)
        if gameOver:
            print("YEAH!")
            return i
        else:
            i+=1
    return -1

def losingPlay(G,p,i): # search losing plays
    G1=copy.copy(G)
    G1,p1,gameOver = play(G1,p,i)
    G1,p1,gameOver = play(G1,p1,i)
    if gameOver:
        return -1
    else:
        return 0

def possibilities(G,p): # give all tactical play
    l=freeCol(G)
    i=0
    while i<len(l):
        if losingPlay(G,p,l[i])==-1:
            print(i)
            l.remove(l[i])
        else:
            i+=1
    return l
def freeCol(G):
    l=[]
    for i in range(7):
        if G[0][i]==0:
            l.append(i)
    return l


# TEST

# MAIN
G,gameOver = newGame()
p = 1
printGame(G,p)
while (gameOver != True):
    if p==1:
        try:
            c = input("Inserer le numero de colonne : ")
            G, p,gameOver = play(G, p, int(c))
        except:
            pass
    else:
        print("IA's turn")
        c = IA(G, p)
        G, p,gameOver = play(G, p,c)
        printGame(G,p)