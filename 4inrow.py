import numpy as np

def newGame():
    w, h = 7, 6
    matrix = np.arange(42).reshape(6,7)*0
    return matrix

def printGame(G):
    
    for L in G:
        print(L)

def play(G,p,c):
    i=5
    while (i != -1): 
        if G[i][c] == 0:
            G[i][c] = p
            print(neighbours (G, i, c))
            return G, True
        else:
            i-=1
    print("Colonne complete, rejouez!")
    return G, False

def neighbours(G,l,c):
    
    g = G[l][0:c]
    d = G[l][c+1:7]
    h = []
    b = []
    dhg = []
    dhd = []
    for i in range(l):
        h.append(G[i][c])
    for i in range(5-l):
        b.append(G[i+l+1][c])
    return[g,d,h,b,dhd,dhg]


def next(p):
    if (p==1):
        return 2
    return 1

try:
    print(array)
    G = newGame()
    gameOver = False
    p = 1
    while (gameOver != True):
        printGame(G)
        try:
            
            c = input("Inserer le numero de colonne : ")
            G, change = play(G, p, int(c))
            if (change): 
                p = next(p)
        except:
            continue
except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program stop() will be  executed.
	      stop()


