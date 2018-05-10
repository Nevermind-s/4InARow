#!/usr/bin/env python2
import numpy as np
import random
import copy
import matplotlib.pyplot as plt


# init game
def newGame():
    w, h = 7, 6
    matrix = np.arange(42).reshape(6, 7)*0
    return matrix, False
# print game status


def printGame(G, p):
    print(G)
    # plt.matshow(G)
    # plt.show()
# adapt the position of the coin after insersion in one column


def play(G, p, c):
    i = 5
    while (i != -1):
        if G[i][c] == 0:
            G[i][c] = p
            gameOver = iswon(G, i, c)
            p = next(p)
            return G, p, gameOver
        else:
            i -= 1
    return G, p, False
# determine next guy who has to play


def next(p):
    if (p == 2):
        return 1
    return 2
# return vertical, horizontal and both diagonals of one position


def neighbours(G, l, c):
    h = G[l]
    v = []
    for i in range(6):
        v.append(G[i][c])
    d1 = np.diag(G, c-l)
    d2 = np.diag(np.fliplr(G), (6-c)-l)
    return h, v, d1, d2
# determine if the game is won or not


def iswon(G, l, c):
    for arr in neighbours(G, l, c):
        w = 0
        if len(arr) > 3:
            for i in range(len(arr)-1):
                if arr[i+1] == arr[i] & arr[i] != 0:
                    w += 1
                    if w > 2:
                        return True
                else:
                    w = 0
    return False
# implementation of the IA


def IA(G, p):
    c = winingPlay(G, p)  # win if winable
    if c == []:
        c = winingPlay(G, next(p))  # prevent lose if loseable
    if c == []:
        l = possibilities(G, p)
        if len(l) == 0:
            l0 = freeCol(G)
            return l0[0]
        else:
            l2 = possibilities(G, next(p))
            a = list(set(l).intersection(l2))
            if len(a) != 0:
                L = min(a, key=lambda x: abs(x-3))
                return L
            else:
                return min(l, key=lambda x: abs(x-3))

            # i = random.randint(0,len(a)-1) # choose randomly
            # return l[i]
    else:
        return c[0]


def getposition(G, a):
    L = []
    for elem in a:
        for i in range(5):
            if G[5-i][elem] != 0:
                L.append(5-i)
    return L


def winingPlay(G, p):  # search one-turn winning play
    gameOver = False
    l = []
    i = 0
    while (i < 7):
        G1 = copy.copy(G)
        G1, p1, gameOver = play(G1, p, i)
        if gameOver:
            l.append(i)
        i += 1
    return l


def losingPlay(G, p0, i):  # search losing plays
    G1 = copy.copy(G)
    G1, p1, gameOver = play(G1, p0, i)
    G2 = copy.copy(G1)
    G2, p2, gameOver = play(G2, p1, i)
    if gameOver:
        return -1
    else:
        for j in range(1, 6):
            G3 = copy.copy(G1)
            G3, p3, gameOver = play(G3, p1, j)
            if len(winingPlay(G3, p1)) > 1:
                return -1
            elif len(winingPlay(G3, p1)) == 1:
                G3, p3, gameOver = play(G3, p3, winingPlay(G3, p1)[0])
                if len(winingPlay(G3, p1)) > 0:
                    return -1
        return 0


def possibilities(G, p):  # give all tactical play
    l = freeCol(G)
    i = 0
    while i < len(l):
        if losingPlay(G, p, l[i]) == -1:
            l.remove(l[i])
        else:
            i += 1
    return l


def freeCol(G):
    l = []
    for i in range(7):
        if G[0][i] == 0:
            l.append(i)
    return l
