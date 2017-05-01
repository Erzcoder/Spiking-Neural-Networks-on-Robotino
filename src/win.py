import numpy as np

BOARDWIDTH = 6
BOARDHEIGHT = 7

def findWinner(gameState):

	if isWinner(gameState,2):
		print("roboy win")
	if isWinner(gameState,1):
		print("player win")


def isWinner(game_state, tile):
    # check horizontal spaces
    board=np.reshape(game_state,(6,7))
    #print(board)

    for y in range(BOARDHEIGHT):
        for x in range(BOARDWIDTH - 3):
            if board[x][y] == tile and board[x+1][y] == tile and board[x+2][y] == tile and board[x+3][y] == tile:
                return True

    # check vertical spaces
    for x in range(BOARDWIDTH):
        for y in range(BOARDHEIGHT - 3):
        	#print(x,y)
            if board[x][y] == tile and board[x][y+1] == tile and board[x][y+2] == tile and board[x][y+3] == tile:
                return True

    # check / diagonal spaces
    for x in range(BOARDWIDTH - 3):
        for y in range(3, BOARDHEIGHT):
            if board[x][y] == tile and board[x+1][y-1] == tile and board[x+2][y-2] == tile and board[x+3][y-3] == tile:
                return True

    # check \ diagonal spaces
    for x in range(BOARDWIDTH - 3):
        for y in range(BOARDHEIGHT - 3):
        	#print("test")
            if board[x][y] == tile and board[x+1][y+1] == tile and board[x+2][y+2] == tile and board[x+3][y+3] == tile:
                return True

    return False


state=[2,0,0,0,1,0,0,
	   0,2,0,1,0,0,0,
	   0,0,2,0,0,1,0,
	   0,1,0,2,0,2,0,
	   0,0,0,0,2,0,0,
	   0,0,1,2,0,1,0]

findWinner(state)


