# ----------
# User Instructions:
# 
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']


def search(grid, init, goal, cost):
    # ----------------------------------------
    # insert code here
    # ----------------------------------------

    # define the array for 'done or not'
    closed = [[0 for _row in range(len(grid[0]))] for _col in range(len(grid))]
    closed[init[0]][init[1]] = 1
#    expand = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
#    action = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]

    row, col = init
    g = 0  # g-value (the number of step)
    count = 1
    open = [[g, row, col]]

    # define flag
    flg_found = False  # when search is done
    flg_resign = False  # when dead end is found

    # loop until the either of flg becomes true
    while not flg_found and not flg_resign:

        # if dead end, return 'fail'
        if len(open) == 0:
            flg_resign = True
            return 'fail'

        else:
            # sort the candidates and take out the least g-value one
            open.sort()
            next = open.pop(0)
            row = next[1]
            col = next[2]
            g = next[0]

            # complete condition
            if row == goal[0] and col == goal[1]:
                found = True
                return [g, row, col]
            
            # else, add the next step to open
            else:
                for i in range(len(delta)):  # apply all the pattern
                    row2 = row + delta[i][0]
                    col2 = col + delta[i][1]

                    # edge condition
                    if (row2 >= 0 and row2 <= (len(grid)-1)
                            and col2 >= 0 and col2 <= (len(grid[0])-1)):
                        
                        # new possible step
                        if closed[row2][col2] == 0 and grid[row2][col2] == 0:
                            g2 = g + cost  # update g-value
                            open.append([g2, row2, col2])  # add new step to open
                            closed[row2][col2] = 1  # update closed

    # outcome
    path = g
    
    return path

print(search(grid, init, goal, cost))
