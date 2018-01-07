# ----------
# User Instructions:
# 
# Write a function optimum_policy that returns
# a grid which shows the optimum policy for robot
# motion. This means there should be an optimum
# direction associated with each navigable cell from
# which the goal can be reached.
# 
# Unnavigable cells as well as cells from which 
# the goal cannot be reached should have a string 
# containing a single space (' '), as shown in the 
# previous video. The goal cell should have '*'.
# ----------

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1  # the cost associated with moving from a cell to an adjacent one

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']


def optimum_policy(grid, goal, cost):
    # ----------------------------------------
    # modify code below
    # ----------------------------------------
    value = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]
    policy = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    change = True  # flag for

    # initialize the goal
    value[goal[0]][goal[1]] = 0
    policy[goal[0]][goal[1]] = '*'

    # loop till nothing to be changed (backward propagation)
    # (row, col) : further from goal, (row2, col2) : closer to goal
    while change:
        change = False

        # if something is changed, update 'change'
        for row in range(len(grid)):
            for col in range(len(grid[0])):

                # if current location is navigable
                if grid[row][col] == 0:

                    # check potential step
                    for a in range(len(delta)):
                        row2 = row + delta[a][0]
                        col2 = col + delta[a][1]

                        # edge and non-barrier condition
                        if (row2 >= 0 and row2 < len(grid) and
                            col2 >= 0 and col2 < len(grid[0])) and grid[row2][col2] == 0:

                            # value at the current position (backward)
                            v2 = value[row2][col2] + cost

                            # if the current value is smaller than the past knowledge, update.
                            if v2 < value[row][col]:
                                change = True
                                value[row][col] = v2
                                policy[row][col] = delta_name[a]

    for p in policy:
        print(p)

    return policy


if __name__ == "__main__":
    optimum_policy(grid, goal, cost)
