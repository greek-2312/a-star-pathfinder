import numpy as np
from scipy.stats import expon

#generate exponential distribution with sample size 10000
EXPONENTIAL_DISTRIBUTION = expon.rvs(scale=20, size=10000)


class Cell:
    def __init__(self, x, y, total_rows, total_columns, obstacle) -> None:
        self.x = x
        self.y = y

        self.f = 0
        self.h = 0
        self.g = 0

        self.total_rows = total_rows
        self.total_columns = total_columns

        self.start = False
        self.end = False
        
        self.obstacle = True if obstacle > self.draw_probability() else False
        
        self.neighbours = []
        self.predecessor = None

    def draw_probability(self):
        return np.random.randint(0, 100, 1)# np.random.choice(EXPONENTIAL_DISTRIBUTION, 1)
    
# np.random.choice(expon.rvs(scale=10, size=10000)) 
    def add_neighbours(self, grid: list) -> None: 
        '''
                Function to discover the neighbours of the cell, with respect to the borders
            of the grid.
            Input:
                - grid: 2D list;
        '''
        x_coordinate, y_coordinate = self.x, self.y

        if x_coordinate < self.total_columns - 1:
            self.neighbours.append(grid[y_coordinate][x_coordinate + 1])
        if x_coordinate > 0:
            self.neighbours.append(grid[y_coordinate][x_coordinate - 1])

        if y_coordinate < self.total_rows - 1:
            self.neighbours.append(grid[y_coordinate + 1][x_coordinate])
        if y_coordinate > 0: 
            self.neighbours.append(grid[y_coordinate - 1][x_coordinate])


class PathFinder:
    def __init__(self, rows, columns, initial_state, goal_state, obstacle_probability) -> None:
        self.rows = rows
        self.columns = columns
        self.initial_state = initial_state
        self.goal_state = goal_state
        self.obstacle_probability = obstacle_probability

    def estimate_heuristic(self, current_node: Cell, goal_state: Cell) -> int:
        '''
                Function to estimate the heuristic distance between the current node 
            and the end node.
            Input: 
                - current_node: Cell type object;
                - goal_state: Cell type object;
            Return:
                An integer that represents the distance between 2 points.
        '''
        return abs(current_node.x - goal_state[0]) + abs(current_node.y - goal_state[1])
    
    def get_neighbours(self, grid: list) -> list:
        '''
            Function to add the neighbours of the cell.
            Input:
                - grid: 2D list;
            Return:
                A 2D list.
        '''
        for i in range(self.columns):
            for j in range(self.rows):
                grid[i][j].add_neighbours(grid)
        return grid

    def generate_grid(self):
        '''
            Function to generate a 2D matrix and populate it with objects of type Cell.
        Input: None;
        Return: 2D matrix
        '''
        grid = [[Cell(x=x, y=y, total_rows=self.rows, total_columns=self.columns, obstacle=self.obstacle_probability) \
                 for x in range(self.rows)] for y in range(self.columns)]

        grid[self.initial_state[1]][self.initial_state[0]].obstacle = False
        grid[self.goal_state[1]][self.goal_state[0]].obstacle = False

        return grid

    def get_optimal_solution(self, node: Cell, path: list) -> list:
        '''
            Function for backward propagation for finding each coordinate for building the whole path.
        Input: 
            - node: Cell;
            - path: list;
        Returns:
            A list containing the coordinates of the final path.
        '''

        while node.predecessor:
            path.append([node.predecessor.x, node.predecessor.y])
            node = node.predecessor
        return path

    def get_obstacle_grid(self, grid: list) -> list:
        '''
            Function to obtain a matrix where each element is is a boolean value, that gives an intuition on
        the presence of an obstacle in the current cell.
        Input:
            - grid: 2D matrix;
        Returns:
            A list where each element is a boolean value that tells if there is an obstacle in the specific element.
        '''
        return [item.obstacle for row in grid for item in row]

    def select_way(self, path_open: list) -> int:
        ''' 
            Function to find the best way given the open set values.
        Input:
            - path_open: list;
        Returns:
            An integer that represents the index of element that is the next step.
        '''
        best_way = 0

        for i in range(len(path_open)):
            if path_open[i].f < path_open[best_way].f:
                best_way = i

        return best_way

    def find_path(self, path_open: list, path_closed: list, level_idx: int, level_mapper: dict):
        '''
            Recursive function to find the optimal path in a grid environment.
        Input:
            - path_open: list;
            - path_closed: list;
            - level_idx: int;
            - level_mapper: dict;
        If the goal state is reached:
            Returns:
                The current node and level mapper that allows to find the optimal solution.
        Else:
            Returns:
                Calls the function recursively until the optimal solution is found. 
        '''
        way = self.select_way(path_open=path_open)
        current_node = path_open[way]

        if current_node.x == self.goal_state[0] and current_node.y == self.goal_state[1]:
            return current_node, level_mapper
        
        level_mapper[level_idx] = []

        path_open.remove(current_node)
        path_closed.append(current_node)
        neighbours = current_node.neighbours

        for neighbour in neighbours:
            if (neighbour not in path_closed) and (not neighbour.obstacle):
                temp_g = current_node.g + 1
                control_flag = 0

                for k in range(len(path_open)):
                    if neighbour.x == path_open[k].x and neighbour.y == path_open[k].y:
                        if temp_g < path_open[k].g:

                            path_open[k].g = temp_g
                            path_open[k].h = self.estimate_heuristic(path_open[k], self.goal_state)
                            path_open[k].f = path_open[k].g + path_open[k].h
                            path_open[k].predecessor = current_node
                        control_flag = 1

                if control_flag != 1:
                    neighbour.g = temp_g
                    neighbour.h = self.estimate_heuristic(neighbour, self.goal_state)
                    neighbour.f = neighbour.g + neighbour.h
                    neighbour.predecessor = current_node
                    path_open.append(neighbour)
                    level_mapper[level_idx].append([neighbour.x, neighbour.y])
        level_idx += 1
        return self.find_path(path_open, path_closed, level_idx, level_mapper)
    

    def start(self):
        '''
            Function to start the environment generation and optimal solution finding.
        Returns:
            The function returns the optimal solution, the flatten grid for visualization matters, level mapper
        that allows to visualize the nodes that were explored alongside the optimal solution.
        '''
        grid = self.generate_grid()
        grid = self.get_neighbours(grid)
        current_node = grid[self.initial_state[1]][self.initial_state[0]]

        goal_state, level_mapper = self.find_path(path_open=[current_node], path_closed=[], level_idx=0, level_mapper={})
        optimal_solution = [[goal_state.x, goal_state.y]]
        optimal_solution = self.get_optimal_solution(goal_state, optimal_solution)

        grid_flatten = [int(item.obstacle) for sub_list in grid for item in sub_list]

        return optimal_solution, grid_flatten, level_mapper
    

# if __name__ == "__main__":
#     path_finder = PathFinder(rows=10, columns=10, initial_state=[0, 0], goal_state=[6, 9],\
#         obstacle_probability=20)

#     goal_states, grid = path_finder.start()


