import json
import numpy as np
from flask import Flask, request, render_template
from a_star import PathFinder
from functools import wraps
from datetime import datetime

app = Flask(__name__)
app.static_folder = "static"

def retry(num_iterations):
    '''
        Decorator function for rerunning the algorithm the given number of iterations.
    '''
    def retry_decorator(func):
        @wraps(func)
        def retried_function(*args, **kwargs):
            for _ in range(num_iterations):
                try:
                    response = func(*args, **kwargs)
                    return response
                except:  
                    print(f'{datetime.now().strftime("%H:%M:%S")} [ERROR] Could not generate an environment with an existing path.')

        return retried_function

    return retry_decorator


@retry(20)
def start_path_finder(finder):
    '''
        Function to start the build of the algorithm and solution finding.
    Input:
        - finder: PathFinder type;
    Returns:
        The optimal solution found for the generated environment, flatten grid and level mapper for
    visualization matters.
    '''
    optimal_solution, grid_flatten, level_mapper = finder.start()
    return optimal_solution, grid_flatten, level_mapper


def open_set_comprehension(data):
    ''' 
        Basic and unoptimzed function for creating combined frames for level mapper values.
    '''
    general_li = []
    for idx in np.arange(len(data))[::3]:
        new_li = []
        try: 
            new_li.extend(data[idx])
            new_li.extend(data[idx + 1])
            new_li.extend(data[idx + 2])
        except:
            pass
        general_li.append(new_li)
    return general_li


@app.route('/', methods=['POST', 'GET'])
def simulate():

    # If the request is of type POST and contains the values for custom generation
    if request.form.get('start-x'):
        initial_state = [int(request.form.get('start-x')), int(request.form.get('start-y'))]
        goal_state = [int(request.form.get('end-x')), int(request.form.get('end-y'))]
        obstacle_probability = float(request.form.get('obstacle-probability')) * 100
        speed = int(request.form.get('speed'))

        path_finder = PathFinder(rows=20, columns=20, \
                                initial_state=initial_state,
                                goal_state=goal_state,
                                obstacle_probability=obstacle_probability)
        
        obstacle_probability = obstacle_probability / 100
    # Else use the default configuration for generation
    else:
        path_finder = PathFinder(rows=20, columns=20, initial_state=[0, 0], \
                        goal_state=[19, 19],
                        obstacle_probability=30)
        initial_state = [0, 0]
        goal_state = [19, 19]
        obstacle_probability = 0.3
        speed = 100
    
    # Find the dolution and reorder the nodes 
    optimal_solution, grid_flatten, level_mapper = start_path_finder(path_finder)# path_finder.start()
    optimal_solution.reverse()
    optimal_solution = optimal_solution[1: -1]

    # Exclude the elements in optimal solution from the level mapper
    # level_mapper = {str(key): element for key, element in level_mapper.items()}
    level_mapper = [[element for element in level if (element not in optimal_solution) and element != goal_state] \
                    for level in level_mapper.values()]

    # Create frames of 3 elements for a faster visualization
    level_mapper = open_set_comprehension(level_mapper)

    # Create the context dictionary 
    context = {
        'terrain_width': 20, 
        'terrain_height': 20, 
        'terrain': grid_flatten, 
        'optimal_solution': optimal_solution,
        'initial_state': initial_state,
        'goal_state': goal_state,
        'obstacle_probability': obstacle_probability,
        'level_mapper': level_mapper,
        'speed': speed
        }
    
    return render_template('index-grid.html', **context)

if __name__ == "__main__":
    app.run(debug=True, port='8282')