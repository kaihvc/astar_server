import flask
import numpy as np
import heapq

def e_dist(x1, x2):
    return np.sqrt((x1[0] - x2[0])**2 + (x1[1] - x2[1])**2)

def convert_to_graph(env_map, x_size, y_size):

    graph_dict = {}
    for x in range(x_size):
        for y in range(y_size):
            neighbors = {}
            for i in range(-1, 2):
                for j in range(-1, 2):
                    if x + i in range(x_size) and y + j in range(y_size) and (i != 0 or j != 0):
                        if env_map[x + i][y + j] == 0:
                            neighbors[(x + i, y + j)] = np.sqrt(i**2 + j**2)
            graph_dict[(x, y)] = neighbors
    return graph_dict

def path(prev, goal):
    if goal not in prev:
        return []
    else:
        return path(prev, prev[goal]) + [goal]

class Frontier_PQ:

    def __init__(self, start, cost):
        self.start = start
        self.cost = cost
        self.states = {start: cost}
        self.q = [(cost, start)]
        heapq.heapify(self.q)

    def contains(self, node):
        for elt in self.q:
            if elt[1] == node:
                return True
        return False

    def add(self, state, cost):
        heapq.heappush(self.q, (cost, state))
        if(state not in self.states):
            self.states[state] = cost

    def pop(self):
        return heapq.heappop(self.q)

    def replace(self, state, cost):
        self.states[state] = cost

def astar_search(start, goal, state_graph, heuristic, return_cost = False):

    # initialize explored set, frontier queue, and previous dict to track path
    explored = []
    frontier = Frontier_PQ(start, 0)
    previous = {start: None}

    # goal check
    if(start == goal):
        previous[goal] = start
        return (path(previous, goal), 0) if return_cost else (path(previous, goal))

    # loop while frontier isn't empty
    while(frontier.q):
        node = frontier.pop()
        while node[1] in explored:
            node = frontier.pop()

        neighbors = state_graph[node[1]]

        for succ in neighbors:

            if((not frontier.contains(succ)) and succ not in explored):

                # cost_to_succ is g, the cost to get to the successor; succ score is f = g + h
                cost_to_succ = frontier.states[node[1]] + state_graph[node[1]][succ]
                succ_score = cost_to_succ + heuristic(succ, goal)
                frontier.add(succ, succ_score)

                if(cost_to_succ <= frontier.states[succ]):
                    previous[succ] = node[1]
                    frontier.replace(succ, cost_to_succ)

        explored.append(node[1])
        if(goal in explored):
            pathToGoal = path(previous, goal)
            return (pathToGoal, pathcost(pathToGoal, state_graph)) if return_cost else (pathToGoal)

    print(explored[-50:-1])
    return ["failure"], -1 if return_cost else ["failure"]

def path_planner(sg, start, end):
    '''
    :param map: A 2D numpy array of size 360x360 representing the world's cspace with 0 as free space and 1 as obstacle
    :param start: A tuple of indices representing the start cell in the map
    :param end: A tuple of indices representing the end cell in the map
    :return: A list of tuples as a path from the given start to the given end in the given maze
    '''
    map_path = astar_search(start, end, sg, e_dist)
    print(map_path)
    world_path = []
    interval = max(int(len(map_path) / 20), 1)
    for i, elt in enumerate(map_path):
        if i % interval == 0:
            world_path.append((elt[0] * 7 / 350, elt[1] * 7 / 350))

    final = (map_path[1][0] * 7 / 350, map_path[1][1] * 7 / 350)
    if final not in world_path: world_path.append(final)

    return world_path

app = flask.Flask(__name__)
env_map = np.transpose(np.load('map_processed.npy'))
state_graph = convert_to_graph(env_map, len(env_map), len(env_map[0]))

world_size = 7
map_size = len(env_map)

@app.route('/', methods=['POST'])
def get_path():
    print('Request received')
    req = flask.request.get_json()
    start_w = req['start']
    end_w = req['goal']

    # Convert the start_w and end_W from webot's coordinate frame to map's
    start = (int(start_w[0] * map_size / world_size),
             int(start_w[1] * map_size / world_size))
    end = (int(end_w[0] * map_size / world_size),
             int(end_w[1] * map_size / world_size))

    print('Planning...')
    map_path = path_planner(state_graph, start, end)
    print('Planning complete.')
    return flask.jsonify({'path': map_path})

if __name__ == "__main__":
    app.run(debug=True)
