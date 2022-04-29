import http.server
import socketserver
import requests


# Mouse 2: 192.168.43.54
# Mouse 3: 192.168.43.46

# Globals

PATH_DATA_SETS = {
    'n': {
        'forward': {
            'x': 0,
            'y': 1,
            'to': 'n',
            'from': 's',
            'facing': 'n'
        },
        'left': {
            'x': -1,
            'y': 0,
            'to': 'w',
            'from': 'e',
            'facing': 'w'
        },
        'right': {
            'x': 1,
            'y': 0,
            'to': 'e',
            'from': 'w',
            'facing': 'e'
        },
        'forward to left': {
            'x': -1,
            'y': 1,
            'to': 'nw',
            'from': 'es',
            'facing': 'w'
        },
        'forward to right': {
            'x': 1,
            'y': 1,
            'to': 'ne',
            'from': 'ws',
            'facing': 'e'
        },
        'left to forward': {
            'x': -1,
            'y': 1,
            'to': 'wn',
            'from': 'se',
            'facing': 'n'
        },
        'right to forward': {
            'x': 1,
            'y': 1,
            'to': 'en',
            'from': 'sw',
            'facing': 'n'
        },
        'left to backward': {
            'x': -1,
            'y': -1,
            'to': 'ws',
            'from': 'ne',
            'facing': 's'
        },
        'right to backward': {
            'x': 1,
            'y': -1,
            'to': 'es',
            'from': 'nw',
            'facing': 's'
        }
    },
    'e': {
        'forward': {
            'x': 1,
            'y': 0,
            'to': 'e',
            'from': 'w',
            'facing': 'e'
        },
        'left': {
            'x': 0,
            'y': 1,
            'to': 'n',
            'from': 's',
            'facing': 'n'
        },
        'right': {
            'x': 0,
            'y': -1,
            'to': 's',
            'from': 'n',
            'facing': 's'
        },
        'forward to left': {
            'x': 1,
            'y': 1,
            'to': 'en',
            'from': 'sw',
            'facing': 'n'
        },
        'forward to right': {
            'x': 1,
            'y': -1,
            'to': 'es',
            'from': 'nw',
            'facing': 's'
        },
        'left to forward': {
            'x': 1,
            'y': 1,
            'to': 'ne',
            'from': 'ws',
            'facing': 'e'
        },
        'right to forward': {
            'x': 1,
            'y': -1,
            'to': 'se',
            'from': 'wn',
            'facing': 'e'
        },
        'left to backward': {
            'x': -1,
            'y': 1,
            'to': 'nw',
            'from': 'es',
            'facing': 'w'
        },
        'right to backward': {
            'x': -1,
            'y': -1,
            'to': 'sw',
            'from': 'en',
            'facing': 'w'
        }
    },
    's': {
        'forward': {
            'x': 0,
            'y': -1,
            'to': 's',
            'from': 'n',
            'facing': 's'
        },
        'left': {
            'x': 1,
            'y': 0,
            'to': 'e',
            'from': 'w',
            'facing': 'e'
        },
        'right': {
            'x': -1,
            'y': 0,
            'to': 'w',
            'from': 'e',
            'facing': 'w'
        },
        'forward to left': {
            'x': 1,
            'y': -1,
            'to': 'se',
            'from': 'wn',
            'facing': 'e'
        },
        'forward to right': {
            'x': -1,
            'y': -1,
            'to': 'sw',
            'from': 'en',
            'facing': 'w'
        },
        'left to forward': {
            'x': 1,
            'y': -1,
            'to': 'es',
            'from': 'nw',
            'facing': 's'
        },
        'right to forward': {
            'x': -1,
            'y': -1,
            'to': 'ws',
            'from': 'ne',
            'facing': 's'
        },
        'left to backward': {
            'x': 1,
            'y': 1,
            'to': 'en',
            'from': 'sw',
            'facing': 'n'
        },
        'right to backward': {
            'x': -1,
            'y': 1,
            'to': 'wn',
            'from': 'se',
            'facing': 'n'
        }
    },
    'w': {
        'forward': {
            'x': -1,
            'y': 0,
            'to': 'w',
            'from': 'e',
            'facing': 'w'
        },
        'left': {
            'x': 0,
            'y': -1,
            'to': 's',
            'from': 'n',
            'facing': 's'
        },
        'right': {
            'x': 0,
            'y': 1,
            'to': 'n',
            'from': 's',
            'facing': 'n'
        },
        'forward to left': {
            'x': -1,
            'y': -1,
            'to': 'ws',
            'from': 'ne',
            'facing': 's'
        },
        'forward to right': {
            'x': -1,
            'y': 1,
            'to': 'wn',
            'from': 'se',
            'facing': 'n'
        },
        'left to forward': {
            'x': -1,
            'y': -1,
            'to': 'sw',
            'from': 'en',
            'facing': 'w'
        },
        'right to forward': {
            'x': -1,
            'y': 1,
            'to': 'nw',
            'from': 'es',
            'facing': 'w'
        },
        'left to backward': {
            'x': 1,
            'y': -1,
            'to': 'se',
            'from': 'wn',
            'facing': 'e'
        },
        'right to backward': {
            'x': 1,
            'y': 1,
            'to': 'ne',
            'from': 'ws',
            'facing': 'e'
        }
    }
}

INSTRUCTION_SETS = {
    'n': {
        'n': 'F',
        'e': 'RF',
        's': 'BF',
        'w': 'LF',
        'ne': 'C',
        'en': 'RC',
        'se': 'BC',
        'es': 'RC',
        'sw': 'BC',
        'ws': 'LC',
        'nw': 'C',
        'wn': 'LC'
    }, 'e': {
        'n': 'LF',
        'e': 'F',
        's': 'RF',
        'w': 'BF',
        'ne': 'LC',
        'en': 'C',
        'se': 'RC',
        'es': 'C',
        'sw': 'RC',
        'ws': 'BC',
        'nw': 'LC',
        'wn': 'BC'
    }, 's': {
        'n': 'BF',
        'e': 'LF',
        's': 'F',
        'w': 'RF',
        'ne': 'BC',
        'en': 'LC',
        'se': 'C',
        'es': 'LC',
        'sw': 'C',
        'ws': 'RC',
        'nw': 'BC',
        'wn': 'RC'
    }, 'w': {
        'n': 'RF',
        'e': 'BF',
        's': 'LF',
        'w': 'F',
        'ne': 'RC',
        'en': 'BC',
        'se': 'LC',
        'es': 'BC',
        'sw': 'LC',
        'ws': 'C',
        'nw': 'RC',
        'wn': 'C'
    }
}

PORT = 8000




# Classes

class Mouse:
    def __init__(self, name, ip, x, y, facing, vip):
        self.state = 'Idle'
        self.name = name
        self.ip = ip
        self.x = x
        self.y = y
        self.facing = facing
        self.vip = vip
    
    def setLocation(self, x, y, facing):
        self.x = x
        self.y = y
        self.facing = facing
    
    
    # Returns formatted data string of mouse data
    def getDisplay(self):
        return f"mouse={self.x},{self.y},{self.facing},{self.name},{self.state},{self.vip}"


class Node: 
    def __init__(self, x, y):
        self.visited = False
        self.directions = {
            'n': False,
            'e': False,
            's': False,
            'w': False,
            'ne': False,
            'en': False,
            'se': False,
            'es': False,
            'sw': False,
            'ws': False,
            'nw': False,
            'wn': False
        }
        self.x = x
        self.y = y
    

    # Gets this node's unvisited_neighbors neighbors
    # Returns list of the following unvisted node dictionary
    # {
    #   'node': an unvisited_neighbors node,
    #   'facing': the direction the mouse would be 'facing' after traveling to it
    # }
    def getUnvisitedNeighbors(self, maze):
        unvisited_neighbors = []
        
        if self.directions['n'] and not maze.nodes[(self.x, self.y+1)].visited:
            unvisited_neighbors.append({
                'node': maze.nodes[(self.x, self.y+1)],
                'facing': 'n'
            })
        if self.directions['e'] and not maze.nodes[(self.x+1, self.y)].visited:
            unvisited_neighbors.append({
                'node': maze.nodes[(self.x+1, self.y)],
                'facing': 'e'
            })
        if self.directions['s'] and not maze.nodes[(self.x, self.y-1)].visited:
            unvisited_neighbors.append({
                'node': maze.nodes[(self.x, self.y-1)],
                'facing': 's'
            })
        if self.directions['w'] and not maze.nodes[(self.x-1, self.y)].visited:
            unvisited_neighbors.append({
                'node': maze.nodes[(self.x-1, self.y)],
                'facing': 'w'
            })
        
        if self.directions['ne'] and not maze.nodes[(self.x+1, self.y+1)].visited:
            unvisited_neighbors.append({
                'node': maze.nodes[(self.x+1, self.y+1)],
                'facing': 'e'
            })
        if self.directions['en'] and not maze.nodes[(self.x+1, self.y+1)].visited:
            unvisited_neighbors.append({
                'node': maze.nodes[(self.x+1, self.y+1)],
                'facing': 'n'
            })
        
        if self.directions['se'] and not maze.nodes[(self.x+1, self.y-1)].visited:
            unvisited_neighbors.append({
                'node': maze.nodes[(self.x+1, self.y-1)],
                'facing': 'e'
            })
        if self.directions['es'] and not maze.nodes[(self.x+1, self.y-1)].visited:
            unvisited_neighbors.append({
                'node': maze.nodes[(self.x+1, self.y-1)],
                'facing': 's'
            })

        if self.directions['sw'] and not maze.nodes[(self.x-1, self.y-1)].visited:
            unvisited_neighbors.append({
                'node': maze.nodes[(self.x-1, self.y-1)],
                'facing': 'w'
            })
        if self.directions['ws'] and not maze.nodes[(self.x-1, self.y-1)].visited:
            unvisited_neighbors.append({
                'node': maze.nodes[(self.x-1, self.y-1)],
                'facing': 's'
            })
        
        if self.directions['nw'] and not maze.nodes[(self.x-1, self.y+1)].visited:
            unvisited_neighbors.append({
                'node': maze.nodes[(self.x-1, self.y+1)],
                'facing': 'w'
            })
        if self.directions['wn'] and not maze.nodes[(self.x-1, self.y+1)].visited:
            unvisited_neighbors.append({
                'node': maze.nodes[(self.x-1, self.y+1)],
                'facing': 'n'
            })
        
        return unvisited_neighbors
    

    # Returns list of this node's neighbor nodes
    def getNeighbors(self, maze):
        neighbors = []

        if self.directions['n']:
            neighbors.append(maze.nodes[(self.x, self.y+1)])
        if self.directions['e']:
            neighbors.append(maze.nodes[(self.x+1, self.y)])
        if self.directions['s']:
            neighbors.append(maze.nodes[(self.x, self.y-1)])
        if self.directions['w']:
            neighbors.append(maze.nodes[(self.x-1, self.y)])
        
        if self.directions['ne']:
            neighbors.append(maze.nodes[(self.x+1, self.y+1)])
        if self.directions['en']:
            neighbors.append(maze.nodes[(self.x+1, self.y+1)])
        
        if self.directions['se']:
            neighbors.append(maze.nodes[(self.x+1, self.y-1)])
        if self.directions['es']:
            neighbors.append(maze.nodes[(self.x+1, self.y-1)])

        if self.directions['sw']:
            neighbors.append(maze.nodes[(self.x-1, self.y-1)])
        if self.directions['ws']:
            neighbors.append(maze.nodes[(self.x-1, self.y-1)])
        
        if self.directions['nw']:
            neighbors.append(maze.nodes[(self.x-1, self.y+1)])
        if self.directions['wn']:
            neighbors.append(maze.nodes[(self.x-1, self.y+1)])
        
        return neighbors


    # Returns formatted data string of node data
    def getDisplay(self):
        found = False
        url = ''
        temp_url = f"{self.x},{self.y}="
        for d in self.directions:
            if self.directions[d]: 
                temp_url = f"{temp_url}{d},"
                found = True
        
        if found: url = temp_url[:-1]
        return url


class Maze:

    def __init__(self, mouse1, mouse2, mouse3, camera_ip_addr):
        self.nodes = {}
        self.nodes[(0,0)] = Node(0,0)
        self.stack = []
        self.vips = {}
        self.end = None
        self.mouse = mouse1
        self.mouse.state = 'Searching for VIP'
        self.mouse1 = mouse1
        self.mouse2 = mouse2
        self.mouse3 = mouse3
        self.camera_ip_addr = camera_ip_addr
        self.is_complete = False
        self.instruction = ''
        self.prev_x = 0
        self.prev_y = 0
        self.prev_facing = 'n'


    # Breadth-first search from one node to another
    # Returns tuple of instructions for the mouse to take and the direction the mouse will be facing after taking the path
    def bfs(self, facing, x, y, x2, y2):
        queue = []
        path = []
        neighbors = []

        curr = self.nodes[(x,y)]

        bfs_visited = {}
        bfs_visited[(x,y)] = None

        while curr.x != x2 or curr.y != y2:
            neighbors = curr.getNeighbors(self)

            for node in neighbors:
                if (node.x, node.y) not in bfs_visited:
                    bfs_visited[(node.x, node.y)] = (curr.x, curr.y)
                    queue.append(node)

            curr = queue.pop(0)
        
        path.insert(0, curr)

        while curr.x != x or curr.y != y:
            curr_x, curr_y = bfs_visited[(curr.x, curr.y)]
            curr = self.nodes[(curr_x,curr_y)]
            path.insert(0, curr)
        
        instruction = ''

        for i in range(len(path) - 1):
            next_instruction, next_facing = self.getSingleInstruction(facing, path[i], path[i + 1])
            instruction = instruction + next_instruction
            facing = next_facing

        return instruction, facing


    # Gets single path from one node to another
    # Returns tuple of the single instruction and the direction the mouse will be facing after taking the path
    def getSingleInstruction(self, facing, node, next_node):
        instruction_set = INSTRUCTION_SETS[facing]
        offset = (next_node.x - node.x, next_node.y - node.y)
        direction = ''
        
        if offset == (0,1): direction = 'n'
        elif offset == (1,0): direction = 'e'
        elif offset == (0,-1): direction = 's'
        elif offset == (-1,0): direction = 'w'
        elif offset == (1,1) and node.directions['ne']: direction = 'ne'
        elif offset == (1,1) and node.directions['en']: direction = 'en'
        elif offset == (1,-1) and node.directions['se']: direction = 'se'
        elif offset == (1,-1) and node.directions['es']: direction = 'es'
        elif offset == (-1,-1) and node.directions['sw']: direction = 'sw'
        elif offset == (-1,-1) and node.directions['ws']: direction = 'ws'
        elif offset == (-1,1) and node.directions['nw']: direction = 'nw'
        elif offset == (-1,1) and node.directions['wn']: direction = 'wn'

        return instruction_set[direction], direction[-1]


    # Updates the display with the location of mice, VIPs and the end of the maze + the last instruction
    # Returns the "url" containing the data sent to the display
    def sendDisplayData(self):
        url = f"?{self.mouse1.getDisplay()}&{self.mouse2.getDisplay()}&{self.mouse3.getDisplay()}"

        for v in self.vips:
            vip = self.vips[v]
            url = f"{url}&vip={vip['x']},{vip['y']},{vip['name']}"

        if self.end is not None:
            url = f"{url}&end={self.end['x']},{self.end['y']}"
        
        for n in self.nodes:
            url = f"{url}&{self.nodes[n].getDisplay()}"

        url = f"{url}&i={self.prev_x},{self.prev_y},{self.prev_facing},{self.instruction}"

        with open(r'./Maze Display/data/data.txt', 'w') as file:
            file.write(url)
            file.close()

        return url


    # Delivers an instruction for the current mouse via GET request to the current mouse's IP address
    # Returns the request's response object
    def sendInstruction(self, instruction):
        self.instruction = self.instruction + instruction
        print(f"http://{self.mouse.ip}/{self.instruction}/instruct")
        return requests.get(f"http://{self.mouse.ip}/{self.instruction}/instruct")
    

    # Get available paths, VIP name if present, and whether or not the current node is the end of the maze from the camera via GET request to the camera server's IP address for current mouse to get 
    # Returns the request's response object
    def getCameraData(self):
        return requests.get(f"http://{self.camera_ip_addr}:9000/{self.mouse.name}")


    # Calculates and sends the next instruction for the current mouse
    # Returns the request's response object
    def serveNextInstruction(self):

        # Update the display
        self.sendDisplayData()

        # Store previous mouse location
        self.prev_x = self.mouse.x
        self.prev_y = self.mouse.y
        self.prev_facing = self.mouse.facing

        # Reset the instruction
        self.instruction = ''




        # CASE: SEARCHING FOR VIP ####################################

        if self.mouse.state == 'Searching for VIP':

            if (self.mouse.x, self.mouse.y) not in self.nodes:
                node = Node(self.mouse.x, self.mouse.y)
                self.nodes[(self.mouse.x, self.mouse.y)] = node
            
            node = self.nodes[(self.mouse.x, self.mouse.y)]
            node.visited = True


            camera_data = self.getCameraData()


            if camera_data['is_end'] and self.end is None:    
                self.end = {
                    'x': self.mouse.x, 
                    'y': self.mouse.y
                }


            vip = camera_data['vip']

            # If a VIP is present
            if vip is not None:
                self.vips[vip] = {
                    'x': self.mouse.x, 
                    'y': self.mouse.y,
                    'name': vip
                }

                if self.mouse.vip == vip and self.end is None:
                    self.mouse.state = 'Searching for end'
                    self.instruction = f"Y"
                    
                elif self.mouse.vip == vip and not self.end is None:
                    self.mouse.state = 'Traveling to end'
                    instruction, facing = self.bfs(self.mouse.facing, self.mouse.x, self.mouse.y, self.end['x'], self.end['y'])
                    self.mouse.setLocation(self.end['x'], self.end['y'], facing)
                    return self.sendInstruction(f"Y{instruction}")
                
                else:
                    self.instruction = f"N"

            paths = camera_data['paths']

            for path in filter(lambda p: paths[p], paths):

                # Get the positional offsets and cardinal directions for the available path
                path_data_set = PATH_DATA_SETS[self.mouse.facing][path]
                
                next_x = self.mouse.x + path_data_set['x']
                next_y = self.mouse.y + path_data_set['y']

                if (next_x, next_y) not in self.nodes:
                    next_node = Node(next_x, next_y)
                    self.nodes[(next_x, next_y)] = next_node

                next_node = self.nodes[(next_x,next_y)]

                # Mark the path between the current node and the next node as available
                node.directions[path_data_set['to']] = True
                next_node.directions[path_data_set['from']] = True


            unvisited_neighbors = node.getUnvisitedNeighbors(self)
            is_neighbor = len(unvisited_neighbors) > 0

            while len(unvisited_neighbors) == 0:
                temp_node = self.stack.pop()
                unvisited_neighbors = temp_node.getUnvisitedNeighbors(self)

            next_node = unvisited_neighbors[0]['node']
            next_node.visited = True
            self.stack.append(next_node)
            
            if is_neighbor:
                instruction, facing = self.getSingleInstruction(self.mouse.facing, node, next_node)
            else:
                instruction, facing = self.bfs(self.mouse.facing, self.mouse.x, self.mouse.y, next_node.x, next_node.y)
            
            self.mouse.setLocation(next_node.x, next_node.y, facing)
            return self.sendInstruction(instruction)



        
        # CASE: TRAVELING TO VIP #####################################

        elif self.mouse.state == 'Traveling to VIP':

            # If the end hasn't been found yet, the mouse travels to their vip, then searches for the end
            if self.end is None:
                self.mouse.state = 'Searching for end'
                vip = self.vips[self.mouse.vip]

                # Get path to the mouse's VIP and send instruction
                instruction, facing = self.bfs(self.mouse.facing, self.mouse.x, self.mouse.y, vip['x'], vip['y'])
                self.mouse.setLocation(self.end['x'], self.end['y'], facing)
                return self.sendInstruction(f"{instruction}Y")
            
            # If the end has been found, the mouse travels to their vip and then travels to the end
            else:
                self.mouse.state = 'Traveling to end'
                vip = self.vips[self.mouse.vip]

                # Get path to the mouse's VIP + the path from the mouse's VIP to the end and send instruction
                vip_instruction, facing = self.bfs(self.mouse.facing, self.mouse.x, self.mouse.y, vip['x'], vip['y'])
                end_instruction, facing = self.bfs(facing, vip['x'], vip['y'], self.end['x'], self.end['y'])
                self.mouse.setLocation(self.end['x'], self.end['y'], facing)
                return self.sendInstruction(f"{vip_instruction}Y{end_instruction}")




        # CASE: SEARCHING FOR END ####################################

        elif self.mouse.state == 'Searching for end':

            if (self.mouse.x, self.mouse.y) not in self.nodes:
                node = Node(self.mouse.x, self.mouse.y)
                self.nodes[(self.mouse.x, self.mouse.y)] = node
            
            node = self.nodes[(self.mouse.x, self.mouse.y)]
            node.visited = True
            

            camera_data = self.getCameraData()


            if camera_data['is_end'] and self.end is None:
                self.end = {
                    'x': self.mouse.x, 
                    'y': self.mouse.y
                }

                self.mouse.state = 'Traveling to end'
                return self.sendInstruction(f"Y")
            
            
            vip = camera_data['vip']

            if vip is not None and vip != self.mouse.vip:
                self.vips[vip] = {
                    'x': self.mouse.x, 
                    'y': self.mouse.y,
                    'name': vip
                }
                self.instruction = 'N'
            
            paths = camera_data['paths']

            for path in filter(lambda p: paths[p], paths):

                # Get the positional offsets and cardinal directions for the available path
                path_data_set = PATH_DATA_SETS[self.mouse.facing][path]
                
                next_x = self.mouse.x + path_data_set['x']
                next_y = self.mouse.y + path_data_set['y']

                if (next_x, next_y) not in self.nodes:
                    next_node = Node(next_x, next_y)
                    self.nodes[(next_x, next_y)] = next_node

                next_node = self.nodes[(next_x, next_y)]

                # Mark the path between the current node and the next node as available
                node.directions[path_data_set['to']] = True
                next_node.directions[path_data_set['from']] = True


            unvisited_neighbors = node.getUnvisitedNeighbors(self)
            is_neighbor = len(unvisited_neighbors) > 0

            while len(unvisited_neighbors) == 0:
                temp_node = self.stack.pop()
                unvisited_neighbors = temp_node.getUnvisitedNeighbors(self)

            next_node = unvisited_neighbors[0]['node']
            next_node.visited = True
            self.stack.append(next_node)
            
            if is_neighbor:
                instruction, facing = self.getSingleInstruction(self.mouse.facing, node, next_node)
            else:
                instruction, facing = self.bfs(self.mouse.facing, self.mouse.x, self.mouse.y, next_node.x, next_node.y)
            
            self.mouse.setLocation(next_node.x, next_node.y, facing)
            return self.sendInstruction(instruction)




        # CASE: TRAVELING TO END #####################################

        elif self.mouse.state == 'Traveling to end' and self.mouse.name == self.mouse1.name:
            self.mouse.state = 'Complete'
            self.mouse = self.mouse2
            if self.vips[self.mouse.vip] is not None:
                self.mouse.state = 'Traveling to VIP'
            else:
                self.mouse.state = 'Searching for VIP'
        
        elif self.mouse.state == 'Traveling to end' and self.mouse.name == self.mouse2.name:
            self.mouse.state = 'Complete'
            self.mouse = self.mouse3
            if self.vips[self.mouse.vip] is not None:
                self.mouse.state = 'Traveling to VIP'
            else:
                self.mouse.state = 'Searching for VIP'
            
        elif self.mouse.state == 'Traveling to end' and self.mouse.name == self.mouse3.name:
            self.mouse.state = 'Complete'
            print('Demo Complete!')
            self.is_complete = True
            return self.sendInstruction(f"S")




# Helper Functions

def get_camera_data():
    # Simulate HTTP GET Request to camera server
    # responds with available paths, present vip, and if the node is the end

    f = input('Forward? ')
    l = input('Left? ')
    r = input('Right? ')
    fl = input('Forward to Left? ')
    fr = input('Forward to Right? ')
    lf = input('Left to Forward? ')
    rf = input('Right to Forward? ')
    lb = input('Left to Backward? ')
    rb = input('Right to Backward? ')

    vip = input('VIP? ')

    if vip == '':
        vip = None
    
    end = input('At the end? ')

    paths =  {
        'forward': f == 'y',
        'left': l == 'y',
        'right': r == 'y',
        'forward to left': fl == 'y',
        'forward to right': fr == 'y',
        'left to forward': lf == 'y',
        'right to forward': rf == 'y',
        'left to backward': lb == 'y',
        'right to backward': rb == 'y'
    }

    is_end = end == 'y'

    return {
        'paths': paths,
        'vip': vip,
        'is_end': is_end
    }


def initialize_demo():
    print()

    mouse_one_vip, mouse_two_vip, mouse_three_vip = input('Enter Mouse VIP Names: ').split()
    print(f"Mouse One VIP: {mouse_one_vip}")
    print(f"Mouse Two VIP: {mouse_two_vip}")
    print(f"Mouse Three VIP: {mouse_three_vip}")
    print()

    mouse_one_ip_addr, mouse_two_ip_addr, mouse_three_ip_addr = input('Enter Mouse IP Addresses: ').split()
    print(f"Mouse One IP Address: {mouse_one_ip_addr}")
    print(f"Mouse Two IP Address: {mouse_two_ip_addr}")
    print(f"Mouse Three IP Address: {mouse_three_ip_addr}")
    print()

    camera_ip_addr = input('Enter Camera IP Address: ')

    mouse1 = Mouse('Mouse 1', mouse_one_ip_addr, 0, 0, 'n', mouse_one_vip)
    mouse2 = Mouse('Mouse 2', mouse_two_ip_addr, 0, 0, 'n', mouse_two_vip)
    mouse3 = Mouse('Mouse 3', mouse_three_ip_addr, 0, 0, 'n', mouse_three_vip)

    global maze
    maze = Maze(mouse1, mouse2, mouse3, camera_ip_addr)

    print()


def test_maze():
    for x in range(1000):
        if not maze.is_complete:
            print(maze.serveNextInstruction())




# Server Functions

def serve_next_instruction():
    global maze
    print(maze.serveNextInstruction())


class Handler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/next':
            serve_next_instruction()

        self.send_response(200)


with socketserver.TCPServer(('', PORT), Handler) as httpd:
    initialize_demo()
    test_maze()
    print("serving at port", PORT)
    httpd.serve_forever()