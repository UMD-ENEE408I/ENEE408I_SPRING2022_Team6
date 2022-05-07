import websocket
import json

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
        'e': 'R',
        's': 'B',
        'w': 'L',
        'ne': 'F',
        'en': 'R',
        'se': 'B',
        'es': 'R',
        'sw': 'B',
        'ws': 'L',
        'nw': 'F',
        'wn': 'L'
    }, 'e': {
        'n': 'L',
        'e': 'F',
        's': 'R',
        'w': 'B',
        'ne': 'L',
        'en': 'F',
        'se': 'R',
        'es': 'F',
        'sw': 'R',
        'ws': 'B',
        'nw': 'L',
        'wn': 'B'
    }, 's': {
        'n': 'B',
        'e': 'L',
        's': 'F',
        'w': 'R',
        'ne': 'B',
        'en': 'L',
        'se': 'F',
        'es': 'L',
        'sw': 'F',
        'ws': 'R',
        'nw': 'B',
        'wn': 'R'
    }, 'w': {
        'n': 'R',
        'e': 'B',
        's': 'L',
        'w': 'F',
        'ne': 'R',
        'en': 'B',
        'se': 'L',
        'es': 'B',
        'sw': 'L',
        'ws': 'F',
        'nw': 'R',
        'wn': 'F'
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
    
    def hasEastWest(self):
        return self.directions['e'] or self.directions['en'] or self.directions['es'] or self.directions['w'] or self.directions['wn'] or self.directions['ws']
    
    def hasNorthSouth(self):
        return self.directions['n'] or self.directions['ne'] or self.directions['nw'] or self.directions['s'] or self.directions['sw'] or self.directions['se']
    

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
        self.is_complete = False
        self.instruction = ''
        self.prev_x = 0
        self.prev_y = 0
        self.prev_facing = 'n'
        self.camera_ip_addr = camera_ip_addr
        self.camera_socket = websocket.WebSocket()
        self.camera_socket.connect(f"ws://{self.camera_ip_addr}")
        print(f"[Connected to Junction Processor]")
        print()
        self.mouse_socket = websocket.WebSocket()
        self.mouse_socket.connect(f"ws://{self.mouse.ip}")
        print(f"[Connected to {self.mouse.name}]")
        print()


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

        instruction = instruction_set[direction], direction[-1]

        if (facing == 'n' or facing == 's') and (instruction_set[direction] == 'F' or instruction_set[direction] == 'C'):
            if node.hasEastWest():
                instruction = 'F', direction[-1]
            else:
                instruction = '', direction[-1]
        elif (facing == 'e' or facing == 'w') and (instruction_set[direction] == 'F' or instruction_set[direction] == 'C'):
            if node.hasNorthSouth():
                instruction = 'F', direction[-1]
            else:
                instruction = '', direction[-1]
        elif instruction_set[direction] == 'R':
            instruction = 'R', direction[-1]
        elif instruction_set[direction] == 'L':
            instruction = 'L', direction[-1]
        
        return instruction


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
        
        #with open(r'./Maze Display/data/data.txt', 'w') as file:
        #    file.write(url)
        #    file.close()

        return url


    # Delivers an instruction for the current mouse via GET request to the current mouse's IP address
    # Returns the request's response object
    def sendInstruction(self, instruction):
        self.instruction = self.instruction + instruction
        print(f"Sending {self.instruction} to {self.mouse.name}")
        self.mouse_socket.send(self.instruction)
        mouse_data = self.mouse_socket.recv()
        print(mouse_data)
        self.handleMouseData(mouse_data)
        self.serveNextInstruction(mouse_data[-1])
        return mouse_data

    
    # Creates paths and sets mouse location at current dead end or junction
    def handleMouseData(self, mouse_data):
        node = self.nodes[(self.mouse.x, self.mouse.y)]
        facing = self.mouse.facing

        for instruction in mouse_data:
            if (instruction == 'F'): path_data_set = PATH_DATA_SETS[facing]['forward']
            elif (instruction == 'L'): path_data_set = PATH_DATA_SETS[facing]['forward to left']
            elif (instruction == 'R'): path_data_set = PATH_DATA_SETS[facing]['forward to right']
            else: return
            
            next_x = node.x + path_data_set['x']
            next_y = node.y + path_data_set['y']

            if (next_x, next_y) not in self.nodes:
                next_node = Node(next_x, next_y)
                self.nodes[(next_x, next_y)] = next_node

            next_node = self.nodes[(next_x,next_y)]

            # Mark the path between the current node and the next node as available
            node.directions[path_data_set['to']] = True
            next_node.directions[path_data_set['from']] = True

            node = next_node
            facing = path_data_set['facing']
        
        self.mouse.setLocation(node.x, node.y, facing)

        return


    # Get available paths, VIP name if present, and whether or not the current node is the end of the maze from the camera via GET request to the camera server's IP address for current mouse to get 
    # Returns the request's response object
    def getJunctionData(self, type):
        if(type == 'D'):
            return {
                "paths": {
                    "forward": False,
                    "left": False,
                    "right": False,
                    "forward to left": False,
                    "forward to right": False,
                    "left to forward": False,
                    "right to forward": False,
                    "left to backward": False,
                    "right to backward": False
                },
                "is_end": False,
                "vip": ""
            }
        
        print()
        print(f"[Requesting junction data]")
        print()
        self.camera_socket.send(f"{self.mouse.name}")
        result = self.camera_socket.recv()
        data = json.loads(result)
        self.camera_socket.connect(f"ws://{self.camera_ip_addr}")
        return data


    # Calculates and sends the next instruction for the current mouse
    # Returns the request's response object
    def serveNextInstruction(self, type):

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


            camera_data = self.getJunctionData(type)


            vip = camera_data['vip']

            # If a VIP is present
            if vip is not None and vip != '':
                self.vips[vip] = {
                    'x': self.mouse.x, 
                    'y': self.mouse.y,
                    'name': vip
                }

                if self.mouse.vip == vip and self.end is None:
                    self.mouse.state = 'Searching for end'
                    return self.sendInstruction(f"Y")
                    
                elif self.mouse.vip == vip and not self.end is None:
                    self.mouse.state = 'Traveling to end'
                    instruction, facing = self.bfs(self.mouse.facing, self.mouse.x, self.mouse.y, self.end['x'], self.end['y'])
                    self.mouse.setLocation(self.end['x'], self.end['y'], facing)
                    return self.sendInstruction(f"Y{instruction}")
                
                else:
                    return self.sendInstruction(f"N")


            if camera_data['is_end']:    
                self.end = {
                    'x': self.mouse.x, 
                    'y': self.mouse.y
                }
                return self.sendInstruction(f"Y")


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

                if (len(unvisited_neighbors) > 1):
                    self.stack.append(temp_node)
            
            next_node = unvisited_neighbors[0]['node']
            next_node.visited = True
            self.stack.append(next_node)
            
            if is_neighbor:
                print(self.mouse.facing)
                print(node)
                print(next_node)
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
            

            camera_data = self.getJunctionData()


            vip = camera_data['vip']

            if vip is not None and vip != '' and vip != self.mouse.vip:
                self.vips[vip] = {
                    'x': self.mouse.x, 
                    'y': self.mouse.y,
                    'name': vip
                }
                return self.sendInstruction(f"N")


            if camera_data['is_end']:
                self.end = {
                    'x': self.mouse.x, 
                    'y': self.mouse.y
                }

                self.mouse.state = 'Traveling to end'
                return self.sendInstruction(f"Y")
            
            
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
                
                if (len(unvisited_neighbors) > 1):
                    self.stack.append(temp_node)
            
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
            if self.mouse.vip in self.vips:
                self.mouse.state = 'Traveling to VIP'
            else:
                self.mouse.state = 'Searching for VIP'

            self.mouse_socket.close()
            self.mouse_socket.connect(f"ws://{self.mouse.ip}")
            print(f"Connected to {self.mouse.name}")

        
        elif self.mouse.state == 'Traveling to end' and self.mouse.name == self.mouse2.name:
            self.mouse.state = 'Complete'
            self.mouse = self.mouse3
            if self.mouse.vip in self.vips:
                self.mouse.state = 'Traveling to VIP'
            else:
                self.mouse.state = 'Searching for VIP'
            
            self.mouse_socket.close()
            self.mouse_socket.connect(f"ws://{self.mouse.ip}")
            print(f"Connected to {self.mouse.name}")
            
        elif self.mouse.state == 'Traveling to end' and self.mouse.name == self.mouse3.name:
            self.mouse.state = 'Complete'
            self.is_complete = True
            self.sendInstruction(f"S")
            self.camera_socket.close()
            self.mouse_socket.close()
            self.sendDisplayData()
            print('Demo Complete!')




# Helper Functions

def initialize_demo():
    global maze

    print()

    """mouse_one_vip, mouse_two_vip, mouse_three_vip = input('Enter Mouse VIP Names: ').split()
    print(f"Mouse One VIP: {mouse_one_vip}")
    print(f"Mouse Two VIP: {mouse_two_vip}")
    print(f"Mouse Three VIP: {mouse_three_vip}")
    print()

    mouse_one_ip_addr, mouse_two_ip_addr, mouse_three_ip_addr = input('Enter Mouse IP Addresses: ').split()
    print(f"Mouse One IP Address: {mouse_one_ip_addr}")
    print(f"Mouse Two IP Address: {mouse_two_ip_addr}")
    print(f"Mouse Three IP Address: {mouse_three_ip_addr}")
    print()

    camera_ip_addr = input('Enter Camera IP Address: ')"""

    mouse1 = Mouse('Mouse 1', '192.168.43.54', 0, 0, 'n', 'a')
    mouse2 = Mouse('Mouse 2', 'a', 0, 0, 'n', 'a')
    mouse3 = Mouse('Mouse 3', 'a', 0, 0, 'n', 'a')

    maze = Maze(mouse1, mouse2, mouse3, '127.0.0.1:9000')

    print()


def run_demo():
    print("Starting Demo")
    global maze
    while not maze.is_complete:
        maze.sendInstruction('E')

initialize_demo()
input("Press Enter to Start...")
print()
run_demo()