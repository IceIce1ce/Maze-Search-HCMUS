import queue
from collections import deque
import heapq

def readFile(myFile):
    f = open(myFile, "r") 
    myfile_data = f.readline() # read size of the maze
    n = int(myfile_data) # convert line string to int to take size of maze
    adjacent_list = []
    # read adjacent list
    for i in range(64):
        myfile_data = f.readline()
        blank_space = myfile_data.split()
        adjacent_list.append(blank_space)
        # convert list of str to list of int
        # map_object = map(int, blank_space)
        # list_of_int = list(map_object)
        # adjacent_list.append(list_of_int)
    myfile_data = f.readline() # read exit node
    end_node = int(myfile_data)
    f.close()
    return adjacent_list, n, end_node

def writeFile(myFile, path):
    ans = ""
    for i in range(len(path)):
        ans += str(path[i]) 
    f = open(myFile, 'w')
    f.write(ans)
    f.close()

# BFS implementation
# Solution 1 for BFS not added goal to explored set
'''
def BFS(adjacent_list, src, dest):
    explored = [] # list store each node explored
    q = queue.deque()
    q.append(src) # make src node as visited and enqueue it
    # if src = goal state return
    if src == dest:
        return "source = destination"
    # loop until the queue is empty
    while q:
        path = q.popleft() # dequeue the path from queue, use popleft() to reduce time complexity 
        last_node = path[-1] # take last node from the path for checking visited or not, follow FIFO
        # if that node not in explored set
        if last_node not in explored:
            neighbours = adjacent_list[last_node] # take neighbours of that node 
            # then we add neighbours of that node to the queue and make that node as explored
            for neighbour in neighbours:
                path1 = list(path)
                path1.append(neighbour)
                q.append(path1)
                # do until we have neighbour = goal
                if neighbour == dest:
                    return "Time to escape the maze use bfs: " + str(len(explored)) + '\n' + "List of explored nodes use bfs: " + str(explored) + '\n' + "List of nodes on the path found use bfs: " + str(path1)
            explored.append(last_node) # make node as explored after adding neighbours to queue
    # if no path found from src to dest, return
    return "No path is found"
'''
# BFS implementation

# Solution 2 for BFS added goal to explored set
def newBFS(adjacent_list, src, dest):
    explored = [src] # list store each node explored
    frontier = deque([src]) # node wait in queue to be explored
    parentNode = {src: None} # keep parent of node, src is None because it's root 
    # if frontier is not empty  
    while len(frontier) > 0:
        currentNode = frontier.pop() # dequeue the current node from queue
        # if node after pop from frontier is goal, return
        if currentNode == dest: 
            return "current node = dest"
        neighbors = adjacent_list[currentNode] # take neighbours of current node
        for neighbor in neighbors:
            # if node neighbour still not in explored, add it to explored and make it explored
            if neighbor not in explored: 
                explored.append(neighbor) # make neighbour as explored
                frontier.appendleft(neighbor) # add neighbor to left and remove from the right follow FIFO
                parentNode[neighbor] = currentNode # current node we had already pop from frontier is parent of neighbour node
                # do until we have neighbor == goal
                if neighbor == dest:
                    return parentNode, explored
    # if no path from src to dest, return
    return "No path found"

def printNewBFS(src, dest):
        parent = src[dest]
        if parent:
            printNewBFS(src, parent)    
            f = open("output_bfs.txt", "a")
            f.write(parent)
            f.write(' -> ')
            f.write(dest)
            f.write('\n')
            f.close()
        else: 
            f1 = open("output_bfs.txt", "a")
            f1.write("List of nodes on the path found use bfs:")
            f1.write('\n')
            f1.close()    
            print("List of nodes on the path found use bfs:", dest, end='')
            return
        print(' ->', dest, end='')
# Solution 2 for BFS added goal to explored set

# IDS implementation
INFINITY = 30 # set max depth for searching is 30

def DLS(adjacent_list, src, dest, maxDepth = INFINITY):
        frontier = deque([(0, src)]) # node wait in queue to be explored
        explored = [src] # keep node explored, avoid using a node twice
        parentNode = {src: None} # keep parent of node, src is None because it's root 
        # if frontier is not empty  
        while len(frontier) > 0:
            depth, currentNode = frontier.pop() # currentNode keep explored node after pop from frontier
            # if node after pop from frontier is goal, return
            if currentNode == dest: 
                return "current node = dest"
            if maxDepth == -1 or depth < maxDepth:
                neighbors = adjacent_list[currentNode] # take neighbours of current node
                for neighbour in neighbors:
                    # if node neighbour still not in explored, add it to explored and make it explored
                    if neighbour not in explored:
                        explored.append(neighbour) # make neighbour as explored
                        frontier.append((depth + 1, neighbour))
                        parentNode[neighbour] = currentNode # current node we had already pop from frontier is parent of neighbour node
                        # do until we have neighbor == goal
                        if neighbour == dest:
                            print("\n\nTime to escape the maze use ids:", len(explored))
                            print("list of explored node use ids:", explored)
                            # write ids to file
                            f = open("output_ids.txt", "a")
                            f.write("Time to escape the maze use ids: ")
                            f.write(str(len(explored)))
                            f.write('\nList of explored node use ids: ')
                            f.write(str(explored))
                            f.close()
                            # write ids to file
                            return parentNode, explored
        # if no path from src to dest, return
        return None, explored
        
def IDS(adjacent_list, src, dest):
        previous_explored = [] 
        maxDepth = 0
        while 1:
            parentNode, explored = DLS(adjacent_list, src, dest, maxDepth)
            if parentNode or len(explored) == len(previous_explored): 
                return parentNode
            else: 
                previous_explored = explored 
                maxDepth += 1
            
def printIDS(src, dest):
        parent = src[dest]
        if parent:
            printIDS(src, parent)    
            f = open("output_ids.txt", "a")
            f.write(parent)
            f.write(' -> ')
            f.write(dest)
            f.write('\n')
            f.close()
        else: 
            f1 = open("output_ids.txt", "a")
            f1.write('\n')
            f1.write("List of nodes on the path found use ids:")
            f1.write('\n')
            f1.close()    
            print("List of nodes on the path found use ids:", dest, end='')
            return
        print(' ->', dest, end='')
# IDS implementation

# GBFS implementation
def calculate_manhattan(node, dest, size_maze):
    iCurrent, jCurrent = divmod(int(node), size_maze)
    iDest, jDest = divmod(int(dest), size_maze)
    return (abs(iCurrent - iDest) + abs(jCurrent - jDest))

def GBFS(adjacent_list, src, dest, size_maze):
        heuristicCost = calculate_manhattan(src, dest, size_maze) # calculate heuristic of node
        frontier =  [(heuristicCost, src)] # node wait in queue to be explored
        explored = [src] # keep node explored, avoid using a node twice
        parentNode = {src: None} # keep parent of node for path returns, src is None because it's root 
        real_path_cost = {src: 0} # cost so far to reach n, g(n) cost
        # if frontier is not empty  
        while len(frontier) > 0:
            # priority queue should be use heap for better
            currentNode = heapq.heappop(frontier)[1] # pop and return smallest item from the heap (node with the lowest cost first) 
            # if node after pop from frontier is goal, return
            if currentNode == dest: 
                return "current node = dest"
            neighborsInit = adjacent_list[currentNode] # take neighbours of current node
            # convert list neighbor from str to int for sorting, then convert to str again
            neighbor_int = [int(i) for i in neighborsInit]
            neighbor_int.sort(reverse=False)
            neighbors = [str(i) for i in neighbor_int]
            for neighbor in neighbors:
                cost = 1 # set edge weight of node and its neighbour is 1
                neighbour_cost = real_path_cost[currentNode] + cost
                # if node neighbour still not in explored, add it to explored and make it explored
                # check if neighbour is in frontier and has a costlier path to neighbour 
                if neighbor not in explored or real_path_cost[neighbor] > neighbour_cost:
                    explored.append(neighbor) # make neighbour as explored
                    parentNode[neighbor] = currentNode # current node we had already heappop from frontier is parent of neighbour node
                    real_path_cost[neighbor] = neighbour_cost # if neighbour cost < real path cost, update real path cost again
                    # push the value (calculate_manhattan(src, dest), neighbor) onto the frontier
                    heapq.heappush(frontier, (calculate_manhattan(neighbor, dest, size_maze), neighbor))
                    # do until we have neighbor == goal
                    if neighbor == dest:
                        return parentNode, explored
        # if no path from src to dest, return
        return "No path is found"
    
def printGBFS(src, dest):
        parent = src[dest]
        if parent:
            printGBFS(src, parent)    
            f = open("output_gbfs.txt", "a")
            f.write(parent)
            f.write(' -> ')
            f.write(dest)
            f.write('\n')
            f.close()
        else: 
            f1 = open("output_gbfs.txt", "a")
            f1.write("List of nodes on the path found use gbfs:")
            f1.write('\n')
            f1.close()    
            print("List of nodes on the path found use gbfs:", dest, end='')
            return
        print(' ->', dest, end='')
# GBFS implementation

# UCS implementation
def UCS(adjacent_list, src, dest):
        frontier =  [(0, src)] # node wait in queue to be explored
        explored = [src] # keep node explored, avoid using a node twice
        parentNode = {src: None} # keep parent of node for path returns, src is None because it's root 
        real_path_cost = {src: 0} # cost so far to reach n, g(n) cost
        # if frontier is not empty  
        while len(frontier) > 0:
            # priority queue should be use heap for better
            currentNode = heapq.heappop(frontier)[1] # pop and return smallest item from the heap (node with the lowest cost first) 
            # if node after pop from frontier is goal, return
            if currentNode == dest: 
                return "current node = dest"
            neighborsInit = adjacent_list[currentNode] # take neighbours of current node
            # convert list neighbor from str to int for sorting, then convert to str again
            neighbor_int = [int(i) for i in neighborsInit]
            neighbor_int.sort(reverse=False)
            neighbors = [str(i) for i in neighbor_int]
            for neighbor in neighbors:
                cost = 1 # set edge weight of node and its neighbour is 1
                neighbour_cost = real_path_cost[currentNode] + cost
                # if node neighbour still not in explored, add it to explored and make it explored
                # check if neighbour is in frontier and has a costlier path to neighbour 
                if neighbor not in explored or real_path_cost[neighbor] > neighbour_cost:
                    explored.append(neighbor) # make neighbour as explored
                    parentNode[neighbor] = currentNode # current node we had already heappop from frontier is parent of neighbour node
                    real_path_cost[neighbor] = neighbour_cost # if neighbour cost < real path cost, update real path cost again
                    # push the value (neighbour_cost, neighbor) onto the frontier
                    heapq.heappush(frontier, (neighbour_cost, neighbor))
                    # do until we have neighbor == goal
                    if neighbor == dest:
                        return parentNode, explored
        # if no path from src to dest, return
        return "No path is found"
    
def printUCS(src, dest):
        parent = src[dest]
        if parent:
            printUCS(src, parent)    
            f = open("output_ucs.txt", "a")
            f.write(parent)
            f.write(' -> ')
            f.write(dest)
            f.write('\n')
            f.close()
        else: 
            f1 = open("output_ucs.txt", "a")
            f1.write("List of nodes on the path found use ucs:")
            f1.write('\n')
            f1.close()    
            print("List of nodes on the path found use ucs:", dest, end='')
            return
        print(' ->', dest, end='')
# UCS implementation

# Astar implementation
def Astar(adjacent_list, src, dest, size_maze):
        heuristicCost = calculate_manhattan(src, dest, size_maze) # calculate heuristic of node
        frontier =  [(heuristicCost, src)] # node wait in queue to be explored
        explored = [src] # keep node explored, avoid using a node twice
        parentNode = {src: None} # keep parent of node for path returns, src is None because it's root 
        real_path_cost = {src: 0} # cost so far to reach n, g(n) cost
        # if frontier is not empty  
        while len(frontier) > 0:
            # priority queue should be use heap for better
            currentNode = heapq.heappop(frontier)[1] # pop and return smallest item from the heap (node with the lowest cost first)
            # if node after pop from frontier is goal, return
            if currentNode == dest: 
                return "current node = dest"
            neighborsInit = adjacent_list[currentNode] # take neighbours of current node
            # convert list neighbor from str to int for sorting, then convert to str again
            neighbor_int = [int(i) for i in neighborsInit]
            neighbor_int.sort(reverse=False)
            neighbors = [str(i) for i in neighbor_int]
            for neighbor in neighbors:
                cost = 1 # set edge weight of current node and its neighbour is 1
                neighbour_cost = real_path_cost[currentNode] + cost
                # if node neighbour still not in explored, add it to explored and make it explored
                # check if neighbour is in frontier and has a costlier path to neighbour 
                if neighbor not in explored or real_path_cost[neighbor] > neighbour_cost:
                    explored.append(neighbor) # make neighbour as explored
                    parentNode[neighbor] = currentNode # current node we had already heappop from frontier is parent of neighbour node
                    real_path_cost[neighbor] = neighbour_cost # if neighbour cost < real path cost, update real path cost again
                    # push the value (calculate_manhattan(neighbor, dest), neighbor) onto the frontier
                    heapq.heappush(frontier, (neighbour_cost + calculate_manhattan(neighbor, dest, size_maze), neighbor))
                    # do until we have neighbor == goal
                    if neighbor == dest:
                        return parentNode, explored
        # if no path from src to dest, return
        return "No path is found"
    
def printAstar(src, dest):
        parent = src[dest]
        if parent:
            printAstar(src, parent)    
            f = open("output_astar.txt", "a")
            f.write(parent)
            f.write(' -> ')
            f.write(dest)
            f.write('\n')
            f.close()
        else: 
            f1 = open("output_astar.txt", "a")
            f1.write("List of nodes on the path found use astar:")
            f1.write('\n')
            f1.close()    
            print("List of nodes on the path found use astar:", dest, end='')
            return
        print(' ->', dest, end='')
# Astar implementation
    
def main():
    adjacent_list, n, endnode = readFile("input.txt") # n is size of maze
    
    # insert parent node at each list in dictionary
    adjacent_list[0].insert(0, '0')
    adjacent_list[1].insert(0, '1')
    adjacent_list[2].insert(0, '2')
    adjacent_list[3].insert(0, '3')
    adjacent_list[4].insert(0, '4')
    adjacent_list[5].insert(0, '5')
    adjacent_list[6].insert(0, '6')
    adjacent_list[7].insert(0, '7')
    adjacent_list[8].insert(0, '8')
    adjacent_list[9].insert(0, '9')
    adjacent_list[10].insert(0, '10')
    adjacent_list[11].insert(0, '11')
    adjacent_list[12].insert(0, '12')
    adjacent_list[13].insert(0, '13')
    adjacent_list[14].insert(0, '14')
    adjacent_list[15].insert(0, '15')
    adjacent_list[16].insert(0, '16')
    adjacent_list[17].insert(0, '17')
    adjacent_list[18].insert(0, '18')
    adjacent_list[19].insert(0, '19')
    adjacent_list[20].insert(0, '20')
    adjacent_list[21].insert(0, '21')
    adjacent_list[22].insert(0, '22')
    adjacent_list[23].insert(0, '23')
    adjacent_list[24].insert(0, '24')
    adjacent_list[25].insert(0, '25')
    adjacent_list[26].insert(0, '26')
    adjacent_list[27].insert(0, '27')
    adjacent_list[28].insert(0, '28')
    adjacent_list[29].insert(0, '29')
    adjacent_list[30].insert(0, '30')
    adjacent_list[31].insert(0, '31')
    adjacent_list[32].insert(0, '32')
    adjacent_list[33].insert(0, '33')
    adjacent_list[34].insert(0, '34')
    adjacent_list[35].insert(0, '35')
    adjacent_list[36].insert(0, '36')
    adjacent_list[37].insert(0, '37')
    adjacent_list[38].insert(0, '38')
    adjacent_list[39].insert(0, '39')
    adjacent_list[40].insert(0, '40')
    adjacent_list[41].insert(0, '41')
    adjacent_list[42].insert(0, '42')
    adjacent_list[43].insert(0, '43')
    adjacent_list[44].insert(0, '44')
    adjacent_list[45].insert(0, '45')
    adjacent_list[46].insert(0, '46')
    adjacent_list[47].insert(0, '47')
    adjacent_list[48].insert(0, '48')
    adjacent_list[49].insert(0, '49')
    adjacent_list[50].insert(0, '50')
    adjacent_list[51].insert(0, '51')
    adjacent_list[52].insert(0, '52')
    adjacent_list[53].insert(0, '53')
    adjacent_list[54].insert(0, '54')
    adjacent_list[55].insert(0, '55')
    adjacent_list[56].insert(0, '56')
    adjacent_list[57].insert(0, '57')
    adjacent_list[58].insert(0, '58')
    adjacent_list[59].insert(0, '59')
    adjacent_list[60].insert(0, '60')
    adjacent_list[61].insert(0, '61')
    adjacent_list[62].insert(0, '62')
    adjacent_list[63].insert(0, '63')
    
    dict_maze = {}
    for l2 in adjacent_list:
        dict_maze[l2[0]] = l2[1:]
        
    endnode_str = str(endnode) 
    # make for old BFS
    # path = BFS(dict_maze, '0', '61')
    # print(path)
    # writeFile("output_bfs.txt", path)
    
    # make for new BFS
    path, explored = newBFS(dict_maze, '0', endnode_str)
    print("Time escape the maze use bfs:", len(explored))
    print("List of explored node use bfs:", explored)
    printNewBFS(path, endnode_str)
    f = open("output_bfs.txt", "a")
    f.write("Time to escape the maze use bfs: ")
    f.write(str(len(explored)))
    f.write("\nList of explored node use bfs: ")
    f.write(str(explored))
    
    path = IDS(dict_maze, '0', endnode_str)
    printIDS(path, endnode_str)
    
    path, explored = GBFS(dict_maze, '0', endnode_str, n)
    print("\n\nTime escape the maze use gbfs:", len(explored))
    print("List of explored node use gbfs:", explored)
    printGBFS(path, endnode_str)
    f = open("output_gbfs.txt", "a")
    f.write("Time to escape the maze use gbfs: ")
    f.write(str(len(explored)))
    f.write("\nList of explored node use gbfs: ")
    f.write(str(explored))
    
    path, explored = UCS(dict_maze, '0', endnode_str)
    print("\n\nTime to escape the maze use ucs:", len(explored))
    print("List of explored node use ucs:", explored)
    printUCS(path, endnode_str)
    f = open("output_ucs.txt", "a")
    f.write("Time to escape the maze use ucs: ")
    f.write(str(len(explored)))
    f.write("\nList of explored node use ucs: ")
    f.write(str(explored))
    
    path, explored = Astar(dict_maze, '0', endnode_str, n)
    print("\n\nTime to escape the maze use astar:", len(explored))
    print("List of explored node use astar:", explored)
    printAstar(path, endnode_str)
    f = open("output_astar.txt", "a")
    f.write("Time to escape the maze use astar: ")
    f.write(str(len(explored)))
    f.write("\nList of explored node use astar: ")
    f.write(str(explored))
    
if __name__ == '__main__':
	main()