
from queue import PriorityQueue
import matplotlib.pyplot as plt
# %matplotlib inline
"""
A* is F = G + H 
 G is the distance between the current node and the start node.
 H is the heuristic — estimated distance from the current node to the end node.
"""
class Nodes:
	"""docstring for Nodes"""
	def __init__(self, prviousNode=None, currentNude = None):
		super(Nodes, self).__init__()
		self.prviousNode = prviousNode
		self.currentNude = currentNude
		self.parent = None
		self.F = 0
		self.G = 0
		self.H = 0
		

	def update_location(self,  colums,rows):
		return self.colums, self.rows

	def get_location(self, colums,rows):
		return self.colums, self.rows

	def __lt__(self, other):
		return False


# function reconstruct_path(cameFrom, current)
#     total_path := {current}
#     while current in cameFrom.Keys:
#         current := cameFrom[current]
#         total_path.prepend(current)
#     return total_path

def path_safer(prviousNode, currentNude):
		path =[]
		while currentNude != None:
			currentNude = prviousNode[currentNude]
			path.append(currentNude)
		return path
"""
// A* finds a path from start to end.
// map is the heuristic function. map(n) estimates the cost to reach goal from node n.
"""	

def a_star_path_paln(map, start, end):
	start_position = Nodes(None, start)

	end_position = Nodes(None, end)

	visted_NodeList = [] #For node n, visted_NodeList[n] is the node immediately preceding it on the cheapest path from start

	not_visted_NodeList=PriorityQueue()

	not_visted_NodeList.append(start_position)

	while len(not_visted_NodeList) > 0:
		# not_visted_NodeList.sort()
		current = not_visted_NodeList.pop(0)
		visted_NodeList.append(current)

		if current == end_position:
			path =[]
			while current!= end_position:
				path.append(current.currentNude)
				current = current.prviousNode
			return path[::-1]

		(x,y) = current.currentNude

		node_neibers.location = [
		(x-1, y), #up
		(x+1, y), #down
		(x, y-1), #left
		(x, y+1), #right
		]  
		for ii in range((node_neibers)):
			for jj in range(node_neibers):
				
				map_path = map.get(ii),map.get(jj)

				if (map_path!='0'):
					continue

				add_node_to_list = Nodes(ii, current)

				if added_node_list in visted_NodeList:
					pass


 

			# add_node_to_list.G =abs(add_node_to_list.currentNude[0] - start_position.currentNude[0])+abs(add_node_to_list.currentNude[0] - start_position.currentNude[0])
			# add_node_to_list.H =abs(add_node_to_list.currentNude[0] - end_position.currentNude[0])+abs(add_node_to_list.currentNude[0] - end_position.currentNude[0]) 
			# add_node_to_list.F = add_node_to_list.G + add_node_to_list.H 
			add_node_to_list.F[start] = heuristic(start_position.get_location(), end_position.get_location())

			for node in range(not_visted_NodeList):
				if (add_node_to_list == node and add_node_to_list.F >= node.F):
					not_visted_NodeList.append(add_node_to_list)

				pass
		return None




	pass

def heuristic(point1, point2):
	x1,y1 = point1
	x2, y2= point2
	return abs(x1 - x2) + abs(y1 - y2)








print("Everything is Okay sofar")

		
