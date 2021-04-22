
from queue import PriorityQueue
import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt
# %matplotlib inline

# A* is F = G + H
# where F is the total cost of the node.
# 		G is the distance between the current node and the start node.
# 		H is the heuristic — estimated distance from the current node to the end node.
class Nodes:
	"""docstring for Nodes"""
	def __init__(self, prviousNode=None, currentNude = None ):
		super(Nodes, self).__init__()
		self.prviousNode = prviousNode
		self.currentNude = currentNude
		self.parent = None
		self.F = 0
		self.G = 0
		self.H = 0
		

	def update_location(self, dirctions):
		self.location =[]


		pass


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
			return path

			pass
		pass

def a_star_path_paln(map, start, end):
	start_position = (None, start)

	end_position = (None, end)

	visted_NodeList = []

	not_visted_NodeList=[]

	not_visted_NodeList.append(start_position)

	while len(not_visted_NodeList) > 0:
		# not_visted_NodeList.sort()
		current = not_visted_NodeList.pop(0)
		visted_NodeList.append(current)

		if current == end_position:
			path =[]
			while current!= start_position:
				path.append(current.currentNude)
				current = current.prviousNode
			return path[::-1]

		(x,y) = current.currentNude

		node_neibers = [
		(x-1, y), #up
		(x+1, y), #down
		(x, y-1), #left
		(x, y+1), #right
		]  



	pass

def heuristic(p1, p2):
	x1,y1 = p1
	x2, y2= p2
	return abs(x1 - x2) + abs(y1 - y2)








print("Everything is Okay sofar")

		
