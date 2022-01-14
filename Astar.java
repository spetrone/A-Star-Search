package Astar;

/***********************************************
 * A* Search
 ************************************************/

import java.util.PriorityQueue;
import java.util.Stack;

public class Astar {
	
	
	
	public static void main (String[] args) {
	
		//cost matrix 
		int[][] cost_matrix = {{0,0,0,6,1,0,0,0,0,0}, //A - 0
				{5,0,2,0,0,0,0,0,0,0}, //B - 1
				{9,3,0,0,0,0,0,0,0,0}, //C - 2
				{0,0,1,0,2,0,0,0,0,0}, //D - 3
				{6,0,0,0,0,2,0,0,0,0}, //E - 4
				{0,0,0,7,0,0,0,0,0,0}, //H - 5
				{0,0,0,0,2,0,0,0,0,0}, //J - 6
				{0,9,0,0,0,0,0,0,0,0}, //G1 - 7
				{0,0,0,5,0,0,0,0,0,0},  //G2 - 8
				{0,0,0,0,0,8,7,0,0,0}};  //G3 - 9
		
		int[] heuristic_vector = {5,7,3,4,6,8,5,0,0,0};
		
		int vistedCount = 0; //accumulator for counting cycles
		int cycleCount = 0; //accumulator for counting node expansions
		
				
		/*frontier list (priority queue based on f-value)*/
		//f-value comparator built into Node class
		PriorityQueue<Node> frontier = new PriorityQueue<Node>();
		
		/*closed list (priority queue based on f-value)*/
		//f-value comparator built into Node class
		PriorityQueue<Node> explored = new PriorityQueue<Node>();
		
		Node goalNode = null; //holds goal node if found
		
		//declare nodes array, save nodes in array to access when comparing
		//f-values from expanded neighbors if they are found to be in the frontier
		Node[] nodes = new Node[10];
		
		//create nodes, initializing with their heuristic value and add to array
		for(int i = 0; i < 10; i++) {
			nodes[i] = new Node(i, heuristic_vector[i]);
		}		
		
		/*put goal nodes (goal states) into vector; this vector 
		is used by the function isGoalState(node n) to test
		if a goal state has been reached */
		Node[] goal_states = {nodes[7], nodes[8], nodes[9]};
		
		

		/*******************************
		 * 
		 * A* search
		 * 
		 **************************/
		
		//sentinel to stop search if goal found 
		//(will be optimal goal given heuristic is admissible (hardcoded as such in this program))
		boolean goalReached = false;		
		
		//Add start node (A - nodes[0]) to the frontier 		
		frontier.add(nodes[0]);		
		
		
		//continue A* until goal state reached or frontier list is empty
		while(!goalReached && frontier.size() > 0) {
			
			cycleCount++; //node is being expanded/explored
						
			//remove head of frontier priority queue
			Node currentNode = frontier.poll();			
					
			//add node to the closed list
			explored.add(currentNode);
			
			//test if it is a goal state
			if (isGoalState(currentNode, goal_states)) {
				goalReached = true;
				goalNode = currentNode; //save goal node to its own variable
				vistedCount++; 
			}
				
			
			//if not goal state, continue A*, otherwise program will now exit the loop
			if(!goalReached) {
				
				//expand current node using node's index and the cost matrix
				//to see if there is a path to any neighbors, note that the node's indices
				//are associated with columns in the matrix
				
				int currentIndex = currentNode.index;
				

				/*Expand node, find all neighbors;
				 * - update all cost values of neighbors (g(n) and f(n) according
				 *     to the realized g(n) value of the current node; 
				 * - check for better paths in neighbors relating to 
				 *     nodes in the frontier list; 
				 * - Adjust frontier list accordingly; 
				 * - add current node to the explored list; */
				
				for (int i = 0; i < cost_matrix[0].length; i++) {
					
					//check edge value of potential neighbor (at index i)
					int edgeValue = cost_matrix[i][currentIndex];
					
					
					if(edgeValue > 0) {    //neighbor exists
						
						vistedCount++;
						
						Node neighborNode = nodes[i]; //node indexed by matrix index
						
						
						//check if node is in open and closed lists	
						//if not in either list, update cost values and parent value, then add to frontier
						if(!(frontier.contains(neighborNode)) && !(explored.contains(neighborNode))) {
							
							neighborNode.parent = currentNode;
							//add step cost to g_value of current node for neighbor's g_value
							neighborNode.g_value = currentNode.g_value + edgeValue;
							neighborNode.setCost();
							//add node to frontier
							frontier.add(neighborNode);
						}
						
						//if neighbor node is already in frontier, compare and keep the better node/path
						else if(frontier.contains(neighborNode)) {
							//if newly found path is better (comparing g-values), change parent of node to currentNode,
							//and update cost values
							if (currentNode.g_value + edgeValue < neighborNode.g_value) {
								neighborNode.parent = currentNode;
								//add step cost with g_value of current node for neighbor's g_value
								neighborNode.g_value = currentNode.g_value + edgeValue;
								neighborNode.setCost();
							}//otherwise do nothing (i.e. ignore this node)					
						}
						/*Re-expanding nodes in the explored list is only necessary if the
						 * heuristic is not consistent. The given heuristic is consistent
						 * so nodes in the explored list are never re-expanded.
						 * Otherwise - if neighbor node is already explored, compare and keep the better node/path; if new 
						 * path is better, remove from explored list and put node back on frontier to be
						 * re-expanded */
						else if(explored.contains(neighborNode)) {
							if (currentNode.g_value + edgeValue < neighborNode.g_value) {
								neighborNode.parent = currentNode;
								//add step cost to g_value of current node for child's g_value
								neighborNode.g_value = currentNode.g_value + edgeValue;
								neighborNode.setCost();
								explored.remove(neighborNode);
								frontier.add(neighborNode);
						    }
					    }
					}
				}				
				
			}
						
		}
		
		//print the results; if no goal found it is a failure, otherwise print the path		
		if (!goalReached)
			System.out.println("\nFailed to find solution...\n");
		else {
			printPath(goalNode);
		}
		
		//print the number of cycles
		System.out.println("\nNodes Visited: " + vistedCount);
		System.out.println("Cycle Count (Nodes expanded, or goal reached): " + cycleCount);
		
	}
	
	/*************
	 *Print the path to a given node
	 *Will print from start towards the goal as the path is reversed
	 *by using a stack
	 *
	 *****************/
	public static void printPath(Node n) {
		
		Node currentNode = n; //start at n and work upwards through graph
		Stack<String> nameStack = new Stack<String>();
		String nodeName = "Z"; //holds name of current node; initialized to z (not a node = error)
		
		/* Backtrack from the goal node to the start node using the 
		 * parent parameter of the Node object. Label the node according
		 * to its index, then place on stack; Print while popping off
		 * stack to reverse path so it starts from the start Node. 
		 */
		while (currentNode.index > 0) {
			
			
			int ind = currentNode.index;
			
			//switch numeric indices to node names for this graph
			switch (ind) {
				case 0: nodeName = "A";
				break;				
				case 1: nodeName = "B";
				break;				
				case 2: nodeName = "C";
				break;				
				case 3: nodeName = "D";
				break;				
				case 4: nodeName = "E";
				break;				
				case 5: nodeName = "H";
				break;				
				case 6: nodeName = "J";
				break;				
				case 7: nodeName = "G1";
				break;				
				case 8: nodeName = "G2";
				break;				
				case 9: nodeName = "G3";
				break;				
				default: nodeName = "Z"; //error				
			}
				
			//add each node to stack
			nameStack.push(nodeName);
			currentNode = currentNode.parent;
		}
		
		/*Print the cheapest path */
		//print while popping off stack 
		System.out.print("Cheapest Path: ");
		
		 System.out.print( " A --> ");
		while (!nameStack.isEmpty()) {
			nodeName = nameStack.pop();
			if(nameStack.size() > 0) //not last node
			 System.out.print(nodeName + " --> ");
			else //last node, don't print arrow
			 System.out.print(nodeName);
		}
		
		//print the goal state:
		System.out.println("\n\nGoal State: " + nodeName); //goal is last node name from above (end of path)
			
	}	
	
	
	/****************
	 * Function to test if the current state is a goal state (i.e.
	 * at a goal node, with inputs of the potential goal node and the goal vector)
	 ****************/
	public static boolean isGoalState(Node n, Node[] gVector) {
		
		boolean match = false; //initialize to no match
		
		//go through vector searching for match
		for(int i = 0 ; i < gVector.length; i++) {
			if (gVector[i].index == n.index)
					match = true;
		}
		
		return match;
	}
		
	 
}
		
