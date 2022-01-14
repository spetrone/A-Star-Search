package Astar;

/*
 * 
 * Node object for A* search
 */

public class Node implements Comparable<Node>{
	
	public int f_value;  //f-value, f(n) = g(n) + h(n)
	public int g_value;  //cost from start node to this node
	public int h_value;  //heuristic, estimated value to goal node
	public int index; //identifying index (associated with matrix in the main function)
	public Node parent;
		
		
	//constructor
	public Node(int i, int h){
		this.index = i;
		this.h_value = h;
		
		//initialize g-value to 0		
		this.g_value = 0;		
		this.setCost(); //set f-value
	}		

	//compareTo; nodes are compared by f-value
	 @Override
     public int compareTo(Node n) {		
		 if (this.f_value < n.f_value)  
			 return -1;		
		 else return 1;  
	 }
	 
	 

	 /*****
	  * Override equal function to test indices as nodes being equal
	  * for checking if a node is in the frontier or closed list 
	  * priority queues
	  */
	 @Override
	    public boolean equals(Object o) {
		 
		 boolean equal = false; 
		 
		// If the object is compared with itself then return true 
	        if (o == this) {
	            equal = true;
	        }
	 
	        /* Check if o is an instance of Node */
	        if (!(o instanceof Node)) {	            
	        	equal = false;			 	
	        }
	        else {
	        	Node n = (Node) o; //type cast to compare indices
	 		 	if (n.index == this.index) {
	 		 		equal = true; //same node if indices are the same
	 	        }
	        }
	 
	      return equal;
	 }
	       
	 
	 //setter function for cost
	 public void setCost() {
		 this.f_value = this.g_value + this.h_value;
	 }
	 

}
