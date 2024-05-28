
import java.util.*;
 
//Edge class
/*Properties:
* - int sourceNode: The source node chosen by the algorithm.
* - int targetNode: The desired target node to travel to from the source node.
* - int weight: The weight value on the edge between two nodes.
* 
* input constructor:
* used to just initialize the edge values.
*/
class Edge
{
  int sourceNode; //Source node (given from the for loop in this case)
  int targetNode; //Target node; node we want to travel to from the source node (given from the for loop as well)
  int weight; //Weight between two nodes on the edges
  
  /*Edge
   * input constructor used to set the edge object's property values from the given values during object creation.
   */
  public Edge(int sourceNode, int targetNode, int weight)
  {
      this.sourceNode = sourceNode;
      this.targetNode = targetNode;
      this.weight = weight;
  }
}
 
// A class to store a heap node
class Node
{
    int nodeNum;
    int weight;
 
    public Node(int num, int weight)
    {
        this.nodeNum = num;
        this.weight = weight;
    }
}
 
// A class to represent a graph object
class Graph
{
    // A list of lists to represent an adjacency list
    List<List<Edge>> adjList = null;
 
    // Constructor
    Graph(List<Edge> edges, int max_nodes)
    {
        adjList = new ArrayList<>();
 
        for (int i = 0; i < max_nodes; i++) {
            adjList.add(new ArrayList<>());
        }
 
        // add edges to the directed graph
        for (Edge edge: edges) {
            adjList.get(edge.sourceNode).add(edge);
        }
    }
}
 
class TestGreedDijk
{
	/*GPS
     * a recursive function used mainly to track the path the algorithm takes to
     * get the shortest route between the two nodes and to return it's value for display
     * purposes.
     */
    public static void GPS(int[] prev, int current_node, List<Integer> path)
    {
        if (current_node >= 0)
        {
            GPS(prev, prev[current_node], path);
            path.add(current_node);
        }
    }
 
    // Run Dijkstra’s algorithm on a given graph
    public static void Dijkstra(Graph graphObj, int sourceNode, int max_nodes)
    {
        PriorityQueue<Node> minHeap; //Uses a priority queue to store the heap
        minHeap = new PriorityQueue<>(Comparator.comparingInt(node -> node.weight)); //Makes a heap of the minimum weight values and compares all of the node's weight values together
        minHeap.add(new Node(sourceNode, 0)); //Initializes the sourcenode's value to 0 since it is the starting node
 
        List<Integer> dist; //Used to store the distance between nodes
        dist = new ArrayList<>(Collections.nCopies(max_nodes, Integer.MAX_VALUE)); //Initializes the default distance values of every other unvisited node to infinity (like in the lecture)
 
        dist.set(sourceNode, 0); //Sets the sourceNode's distance to 0 because there is no distance between itself
 
        boolean[] ShortPathFound = new boolean[max_nodes]; //Used to mark if when the shortest path has been found at the current node
        ShortPathFound[sourceNode] = true; //Sets the source node's pathFound boolean to true because we are starting there
 
        int[] prev = new int[max_nodes]; //Stores the previous node from the current node for display purposes at the end of the algorithm
        prev[sourceNode] = -1;
 
        while (!minHeap.isEmpty()) //While the heap isn't empty, run through the heap and compare the shortest weight values between the edges and if the shortest path has been found, mark it true in the boolean list
        {
            Node node = minHeap.poll(); //Removes and returns the shortest next node to travel to
 
            int curr_node_num = node.nodeNum; //gets nodeNum (number of the given node/current node)
 
            // do for each neighbor `v` of `u`
            for (Edge edge: graphObj.adjList.get(curr_node_num))
            {
                int destNode = edge.targetNode; //Gets the target node (node we want to travel to from source node) renamed to destNode for clarity
                int weight = edge.weight; //gets the weight of the edge given from the graphObj
                
                /*If the shortestpathfound variable is FALSE (shortest path hasn't been found yet) 
                 * then run through the edges of the current sourceNode and compare them to each other
                 * until we find the smallest/shortest distance between them, and add the shortest distance to the heap 
                 * to record the shortest distance from the current source node to the destination node (destNode)
                 */
                if (!ShortPathFound[destNode] && (dist.get(curr_node_num) + weight) < dist.get(destNode))
                {
                    dist.set(destNode, dist.get(curr_node_num) + weight);
                    prev[destNode] = curr_node_num;
                    minHeap.add(new Node(destNode, dist.get(destNode)));
                }
            }
 
            ShortPathFound[curr_node_num] = true; //Once the algorithm is done comparing and gets the shortest distance from the current_node, mark it true in our boolean list to not double check it accidentally
        }
 
        //Path list used for display purposes to track the buildings visited to get to the destination node
        List<Integer> path = new ArrayList<>();
 
        
        //Display for loop that displays the path taken from sourcenode to the given target node
        for (int i = 0; i < max_nodes; i++)
        {
            if (i != sourceNode && dist.get(i) != Integer.MAX_VALUE)
            {
                GPS(prev, i, path);
                System.out.println("The shortest distance from node " + sourceNode + " to " +
                		i + " is " + dist.get(i) + ". Path taken: " + path);
               // System.out.printf("Path (%d —> %d): Minimum cost = %d, Route = %s\n",
                         //       source, i, dist.get(i), route);
               path.clear(); //Resets path list to regenerate for new iteration
            }
        }
    }
 
    /*NUMBER REPRESENTATIONS OF THE BUILDINGS:
    0 - College Square
    1 -  Lewis Science Center
    2 -  Speech Language Hearing 
    3 -  Prince Center
    4 -  Computer Science
    5 -  Burdick
    6 -  Torreyson Library
    7 -  Maintenance College
    8 -  Police Dept
    9 -  Old Main
    10 -  McAlister Hall
    11 -  Fine Art
    12 -  Student health Center
    13 -  Student Center 
    14 -  New Business Building
    15 -  oak Tree Apt
    16 -  Brewer-Hegema
    17 -  Bear Village Apt
    18 - Wingo
    */
    
    
    public static void main(String[] args)
    {
    	// total number of nodes in the graph (labelled from 0 to 4)
        int max_nodes = 19;
        
        //Form the edges based on the given graph from Project3 document
        List<Edge> edges = Arrays.asList(
        		//(node 1, node 2, weight between node 1 and node 2 to create edge)
        		//College Square Edges and weights
        		new Edge(0, 1, 200), 
        		new Edge(0, 3, 300),
        		//Lewis Science center edges and weights
        		new Edge(1, 0, 200),
        		new Edge(1, 4, 150),
        		new Edge(1, 2, 250),
        		//Speech Language Hearing
        		new Edge(2, 1, 250),
        		new Edge(2, 5, 100),
        		new Edge(2, 7, 120),
        		//Prince center
        		new Edge(3, 0, 300),
        		new Edge(3, 4, 80),
        		new Edge(3, 6, 30),
        		new Edge(3, 8, 100),
        		//Computer science
        		new Edge(4, 3, 80),
        		new Edge(4, 1, 150),
        		new Edge(4, 5, 30),
        		new Edge(4, 6, 40),
        		//Burdick
        		new Edge(5, 4, 30),
        		new Edge(5, 2, 100),
        		new Edge(5, 6, 80),
        		new Edge(5, 10, 200),
        		new Edge(5, 7, 300),
        		//TorreySon Library
        		new Edge(6, 3, 30),
        		new Edge(6, 4, 40),
        		new Edge(6, 5, 80),
        		new Edge(6, 9, 30),
        		//Maintenance college
        		new Edge(7, 5, 300),
        		new Edge(7, 2, 120),
        		new Edge(7, 10, 150),
        		new Edge(7, 18, 100),
        		new Edge(7, 14, 150),
        		new Edge(7, 15, 160),
        		//Police deptartment
        		new Edge(8, 3, 100),
        		new Edge(8, 9, 200),
        		new Edge(8, 11, 50),
        		new Edge(8, 12, 100),
        		//Old main
        		new Edge(9, 8, 200),
        		new Edge(9, 6, 30),
        		new Edge(9, 10, 100),
        		new Edge(9, 11, 90),
        		//McAlister Hall
        		new Edge(10, 9, 100),
        		new Edge(10, 5, 200),
        		new Edge(10, 7, 150),
        		new Edge(10, 11, 180),
        		new Edge(10, 13, 100),
        		new Edge(10, 18, 50),
        		//Fine Art
        		new Edge(11, 8, 50),
        		new Edge(11, 9, 90),
        		new Edge(11, 10, 180),
        		new Edge(11, 13, 80),
        		//Student health center
        		new Edge(12, 8, 100),
        		new Edge(12, 13, 50),
        		new Edge(12, 16, 200),
        		//Student center
        		new Edge(13, 12, 50),
        		new Edge(13, 11, 80),
        		new Edge(13, 10, 100),
        		new Edge(13, 18, 100),
        		new Edge(13, 14, 110),
        		//New business building
        		new Edge(14, 13, 110),
        		new Edge(14, 18, 50),
        		new Edge(14, 7, 150),
        		new Edge(14, 15, 30),
        		new Edge(14, 16, 20),
        		//Oak tree apt
        		new Edge(15, 7, 160),
        		new Edge(15, 14, 30),
        		new Edge(15, 16, 40),
        		//Brewer hegema
        		new Edge(16, 12, 200),
        		new Edge(16, 14, 20),
        		new Edge(16, 15, 40),
        		new Edge(16, 17, 350),
        		//Bear village Apt
        		new Edge(17, 16, 350),
        		//Wingo
        		new Edge(18, 13, 100),
        		new Edge(18, 10, 50),
        		new Edge(18, 7, 100),
        		new Edge(18, 14, 50));
  
        //Create graph object
        Graph graph = new Graph(edges, max_nodes);
 
        //Legend/Guide for what node numbers correspond to what building:
        System.out.println(
        		"NODE NUMBER REPRESENTATIONS OF THE BUILDINGS:\r\n"
        		+ "  node  0 - College Square\r\n"
        		+ "  node  1 -  Lewis Science Center\r\n"
        		+ "  node  2 -  Speech Language Hearing \r\n"
        		+ "  node  3 -  Prince Center\r\n"
        		+ "  node  4 -  Computer Science\r\n"
        		+ "  node  5 -  Burdick\r\n"
        		+ "  node  6 -  Torreyson Library\r\n"
        		+ "  node  7 -  Maintenance College\r\n"
        		+ "  node  8 -  Police Dept\r\n"
        		+ "  node  9 -  Old Main\r\n"
        		+ "  node  10 -  McAlister Hall\r\n"
        		+ "  node  11 -  Fine Art\r\n"
        		+ "  node  12 -  Student health Center\r\n"
        		+ "  node  13 -  Student Center \r\n"
        		+ "  node  14 -  New Business Building\r\n"
        		+ "  node  15 -  oak Tree Apt\r\n"
        		+ "  node  16 -  Brewer-Hegema\r\n"
        		+ "  node  17 -  Bear Village Apt\r\n"
        		+ "  node  18 - Wingo"
        		+ "\n-------------------------------------------------------------------------"
                );
        
        
        
        // run the Dijkstra’s algorithm from every node
        for (int sourceNode = 0; sourceNode < max_nodes; sourceNode++) {
        	Dijkstra(graph, sourceNode, max_nodes);
        }
    }
}