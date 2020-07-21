/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.List;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.PriorityQueue;

import geography.GeographicPoint;
import util.GraphLoader;


/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph{
	//TODO: Add your member variables here in WEEK 3
	private HashMap<GeographicPoint,MapNode> map;
	private HashSet<MapEdge> edges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		map=new HashMap<>();
		edges=new HashSet<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return map.values().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return map.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return edges.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		if (map.containsKey(location) || location.equals(null))
		return false;
		MapNode v=new MapNode(location);
		map.put(location,v);
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 3
		MapNode n1 = map.get(from);
		MapNode n2 = map.get(to);
		
		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: from:" + from + "is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: to:" + to + "is not in graph");
		MapEdge e=new MapEdge(n1, n2, roadName, roadType, length);
		edges.add(e);
		n1.addedge(e);
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		if (start == null || goal == null) {
			throw new NullPointerException("Start or goal node is null!  No path exists.");
		}
		
		MapNode startNode = map.get(start);
		MapNode endNode = map.get(goal);
		if (startNode == null || endNode == null) {
			System.out.println("Start or goal node is null! No path exists.");
			return null;
		}
		
		HashMap<MapNode,MapNode> parentMap = new HashMap<>();
		boolean found= search(startNode,endNode,parentMap,nodeSearched);
		//add
		
        if (!found) {
			System.out.println("No path exists");
			return new ArrayList<GeographicPoint>();
		}
		// reconstruct the path
		return reconstruct_path(startNode,endNode,parentMap);
	
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

	}
	
	private static boolean search(MapNode start, 
			 					     MapNode goal,HashMap<MapNode,MapNode> parentMap,
			 					     Consumer<GeographicPoint> nodeSearched)
	{   HashSet<MapNode> visited = new HashSet<MapNode>();
	    Queue<MapNode> toExplore = new LinkedList<>();
		toExplore.add(start);
		boolean found = false;
		while (!toExplore.isEmpty()) {
			MapNode curr = toExplore.remove();
			// hook for visualization
						nodeSearched.accept(curr.getlocation());
			if (curr.equals(goal)) {
				found = true;
				break;
			}
			//List<MapEdge> neighbors = curr.getedges();
			Set<MapNode> neighbors=curr.getNeighbors();
			for(MapNode next: neighbors ) {	
			   if (!visited.contains(next))
				{ 
					visited.add(next);
					parentMap.put(next, curr);
					toExplore.add(next);
				}
			}
		}
		return found;
	}
	
	private List<GeographicPoint> reconstruct_path(MapNode start, 
			 					     MapNode goal,HashMap<MapNode, MapNode> parentMap)
	{LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
	MapNode curr = goal;
	while (!curr.equals(start)) {
		path.addFirst(curr.getlocation());
		curr = parentMap.get(curr);
	}
	path.addFirst(start.getlocation());
	return path;}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		if (start == null || goal == null) {
			throw new NullPointerException("Start or goal node is null!  No path exists.");
		}
		
		MapNode startNode = map.get(start);
		MapNode endNode = map.get(goal);
		if (startNode == null || endNode == null) {
			System.out.println("Start or goal node is null! No path exists.");
			return null;
		}
		
		HashMap<MapNode,MapNode> parentMap = new HashMap<>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
	    PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>();
	    //initialize all distance
	    for(MapNode m:map.values()) {
	    	m.updatecurrentdist(Double.POSITIVE_INFINITY);
	    }
	    
		boolean found= dijsearch(startNode,endNode,parentMap,nodeSearched,visited,toExplore);
		//add
		
        if (!found) {
			System.out.println("No path exists");
			return new ArrayList<GeographicPoint>();
		}
		// reconstruct the path
		return reconstruct_path(startNode,endNode,parentMap);
	
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

	}
	
	private static boolean dijsearch(MapNode start, 
			 					     MapNode goal,HashMap<MapNode,MapNode> parentMap,
			 					     Consumer<GeographicPoint> nodeSearched,
			 					     HashSet<MapNode> visited,PriorityQueue<MapNode> toExplore)
	{   
	    toExplore.add(start);
		boolean found = false;
		start.updatecurrentdist(0);
		int count=0;
		while (!toExplore.isEmpty()) {
			MapNode curr = toExplore.poll();
			//count++;
			// hook for visualization
		    nodeSearched.accept(curr.getlocation());
		    if (!visited.contains(curr))
		    {   visited.add(curr);
		        count++;
		    	if (curr.equals(goal)) {
				found = true;
				System.out.println("d "+count);
				break;
			    }
			Set<MapNode> neighbors=curr.getNeighbors();
		    HashMap<MapNode,Double> dis=curr.getNeighborsDistance();
			double currentdist=0;
			for(MapNode next: neighbors ) {	
			   if (!visited.contains(next))
				{  currentdist=curr.getstartdis()+dis.get(next); 
				   if (next.getstartdis()>currentdist)
					 parentMap.put(next,curr);
					 next.updatecurrentdist(currentdist);
					 toExplore.add(next);
				}
			}
		}
	}
		return found;
	}
	


	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		if (start == null || goal == null) {
			throw new NullPointerException("Start or goal node is null!  No path exists.");
		}
		
		MapNode startNode = map.get(start);
		MapNode endNode = map.get(goal);
		if (startNode == null || endNode == null) {
			System.out.println("Start or goal node is null! No path exists.");
			return null;
		}
		
		HashMap<MapNode,MapNode> parentMap = new HashMap<>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
	    PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>();
	    //initialize all distance
	    for(MapNode m:map.values()) {
	    	m.updatecurrentdist(Double.POSITIVE_INFINITY);
	    	m.updateenddist(Double.POSITIVE_INFINITY);
	    }
	    
		boolean found= astarsearch(startNode,endNode,parentMap,nodeSearched,visited,toExplore);
		//add
		
        if (!found) {
			System.out.println("No path exists");
			return new ArrayList<GeographicPoint>();
		}
		// reconstruct the path
		return reconstruct_path(startNode,endNode,parentMap);
	}

	private static boolean astarsearch(MapNode start, 
		     MapNode goal,HashMap<MapNode,MapNode> parentMap,
		     Consumer<GeographicPoint> nodeSearched,
		     HashSet<MapNode> visited,PriorityQueue<MapNode> toExplore)
    {   
           toExplore.add(start);
           boolean found = false;
           int count2=0;
           start.updatecurrentdist(0);
           start.updateenddist(0);
           while (!toExplore.isEmpty()) {
           MapNode curr = toExplore.poll();
           // hook for visualization
           nodeSearched.accept(curr.getlocation());
           if (!visited.contains(curr))
           {   visited.add(curr);
               count2++;
               if (curr.equals(goal)) {
               found = true;
               System.out.println(count2);
               break;
               }
            Set<MapNode> neighbors=curr.getNeighbors();
            HashMap<MapNode,Double> dis=curr.getNeighborsDistance();
            double currentdist=0;
            for(MapNode next: neighbors ) {	
                if (!visited.contains(next))
                {  currentdist=curr.getstartdis()+dis.get(next);
                   double enddis=(next.getlocation()).distance(goal.getlocation());
                   double total=currentdist+enddis;
                    if (next.getenddis()>total)
                       parentMap.put(next,curr);
                       next.updatecurrentdist(currentdist);
                       next.updateenddist(total);
                       toExplore.add(next);
						}
						}
						}
						}
	  return found;
    }

	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		System.out.println("Num nodes: " + firstMap.getNumVertices()); // should be 9
		System.out.println("Num edges: " + firstMap.getNumEdges()); // should be 22
		
		List<GeographicPoint> route3 = firstMap.bfs(new GeographicPoint(1.0, 1.0), new GeographicPoint(8.0, -1.0));
		System.out.println(route3);
		
		MapGraph firstMap2 = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap2);
		List<GeographicPoint> sroute = firstMap2.dijkstra(new GeographicPoint(1.0, 1.0), new GeographicPoint(8.0, -1.0));
		System.out.println(sroute);
		
		MapGraph firstMap1 = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap1);
		List<GeographicPoint> aroute = firstMap1.aStarSearch(new GeographicPoint(1.0, 1.0), new GeographicPoint(8.0, -1.0));
		System.out.println(aroute);
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		MapGraph theMap1 = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap1);
		List<GeographicPoint> route2 = theMap1.aStarSearch(start,end);

		
		
	}
	
}
