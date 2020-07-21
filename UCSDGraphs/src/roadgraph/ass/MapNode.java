package roadgraph;

import geography.GeographicPoint;
import java.util.*;

public class MapNode implements Comparable{
	private GeographicPoint location;
    private HashSet<MapEdge> edges;
    private double startdis; 
    private double enddis;
    
    public MapNode(GeographicPoint location) {
    	this.location=location;
    	this.edges=new HashSet<>();
    	this.startdis=0;
    	this.enddis=0;
    }    
    
    public GeographicPoint getlocation() {
    	return location;
    }
    
    public HashSet<MapEdge> getedges(){
    	return edges;
    }
    
    public void addedge(MapEdge e){
    	edges.add(e);
    }
   
    /** Return the neighbors of this MapNode */
	public Set<MapNode> getNeighbors()
	{
		Set<MapNode> neighbors = new HashSet<MapNode>();
		for (MapEdge edge : edges) {
			neighbors.add(edge.getOtherNode(this));
		}
		return neighbors;
	}
	
	/*returns current node dist from start*/
	public HashMap<MapNode,Double> getNeighborsDistance()
	{   HashMap<MapNode,Double> dis=new HashMap<>();
	    for (MapEdge edge : edges) {
		dis.put(edge.getOtherNode(this),edge.getlength());
	    }
	    return dis;
	}
	
	
	public void updatecurrentdist(double current)
	{  
		    this.startdis=current;
    }
	 
	public double getstartdis()
	{return this.startdis;
	}
	
	//for dist from goal
	public void updateenddist(double current)
	{  
		    this.enddis=current;
    }
	 
	public double getenddis()
	{return this.enddis;
	}
	
	
	/** Returns whether two nodes are equal.
	 * Nodes are considered equal if their locations are the same, 
	 * even if their street list is different.
	 */
	public boolean equals(Object o)
	{
		if (!(o instanceof MapNode) || (o == null)) {
			return false;
		}
		MapNode node = (MapNode)o;
		return node.location.equals(this.location);
	}
	
	// Code to implement Comparable
		public int compareTo(Object o) {
			// convert to map node, may throw exception
			MapNode m = (MapNode)o; 
			return ((Double)this.getstartdis()).compareTo((Double) m.getstartdis());
		}
}
