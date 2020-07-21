package roadgraph;

import geography.GeographicPoint;

public class MapEdge  {
	private MapNode from;
	private MapNode to;
	private String roadName;
	private String roadType;
	private double length;
  
	public MapEdge(	MapNode n1,MapNode n2,String roadName,
			String roadType,double length) {
		from=n1;
		to=n2;
	    this.roadName=roadName;
		this.roadType=roadType;
		this.length=length;
	}
	
	// given one node in an edge, return the other node
		MapNode getOtherNode(MapNode node)
		{
			if (node.equals(from)) 
				return to;
			else if (node.equals(to))
				return from;
			throw new IllegalArgumentException("Looking for " +"a point that is not in the edge");
		}
		
	public String getroadName() {
		return roadName;
	}
	
	public String getroadType() {
		return roadType;
	}

	public double getlength() {
		return length;
	}
}
