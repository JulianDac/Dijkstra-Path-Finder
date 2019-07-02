package map;

import java.util.ArrayList;
import java.util.List;

public class Node {

    public Coordinate coordinate;
    public List<Edge> edges;

    public int shortestDistanceFromOrigin = Integer.MAX_VALUE;
    public Node previousNode = null; // in the shortest path found so far

    public Node(Coordinate coordinate) {
	this.coordinate = coordinate;
	this.edges = new ArrayList<Edge>();
    }
}
