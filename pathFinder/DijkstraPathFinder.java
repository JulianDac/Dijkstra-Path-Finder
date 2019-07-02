package pathFinder;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import map.Coordinate;
import map.Edge;
import map.Node;
import map.PathMap;

public class DijkstraPathFinder implements PathFinder {
    PathMap map;
    List<Node> graph;
    List<Node> originNodes;
    List<Node> destNodes;
    List<Node> waypointNodes;
    Set<Node> allExploredNodes;

    public DijkstraPathFinder(PathMap map) {
	this.map = map;
	this.graph = new ArrayList<Node>();
	this.originNodes = new ArrayList<Node>();
	this.destNodes = new ArrayList<Node>();
	this.waypointNodes = new ArrayList<Node>();
	this.allExploredNodes = new HashSet<Node>();

	convertMapToGraph(map, this.graph);
    } // end of DijkstraPathFinder()

    @Override
    public List<Coordinate> findPath() {
	List<Coordinate> path = new ArrayList<Coordinate>();

	List<Node> shortestNodePath = null;

	// handling multiple origins and destinations
	for (Node originNode : this.originNodes) {
	    for (Node destNode : this.destNodes) {
		int shortest = shortestNodePath != null ? shortestNodePath.size() : Integer.MAX_VALUE;
		List<Node> nodePath = findShortestThroughWaypoints(originNode, destNode);
		if (nodePath.size() < shortest) {
		    shortestNodePath = nodePath;
		}
	    }
	}

	for (Node node : shortestNodePath) {
	    path.add(node.coordinate);
	}

	return path;
    } // end of findPath()

    @Override
    public int coordinatesExplored() {
	return this.allExploredNodes.size();
    } // end of cellsExplored()

    private void convertMapToGraph(PathMap map, List<Node> graph) {
	// initialize isolated Nodes
	for (int i = 0; i < map.sizeR; i++) {
	    for (int j = 0; j < map.sizeC; j++) {

		Coordinate coordinate = map.cells[i][j];

		if (!coordinate.getImpassable()) {
		    Node node = new Node(coordinate);

		    graph.add(node);

		    if (map.originCells.contains(coordinate)) {
			this.originNodes.add(node);
		    }

		    if (map.destCells.contains(coordinate)) {
			this.destNodes.add(node);
		    }

		    if (map.waypointCells.contains(coordinate)) {
			this.waypointNodes.add(node);
		    }
		}

	    }
	}

	// connect Nodes
	for (Node fromNode : graph) {
	    int r = fromNode.coordinate.getRow();
	    int c = fromNode.coordinate.getColumn();

	    // connect to the below node
	    if (map.isIn(r - 1, c)) {
		Coordinate toCoordinate = map.cells[r - 1][c];
		if (!toCoordinate.getImpassable()) {
		    addEdge(fromNode, toCoordinate);
		}
	    }

	    // connect to the left node
	    if (map.isIn(r, c - 1)) {
		Coordinate toCoordinate = map.cells[r][c - 1];
		if (!toCoordinate.getImpassable()) {
		    addEdge(fromNode, toCoordinate);
		}
	    }

	    // connect to the right node
	    if (map.isIn(r, c + 1)) {
		Coordinate toCoordinate = map.cells[r][c + 1];
		if (!toCoordinate.getImpassable()) {
		    addEdge(fromNode, toCoordinate);
		}
	    }

	    // connect to the left node
	    if (map.isIn(r + 1, c + 1)) {
		Coordinate toCoordinate = map.cells[r + 1][c];
		if (!toCoordinate.getImpassable()) {
		    addEdge(fromNode, toCoordinate);
		}
	    }
	}
    }

    private void addEdge(Node fromNode, Coordinate toCoordinate) {
	Node toNode = findNodeByCoordinate(toCoordinate.getRow(), toCoordinate.getColumn());
	Edge edge = new Edge(toNode, toCoordinate.getTerrainCost());
	fromNode.edges.add(edge);
    }

    private Node findNodeByCoordinate(int r, int c) {
	for (Node node : this.graph) {
	    if ((node.coordinate.getRow() == r) && (node.coordinate.getColumn() == c)) {
		return node;
	    }
	}
	return null;
    }

    private List<Node> findShortestThroughWaypoints(Node originNode, Node destNode) {

	if (!(this.waypointNodes.size() > 0)) {
	    return findShortest(originNode, destNode);
	}

	List<Node> shortestPath = null;

	List<List<Node>> waypointSequencePermutations = new ArrayList<List<Node>>();

	for (int i = 0; i < this.waypointNodes.size() * this.waypointNodes.size() * this.waypointNodes.size(); i++) {
	    List<Node> permutation = new ArrayList<Node>();
	    while (permutation.size() < this.waypointNodes.size()) {
		Node waypointNode = this.waypointNodes.get((int) (Math.random() * this.waypointNodes.size()));
		if (!permutation.contains(waypointNode)) {
		    permutation.add(waypointNode);
		}
	    }
	    waypointSequencePermutations.add(permutation);
	}

	for (List<Node> waypointSequencePermutation : waypointSequencePermutations) {

	    int shortest = shortestPath != null ? shortestPath.size() : Integer.MAX_VALUE;
	    List<Node> path = new ArrayList<Node>();

	    // origin -> first waypoint
	    List<Node> pathFromOriginToFirstWaypoint = findShortest(originNode, waypointSequencePermutation.get(0));
	    pathFromOriginToFirstWaypoint.remove(pathFromOriginToFirstWaypoint.size() - 1);
	    path.addAll(pathFromOriginToFirstWaypoint);

	    // waypoint -> waypoint
	    for (int i = 0; i < waypointSequencePermutation.size() - 1; i++) {
		List<Node> pathFromWaypointToWayPoint = findShortest(waypointSequencePermutation.get(i),
			waypointSequencePermutation.get(i + 1));
		pathFromWaypointToWayPoint.remove(pathFromWaypointToWayPoint.size() - 1);
		path.addAll(pathFromWaypointToWayPoint);
	    }

	    // last waypoint -> destination
	    List<Node> pathFromLastWaypointToDestination = findShortest(
		    waypointSequencePermutation.get(waypointSequencePermutation.size() - 1), destNode);
	    path.addAll(pathFromLastWaypointToDestination);

	    if (path.size() < shortest) {
		shortestPath = path;
	    }
	}

	return shortestPath;

    }

    private List<Node> findShortest(Node originNode, Node destNode) {
	for (Node node : this.graph) {
	    node.shortestDistanceFromOrigin = Integer.MAX_VALUE;
	}
	List<Node> path = new ArrayList<Node>();
	Set<Node> exploredNodes = new HashSet<Node>();

	// initialize origin
	originNode.shortestDistanceFromOrigin = 0;
	for (Edge edge : originNode.edges) {
	    Node adjacentNode = edge.toNode;
	    adjacentNode.shortestDistanceFromOrigin = edge.cost;
	    adjacentNode.previousNode = originNode;
	}
	exploredNodes.add(originNode);
	this.allExploredNodes.add(originNode);

	// explore
	for (int i = 0; i <= this.graph.size(); i++) {

	    // if (exploredNodes.size() == this.graph.size()) {
	    // break;
	    // }

	    Node shortestDistanceUnexploredNode = findNextShortestUnexploredNode(exploredNodes);

	    if (shortestDistanceUnexploredNode != null) {
		// recalculate adjacent nodes' distance
		for (Edge edge : shortestDistanceUnexploredNode.edges) {
		    Node adjacentNode = edge.toNode;
		    int distanceFromOrigin = shortestDistanceUnexploredNode.shortestDistanceFromOrigin + edge.cost;
		    if (distanceFromOrigin < adjacentNode.shortestDistanceFromOrigin) {
			adjacentNode.shortestDistanceFromOrigin = distanceFromOrigin;
			adjacentNode.previousNode = shortestDistanceUnexploredNode;
		    }
		}

		exploredNodes.add(shortestDistanceUnexploredNode);
		this.allExploredNodes.add(shortestDistanceUnexploredNode);
	    }

	}

	// find path from destination
	Node node = destNode;
	while (node != originNode) {
	    path.add(0, node);
	    node = node.previousNode;
	}
	path.add(0, originNode);

	return path;
    }

    // could be better optimized
    private Node findNextShortestUnexploredNode(Set<Node> exploredNodes) {

	int shortestDistance = Integer.MAX_VALUE;
	Node shortestDistanceUnexploredNode = null;

	for (Node exploredNode : exploredNodes) {
	    for (Edge edge : exploredNode.edges) {
		Node adjacentNode = edge.toNode;
		if (!exploredNodes.contains(adjacentNode)) {
		    int distanceFromOrigin = exploredNode.shortestDistanceFromOrigin + edge.cost;
		    if (shortestDistanceUnexploredNode != null) {
			if (distanceFromOrigin < shortestDistance) {
			    shortestDistance = distanceFromOrigin;
			    shortestDistanceUnexploredNode = adjacentNode;
			}
		    } else {
			shortestDistance = distanceFromOrigin;
			shortestDistanceUnexploredNode = adjacentNode;
		    }
		}
	    }
	}

	return shortestDistanceUnexploredNode;
    }

} // end of class DijsktraPathFinder
