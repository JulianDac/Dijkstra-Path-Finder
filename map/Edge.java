package map;

public class Edge {

    public Node toNode;
    public int cost;

    public Edge(Node toNode, int cost) {
	this.toNode = toNode;
	this.cost = cost;
    }
}
