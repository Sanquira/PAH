package student.tasks;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.TreeSet;

import javax.swing.Timer;
import javax.vecmath.Point2d;

import student.tasks.task2.Node;
import student.tasks.task2.Tree;
import cz.agents.alite.pahtactical.vis.LabeledPointLayer;
import cz.agents.alite.pahtactical.vis.PathLayer;
import cz.agents.alite.pahtactical.vis.Polygons2dLayer;
import cz.agents.alite.tactical.util.Point;
import cz.agents.alite.tactical.util.Polygon2d;
import cz.agents.alite.tactical.util.Visibility;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.common.VisInfoLayer;
import cz.agents.alite.vis.layer.terminal.PointLayer;

public class Task2 {

	private static final int sizeX = 1024;
	private static final int sizeY = 768;

	private static final boolean drawLinies = true;

	/**
	 * This method should return a "reasonable" path for a vehicle with given
	 * footprint and minimum turn radius that begins at start configuration and
	 * ends at the goal configuration and the vehicle will avoid collisions with
	 * given polygonal obstacles. Use RRT* algorithm with Dubin's curve
	 * extensions.
	 *
	 * @return poly-line representation of the found path
	 */

	public static Point2d[] findPath(final Point2d startP, final Point2d endP, final Collection<Polygon2d> obstacles) {

		/* params */
		int maxEdgeLength = 25;

		Tree tree = new Tree();
		Node start = new Node(startP);
		Node goal = new Node(endP);

		start.setPathLengthFromRoot(0);
		tree.addNode(start, null);

		Node randomNode;
		Node nearestNode;
		Node newNode = null;
		double radius = 9999;
		ArrayList<Node> nearestNodes = new ArrayList<Node>();
		boolean nodeAdded = false;
		boolean run = true;
		int tryGoalFactor = 1;
		Visibility vis = new Visibility(obstacles);
		int iteration = 0;
		ArrayList<Node> path = new ArrayList<Node>();

		int maxIter = 3000;

		while (run && iteration <= maxIter) {
			randomNode = getRandomState(obstacles);
			nodeAdded = false;
			// if (iteration % tryGoalFactor == 0) {
			if (iteration == maxIter) {
				randomNode = new Node(endP);
			}
			nearestNodes = getNearestNode(tree, randomNode, radius);

			if (nearestNodes.size() > 200) {
				radius = getEuclidDist(randomNode, nearestNodes.get(200));
			}

			for (int i = 0; i < nearestNodes.size(); i++) {
				nearestNode = nearestNodes.get(i);
				if (vis.isVisible(nearestNode.toPoint(), randomNode.toPoint())) {
					do {
						newNode = goTowardsNode(nearestNode, randomNode, maxEdgeLength);
						tree.addChild(nearestNode, newNode);
						tree.addNode(newNode, nearestNode);
						Point2d[] ppp = { nearestNode.toPoint2d(), newNode.toPoint2d() };
						VisManager.registerLayer(PathLayer.create(ppp, Color.BLACK, 1));
						VisManager.registerLayer(LabeledPointLayer.create(newNode.toPoint2d(), ""));
						nearestNode = newNode;
					} while (getEuclidDist(newNode, randomNode) > 0);

					if (getEuclidDist(newNode, goal) == 0) {
						System.out.println("tu");
						newNode.setCoordinate(endP);
						goal = newNode;
						path = getPath(start, goal, tree);
						run = false;
					}
					tryGoalFactor = 1;
					nodeAdded = true;
				} else if (tryGoalFactor < 10) {
					tryGoalFactor++;
				}

				if (nodeAdded) {
					break;
				}
			}
			if (nodeAdded) {
				for (int i = 0; i < nearestNodes.size(); i++) {
					Node thisNode = nearestNodes.get(i);
					double newNodePathLength = newNode.getPathLengthFromRoot();
					double thisNodePathLength = thisNode.getPathLengthFromRoot();
					double distance = getEuclidDist(newNode, thisNode);

					if (newNodePathLength + distance < thisNodePathLength) {
						if (vis.isVisible(newNode.toPoint(), thisNode.toPoint())) {
							tree.changeParent(thisNode, newNode);
							tree.addChild(newNode, thisNode);
							thisNode.setPathLengthFromRoot(newNodePathLength + distance);
							Point2d[] ppp = { thisNode.toPoint2d(),
									newNode.toPoint2d() };
							VisManager.registerLayer(LabeledPointLayer.create(newNode.toPoint2d(),
									""));
							VisManager.registerLayer(PathLayer.create(ppp, Color.GREEN, 1));
						}
					}
				}
			}
			iteration++;
		}

		return reconstructPath(path);
	}

	private static Point2d[] reconstructPath(ArrayList<Node> path) {
		Point2d[] pnts = new Point2d[path.size()];
		for (int i = 0; i < pnts.length; i++) {
			Node n = path.remove(path.size() - 1);
			pnts[i] = new Point2d(n.getX(), n.getY());
		}
		return pnts;
	}

	private static ArrayList<Node> getPath(Node start, Node newNode, Tree tree) {
		ArrayList<Node> path = new ArrayList<Node>();
		path.add(newNode);
		Node n = newNode;
		while (true) {
			n = tree.getParent(n);
			path.add(n);
			if (n == start) {
				return path;
			}
		}
	}

	private static Node goTowardsNode(Node nearestNode, Node randomNode, int maxEdgeLength) {
		double vx = nearestNode.getX();
		double vy = nearestNode.getY();
		double rx = randomNode.getX();
		double ry = randomNode.getY();

		Node newNode = new Node(rx, ry);
		double v1 = vx - rx;
		double v2 = vy - ry;
		double dx = getEuclidDist(randomNode, nearestNode);

		for (int i = 0; i < (int) (dx * 10); i++) {
			newNode.setCoordinate(rx + v1 * ((double) i / (double) dx), ry + v2 * ((double) i / (double) dx));
			double d = getEuclidDist(nearestNode, newNode);
			if (d <= maxEdgeLength) {
				newNode.setPathLengthFromRoot(nearestNode.getPathLengthFromRoot() + d);
				return newNode;
			}
		}
		return null;
	}

	private static ArrayList<Node> getNearestNode(Tree tree, Node node, double radius) {
		HashMap<Node, Double> nearestNodes = new HashMap<Node, Double>();
		double dist;
		for (int i = 0; i < tree.getNodeCount(); i++) {
			dist = getEuclidDist(tree.getNode(i), node);
			if (dist < radius) {
				nearestNodes.put(tree.getNode(i), dist + tree.getNode(i).getPathLengthFromRoot());
			}
		}
		HashMap<Node, Double> sortedNodes = new HashMap<Node, Double>();
		sortedNodes = sortHashMap(nearestNodes);

		ArrayList<Node> candidateNodes = new ArrayList<Node>();

		for (Node n : sortedNodes.keySet()) {
			candidateNodes.add(n);
		}

		return candidateNodes;
	}

	private static HashMap<Node, Double> sortHashMap(HashMap<Node, Double> input) {

		Map<Node, Double> tempMap = new HashMap<Node, Double>();

		for (Node wsState : input.keySet()) {
			tempMap.put(wsState, input.get(wsState));
		}

		ArrayList<Node> mapKeys = new ArrayList<Node>(tempMap.keySet());
		ArrayList<Double> mapValues = new ArrayList<Double>(tempMap.values());
		HashMap<Node, Double> sortedMap = new LinkedHashMap<Node, Double>();
		TreeSet<Double> sortedSet = new TreeSet<Double>(mapValues);
		Object[] sortedArray = sortedSet.toArray();

		int size = sortedArray.length;
		for (int i = 0; i < size; i++) {
			sortedMap.put(mapKeys.get(mapValues.indexOf(sortedArray[i])), (Double) sortedArray[i]);
		}
		return sortedMap;
	}

	private static double getEuclidDist(Node n1, Node n2) {
		return Math.sqrt(Math.pow(n1.getX() - n2.getX(), 2) + Math.pow(n1.getY() - n2.getY(), 2));
	}

	private static Node getRandomState(Collection<Polygon2d> obstacles) {
		boolean ko = true;
		double x = -1, y = -1;
		do {
			x = (Math.random() * sizeX) - (sizeX / 2.);
			y = (Math.random() * sizeY) - (sizeY / 2.);

			for (Polygon2d poly : obstacles) {
				if (poly.isInside(new Point2d(x, y))) {
					ko = true;
					break;
				} else {
					ko = false;
				}
			}
		} while (ko);
		return new Node(x, y);
	}

	public static void main(String[] args) {
		Point2d start = new Point2d(0, -200);
		Point2d end = new Point2d(0, 120);

		Collection<Polygon2d> polygons = new LinkedList<Polygon2d>();
		polygons.add(getRect(0, 0, 200, 200));
		polygons.add(getRect(100, 100, 170, 170));
		polygons.add(getRect(-100, 100, 100, 170));

		VisManager.registerLayer(ColorLayer.create(Color.WHITE));
		VisManager.registerLayer(Polygons2dLayer.create(polygons, Color.BLACK, 2));
		VisManager.registerLayer(LabeledPointLayer.create(start, "start"));
		VisManager.registerLayer(LabeledPointLayer.create(end, "end"));

		VisManager.registerLayer(VisInfoLayer.create());

		VisManager.setInitParam("PAH Trajectory Planning Task", sizeX, sizeY);
		VisManager.init();

		Point2d[] path = findPath(start, end, polygons);
		// for (int i = 0; i < path.length - 1; i++) {
		// VisManager.registerLayer(LabeledPointLayer.create(path[i], i+""));
		// }
		// VisManager.registerLayer(LabeledPointLayer.create(path[path.length -
		// 1], path.length - 1 + ""));
		// System.out.println(path.length);

		Timer tim = new Timer(2000, new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent e) {
				System.exit(0);
			}
		});
		// tim.start();

		VisManager.registerLayer(PathLayer.create(path, Color.BLUE, 3));
	}

	public static Polygon2d getRect(int x, int y, int w, int h) {
		return new Polygon2d(new Point2d[] {
				new Point2d(x - w / 2, y - h / 2),
				new Point2d(x + w / 2, y - h / 2),
				new Point2d(x + w / 2, y + h / 2),
				new Point2d(x - w / 2, y + h / 2) });
	}

}
