package student.tasks;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Random;
import java.util.TreeSet;

import javax.vecmath.Point2d;

import student.tasks.task4.Node;
import student.tasks.task4.Tree;
import cz.agents.alite.pahtactical.vis.LabeledPointLayer;
import cz.agents.alite.pahtactical.vis.PathLayer;
import cz.agents.alite.pahtactical.vis.Polygons2dLayer;
import cz.agents.alite.pahtactical.vis.VehicleLayer;
import cz.agents.alite.tactical.util.Point;
import cz.agents.alite.tactical.util.Polygon2d;
import cz.agents.alite.tactical.util.Visibility;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.common.VisInfoLayer;
import dubins.DubinsCurve;
import dubins.Point2dYaw;

public class Task4 {

	private static final int sizeX = 1024;
	private static final int sizeY = 768;
	private static final double PI2 = Math.PI / 2;
	private static final boolean drawLinies = true;

	/**
	 * This method should return a "reasonable" path for a vehicle with given footprint and minimum turn radius that begins at start configuration and ends at the goal
	 * configuration and the vehicle will avoid collisions with given obstacles.
	 * 
	 * Use RRT* algorithm with Dubin's curve extensions.
	 *
	 * @return poly-line representation of the found path
	 */
	public static Point2d[] findPath(
			final Point2dYaw startP, final Point2dYaw endP, final Collection<Polygon2d> obstacles,
			final Polygon2d footprint, final double rho) {

		int maxEdgeLength = 10;

		Tree tree = new Tree();
		Node start = new Node(startP);
		Node goal = new Node(endP);

		start.setPathLengthFromRoot(0);
		tree.addNode(start, null);

		Node randomNode;
		Node nearestNode = null;
		Node newNode = null;
		double radius = 9999;
		ArrayList<Node> nearestNodes = new ArrayList<Node>();
		boolean nodeAdded = false;
		boolean run = true;
		int tryGoalFactor = 1;
		Collection<Polygon2d> infObs = getInflatedPolygons(obstacles, 15, 0);
		Visibility vis = new Visibility(infObs);
		int iteration = 0;
		ArrayList<Node> path = new ArrayList<Node>();
		DubinsCurve dubin;

		int maxIter = 1000;

		while (run && iteration <= maxIter) {
			randomNode = getRandomState(infObs);
			nodeAdded = false;
			if (iteration == maxIter) {
				randomNode = new Node(endP);
			}
			nearestNodes = getNearestNode(tree, randomNode, radius);
			int maxPnts=1500;
			 if (nearestNodes.size() > maxPnts) {
			 radius = getEuclidDist(randomNode, nearestNodes.get(maxPnts));
			 // System.out.println("Menim radius na: " + radius);
			 }

			Point2dYaw[] shortestDub = new Point2dYaw[0];
			double curr_shortest_dub = Double.POSITIVE_INFINITY;
			Node bestNearest = null;
			for (int i = 0; i < nearestNodes.size(); i++) {
				nearestNode = nearestNodes.get(i);
				dubin = new DubinsCurve(nearestNode.toPoint2dYaw(), randomNode.toPoint2dYaw(), rho);
				Point2dYaw[] dubpnts = dubin.interpolateUniformBy(maxEdgeLength);
				double dub_dist_root = nearestNode.getPathLengthFromRoot();
				boolean useable = true;
				for (int j = 0; j < dubpnts.length - 1; j++) {
					dub_dist_root = dub_dist_root + maxEdgeLength;
					if (!vis.isVisible(new Point(dubpnts[j].x, dubpnts[j].y, 0), new Point(dubpnts[j + 1].x, dubpnts[j + 1].y, 0))) {
						useable = false;
					}
				}
				if (useable && dub_dist_root < curr_shortest_dub) {
					shortestDub = dubpnts;
					curr_shortest_dub = dub_dist_root;
					bestNearest = nearestNode;
				}
			}
			Node tmp;
			for (int i = 1; i < shortestDub.length; i++) {
				tmp = new Node(shortestDub[i]);
				tmp.setPathLengthFromRoot(bestNearest.getPathLengthFromRoot() + maxEdgeLength);
				tree.addChild(bestNearest, tmp);
				tree.addNode(tmp, bestNearest);
//				Point2d[] ppp = { bestNearest.toPoint2d(), tmp.toPoint2d() };
//				VisManager.registerLayer(PathLayer.create(ppp, Color.BLACK, 1));
//				VisManager.registerLayer(LabeledPointLayer.create(tmp.toPoint2d(), ""));
				bestNearest = tmp;
			}

			if (getEuclidDist(randomNode, goal) == 0) {
				// System.out.println("tu");
				path = getPath(start, bestNearest, tree);
				run = false;
			}

			if (shortestDub.length != 0) {
				iteration++;
				// System.out.println(iteration + ", " + tree.getNodeCount());
			}
		}
		// System.out.println("done");

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

	private static double getEuclidDist(Node n1, Node n2) {
		return Math.sqrt(Math.pow(n1.getX() - n2.getX(), 2) + Math.pow(n1.getY() - n2.getY(), 2));
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

	private static Node getRandomState(Collection<Polygon2d> obstacles) {
		boolean ko = true;
		double x = -1, y = -1, yaw = -1;
		do {
			x = (Math.random() * sizeX) - (sizeX / 2.);
			y = (Math.random() * sizeY) - (sizeY / 2.);
			yaw = (Math.random() * 2 * Math.PI) - Math.PI;

			for (Polygon2d poly : obstacles) {
				if (poly.isInside(new Point2d(x, y))) {
					ko = true;
					break;
				} else {
					ko = false;
				}
			}
		} while (ko);
		return new Node(x, y, yaw);
	}

	private static Collection<Polygon2d> getInflatedPolygons(Collection<Polygon2d> polygons, int inflate, int numOfPoints) {
		Collection<Polygon2d> inflatedPolygons = new LinkedList<Polygon2d>();
		for (Polygon2d polygon2d : polygons) {
			inflatedPolygons.add(polygon2d.inflate(inflate, numOfPoints));
		}
		return inflatedPolygons;
	}

	public static void main(String[] args) {

		Polygon2d footprint = getFootprint();

		Point2dYaw start = new Point2dYaw(0, -200, 0);
		Point2dYaw end = new Point2dYaw(0, 250, Math.PI / 3);

		Collection<Polygon2d> polygons = new LinkedList<Polygon2d>();
		polygons.add(getRect(0, 0, 200, 200));
		polygons.add(getRect(100, 100, 500, 10));
		polygons.add(getRect(-100, 100, 100, 170));

		VisManager.registerLayer(ColorLayer.create(Color.WHITE));
		VisManager.registerLayer(Polygons2dLayer.create(polygons, Color.BLACK, 2));
		VisManager.registerLayer(VehicleLayer.create(footprint, start, Color.GREEN));
		VisManager.registerLayer(VehicleLayer.create(footprint, end, Color.RED));

		VisManager.registerLayer(VisInfoLayer.create());

		VisManager.setInitParam("PAH Trajectory Planning Task", 1024, 768);
		VisManager.init();

		Point2d[] path = findPath(start, end, polygons, footprint, 40);

		VisManager.registerLayer(PathLayer.create(path, Color.BLUE, 3));
	}

	public static Polygon2d getRect(int x, int y, int w, int h) {
		return new Polygon2d(new Point2d[] {
				new Point2d(x - w / 2, y - h / 2),
				new Point2d(x + w / 2, y - h / 2),
				new Point2d(x + w / 2, y + h / 2),
				new Point2d(x - w / 2, y + h / 2) });
	}

	public static Polygon2d getFootprint() {
		int h = 20;
		int wFront = 25;
		int wBack = 8;
		return new Polygon2d(new Point2d[] {
				new Point2d(-wBack, -h / 2),
				new Point2d(+wFront, -h / 2),
				new Point2d(+wFront, +h / 2),
				new Point2d(-wBack, +h / 2) });
	}

}
