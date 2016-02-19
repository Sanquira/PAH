package student.tasks;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;

import javax.vecmath.Point2d;

import student.tasks.task2.Node;
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
import cz.agents.alite.vis.layer.terminal.PointLayer;
import dubins.DubinsCurve;
import dubins.Point2dYaw;

public class Task3 {

	private static final double PI2 = Math.PI / 2;

	/**
	 * This method should return a "reasonable" path for a vehicle with given footprint and minimum turn radius that begins at start configuration and ends at the goal
	 * configuration and the vehicle will avoid collisions with given obstacles.
	 * 
	 * Use A* algorithm on an implicit maneuver graph (state-lattice).
	 *
	 * @return poly-line representation of the found path
	 */

	public static Point2d[] findPath(final Point2dYaw start, final Point2dYaw end, final Collection<Polygon2d> obstacles, final Polygon2d footprint, final double rho) {

		HashMap<Point2dYaw, Point2dYaw> cameFrom = new HashMap<Point2dYaw, Point2dYaw>();
		ArrayList<Point2dYaw> closedset = new ArrayList<Point2dYaw>();
		ArrayList<Point2dYaw> openset = new ArrayList<Point2dYaw>();
		openset.add(start);
		HashMap<Point2dYaw, Double> g_score = new HashMap<Point2dYaw, Double>();
		HashMap<Point2dYaw, Double> h_score = new HashMap<Point2dYaw, Double>();
		HashMap<Point2dYaw, Double> f_score = new HashMap<Point2dYaw, Double>();
		g_score.put(start, 0.); // Délka aktuální optimální cesty.
		h_score.put(start, getEuclidDist(start, end)); // Heuristický odhad vzdálenosti
		f_score.put(start, h_score.get(start)); // Předpokládaná délka cesty mezi startem a cílem jdoucí přes y.

		ArrayList<Point2dYaw> pntsOfInt = new ArrayList<Point2dYaw>();
		pntsOfInt.add(end);
		while (!openset.isEmpty()) {
			Point2dYaw x = getPointSmallest(openset, f_score);
			if (x.equals(end)) {
				return alToField(reconstructPath(cameFrom, end, start, rho));
			}
			openset.remove(x);
			closedset.remove(x);
			ArrayList<Point2dYaw> visibles = findVisiblePoints(x, obstacles, pntsOfInt, rho);

			for (Point2dYaw y : visibles) {
				if (closedset.contains(y)) {
					continue;
				}
				double curr_g_score = g_score.get(x) + getYawbasedDist(x, y);
				boolean curr_is_better;
				if (!openset.contains(y)) {
					openset.add(y);
					curr_is_better = true;
				} else if (curr_g_score < g_score.get(y)) {
					curr_is_better = true;
				} else {
					curr_is_better = false;
				}
				if (curr_is_better) {
					cameFrom.put(y, x);
					g_score.put(y, curr_g_score);
					h_score.put(y, getEuclidDist(y, end));
					f_score.put(y, g_score.get(y) + h_score.get(y));
				}
			}
		}

		System.err.println("findPath: error pathfind, no path find");
		return null;
	}

	private static ArrayList<Point2dYaw> reconstructPath(HashMap<Point2dYaw, Point2dYaw> cameFrom, Point2dYaw end, Point2dYaw start, double rho) {
		ArrayList<Point2dYaw> path = new ArrayList<Point2dYaw>();
		Point2dYaw n = end;
		DubinsCurve dubin;
		while (true) {
			// Point2d[] ppp = { n.getPosition(), cameFrom.get(n).getPosition() };
			// VisManager.registerLayer(PathLayer.create(ppp, Color.GREEN, 2));
			dubin = new DubinsCurve(cameFrom.get(n), n, rho - 0.01);
			Point2dYaw[] pnt = dubin.interpolateUniformBy(10);
			for (int i = pnt.length - 1; i > 0; i--) {
				path.add(pnt[i]);
			}
			n = cameFrom.remove(n);
			if (n == start) {
				path.add(n);
				return path;
			}
		}
	}

	private static ArrayList<Point2dYaw> findVisiblePoints(Point2dYaw x, Collection<Polygon2d> obstacles, ArrayList<Point2dYaw> pntsOfInt, double rho) {
		double[] yaws = { x.getYaw() - PI2, x.getYaw(), x.getYaw() + PI2 };
		double[][] sourss = { { rho, -rho }, { rho, 0 }, { rho, rho } };
		ArrayList<Point2dYaw> pnts = new ArrayList<Point2dYaw>();

		// DEBUG
		double[][] sourss2 = { { rho, -rho - 10 }, { 10 + rho, 0 }, { rho, rho + 10 } };

		for (int i = 0; i < sourss.length; i++) {
			pnts.add(new Point2dYaw(roundTo(-sourss[i][1] * Math.sin(x.getYaw()) + sourss[i][0] * Math.cos(x.getYaw()) + x.x, 4),
					roundTo(sourss[i][1] * Math.cos(x.getYaw()) + sourss[i][0] * Math.sin(x.getYaw()) + x.y, 4), roundTo(yaws[i], 4)));
		}

		// Collection<Polygon2d> inf = getInflatedPolygons(obstacles, (int) (rho/ 2)+1, 0);
		Collection<Polygon2d> inf = getInflatedPolygons(obstacles, 12, 0);
		Visibility vis = new Visibility(inf);
		ArrayList<Point2dYaw> pp2 = new ArrayList<Point2dYaw>();
		for (int i = 0; i < pnts.size(); i++) {
			if (vis.isVisible(new Point(x.x, x.y, 0), new Point(pnts.get(i).x, pnts.get(i).y, 0)) && !vis.isInBuilding(new Point(pnts.get(i).x, pnts.get(i).y, 0))) {
				pp2.add(pnts.get(i));
				// Point2dYaw oo = new Point2dYaw(-sourss2[i][1] * Math.sin(x.getYaw()) + sourss2[i][0] * Math.cos(x.getYaw()) + x.x, sourss2[i][1] * Math.cos(x.getYaw())
				// + sourss2[i][0] * Math.sin(x.getYaw()) + x.y, yaws[i]);
				// Point2d[] ppp = { pnts.get(i).getPosition(), oo.getPosition() };
				// VisManager.registerLayer(PathLayer.create(ppp, Color.RED, 1));
				// VisManager.registerLayer(LabeledPointLayer.create(pnts.get(i).getPosition(), ""));
			}
		}
		for (Point2dYaw point2d : pntsOfInt) {
			if (vis.isVisible(new Point(x.x, x.y, 0), new Point(point2d.x, point2d.y, 0)) && !pnts.contains(point2d)) {
				pp2.add(point2d);
			}
		}
		return pp2;
	}

	private static double roundTo(double number, int numOfDecimal) {
		return ((double) Math.round(number * Math.pow(10, numOfDecimal))) / Math.pow(10, numOfDecimal);
	}

	private static Collection<Polygon2d> getInflatedPolygons(Collection<Polygon2d> polygons, int inflate, int numOfPoints) {
		Collection<Polygon2d> inflatedPolygons = new LinkedList<Polygon2d>();
		for (Polygon2d polygon2d : polygons) {
			inflatedPolygons.add(polygon2d.inflate(inflate, numOfPoints));
		}
		return inflatedPolygons;
	}

	private static Point2dYaw getPointSmallest(ArrayList<Point2dYaw> openset, HashMap<Point2dYaw, Double> f_score) {
		double f_min = Double.MAX_VALUE;
		Point2dYaw minpnt = null;
		for (Point2dYaw pnt : openset) {
			double curf = f_score.get(pnt);
			if (f_min > curf) {
				f_min = curf;
				minpnt = pnt;
			}
		}
		return minpnt;
	}

	private static double getYawbasedDist(Point2dYaw n1, Point2dYaw n2) {
		double circDist = Math.PI * Math.sqrt((Math.pow(n1.x - n2.x, 2) + Math.pow(n1.y - n2.y, 2)) / 2) / 2;
		double strDist = Math.sqrt(Math.pow(n1.x - n2.x, 2) + Math.pow(n1.y - n2.y, 2));
		if (n1.getYaw() != n2.getYaw() && Math.sqrt(Math.pow(n1.x - n2.x, 2) + Math.pow(n1.y - n2.y, 2)) - 40 < 0.001) {
			return circDist;
		}
		return strDist;
	}

	private static double getEuclidDist(Point2dYaw n1, Point2dYaw n2) {
		return Math.sqrt(Math.pow(n1.x - n2.x, 2) + Math.pow(n1.y - n2.y, 2));
	}

	private static Point2d[] alToField(ArrayList<Point2dYaw> al) {
		Point2d[] outpnts = new Point2d[al.size()];
		for (int i = 0; i < outpnts.length; i++) {
			outpnts[i] = al.remove(al.size() - 1).getPosition();
		}
		return outpnts;
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
		for (int i = 0; i < path.length; i++) {
			VisManager.registerLayer(LabeledPointLayer.create(path[i], i + ""));
		}

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
