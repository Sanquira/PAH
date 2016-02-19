package student.tasks;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;

import javax.swing.Timer;
import javax.vecmath.Point2d;

import cz.agents.alite.pahtactical.vis.LabeledPointLayer;
import cz.agents.alite.pahtactical.vis.PathLayer;
import cz.agents.alite.pahtactical.vis.Polygons2dLayer;
import cz.agents.alite.tactical.util.Point;
import cz.agents.alite.tactical.util.Polygon2d;
import cz.agents.alite.tactical.util.Visibility;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.common.VisInfoLayer;

public class Task1 {

	private static final int DEBUG = 1;

	/**
	 * This method should return a shortest path from the start point to the end
	 * point that avoids the given polygonal obstacles.
	 * 
	 * Use A* algorithm on visibility graph to find the path.
	 * 
	 * @return poly-line representation of the found path
	 */

	public static Point2d[] findPath(Point2d start, Point2d end, Collection<Polygon2d> obstacles) {
		debugPrint("findPath: start finding A* path", 3);
		HashMap<Point2d, Point2d> cameFrom = new HashMap<Point2d, Point2d>();
		ArrayList<Point2d> closedset = new ArrayList<Point2d>();
		ArrayList<Point2d> openset = new ArrayList<Point2d>();
		openset.add(start);
		HashMap<Point2d, Double> g_score = new HashMap<Point2d, Double>();
		HashMap<Point2d, Double> h_score = new HashMap<Point2d, Double>();
		HashMap<Point2d, Double> f_score = new HashMap<Point2d, Double>();
		g_score.put(start, 0.);
		h_score.put(start, sqrEuclidDist(start, end));
		f_score.put(start, h_score.get(start));

		debugPrint("findPath: g_score size: " + g_score.size() + ", value: " + g_score.get(start));
		debugPrint("findPath: h_score size: " + h_score.size() + ", value: " + h_score.get(start));
		debugPrint("findPath: f_score size: " + f_score.size() + ", value: " + f_score.get(start));

		Collection<Point2d> pntOfInt = new LinkedList<Point2d>();
		pntOfInt.add(end);

		while (!openset.isEmpty()) {
			Point2d x = getPointSmallest(openset, f_score);
			if (x.equals(end)) {

				debugPrint("findPath: path generated sucessfully", 3);
				return alToField(reconstructPath(cameFrom, end));
			}
			openset.remove(x);
			closedset.add(x);
			ArrayList<Point2d> visibles = findVisiblePoints(x, obstacles, pntOfInt);

			for (Point2d y : visibles) {
				if (closedset.contains(y)) {
					continue;
				}
				double curr_q_score = g_score.get(x) + sqrEuclidDist(x, y);
				boolean curr_is_better;
				if (!openset.contains(y)) {
					openset.add(y);
					curr_is_better = true;
				} else if (curr_q_score < g_score.get(y)) {
					curr_is_better = true;
				} else {
					curr_is_better = false;
				}
				if (curr_is_better) {
					cameFrom.put(y, x);
					g_score.put(y, curr_q_score);
					h_score.put(y, sqrEuclidDist(y, end));
					f_score.put(y, g_score.get(y) + h_score.get(y));
				}
			}

		}
		debugPrint("findPath: error pathfind, no path find", 3);
		return null;
	}

	private static ArrayList<Point2d> reconstructPath(HashMap<Point2d, Point2d> cameFrom, Point2d point2d) {
		if (cameFrom.get(point2d) != null) {
			ArrayList<Point2d> p = reconstructPath(cameFrom, cameFrom.get(point2d));
			p.add(point2d);
			return p;
		} else {
			ArrayList<Point2d> p = new ArrayList<Point2d>();
			p.add(point2d);
			return p;
		}
	}

	private static ArrayList<Point2d> findVisiblePoints(Point2d pnt, Collection<Polygon2d> obstacles, Collection<Point2d> pntOfInt) {
		ArrayList<Point2d> pnts = new ArrayList<Point2d>();
		Collection<Polygon2d> inf = getInflatedPolygons(obstacles, 1, 0);
		Visibility vis = new Visibility(obstacles);
		for (Polygon2d polygon2d : inf) {
			for (Point2d point2d : polygon2d.getPoints()) {
				if (vis.isVisible(new Point(pnt.x, pnt.y, 0), new Point(point2d.x, point2d.y, 0)) && !pnts.contains(point2d)) {
					pnts.add(point2d);
				}
			}
		}
		for (Point2d point2d : pntOfInt) {
			if (vis.isVisible(new Point(pnt.x, pnt.y, 0), new Point(point2d.x, point2d.y, 0)) && !pnts.contains(point2d)) {
				pnts.add(point2d);
			}
		}
		debugPrint("findVisiblePoints: found " + pnts.size());
		return pnts;
	}

	private static Point2d getPointSmallest(ArrayList<Point2d> openset, HashMap<Point2d, Double> f_score) {
		double f_min = Double.MAX_VALUE;
		Point2d minpnt = null;
		for (Point2d pnt : openset) {
			double curf = f_score.get(pnt);
			if (f_min > curf) {
				f_min = curf;
				minpnt = pnt;
			}

		}
		debugPrint("getPointSmallest: found point: " + minpnt + " with f_score: " + f_min);
		return minpnt;
	}

	private static Collection<Polygon2d> getInflatedPolygons(Collection<Polygon2d> polygons, int inflate, int numOfPoints) {
		Collection<Polygon2d> inflatedPolygons = new LinkedList<Polygon2d>();
		for (Polygon2d polygon2d : polygons) {
			inflatedPolygons.add(polygon2d.inflate(inflate, numOfPoints));
		}
		return inflatedPolygons;
	}

	private static Point2d[] alToField(ArrayList<Point2d> al) {
		Point2d[] outpnts = new Point2d[al.size()];
		for (int i = 0; i < outpnts.length; i++) {
			outpnts[i] = al.remove(0);
		}
		return outpnts;
	}

	private static double sqrEuclidDist(Point2d pnt1, Point2d pnt2) {
		return Math.sqrt(Math.pow(pnt1.x - pnt2.x, 2) + Math.pow(pnt1.y - pnt2.y, 2));
//		return Math.pow(pnt1.x - pnt2.x, 2) + Math.pow(pnt1.y - pnt2.y, 2);
	}

	private static void debugPrint(String message) {
		debugPrint(message, 0);
	}

	private static void debugPrint(String message, int prio) {
		if (prio >= DEBUG) {
			System.out.println(message);
		}
	}

	public static void main(String[] args) {
//		Point2d start = new Point2d(0, -200);
//		Point2d end = new Point2d(0, 120);
		
		Point2d start = new Point2d(100, -200);
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

		VisManager.setInitParam("PAH Trajectory Planning Task", 1024, 768);
		VisManager.init();

		Point2d[] path = findPath(start, end, polygons);
//		for (int i = 0; i < path.length; i++) {
//			VisManager.registerLayer(LabeledPointLayer.create(path[i], i + ""));
//		}

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
