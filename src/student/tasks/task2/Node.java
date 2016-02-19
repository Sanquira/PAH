package student.tasks.task2;

import javax.vecmath.Point2d;

import cz.agents.alite.tactical.util.Point;

/**
 *
 * @author javedhossain
 */

public class Node {

	private double x;
	private double y;
	private double pathLengthFromRoot;
	private double distanceFromGoal;

	public Node() {
		x = -1;
		y = -1;
		pathLengthFromRoot = -1;
		distanceFromGoal = Double.POSITIVE_INFINITY;
	}

	public Node(Point2d pnt) {
		x = pnt.x;
		y = pnt.y;
		pathLengthFromRoot = -1;
		distanceFromGoal = Double.POSITIVE_INFINITY;
	}

	public Node(double eks, double wye) {
		x = eks;
		y = wye;
		pathLengthFromRoot = -1;
		distanceFromGoal = Double.POSITIVE_INFINITY;
	}

	public Node(double eks, double wye, double pathLength, double distFromGoal) {
		x = eks;
		y = wye;
		pathLengthFromRoot = pathLength;
		distanceFromGoal = distFromGoal;
	}

	public void setCoordinate(Point2d pnt) {
		x = pnt.x;
		y = pnt.y;
	}

	public void setCoordinate(double eks, double wye) {
		x = eks;
		y = wye;
	}

	public void setPathLengthFromRoot(double pathLength) {
		pathLengthFromRoot = pathLength;
	}

	public void setDistFromGoal(double dist) {
		distanceFromGoal = dist;
	}

	public double getDistFromGoal() {
		return distanceFromGoal;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public double getPathLengthFromRoot() {
		return pathLengthFromRoot;
	}

	public String toString() {
		return "N = (" + x + "," + y + ")";
	}

	public Point2d toPoint2d() {
		return new Point2d(x, y);
	}
	
	public Point toPoint(){
		return new Point(x,y,0);
	}

}
