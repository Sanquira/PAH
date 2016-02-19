package student.tasks.task4;

import javax.vecmath.Point2d;

import cz.agents.alite.tactical.util.Point;
import dubins.Point2dYaw;

/**
 *
 * @author javedhossain
 */

public class Node {

	private double x;
	private double y;
	private double yaw;
	private double pathLengthFromRoot;
	private double distanceFromGoal;

	public Node() {
		x = -1;
		y = -1;
		yaw = -1;
		pathLengthFromRoot = -1;
		distanceFromGoal = Double.POSITIVE_INFINITY;
	}

	public Node(Point2dYaw pnt) {
		x = pnt.x;
		y = pnt.y;
		yaw = pnt.getYaw();
		pathLengthFromRoot = -1;
		distanceFromGoal = Double.POSITIVE_INFINITY;
	}

	public Node(double eks, double wye, double yawfa) {
		x = eks;
		y = wye;
		yaw = yawfa;
		pathLengthFromRoot = -1;
		distanceFromGoal = Double.POSITIVE_INFINITY;
	}

	public Node(double eks, double wye, double yawfa, double pathLength, double distFromGoal) {
		x = eks;
		y = wye;
		yaw = yawfa;
		pathLengthFromRoot = pathLength;
		distanceFromGoal = distFromGoal;
	}

	public void setCoordinate(Point2dYaw pnt) {
		x = pnt.x;
		y = pnt.y;
		yaw = pnt.getYaw();
	}

	public void setCoordinate(double eks, double wye) {
		x = eks;
		y = wye;
	}

	public void setCoordinate(double eks, double wye, double yawfa) {
		x = eks;
		y = wye;
		yaw = yawfa;
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

	public double getYaw() {
		return yaw;
	}

	public double getPathLengthFromRoot() {
		return pathLengthFromRoot;
	}

	public String toString() {
		return "N = (" + x + "," + y + "," + yaw + ")";
	}

	public Point2d toPoint2d() {
		return new Point2d(x, y);
	}

	public Point toPoint() {
		return new Point(x, y, yaw);
	}

	public Point2dYaw toPoint2dYaw() {
		return new Point2dYaw(x, y, yaw);
	}

}
