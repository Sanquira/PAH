package dubins;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;

public class Point2dYaw extends Point3d {

	public Point2dYaw(Point2d position, double yaw) {
		super(position.x, position.y, yaw);
	}

	public Point2dYaw(double x, double y, double z) {
		super(x, y, z);
	}

	public Point2dYaw(Tuple3d t1) {
		super(t1);
	}

	public Point2d getPosition() {
		return new Point2d(x,y);
	}

	/* Yaw in radians: (-pi/2 to +pi/2) */
	public double getYaw() {
		return z;
	}

}
