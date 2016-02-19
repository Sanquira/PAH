package student.creator;

import java.util.HashSet;
import java.util.Set;

import org.apache.log4j.Logger;

import cz.agents.alite.pahtactical.creator.DefaultCreator;
import cz.agents.alite.tactical.universe.entity.embodiment.Car.VehicleType;
import cz.agents.alite.tactical.util.Point;

public class StudentTestingCreator extends DefaultCreator {

    private static final Logger LOGGER = Logger.getLogger(StudentTestingCreator.class);
    private static final double Z_OFFSET = 1.5;

    @Override
    protected void createEntities() {
        LOGGER.info(">>> CREATING UGV ENTITY");

        // Generate the checkpoints
        Point initLocation = new Point (569, 800, getAltitude(569,800) + Z_OFFSET);
        Set<Point> checkpoints = new HashSet<Point>();
        checkpoints.add(getPoint(724,550));
        checkpoints.add(getPoint(648,554));
        checkpoints.add(getPoint(616,586));
        checkpoints.add(getPoint(847,461));
        checkpoints.add(getPoint(790,366));

        checkpoints.add(getPoint(908,495));
        checkpoints.add(getPoint(949.76,828.7));
        checkpoints.add(getPoint(900,838));
        checkpoints.add(getPoint(915,899));
        checkpoints.add(getPoint(873,846));

        // Add a UGV
        addCarAgent(initLocation, checkpoints, 3.5, VehicleType.V3S); // Change to VehicleType.V3S to use the truck
    }

	protected Point getPoint(double x, double y) {
        return new Point(x, y, getAltitude(x, y) + Z_OFFSET);
    }

    public static void main(String[] args) {
        StudentTestingCreator creator = new StudentTestingCreator();
        creator.init(args);
        creator.enableVis2d(true);
        // Here you can enable/disable 3-d Visualization
        creator.enableVis3d(true);
        creator.create();
    }
}
