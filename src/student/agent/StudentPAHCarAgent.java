package student.agent;

import java.awt.Color;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import com.jme3.math.Vector3f;

import cz.agents.alite.pahtactical.agent.PAHCarAgent;
import cz.agents.alite.pahtactical.vis.PlanLayer;
import cz.agents.alite.pahtactical.vis.PlanLayer.PlanProvider;
import cz.agents.alite.tactical.universe.entity.embodiment.Car;
import cz.agents.alite.tactical.universe.entity.embodiment.Car.VehicleType;
import cz.agents.alite.tactical.universe.environment.TacticalEnvironment.TacticalEnvironmentHandler;
import cz.agents.alite.tactical.universe.world.map.Building;
import cz.agents.alite.tactical.universe.world.map.UrbanMap;
import cz.agents.alite.tactical.universe.world.physics.Terrain;
import cz.agents.alite.tactical.util.Point;
import cz.agents.alite.tactical.util.Polygon2d;
import cz.agents.alite.tactical.util.Visibility;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.element.StyledPoint;
import cz.agents.alite.vis.element.aggregation.StyledPointElements;
import cz.agents.alite.vis.element.implemetation.StyledPointImpl;
import cz.agents.alite.vis.layer.terminal.StyledPointLayer;
/**
 * Example PAH car agent for the first assignment of PAH course illustrating
 * the use of API.
 *
 * Takes implicit order of obstacles and tries to reach them by
 * straight line while ignoring obstacles.
 */
public class StudentPAHCarAgent extends PAHCarAgent {

    final static float CAR_SPEED = 30.0f;
    List<Point> checkpointsToGo;

    public StudentPAHCarAgent(String name, UrbanMap map, Terrain terrain, TacticalEnvironmentHandler handler, Set<Point> checkpoints) {
        super(name, map, terrain, handler, checkpoints);
    }

    @Override
    protected void ready() {
        // This method is called when the simulation has been properly initialized.

        System.out.println("Ready to go... ");

        // Current position can be obtained like this
        Point currentPosition = getCurrentPosition();
        System.out.println("My current position is " + currentPosition);

        // Current heading of the car can be obtained like this
        Vector3f currentHeading = getCurrentHeading();
        System.out.println("My current heading is " + currentHeading);

        // Current speed of the car can be obtained like this
        float currentSpeed = getCurrentSpeed();
        System.out.println("My current speed is " + currentSpeed);

        // The type of vehicle your agent currently controls
        VehicleType vehicleType = getVehicleType();
        System.out.println("The vehicle is: " + vehicleType);

        // The set of checkpoints can be obtained like this:
        Set<Point> checkpoints = getCheckpoints();
        System.out.println("Checkpoints: " + checkpoints);

        // The buildings (obstacles) can be obtained like this:
        List<Building> buildings = getMap().getBuildings();
        // The points defining the base of the building can be obtained like this:
        buildings.get(0).getPoints();
        // The polygon representing the base of the building can be obtained using:
        Polygon2d base = buildings.get(0).getBaseAsPolygon2d();
        // The base can be inflated using inflate method. The polygon is inflated by by 5m, where the arcs at corners are approximated by 2 points.
        Polygon2d inflatedBase = base.inflate(5.0, 2);

        // You can check if the points are straight-line visible (w.r.t. the buildings) like this:
        // E.g. the point 425, 979 is not visible from initial position
        boolean visible1 = getMap().getVisibility().isVisible(currentPosition, new Point(425,979, getAltitude(425, 979) + 1.0), Double.MAX_VALUE);

        // E.g. the point 650, 850 is visible from initial position
        boolean visible2 = getMap().getVisibility().isVisible(currentPosition, new Point(650, 850, getAltitude(650, 850) + 1.0), Double.MAX_VALUE);

        // Visibility with respect to customized polygons can be obtained by creating your own Visibility object:
        Visibility inflatedVisibility = new Visibility(Collections.singleton(inflatedBase));
        inflatedVisibility.isVisible(currentPosition, new Point(425,979, getAltitude(425, 979) + 1.0));

        // To get the z-coordinate of the terrain use:
        getAltitude(425, 979);

        // Order the checkpoints implicitly
        checkpointsToGo = new LinkedList<Point>(getCheckpoints());

        // Start moving towards the first checkpoint
        // The arguments are:
        // 1. Target waypoint,
        // 2. Target speed (km/h),
        // 3. Direction: true=forward, false=reverse,
        // 4. Waypoint reached tolerance -- the target waypoint is considered reached when the vehicle is at most this far
        goToWaypoint(checkpointsToGo.get(0), CAR_SPEED, true, 3.5f);

        // Create a 2D-Vis layer to visualize your planned path
        VisManager.registerLayer(PlanLayer.create(new PlanProvider() {

            @Override
            public List<Point> getPlan() {
                return getPlanForVis();
            }
        }, Color.BLUE, 2));

        // Create a 2D-Vis layer to visualize your checkpoints
        VisManager.registerLayer(StyledPointLayer.create( new StyledPointElements() {

			@Override
			public Iterable<? extends StyledPoint> getPoints() {
				LinkedList<StyledPoint> points = new LinkedList<StyledPoint>();

				for (Point checkPoint : checkpointsToGo) {
					points.add(new StyledPointImpl(checkPoint, Color.RED, 8));
				};
				return points;
			}
		}));



    }

    @Override
    protected void waypointReached(Point waypoint) {
    	 // Called when the waypoint set using the goToWaypoint method has been reached.

         System.out.println("Hurray, the waypoint " + waypoint + " has been reached!");
         if (waypoint != null && waypoint.epsilonEquals(checkpointsToGo.get(0), 1.0)) {

             halt();
             checkpointsToGo.remove(0);

             // go to next checkpoint
             if (!checkpointsToGo.isEmpty()) {
                 goToWaypoint(checkpointsToGo.get(0), CAR_SPEED, true, 3.5f);
             } else {
                 System.out.println("Hurray Hurray, done completely!!!");
             }
         }
    }

    @Override
    protected void positionChanged(Point newPosition) {
       // Called every time the position of the car changes.

       // System.out.println("Position changed to " + newPosition);
       // System.out.println("My current position is " + getCurrentPosition());
       // System.out.println("My current heading is " + getCurrentHeading());
       // System.out.println("My current speed is " + getCurrentSpeed());
    }

    @Override
    protected void tick(long simulationTime) {
       // Called with a given period of the simulation time. Roughly 1000 ms.
       // System.out.println("Tick. Distance driven so far: " + getDistance());
    }

    private List<Point> getPlanForVis() {
    	// The returned sequence of points will be shown in 2D Vis.

    	List<Point> plan = new LinkedList<Point>();

    	if (getCurrentPosition() != null) {
    		plan.add(getCurrentPosition());
    	}

    	plan.addAll(checkpointsToGo);
        return plan;
    }


}
