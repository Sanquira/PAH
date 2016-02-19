package cz.agents.alite.pahtactical.agent;

import java.util.HashSet;
import java.util.Set;

import javax.vecmath.Point2d;

import com.jme3.math.Vector3f;

import cz.agents.alite.common.entity.Entity;
import cz.agents.alite.tactical.universe.entity.embodiment.Car;
import cz.agents.alite.tactical.universe.environment.TacticalEnvironment.TacticalEnvironmentHandler;
import cz.agents.alite.tactical.universe.environment.actuator.GoToWaypointCarActuator;
import cz.agents.alite.tactical.universe.environment.sensor.AlarmSensor;
import cz.agents.alite.tactical.universe.environment.sensor.CarSensor;
import cz.agents.alite.tactical.universe.environment.sensor.callback.AlarmCallback;
import cz.agents.alite.tactical.universe.environment.sensor.callback.PositionChangeCallback;
import cz.agents.alite.tactical.universe.environment.sensor.callback.WaypointReachedCallback;
import cz.agents.alite.tactical.universe.world.map.UrbanMap;
import cz.agents.alite.tactical.universe.world.physics.Terrain;
import cz.agents.alite.tactical.util.Point;

public abstract class PAHCarAgent extends Entity {

    public interface AllCheckpointsVisitedCallback {
        void allCheckPointsVisited(double distance);
    };

    private double checkpointReachedTolerance = 6f;

    protected final TacticalEnvironmentHandler handler;

    private Set<Point> checkpoints;
    private Set<Point> unvisitedCheckpoints;
    private UrbanMap map;
    private Terrain terrain;

    protected final CarSensor carSensor;
    protected final AlarmSensor alarmSensor;
    protected final GoToWaypointCarActuator goToWaypointActuator;
    private AllCheckpointsVisitedCallback allCheckpointsVisitedCallback = null;

    private Point lastPosition = null;

    protected final int TICK_PERIOD = 1000;

    final AlarmCallback tickCallback = new AlarmCallback() {

        @Override
        public void senseAlarm(long simulationTime) {
            tick(simulationTime);
            alarmSensor.registerAlarmCallback(TICK_PERIOD, tickCallback);
        }
    };

    private double distance = 0.0;
    private boolean finished;

    public PAHCarAgent(final String name, final UrbanMap map,
            Terrain terrain, final TacticalEnvironmentHandler handler, Set<Point> checkpoints) {
        super(name);

        this.handler = handler;
        this.map = map;
        this.terrain =  terrain;
        this.checkpoints = checkpoints;
        this.unvisitedCheckpoints = new HashSet<Point>();

        for (Point checkpoint : checkpoints) {
            unvisitedCheckpoints.add(new Point(checkpoint));
        }

        carSensor = handler.addSensor(CarSensor.class, this);
        alarmSensor = handler.addSensor(AlarmSensor.class, this);
        goToWaypointActuator = handler.addAction(GoToWaypointCarActuator.class,
                this);

        alarmSensor.registerAlarmCallback(1, new AlarmCallback() {
            @Override
            public void senseAlarm(long simulationTime) {
                ready();
            }
        });

        carSensor.registerWaypointReachedCallback(new WaypointReachedCallback() {

                    @Override
                    public void waypointReached(Point waypoint) {
                        if (waypoint != null) {
                            System.out.println("Waypoint reached: " + waypoint);
                            PAHCarAgent.this.waypointReached(waypoint);
                        }
                    }
                });

        carSensor.registerMyPositionChangeCallback(new PositionChangeCallback() {

                    @Override
                    public void sensePositionChange(Point position) {
                    	checkPlanFulfilment(position);
                        measureDistance(position);
                        positionChanged(position);
                    }
                });

        alarmSensor.registerAlarmCallback(2, tickCallback);
    }

    private void checkPlanFulfilment(Point waypoint) {
        if (!unvisitedCheckpoints.isEmpty()) {
            Point checkpoint
                = removeNearCheckpoint(unvisitedCheckpoints, waypoint, checkpointReachedTolerance);

            if (checkpoint != null) {
                System.out.println("### Checkpoint " + checkpoint + " visited ###");
            }
        }

        if (unvisitedCheckpoints.isEmpty() && !finished) {
            System.out.println("### All checkpoints visited!  ###");
            System.out.println("### Travelled distance: " + getDistance() + " ###");

            if (allCheckpointsVisitedCallback != null) {
                allCheckpointsVisitedCallback.allCheckPointsVisited(getDistance());
            }
            finished = true;
        }
    }

    private Point removeNearCheckpoint(Set<Point> checkpoints, Point point, double distance) {
        Point nearCheckpoint = null;
        for (Point checkpoint : checkpoints)  {
        	Point2d checkpoint2d = new Point2d(checkpoint.x, checkpoint.y);
        	Point2d point2d = new Point2d(point.x, point.y);

            if (checkpoint2d.distance(point2d) <= distance) {
                nearCheckpoint = checkpoint;
            }
        }

        if (nearCheckpoint != null) {
            checkpoints.remove(nearCheckpoint);
        }

        return nearCheckpoint;
    }

    private void measureDistance(Point position) {
        if (lastPosition != null) {
            distance += position.distance(lastPosition);
        }
        lastPosition = position;
    }

    /**
     * Returns the map of the urban area.
     */
    public UrbanMap getMap() {
        return map;
    }

    /**
     * Returns the set of checkpoints that have to be visited.
     */
    public Set<Point> getCheckpoints() {
        return checkpoints;
    }

    public TacticalEnvironmentHandler getHandler() {
        return handler;
    }

    /**
     * Returns the distance traveled so far by the car.
     */
    public double getDistance() {
        return distance;
    }

    /**
     * This method is called once the simulation is properly initialized.
     * Start executing your plan in this method.
     */
    abstract protected void ready();

    /**
     * This method is called when a waypoint set using GoToWaypoint actuator has been reached.
     */
    abstract protected void waypointReached(Point waypoint);

    /**
     * This method is called periodically when the position of the car changes.
     */
    abstract protected void positionChanged(Point newPosition);

    /**
     * This method is called periodically every 1000 millisecond of simulation time.
     */
    abstract protected void tick(long simulationTime);

    /**
     * Makes the car to go towards the given waypoint at the given target velocity.
     * It can be specified whether the car should go forward, or reverse.
     */
    public void goToWaypoint(Point waypoint, float targetVelocity, boolean isForward, float waypointReachedTolerance) {
        goToWaypointActuator.actGoToWaypoint(waypoint, targetVelocity, isForward, waypointReachedTolerance);
    }

    /**
     * Cancels the current goToWaypoint task.
     */
    public void halt() {
        goToWaypointActuator.actHalt();
    }

    /**
     * Returns the z-coordinate of the terrain surface at the given position.
     */
    public double getAltitude(double x, double y) {
        return terrain.getAltitude((float) x, (float) y);
    }

    /**
     * Returns the current position of the car.
     */
    public Point getCurrentPosition() {
        return carSensor.senseMyPosition();
    }

    /**
     * Returns the current heading of the car.
     */
    public Vector3f getCurrentHeading() {
        return carSensor.senseMyHeading();
    }

    /**
     * Returns the current speed of the car.
     */
    public float getCurrentSpeed() {
        return carSensor.senseMySpeed();
    }

    public Car.VehicleType getVehicleType() {
    	return carSensor.senseCarType();
    }

    public final void registerAllCheckpointsVisistedCallback(AllCheckpointsVisitedCallback callback) {
        allCheckpointsVisitedCallback = callback;
    }

    public void setCheckpointReachedTolerance(double tolerance) {
    	this.checkpointReachedTolerance = tolerance;
    }
}
