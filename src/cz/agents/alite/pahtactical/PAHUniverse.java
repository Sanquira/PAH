package cz.agents.alite.pahtactical;

import cz.agents.alite.common.entity.Entity;
import cz.agents.alite.tactical.creator.config.VillageConfig;
import cz.agents.alite.tactical.universe.TacticalUniverse;
import cz.agents.alite.tactical.universe.entity.embodiment.Car;
import cz.agents.alite.tactical.universe.entity.embodiment.CarModel;
import cz.agents.alite.tactical.universe.entity.embodiment.Team;
import cz.agents.alite.tactical.util.Point;

public class PAHUniverse extends TacticalUniverse {

    public PAHUniverse() {
    }

    public PAHUniverse(VillageConfig villageConfig) {
        super(villageConfig);
    }

    public void addCustomCarEntity(String name, Entity carAgent, Team team, Point waypoint, Car.VehicleType vehicleType) {

        entities.put(name, carAgent);

        // TODO -- here add a new argument that changes the car to a truck
        addCarToStorage(name, team, waypoint, null, null, vehicleType);
    }

}
