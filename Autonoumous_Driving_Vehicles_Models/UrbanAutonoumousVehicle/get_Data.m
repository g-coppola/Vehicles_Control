[scenario, egoVehicle,actors] = ScenarioBuild();

truckObj = actors.truck;
bikeObj  = actors.bicycle;
ped1Obj  = actors.pedestrian;
ped2Obj  = actors.pedestrian1; 

ego_x = []; ego_y = [];
ego_v = []; ego_yaw = [];

truck_x = []; truck_y = []; truck_yaw = []; truck_v = [];
bike_x  = []; bike_y  = []; bike_yaw = []; bike_v = [];
ped1_x  = []; ped1_y  = []; ped1_yaw = []; ped1_v = [];
ped2_x  = []; ped2_y  = []; ped2_yaw = []; ped2_v = [];

t = []; 

while advance(scenario)
    pause(0.01); 
    
    ego_x = [ego_x; egoVehicle.Position(1)];
    ego_y = [ego_y; egoVehicle.Position(2)];

    yaw = deg2rad(egoVehicle.Yaw);
    ego_yaw = [ego_yaw; yaw];

    vx = egoVehicle.Velocity(1);
    vy = egoVehicle.Velocity(2);

    v = vx*cos(yaw) + vy * sin(yaw);
    ego_v = [ego_v; v]; 
    
    if ~isempty(truckObj)
        truck_x = [truck_x; truckObj.Position(1)];
        truck_y = [truck_y; truckObj.Position(2)];

        vx = truckObj.Velocity(1);
        vy = truckObj.Velocity(2);

        yaw = deg2rad(truckObj.Yaw);
        truck_yaw = [truck_yaw; yaw];

        v = vx*cos(yaw) + vy * sin(yaw);

        truck_v = [truck_v; v];

    end
    
    if ~isempty(bikeObj)
        bike_x = [bike_x; bikeObj.Position(1)];
        bike_y = [bike_y; bikeObj.Position(2)];

        vx = bikeObj.Velocity(1);
        vy = bikeObj.Velocity(2);

        yaw = deg2rad(bikeObj.Yaw);
        bike_yaw = [bike_yaw; yaw];

        v = vx*cos(yaw) + vy * sin(yaw);

        bike_v = [bike_v; v];
    end
    
    if ~isempty(ped1Obj)
        ped1_x = [ped1_x; ped1Obj.Position(1)];
        ped1_y = [ped1_y; ped1Obj.Position(2)];

        vx = ped1Obj.Velocity(1);
        vy = ped1Obj.Velocity(2);

        yaw = deg2rad(ped1Obj.Yaw);
        ped1_yaw = [ped1_yaw; yaw];

        v = vx*cos(yaw) + vy * sin(yaw);

        ped1_v = [ped1_v; v];
        
    end
    
    if ~isempty(ped2Obj)
        ped2_x = [ped2_x; ped2Obj.Position(1)];
        ped2_y = [ped2_y; ped2Obj.Position(2)];

        vx = ped2Obj.Velocity(1);
        vy = ped2Obj.Velocity(2);

        yaw = deg2rad(ped2Obj.Yaw);
        ped2_yaw = [ped2_yaw; yaw];

        v = vx*cos(yaw) + vy * sin(yaw);

        ped2_v = [ped2_v; v];
    end
    
    t = [t; scenario.SimulationTime];
end

truck.x = truck_x; truck.y = truck_y; truck.v = truck_v; truck.yaw = truck_yaw;
bike.x = bike_x; bike.y = bike_y; bike.v = bike_v; bike.yaw = bike_yaw;
ped1.x = ped1_x; ped1.y = ped1_y; ped1.v = ped1_v; ped1.yaw = ped1_yaw;
ped2.x = ped2_x; ped2.y = ped2_y; ped2.v = ped2_v; ped2.yaw = ped2_yaw;

save('SimulationData.mat','scenario','t','ego_x','ego_y','ego_v','ego_yaw','truck','bike','ped1','ped2')
clear
clc