
#include "GageDrone.hpp"

int ExampleMain()
{
    using namespace msr::airlib;
    std::unique_ptr<MultirotorRpcLibClient> client;
    try 
    {
        point tmpP; //use this just as a placeholder point variable so you can fill other points easily. (I plan on improving this later)

        float speedA = 5.0f;
        float durationA = 8.0f;
        point startA = tmpP.makePoint(5.0f, 5.0f, 0.0f);
        point endA = tmpP.makePoint(10.0f, 5.0f, 5.0f);
        float heightA = -2.0f;

        float speedB = 8.0f;
        float durationB = 12.0f;
        point startB = tmpP.makePoint(-5.0f, -5.0f, 0.0f);
        point endB = tmpP.makePoint(10.0f, 5.0f, 5.0f);
        float heightB = -4.0f;

        client = std::unique_ptr<MultirotorRpcLibClient>(new MultirotorRpcLibClient());
        client->confirmConnection();
        client->enableApiControl(true);

        GageDrone droneA("Drone A", startA, endA, heightA);
        GageDrone droneB("Drone B", startB, endB, heightB);

        //TODO: the two below methods were supposed to do the all the stuff below them, but can't get em to work rn.
        //SceneDrones sc(droneA, speedA, durationA, droneB, speedB, durationB);
        //sc.runDroneScene();

        
        droneA.spawnDrone();
        droneB.spawnDrone();

        std::vector<std::thread> threads;
        std::this_thread::sleep_for(std::chrono::duration<double>(1.0f));

        threads.push_back(std::thread(&GageDrone::droneMovement, &droneA, speedA, durationA));
        std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
        threads.push_back(std::thread(&GageDrone::droneMovement, &droneB, speedB, durationB));
        std::this_thread::sleep_for(std::chrono::duration<double>(0.1));

        for (auto& toJoin : threads) 
        {
            toJoin.join();
        }
        
    }
    catch (const std::exception&) 
    {
        return -1;
    }

    return 0;
}



GageDrone::GageDrone(std::string name, point start, point end, float height)
{
    if (end.z > 0) end.z *= -1; //if caller forgot that z axis follows NED axis.
    this->name = name;
    this->start = start;
    this->end = end;
    this->height = height;
    this->makeFinalPoint();
}

void GageDrone::makeFinalPoint()
{
    point tmp;
    tmp.copy(this->start);
    this->path = this->end.vectorDistanceTo(tmp);
}


void GageDrone::spawnDrone()
{
    using namespace msr::airlib;
    std::unique_ptr<MultirotorRpcLibClient> client;

    try 
    {
        client = std::unique_ptr<MultirotorRpcLibClient>(new MultirotorRpcLibClient());
        client->confirmConnection();
        Pose pose(this->start.toVector3r(), Quaternionr(0, 0, 0, 0));
        client->simAddVehicle(this->name, "simpleflight", pose, "");
        std::this_thread::sleep_for(std::chrono::duration<double>(2));
        //NEXT NEED TO ADJUST END.Z AGAIN BECAUSE, WHEN SPAWNING, I CAN'T PREDICT IF THEY SPAWN AT WEIRD HEIGHTS!
        point currPos;
        currPos.getCurrPos(client->getMultirotorState(this->name));
        std::cout << currPos.toString() << std::endl;
        this->path.z += currPos.z;

    }
    catch (const std::exception& e) 
    {
        std::cout << e.what() << std::endl;
    }
    
}


void GageDrone::droneMovement(float speed, float duration)
{
    using namespace msr::airlib;
    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> client;
    try {
        client = std::unique_ptr<MultirotorRpcLibClient>(new MultirotorRpcLibClient());
        client->confirmConnection();

        constexpr float takeoff_timeout = 5;
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom;
        const YawMode yaw_mode(false, 0);
        const float timeout = 10.0f;

        //moveByVelocityZ is an offboard operation, so we need to set offboard mode.
        client->enableApiControl(true, this->name);
        client->armDisarm(true, this->name);

        client->takeoffAsync(takeoff_timeout, this->name)->waitOnLastTask();
        // client->moveToZAsync(this->getPath().z, 5.0f, timeout, yaw_mode, -1.0f, 1.0f, this->name)->waitOnLastTask();
        client->moveToZAsync(height, speed, duration, yaw_mode, -1.0f, 1.0f, this->name)->waitOnLastTask();

        // switch to explicit hover mode so that this is the fallback when
        // move* commands are finished.
        client->hoverAsync(this->name)->waitOnLastTask();
        point movPath = this->getPath();

        client->moveToPositionAsync(movPath.x, movPath.y, movPath.z, speed, duration, drivetrain, yaw_mode, -1.0f, 1.0f, this->name);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));

        client->hoverAsync(this->name)->waitOnLastTask();
    }
    catch (rpc::rpc_error& e) {
        //std::cout << e.what() << std::endl;
        const auto msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong. In drone: " << this->name << std::endl
                  << msg << std::endl;
    }
}



int SceneDrones::runDroneScene()
{

    //std::vector<GageDrone> drones = this->getAllDrones();
    std::vector<std::thread> threads;
    try 
    {
        for (DroneTestInfo d : this->drones) 
        {
            d.drone.spawnDrone();
            threads.push_back(std::thread(&GageDrone::droneMovement, &(d.drone), d.speed, d.duration));
            std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
        }
        for (auto& toJoin : threads) 
        {
            toJoin.join();
        }
        return 1;
    }
    catch (const std::exception& e) 
    {
        std::cout << "\n\t\t[ERROR: in runDroneScene]\n"
                  << e.what() << std::endl;
        return -1;
    }
}
