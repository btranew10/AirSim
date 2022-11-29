// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include "GageLidarParsing.hpp"

#pragma region Function Declarations

void runSingleClient(const uint16_t port, const int ordinal);
void gageBasicSoloLidarTestMain();
void testRunSingleClient();


#pragma endregion

int main()
{
    //testRunSingleClient();
    //gageBasicSoloLidarTestMain();
    using namespace msr::airlib;
    std::unique_ptr<MultirotorRpcLibClient> client;
    try 
    {
        point tmpP; //use this just as a placeholder point variable so you can fill other points easily. (I plan on improving this later)

        float speedA = 5.0f;
        float durationA = 8.0f;
        point startA = tmpP.makePoint(5.0f, 5.0f, 0.0f);
        point endA = tmpP.makePoint(10.0f, 5.0f, 5.0f);

        float speedB = 8.0f;
        float durationB = 12.0f;
        point startB = tmpP.makePoint(-5.0f, -5.0f, 0.0f);
        point endB = tmpP.makePoint(10.0f, 5.0f, 5.0f);

        client = std::unique_ptr<MultirotorRpcLibClient>(new MultirotorRpcLibClient());
        client->confirmConnection();
        client->enableApiControl(true);

        GageDrone droneA("Drone A", startA, endA);
        GageDrone droneB("Drone B", startB, endB);

        droneA.spawnDrone();
        droneB.spawnDrone();

        std::vector<std::thread> clientThreads;
        std::this_thread::sleep_for(std::chrono::duration<double>(1.0f));

        clientThreads.push_back(std::thread(&GageDrone::droneMovement, &droneA, speedA, durationA));
        std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
        clientThreads.push_back(std::thread(&GageDrone::droneMovement, &droneB, speedB, durationB));
        std::this_thread::sleep_for(std::chrono::duration<double>(0.1));

        for (auto& toJoin : clientThreads) 
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



int testDroneTools()
{
    using namespace msr::airlib;
    std::unique_ptr<MultirotorRpcLibClient> client;
    try {
        client = std::unique_ptr<MultirotorRpcLibClient>(new MultirotorRpcLibClient());
        client->confirmConnection();
        client->armDisarm(false);

        client->enableApiControl(true);
        client->armDisarm(true);

        point startA;
        startA.makePoint(5.0f, 5.0f, 0.0f);
        point endA;
        endA.makePoint(10.0f, 5.0f, 5.0f);

        point startB;
        startB.makePoint(-5.0f, -5.0f, 0.0f);
        point endB;
        endB.makePoint(10.0f, 5.0f, 5.0f);

        GageDrone droneA("Drone A", startA, endA);

        GageDrone droneB("Drone B", startB, endB);

        droneA.spawnDrone();
        droneB.spawnDrone();

        //droneA.droneMovement(client, 5.0f, 8.0f);
        //droneB.droneMovement(client, 8.0f, 12.0f);

        client->armDisarm(false, droneA.name);
    }
    catch (const std::exception&) {
        return -1;
    }
    return 0;
}


#pragma region Mains 

void gageBasicSoloLidarTestMain()
{

    using namespace msr::airlib;

    //msr::airlib::MultirotorRpcLibClient client;
    std::unique_ptr<MultirotorRpcLibClient> client;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;

    try 
    {
        client = std::unique_ptr<MultirotorRpcLibClient>(new MultirotorRpcLibClient());
        client->confirmConnection();
        client->armDisarm(false);

        client->enableApiControl(true);
        client->armDisarm(true);

        float takeoff_timeout = 5.0f;
        client->takeoffAsync(takeoff_timeout)->waitOnLastTask();
        std::this_thread::sleep_for(std::chrono::duration<double>(5));

        int i = 0;
        while (i < 3) 
        {

            // switch to explicit hover mode so that this is the fall back when
            // move* commands are finished.
            client->hoverAsync()->waitOnLastTask();

            // moveByVelocityZ is an offboard operation, so we need to set offboard mode.
            client->enableApiControl(true);

            auto position = client->getMultirotorState().getPosition();
            float z = position.z(); // current position (NED coordinate system).
            constexpr float speed = 10.0f;
            constexpr float size = 25.0f;
            constexpr float duration = size / speed;
            DrivetrainType drivetrain = DrivetrainType::ForwardOnly;
            YawMode yaw_mode(true, 0);

            //auto lidarData = client.getLidarData();
            lidarScan(client, 5.0f, false, true);

            //I want it to go a bit higher before circling
            float height = -10.0f;
            float ascVel = 5.0f;
            float ascTimeout = 5.0f;
            std::cout << "Ascending further to " << -height << "m..." << std::endl;
            client->moveToZAsync(height, ascVel, ascTimeout, yaw_mode)->waitOnLastTask();

            std::this_thread::sleep_for(std::chrono::duration<double>(2.5));
            z = client->getMultirotorState().getPosition().z(); //update z position so it stays at current alt

            std::cout << "Beginning Square Flight..." << std::endl;
            client->moveByVelocityZAsync(speed, 0, z, duration, drivetrain, yaw_mode);
            std::this_thread::sleep_for(std::chrono::duration<double>(duration));

            client->moveByVelocityZAsync(0, speed, z, duration, drivetrain, yaw_mode);

            lidarScan(client, 5.0f, false, true);

            std::this_thread::sleep_for(std::chrono::duration<double>(duration));

            lidarScan(client, 5.0f, false, true);

            client->moveByVelocityZAsync(-speed, 0, z, duration, drivetrain, yaw_mode);
            std::this_thread::sleep_for(std::chrono::duration<double>(duration));
            client->moveByVelocityZAsync(0, -speed, z, duration, drivetrain, yaw_mode);
            std::this_thread::sleep_for(std::chrono::duration<double>(duration));

            //easyFlyInSquare(speed, size); //[TODO] Cant figure out how to pass client as a param...

            std::this_thread::sleep_for(std::chrono::duration<double>(1.5));
            client->hoverAsync()->waitOnLastTask();
            std::cout << "Descending..." << std::endl;
            client->moveToZAsync(-height, ascVel, ascTimeout, yaw_mode)->waitOnLastTask();

            std::cout << "Beginning Landing..." << std::endl;
            client->landAsync()->waitOnLastTask();
            std::this_thread::sleep_for(std::chrono::duration<double>(5));

            std::cout << "Disarming..." << std::endl;
            client->armDisarm(false);
            std::this_thread::sleep_for(std::chrono::duration<double>(5));
            //client.waitOnLastTask(); causes crash???

            //std::cout << "Parsing Lidar Data..." << std::endl;
            //vector<point> lidarData = parseLidarData(client.getLidarData());
            //printLidarScan(lidarData);

            i++;
        }
    }
    catch (rpc::rpc_error& e) 
    {
        const auto msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
    }
}


void testRunSingleClient()
{
    using namespace msr::airlib;

    uint16_t rpc_port = 41451;
    int num_platforms = 2;

    std::cout << "First port is " << rpc_port << std::endl;
    std::cout << "Num platforms: " << num_platforms << std::endl;
    std::cout << "Making clients..." << std::endl;

    try {

        std::vector<std::thread> clientThreads;

        // Count down, so the first one can easily go the highest (without knowing count)
        int client_ordinal = num_platforms - 1;
        for (int i = 0; i < num_platforms; i++) {
            clientThreads.push_back(std::thread(runSingleClient, rpc_port, client_ordinal));
            client_ordinal--;
            std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
        }

        for (auto& toJoin : clientThreads) {
            toJoin.join();
        }
    }
    catch (rpc::rpc_error& e) {
        const auto msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
    }
}



#pragma endregion

#pragma region GAGEs LIDAR DATA PARSING STUFF

std::vector<point> parseLidarData(msr::airlib::LidarData data)
{
    //NOTE: For set up of the LIDAR --> https://microsoft.github.io/AirSim/lidar/
    /*
    * original point-cloud is just a vector<float> where 
    * data is stored as: {x0,y0,z0,x1,y1,z1,...xn,yn,zn}.
    * I want to parse this into a more organized manner 
    * that's easier to work with and will make sure it
    * is always clear what type of info we are referencing.
    * So, I'm splitting it into a doubly-nested data struct, 
    * so you can reference it as parsedPointCloud[i].x/y/z 
    * where i is index of each different point and you just 
    * specify .x y or z to get the corresponding point.
    */
    std::vector<point> parsedPointCloud;

    int currPoint = 0;
    float tempArr[] = { 0, 0, 0 };
    for (float f : data.point_cloud) 
    {
        if (currPoint == 3) 
        {
            /*
            std::cout << "Point Cloud @ " << lidarData.time_stamp << " was: ["
                      << tempArr[0] << ","
                      << tempArr[1] << ","
                      << tempArr[2] << "]" << 
            std::endl;
            */

            point p;
            p.x = tempArr[0];
            p.y = tempArr[1];
            p.z = tempArr[2];

            parsedPointCloud.push_back(p);

            currPoint = 0; //reset curr point so can grab next 3 items
        }
        tempArr[currPoint++] = f;
    }
    return parsedPointCloud;
}

int printLidarScan(std::vector<point> pointCloud)
{
    int i = 0;
    for (point p : pointCloud) 
    {
        std::cout << "Point " << i++ << " : ["
                  << p.x << ","
                  << p.y << ","
                  << p.z << "]" << std::endl;
    }
    return i;
}
int printLidarScan(std::vector<point> pointCloud, msr::airlib::TTimePoint time)
{
    int i = 0;
    for (point p : pointCloud) 
    {
        std::cout << "[Time: " << time << "] Point " << i++ << " : ["
                  << p.x << ","
                  << p.y << ","
                  << p.z << "]" << std::endl;
    }
    return i;
}

std::vector<point> findPointsInCritRange(msr::airlib::MultirotorState state, std::vector<point> pointCloud, float criticalRange)
{
    point selfPoint;
    selfPoint.getCurrPos(state);

    std::vector<point> dangerCloud;

    for (point p : pointCloud) 
    {
        if (calculateDistance(selfPoint, p) >= criticalRange) dangerCloud.push_back(p);
    }

    return dangerCloud;
}

std::vector<point> lidarScan(std::unique_ptr<msr::airlib::MultirotorRpcLibClient>& client, float critRadius, bool printCriticalCloud, bool printPercent)
{
    using namespace std;
    std::cout << "Parsing Lidar Data..." << std::endl;
    vector<point> lidarData = parseLidarData(client->getLidarData());
    vector<point> criticalPointCloud = findPointsInCritRange(client->getMultirotorState(), lidarData, critRadius);
    if (printCriticalCloud) printLidarScan(criticalPointCloud);
    if (printPercent) {
        std::cout << "\n\n\t% in Critical Range of " << critRadius << "m (" << criticalPointCloud.size()
                  << "/" << lidarData.size() << "):  " << getPercentSurrounded(lidarData.size(), criticalPointCloud.size())
                  << std::endl;
    }
    return criticalPointCloud;
}


#pragma endregion


/// <summary>
/// Works. Used in testRunSingleClient() to 
/// create a basic test where two drones are spawned that fly into eachother
/// according to the Converging Collision scenario type. 
/// TODO: Do not think it is essential to make port, passed.
/// <para>...</para>
/// Author: Malachi Sanderson
/// <para>...</para>
/// Made: 11-12-22
/// <para></para>
/// </summary>
/// <param name="port"></param>
/// <param name="ordinal"></param>
void runSingleClient(const uint16_t port, const int ordinal)
{
    using namespace msr::airlib;
    //msr::airlib::MultirotorRpcLibClient client;
    std::unique_ptr<MultirotorRpcLibClient> client;
    typedef common_utils::FileSystem FileSystem;

    constexpr char host[] = "localhost";
    float timeout_s = 60;


    try 
    {
        client = std::unique_ptr<MultirotorRpcLibClient>(new MultirotorRpcLibClient(host, port, timeout_s));
        client->confirmConnection();

        const std::string drone_A_name = "UAV_1";
        const std::string drone_B_name = "UAV_0";

        const std::string vehicle_name = "UAV_" + std::to_string(ordinal);
        constexpr float takeoff_timeout = 5;
        // Altitude difference between each platform, in meters
        
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom;
        const YawMode yaw_mode(false, 0);
        const float timeout = 10.0f;
        constexpr float speed = 3.0f;
        //want both to be at 10 up.
        constexpr float height = -20.0f;

        const float distMod = 5.0f;
        point locB;
        point locA;
        point startA;
        point startB;
        point targetA;
        point targetB;
        point expectedCollisionPoint;
        point pos;

        startA.makePoint(1.0f*distMod, 1.0f, 0); //(x = 5, y = 1, z = 0)
        startB.makePoint(2.0f * distMod, -1.0f * distMod, 0); //(x = 10, y = -5, z = 0)

        targetA.makePoint(3.0f * distMod, 0, height); //(x = 15, y = 0, 
        targetB.makePoint(0, 2.0f * distMod, height); //(x = 0, y = 10,

        expectedCollisionPoint.makePoint(1.0f * distMod, 0, height);


        // drone B...see startB
        if (ordinal == 0) pos.copy(startB);
        //drone A...see startA
        if (ordinal == 1) pos.copy(startA);

        Pose pose(pos.toVector3r(), Quaternionr(0, 0, 0, 0));
        client->simAddVehicle(vehicle_name, "simpleflight", pose, "");
            
        
        // This is a bit crude, but give it a moment to settle on the ground, else takeoff will fail
        std::this_thread::sleep_for(std::chrono::duration<double>(2));
        locB.getCurrPos(client->getMultirotorState(drone_B_name));
        locA.getCurrPos(client->getMultirotorState(drone_A_name));
        //pos.getCurrPos(client);


        //moveByVelocityZ is an offboard operation, so we need to set offboard mode.
        client->enableApiControl(true, vehicle_name);
        client->armDisarm(true, vehicle_name);

        
        client->takeoffAsync(takeoff_timeout, vehicle_name)->waitOnLastTask();


        //std::cout << vehicle_name << " is ascending..." << std::endl;
        client->moveToZAsync(height, 5.0f, timeout, yaw_mode, -1.0f, 1.0f, vehicle_name)->waitOnLastTask();

        locB.getCurrPos(client->getMultirotorState(drone_B_name));
        locA.getCurrPos(client->getMultirotorState(drone_A_name));
        
        

        // switch to explicit hover mode so that this is the fallback when
        // move* commands are finished.
        client->hoverAsync(vehicle_name)->waitOnLastTask();

        locB.getCurrPos(client->getMultirotorState(drone_B_name));
        locA.getCurrPos(client->getMultirotorState(drone_A_name));

        pos.makePoint(0.0f, 0.0f, 0.0f);
        float duration = 8.0f;

        if (ordinal == 0) // B moves to target B
        { 
            pos.copy(targetB);
            //duration *= 4; // B has a longer delay than A
        }
        if (ordinal == 1) // A moves to target A
        {
            pos.copy(targetA);
            //duration *= 1; // A has a longer delay than B
        }

        client->moveToPositionAsync(pos.x, pos.y, pos.z, speed, duration, drivetrain, yaw_mode, -1.0f, 1.0f, vehicle_name);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));

        client->hoverAsync(vehicle_name)->waitOnLastTask();
        std::this_thread::sleep_for(std::chrono::duration<double>(1.5));

        std::cout << "Disarming: " << vehicle_name << "..." << std::endl;
        client->armDisarm(false, vehicle_name);

    }
    catch (rpc::rpc_error& e) 
    {
        const auto msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
    }
}



#pragma region Easy Drone Flight Pattern Commands


bool easyFlyInSquare(std::unique_ptr<msr::airlib::MultirotorRpcLibClient>& client, float speed, float size)
{
    using namespace msr::airlib;

    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;

    try 
    {
        client->confirmConnection();
        point pos;
        pos.getCurrPos(client);
        float duration = size / speed;
        DrivetrainType drivetrain = DrivetrainType::ForwardOnly;
        YawMode yaw_mode(true, 0);

        //std::cout << "Beginning Square Flight..." << std::endl;
        client->moveByVelocityZAsync(speed, 0, pos.z, duration, drivetrain, yaw_mode);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        client->moveByVelocityZAsync(0, speed, pos.z, duration, drivetrain, yaw_mode);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        client->moveByVelocityZAsync(-speed, 0, pos.z, duration, drivetrain, yaw_mode);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        client->moveByVelocityZAsync(0, -speed, pos.z, duration, drivetrain, yaw_mode);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
    }
    catch (rpc::rpc_error& e) 
    {
        const auto msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
    }

    return false;
}

bool easyFlyInSquare(std::unique_ptr<msr::airlib::MultirotorRpcLibClient>& client, std::string droneName, float speed, float size)
{
    using namespace msr::airlib;

    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;

    try {
        client->confirmConnection();
        point pos;
        pos.getCurrPos(client->getMultirotorState(droneName));
        float duration = size / speed;
        msr::airlib::DrivetrainType drivetrain = msr::airlib::DrivetrainType::ForwardOnly;
        msr::airlib::YawMode yaw_mode(true, 0);

        //std::cout << "Beginning Square Flight..." << std::endl;
        client->moveByVelocityZAsync(speed, 0, pos.z, duration, drivetrain, yaw_mode, droneName);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        client->moveByVelocityZAsync(0, speed, pos.z, duration, drivetrain, yaw_mode, droneName);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        client->moveByVelocityZAsync(-speed, 0, pos.z, duration, drivetrain, yaw_mode, droneName);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        client->moveByVelocityZAsync(0, -speed, pos.z, duration, drivetrain, yaw_mode, droneName);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
    }
    catch (rpc::rpc_error& e) {
        const auto msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
    }

    return false;
}



#pragma endregion



#pragma region My Function Testing Methods

/// <summary>
/// Just a basic example of how to mess with methods that take client as a parameter.
/// <para>...</para>
/// Author: Malachi Sanderson
/// <para>...</para>
/// Made: 11-12-22
/// <para></para>
/// </summary>
void TEST_POINT_GETPOS()
{
    point pos;
    constexpr char host[] = "localhost";
    float timeout_s = 60;
    uint16_t rpc_port = 41451;
    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> client = std::unique_ptr<msr::airlib::MultirotorRpcLibClient>(new msr::airlib::MultirotorRpcLibClient(host, rpc_port, timeout_s));
    pos.getCurrPos(client);
    std::cout << "GOT CLIENT PTR POS: " << pos.toString() << std::endl;
}



#pragma endregion





