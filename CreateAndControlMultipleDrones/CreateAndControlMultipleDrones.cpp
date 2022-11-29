/*
Andrew Livera, Benjamin Ranew
CS 490 Computer Science Capstone Design I
Dr. Akbas
November 27, 2022

CreateAndControlMultipleDrones.cpp

The program "CreateAndControlMultipleDrones.cpp" modifies some of "GageDrone.cpp" and directly relies on "GageDrone.hpp" and indirectly relies on "Point.hpp", and it can run 4 tests with 2 drones at varying initial heights with output from Test 1 written to the file "Test1.txt".

Tests
1) Drone A is moving, Drone B is stationary, and they collide.
2) Drone B is ahead of Drone A, Drone A is faster than Drone B, and they collide.
3) Drone A is moving north, Drone B is moving south, Drone A flies straight into Drone B, and they collide.
4) Drone A is moving north, Drone B is moving west, and they collide.

References
Microsoft Corporation. (2022). AirSim (Version 1.8.1) [Computer software]. Microsoft Corporation. https://github.com/microsoft/AirSim
Sanderson, M. (2022a). GageDrone.cpp [Computer software]. Sanderson, M. https://github.com/MalachiSanderson/UAS-Drone-Simulation-2022-2023-/blob/main/AirSim/GageTestProject/GageDrone.cpp
Sanderson, M. (2022b). GageDrone.hpp [Computer software]. Sanderson, M. https://github.com/MalachiSanderson/UAS-Drone-Simulation-2022-2023-/blob/main/AirSim/GageTestProject/GageDrone.hpp
Sanderson, M. (2022c). Point.hpp [Computer software]. Sanderson, M. https://github.com/MalachiSanderson/UAS-Drone-Simulation-2022-2023-/blob/main/AirSim/GageTestProject/Point.hpp
*/

#include "GageDrone.hpp"

int test1()
// Drone A is moving, Drone B is stationary, and they collide.
{
    using namespace msr::airlib;
    std::unique_ptr<MultirotorRpcLibClient> client;

    try {
        std::ofstream outputFile;
        outputFile.open("Test1.txt");
        
        outputFile << "Test 1" << std::endl;
        outputFile << std::endl;
        
        outputFile << "Scenario Type: Static" << std::endl;
        outputFile << "Test Description: Drone A is moving, Drone B is stationary, and they collide." << std::endl;
        outputFile << std::endl;
        
        outputFile << "Algorithm: PLACEHOLDER" << std::endl;
        outputFile << std::endl;

        point tmpP;

        float speedA = 4.0f;
        float durationA = 6.0f; // Seconds
        point startA = tmpP.makePoint(8.0f, 12.0f, 0.0f);
        point endA = tmpP.makePoint(-4.0f, -6.0f, 5.25f);
        float heightA = -2.0f;

        float speedB = 4.0f;
        float durationB = 3.0f; // Seconds
        point startB = tmpP.makePoint(-4.0f, -6.0f, 0.0f);
        point endB = tmpP.makePoint(-4.0f, -6.0f, 4.0f);
        float heightB = -4.0f;

        outputFile << "A's Initial Velocity: " << speedA << " m/s" << std::endl;
        outputFile << "B's Initial Velocity: " << speedB << " m/s" << std::endl;
        outputFile << std::endl;

        outputFile << "A's Initial Point Distance: (" << abs(endA.x - startA.x) << " m, " << abs(endA.y - startA.y) << " m, " << abs(endA.z - startA.z) << " m)" << std::endl;
        outputFile << "B's Initial Point Distance: (" << abs(endB.x - startB.x) << " m, " << abs(endB.y - startB.y) << " m, " << abs(endB.z - startB.z) << " m)" << std::endl;
        outputFile << std::endl;

        client = std::unique_ptr<MultirotorRpcLibClient>(new MultirotorRpcLibClient());
        client->confirmConnection();
        client->enableApiControl(true);

        GageDrone droneA("Drone A", startA, endA, heightA);
        GageDrone droneB("Drone B", startB, endB, heightB);

        droneA.spawnDrone();
        droneB.spawnDrone();

        std::vector<std::thread> threads;
        std::this_thread::sleep_for(std::chrono::duration<double>(1.0f));

        std::chrono::steady_clock::time_point startTime = std::chrono::high_resolution_clock::now();

        threads.push_back(std::thread(&GageDrone::droneMovement, &droneA, speedA, durationA));
        std::this_thread::sleep_for(std::chrono::duration<double>(0.1));

        threads.push_back(std::thread(&GageDrone::droneMovement, &droneB, speedB, durationB));
        std::this_thread::sleep_for(std::chrono::duration<double>(0.1));

        for (auto& toJoin : threads) {
            toJoin.join();
        }

        std::chrono::steady_clock::time_point endTime = std::chrono::high_resolution_clock::now();

        std::chrono::seconds difTime = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime);

        Vector3r currentPositionA = client->getMultirotorState("Drone A").getPosition();
        point finalA = tmpP.makePoint(abs(abs(currentPositionA.x()) - startA.x), abs(abs(currentPositionA.y()) - startA.y), abs(abs(currentPositionA.z()) - startA.z));
        outputFile << "A's Travel Displacement: (" << finalA.x << " m, " << finalA.y << " m, " << finalA.z << " m)" << std::endl;

        Vector3r currentPositionB = client->getMultirotorState("Drone B").getPosition();
        point finalB = tmpP.makePoint(abs(abs(currentPositionB.x()) - startB.x), abs(abs(currentPositionB.y()) - startB.y), abs(abs(currentPositionB.z()) - startB.z));
        outputFile << "B's Travel Displacement: (" << finalB.x << " m, " << finalB.y << " m, " << finalB.z << " m)" << std::endl;
        outputFile << std::endl;

        outputFile << "A's Avoidance Radius: PLACEHOLDER" << std::endl;
        outputFile << "B's Avoidance Radius: PLACEHOLDER" << std::endl;
        outputFile << std::endl;

        outputFile << "A's Closest Threat: PLACEHOLDER" << std::endl;
        outputFile << "B's Closest Threat: PLACEHOLDER" << std::endl;
        outputFile << std::endl;

        outputFile << "A's Lost Time: " << abs(durationA - (float)difTime.count()) << " s" << std::endl;
        outputFile << "B's Lost Time: " << abs(durationB - (float)difTime.count()) << " s" << std::endl;
        outputFile << std::endl;

        outputFile << "A's Response Time: PLACEHOLDER" << std::endl;
        outputFile << "B's Response Time: PLACEHOLDER" << std::endl;

        outputFile.close();
    }

    catch (const std::exception&) {
        return -1;
    }

    return 0;
}

int test2()
// Drone B is ahead of Drone A, Drone A is faster than Drone B, and they collide.
{
    using namespace msr::airlib;
    std::unique_ptr<MultirotorRpcLibClient> client;

    try {
        point tmpP;

        float speedA = 3.0f;
        float durationA = 12.0f;
        point startA = tmpP.makePoint(12.0f, 6.0f, 0.0f);
        point endA = tmpP.makePoint(0.0f, 6.0f, 2.0f);
        float heightA = -5.0f;

        float speedB = 1.0f;
        float durationB = 3.0f;
        point startB = tmpP.makePoint(6.0f, 6.0f, 0.0f);
        point endB = tmpP.makePoint(0.0f, 6.0f, 1.0f);
        float heightB = -4.0f;

        client = std::unique_ptr<MultirotorRpcLibClient>(new MultirotorRpcLibClient());
        client->confirmConnection();
        client->enableApiControl(true);

        GageDrone droneA("Drone A", startA, endA, heightA);
        GageDrone droneB("Drone B", startB, endB, heightB);

        droneA.spawnDrone();
        droneB.spawnDrone();

        std::vector<std::thread> threads;
        std::this_thread::sleep_for(std::chrono::duration<double>(1.0f));

        threads.push_back(std::thread(&GageDrone::droneMovement, &droneA, speedA, durationA));
        std::this_thread::sleep_for(std::chrono::duration<double>(0.1));

        threads.push_back(std::thread(&GageDrone::droneMovement, &droneB, speedB, durationB));
        std::this_thread::sleep_for(std::chrono::duration<double>(0.1));

        for (auto& toJoin : threads) {
            toJoin.join();
        }
    }

    catch (const std::exception&) {
        return -1;
    }

    return 0;
}

int test3()
// Drone A is moving north, Drone B is moving south, Drone A flies straight into Drone B, and they collide.
{
    using namespace msr::airlib;
    std::unique_ptr<MultirotorRpcLibClient> client;

    try {
        point tmpP;

        float speedA = 3.0f;
        float durationA = 6.0f;
        point startA = tmpP.makePoint(18.0f, 6.0f, 0.0f);
        point endA = tmpP.makePoint(0.0f, 6.0f, 5.5f);
        float heightA = -2.0f;

        float speedB = 3.0f;
        float durationB = 6.0f;
        point startB = tmpP.makePoint(3.0f, 6.0f, 0.0f);
        point endB = tmpP.makePoint(18.0f, 6.0f, 0.5f);
        float heightB = -4.0f;

        client = std::unique_ptr<MultirotorRpcLibClient>(new MultirotorRpcLibClient());
        client->confirmConnection();
        client->enableApiControl(true);

        GageDrone droneA("Drone A", startA, endA, heightA);
        GageDrone droneB("Drone B", startB, endB, heightB);

        droneA.spawnDrone();
        droneB.spawnDrone();

        std::vector<std::thread> threads;
        std::this_thread::sleep_for(std::chrono::duration<double>(1.0f));

        threads.push_back(std::thread(&GageDrone::droneMovement, &droneA, speedA, durationA));
        std::this_thread::sleep_for(std::chrono::duration<double>(0.1));

        threads.push_back(std::thread(&GageDrone::droneMovement, &droneB, speedB, durationB));
        std::this_thread::sleep_for(std::chrono::duration<double>(0.1));

        for (auto& toJoin : threads) {
            toJoin.join();
        }
    }

    catch (const std::exception&) {
        return -1;
    }

    return 0;
}

int test4()
// Drone A is moving north, Drone B is moving west, and they collide.
{
    using namespace msr::airlib;
    std::unique_ptr<MultirotorRpcLibClient> client;

    try {
        point tmpP;

        float speedA = 2.5f;
        float durationA = 6.0f;
        point startA = tmpP.makePoint(-4.0f, 0.0f, 0.0f);
        point endA = tmpP.makePoint(4.0f, 0.0f, 3.75f);
        float heightA = -2.0f;

        float speedB = 3.0f;
        float durationB = 6.0f;
        point startB = tmpP.makePoint(4.0f, 6.0f, 0.0f);
        point endB = tmpP.makePoint(4.0f, -6.0f, 1.75f);
        float heightB = -4.0f;

        client = std::unique_ptr<MultirotorRpcLibClient>(new MultirotorRpcLibClient());
        client->confirmConnection();
        client->enableApiControl(true);

        GageDrone droneA("Drone A", startA, endA, heightA);
        GageDrone droneB("Drone B", startB, endB, heightB);

        droneA.spawnDrone();
        droneB.spawnDrone();

        std::vector<std::thread> threads;
        std::this_thread::sleep_for(std::chrono::duration<double>(1.0f));

        threads.push_back(std::thread(&GageDrone::droneMovement, &droneA, speedA, durationA));
        std::this_thread::sleep_for(std::chrono::duration<double>(0.1));

        threads.push_back(std::thread(&GageDrone::droneMovement, &droneB, speedB, durationB));
        std::this_thread::sleep_for(std::chrono::duration<double>(0.1));

        for (auto& toJoin : threads) {
            toJoin.join();
        }
    }

    catch (const std::exception&) {
        return -1;
    }

    return 0;
}

int main()
{
    test1();
    // test2();
    // test3();
    // test4();

    return 0;
}