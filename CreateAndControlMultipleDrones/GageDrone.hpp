
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"

STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>

//#pragma once //this serves same function as the #ifndef GAGE_DRONE_HPP stuff but isn't supported for all compilers so won't use it.
#ifndef GAGE_DRONE_HPP
#define GAGE_DRONE_HPP

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "Point.hpp"


class GageDrone;

/// <summary>
/// Class I made that mostly serves the purpose of 
/// streamlining the heck out of working with setting up 
/// drone scenes and making them move how you want.
/// 
/// <para>...</para>
/// Author: Malachi Sanderson
/// <para>...</para>
/// Made: 11-16-22
/// </summary>
class GageDrone
{
    #pragma region Attributes 
    
    #pragma region Public Attributes
public:
    std::string name;

    //REQUIRES: caller MUST have start.z = 0.
    point start;

    point end;

    float height;
    #pragma endregion

    #pragma region Private Attributes
private:
    point path;
    #pragma endregion

    #pragma endregion


    #pragma region Methods

    #pragma region Public Methods

public:
    /// <summary>
    /// Constructor for the GageDrone class. 
    /// Use this when you instantiate new drones.
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-16-22
    /// </summary>
    /// <param name="name"></param>
    /// <param name="start"></param>
    /// <param name="end"></param>
    GageDrone(std::string name, point start, point end, float height);

    GageDrone(){};

    /// <summary>
    /// Spawns specified drone.
    /// View implementation for details on spawning specifics.
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-16-22
    /// </summary>
    void spawnDrone();

    /// <summary>
    /// Returns the value of the private path attribute.
    /// You shouldn't be directly messing with this generally.
    /// This is calculated and handled in other internal functions
    /// of this class during setup.
    ///
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-16-22
    /// </summary>
    /// <returns></returns>
    point getPath() const
    {
        return path;
    }

    /// <summary>
    /// Given a drone of type GageDrone, you can call this to make it do its
    /// movement along its defined path, at a passed speed and for a
    /// passed duration.
    /// <para>
    /// Note: This can be mult-threaded!!! Yayyy!
    /// </para>
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-16-22
    /// <para></para>
    /// </summary>
    /// <param name="speed"></param>
    /// <param name="duration"></param>
    void droneMovement(float speed, float duration);

    #pragma endregion

    #pragma region Private Methods

private:
    /// <summary>
    /// When a drone is spawned at a location, the drone
    /// always counts that initially spawned location as (0,0,0).
    /// This can make it difficult to tell multiple drones to go to
    /// go to locations because each drone's perception of that location
    /// is relative to their own start position. This method makes it so
    /// that we can ignore this and just treat it as a global position without
    /// cheating and using simulation info.
    ///
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-16-22
    /// </summary>
    /// <returns></returns>
    void makeFinalPoint();

    #pragma endregion


    #pragma endregion


};


/// <summary>
/// TODO: I am planning something here. Ignore for now. Entirely nonessential.
/// </summary>
class SceneDrones
{

    

private:
    //std::map<GageDrone, std::pair<float, float>> sceneSetup;
    class DroneTestInfo
    {
    public:
        GageDrone drone;
        float speed;
        float duration;
        DroneTestInfo(GageDrone d, float s, float dur)
        {
            this->drone = d;
            this->speed = s;
            this->duration = dur;
        }
    };

    std::vector<DroneTestInfo> drones;

public:
    
    /// <summary>
    /// Lets you create a scene for as many drones as you want.
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-16-22
    /// </summary>
    /// <param name="drones"></param>
    /// <param name="speeds"></param>
    /// <param name="durations"></param>
    SceneDrones(std::vector<GageDrone> drones, std::vector<float> speeds, std::vector<float> durations)
    {
        for (GageDrone d : drones)
        {
            //basically, just pop from top of both lists.

            float speed = speeds.at(0);
            float duration = durations.at(0);
            speeds.erase(speeds.begin()); 
            durations.erase(durations.begin());
            DroneTestInfo di(d,speed,duration);
            this->drones.push_back(di);
            //this->sceneSetup.insert({ d, std::pair<float, float>(speed,duration) });
        }
    }

    /// <summary>
    /// Lets you easily create a scene for two diff drones.
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-16-22
    /// </summary>
    /// <param name="droneA"></param>
    /// <param name="speedA"></param>
    /// <param name="durationA"></param>
    /// <param name="droneB"></param>
    /// <param name="speedB"></param>
    /// <param name="durationB"></param>
    SceneDrones(GageDrone droneA, float speedA, float durationA, GageDrone droneB, float speedB, float durationB)
    {

        DroneTestInfo dA(droneA, speedA, durationA);
        DroneTestInfo dB(droneB, speedB, durationB);
        this->drones.push_back(dA);
        this->drones.push_back(dB);
        //std::pair<float,float> fA = { speedA, durationA };
        //std::pair<float, float> fB = { speedB, durationB };
        
        //this->sceneSetup.insert({ droneA, fA });
        //this->sceneSetup.insert({ droneB, fB });

        //sceneSetup.insert(std::pair<GageDrone, std::pair<float, float>>(droneA, fA));
    }
    
    
    /*
    /// <summary>
    /// Get a list of all drones in this instance of the scene drones class.
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-16-22
    /// </summary>
    /// <returns></returns>
    std::vector<GageDrone> getAllDrones() 
    {
        std::vector<GageDrone> drones;
        for (auto i = this->sceneSetup.begin(); i != this->sceneSetup.end(); i++)
            drones.push_back(i->first);
        return drones;
    }

    /// <summary>
    /// Get speed related to a given drone.
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-16-22
    /// </summary>
    /// <param name="d"></param>
    /// <returns></returns>
    float getSpeed(GageDrone d) 
    {
        return this->sceneSetup.at(d).first;
    }

    /// <summary>
    /// Get duration for a given drone.
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-16-22
    /// </summary>
    /// <param name="d"></param>
    /// <returns></returns>
    float getDuration(GageDrone d) 
    {
        return this->sceneSetup.at(d).second;
    }
    */

    /// <summary>
    /// Easy method to run a built scene.
    ///<para>
    /// REQUIRES: SCENE MUST BE FULLY CONSTRUCTED WITH ACCEPTABLE VALUES.
    /// </para>
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-16-22
    /// </summary>
    /// <returns></returns>
    int runDroneScene();
    
};




#endif
