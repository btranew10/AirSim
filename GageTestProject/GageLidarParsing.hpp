
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
#include "Point.hpp"
#include "GageDrone.hpp"

STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>

#ifndef GAGE_LIDAR_PARSING_HPP
#define GAGE_LIDAR_PARSING_HPP

//hi

/// <summary>
/// Prints out all points in a point cloud 
/// (requires lidar data to have been parsed using 
/// <see cref="parseLidarData(msr::airlib::LidarData)"/>) 
/// <para>...</para>
/// Author: Malachi Sanderson
/// <para>...</para>
/// Made: 11-5-22
/// <para></para>
/// 
/// </summary>
/// <param name="std::vector{point}">parsed point cloud</param>
/// <returns>number of points in point cloud to be printed</returns>
int printLidarScan(std::vector<point>);

/// <summary>
/// Identical to  <see cref="printLidarScan(vector{point})"/> but
/// it also printes a passed time stamp along with the points.
/// <para>...</para>
/// Author: Malachi Sanderson
/// <para></para>
/// Made: 11-5-22
/// <para></para>
/// 
/// </summary>
/// <param name="std::vector{point}">parsed point cloud</param>
/// <param name="msr::airlib::TTimePoint"> timestamp</param>
/// <returns>number of points in point cloud to be printed</returns>
int printLidarScan(std::vector<point>, msr::airlib::TTimePoint);

/// <summary>
/// TODO: Description
/// 
/// <para>...</para>
/// Author: Malachi Sanderson
/// <para></para>
/// Made: 11-5-22
/// <para></para>
/// </summary>
/// <param name="state"></param>
/// <param name="pointCloud"></param>
/// <param name="criticalRange"></param>
/// <returns></returns>
std::vector<point> findPointsInCritRange(msr::airlib::MultirotorState state, std::vector<point> pointCloud, float criticalRange);


/// <summary>
/// Given a passed instance of LidarData, this parses all of the points of the
/// point cloud into a more organized and easily-readable data type; a
/// vector composed of points (having x,y and z values).
/// <para>...</para>
/// Author: Malachi Sanderson
/// <para>...</para>
/// Made: 11-5-22
/// <para></para>
///
/// </summary>
/// <param name="data"> - raw AirSim lidar data instance</param>
/// <returns>parsed point cloud</returns>
std::vector<point> parseLidarData(msr::airlib::LidarData data);


/// <summary>
/// TODO Description
/// TODO: actually make this work usefully.
/// NOTE: assumes the drone is already at the desired height.
/// <para>...</para>
/// Author: Malachi Sanderson
/// <para></para>
/// Made: 11-12-22
/// <para></para>
/// </summary>
/// <param name="client"></param>
/// <param name="speed"></param>
/// <param name="size"></param>
/// <returns></returns>
bool easyFlyInSquare(std::unique_ptr<msr::airlib::MultirotorRpcLibClient>& client, float speed, float size);

/// <summary>
/// TODO Description
/// TODO: actually make this work usefully.
/// NOTE: assumes the drone is already at the desired height.
/// <para>...</para>
/// Author: Malachi Sanderson
/// <para></para>
/// Made: 11-12-22
/// <para></para>
/// </summary>
/// <param name="client"></param>
/// <param name="droneName"> - name of specific drone you wish to fly in the square</param>
/// <param name="speed"></param>
/// <param name="size"></param>
/// <returns></returns>
bool easyFlyInSquare(std::unique_ptr<msr::airlib::MultirotorRpcLibClient>& client, std::string droneName, float speed, float size);


/// <summary>
/// static function that just 
/// returns a percentage of points from stdCloud
/// that are in dangerCloud.
/// <para>...</para>
/// Author: Malachi Sanderson
/// <para>...</para>
/// Made: 11-6-22
/// <para></para>
/// </summary>
/// <param name="stdCloudSize"> - cloud.size() </param>
/// <param name="dangerCloudSize"> - dangerCloud.size()</param>
/// <returns></returns>
static double getPercentSurrounded(std::size_t stdCloudSize, std::size_t dangerCloudSize)
{
    if (stdCloudSize == 0 || dangerCloudSize == 0)
        return 0.0;
    else
        return ((float)dangerCloudSize) / ((float)stdCloudSize);
}




/// <summary>
/// TODO: Description
/// 
/// <para>...</para>
/// Author: Malachi Sanderson
/// <para>...</para>
/// Made: 11-12-22
/// <para></para>
/// </summary>
/// <param name="prePrintMessage"></param>
/// <param name="loc"></param>
void printLocation(std::string prePrintMessage, point loc)
{
    std::cout << prePrintMessage << " " << loc.toString() << std::endl;
}




 /// <summary>
/// Checks lidar and searches lidar data for any points within critRadius and then returns the
/// critical point cloud.
/// <para>...</para>
/// Author: Malachi Sanderson
/// <para>...</para>
/// Made: 11-12-22
/// <para></para>
/// </summary>
///
/// <param name="client"> -
/// client unique pointer. Requires client to be initialized as a
/// unique pointer, NOT JUST AN OBJECT OF TYPE MultirotorRpcLibClient
/// </param>
///
/// <param name="critRadius"> -
/// float defining the radius of which is considered TOO close. Any points from lidar scan that are this
/// close or closer will be added to the critical point cloud.
/// </param>
///
/// <param name="printCriticalCloud"> -
/// bool telling the method whether or not you wish to see a print of all points in the point cloud.
/// Generally suggested to keep this as false unless for testing.
/// </param>
///
/// <param name="printPercent"> -
/// bool telling the method whether or not you wish to see a print of the percentage of points from the
/// standard lidar scan that are within the critical range. This should always be equal to or less than 1.
/// </param>
///
/// <returns>
/// returns a point cloud of all points that are within the critical radius passed.
/// </returns>
std::vector<point> lidarScan(std::unique_ptr<msr::airlib::MultirotorRpcLibClient>& client, float critRadius, bool printCriticalCloud, bool printPercent);


/// <summary>
/// TODO : PUT ALL STUFF FROM THE main.cpp REGION, "GAGE'S LIDAR PARSING STUFF", 
/// IN HERE AND EVENTUALLY MAKE A METHOD THAT WILL BE ABLE TO HANDLE 
/// MULTI-THREADED, CONSTANT LIDAR SCANS!!!!
/// </summary>
class LidarScanner
{
   
};

#endif
