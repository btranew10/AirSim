#pragma once
#ifndef ACTS_LIDAR_SCANNER_HPP
#define ACTS_LIDAR_SCANNER_HPP
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "Point.hpp"

class ACTSLidarScanner
{
    struct PointCloud
    {
        std::vector<point> pointCloud;

        PointCloud(std::vector<point> cl) //constructor.
        {
            this->pointCloud = cl;
        }
        PointCloud() {} //null constructor...just have this here for flexibility.
    };
#pragma region Attributes

private:
    PointCloud stdCloud; //user should generally never need to mess with this specifically. So, keep private.
    PointCloud dangerCl; //we actually wanna hide this from user because this may be accessed synchronously bec it will be updated in its own thread. So we need to protect it.
    //^ may want to employ some basic multi-threading stuff such as semaphores protecting these two clouds as well as utilizing the cpp wait function(??)

    bool isThreadRunning; //this will be adjusted inside startScanThread() and stopScanThread()...
    bool isInDanger;

public:
    //TODO: scan once, then wait for this long before scanning again.
    float scanPeriod;

#pragma endregion

public:
    ACTSLidarScanner(); //Null constructor....just have this here for flexibility.
    //TODO: Prob also want more params later as you flesh this class out.
    ACTSLidarScanner(float period);

    //TODO: make it so you can do ACTSLidarScanner als = new ACTSLidarScanner(); als.startScanThread(); at start of danny's code to simply begin the thread scanning.
    void startScanThread();

    void stopScanThread();

    //TODO: does a single scan. Need some thread protection where if scan is currently running, this either does nothing or just waits for its turn.
    //It will update dangerCl and stdCloud. It will return dangerCl.
    PointCloud scan();

    //TODO: similar to scan() in that it has to wait and be thread-safe but it does no updating of any cloud. It just reads dangerCl and returns its values. "const" means it's a read-only method.
    PointCloud checkRecScan() const;

    //TODO: basically a getter for isInDanger. NOTE: PROB NEED TO DO STUFF TO VERIFY THREAD SAFETY HERE TOO!
    bool inDanger() const
    {
        return this->isInDanger;
    }

private:
    //TODO: this will interrupt other threads in this method and within the CA algo. Needs to preempt what CA is working on to
    void criticalRadiusDetected();
};



#endif