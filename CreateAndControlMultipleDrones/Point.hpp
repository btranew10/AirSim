
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

struct point;
static double calculateDistance(point a, point b);
static void printLocation(point loc);

/// <summary>
/// TODO: Description...
/// TODO: make this a class instead of a struct. Be nice to have a constructor for points 
/// instead of how it currently is.
/// <para>Methods...</para>
/// <para> point::getCurrPos(client) </para>
/// <para>point::getCurrPos(state) </para>
/// <para> point::toString() </para>
/// <para> point::toVector3r() </para>
/// <para> point::makePoint(float nX, float nY, float nZ) </para>
/// <para> point::copy(point) </para>
/// <para> point::copy(Vector3r) </para>
/// <para> point::distanceTo(point) </para>
/// <para> point::vectorDistanceTo(point) </para>
/// <para> point::vectorAbsDistanceTo(point otherP) </para>
/// <para> static void printLocation(point loc) </para>
/// <para> static double calculateDistance(point a, point b) </para>
/// <para>...</para>
/// Author: Malachi Sanderson
/// <para>...</para>
/// Made: 11-12-22
/// </summary>
struct point
{
    float x;
    float y;
    float z;

    /// <summary>
    /// Just gets position from client and sets
    /// x,y,z values for point so I don't have to use their built in vectors.
    /// NOTE: This uses standard getMultirotorState().
    /// Call getCurrentPosition with MultirotorState param
    /// if you want to specify vehicle via its name.
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-12-22
    /// <para></para>
    /// </summary>
    /// <param name="client"></param>
    void getCurrPos(std::unique_ptr<msr::airlib::MultirotorRpcLibClient>& client)
    {
        this->getCurrPos(client->getMultirotorState());
    }

    /// <summary>
    /// Just gets position from client and sets
    /// x,y,z values for point so I don't have to use their built i vectors.
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-12-22
    /// </summary>
    /// <param name="state"></param>
    void getCurrPos(msr::airlib::MultirotorState state)
    {
        msr::airlib::Vector3r v = state.getPosition();
        this->copy(v);
    }

    /// <summary>
    /// Returns a string representation of the x, y and z values of a point.
    /// Formatted with brackets enclosing the string and commas separating each value.
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-12-22
    /// <para></para>
    /// </summary>
    /// <returns></returns>
    std::string toString()
    {
        return "[" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "]";
    }

    /// <summary>
    /// Simply returns a point represented as an airsim vector3r.
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-12-22
    /// <para></para>
    /// </summary>
    /// <returns></returns>
    msr::airlib::Vector3r toVector3r()
    {
        return msr::airlib::Vector3r(this->x, this->y, this->z);
    }

    /// <summary>
    /// Basically a constructor for points. Note, this overwrites
    /// whatever is currently held by the point you call it on.
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-12-22
    /// <para></para>
    /// </summary>
    /// <param name="x"></param>
    /// <param name="y"></param>
    /// <param name="z"></param>
    /// <returns>this point with its new values</returns>
    point makePoint(float nX, float nY, float nZ)
    {
        this->x = nX;
        this->y = nY;
        this->z = nZ;
        return *this;
    }

    /// <summary>
    /// When you wish to copy the values from one point, to another, call this.
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-12-22
    /// <para></para>
    /// </summary>
    /// <param name="otherP"></param>
    /// <returns></returns>
    point copy(point otherP)
    {
        this->x = otherP.x;
        this->y = otherP.y;
        this->z = otherP.z;
        return *this;
    }
    point copy(msr::airlib::Vector3r v)
    {
        this->x = v.x();
        this->y = v.y();
        this->z = v.z();
        return *this;
    }

    /// <summary>
    /// calculates the magnitude of the distance to from this point 
    /// to another point.
    ///
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-12-22
    /// <para></para>
    /// </summary>
    /// <param name="otherP"></param>
    /// <returns></returns>
    const double distanceTo(point otherP)
    {
        return calculateDistance(*this,otherP);
    }

    /// <summary>
    /// Calculates thethe difference between each
    /// component of the vectors and returns the resulting vector.
    /// NOTE: the general form of subtraction done here is: x = this.x - other.x.
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-12-22
    /// <para></para>
    /// </summary>
    /// <param name="otherP"></param>
    /// <returns></returns>
    const point vectorDistanceTo(point otherP)
    {
        point p;
        p.copy(*this);
        p.x = p.x - otherP.x;
        p.y = p.y - otherP.y;
        p.z = p.z - otherP.z;
        return p;
    }

    /// <summary>
    /// Calculates the absolute value of the difference between each
    /// component of the vectors and returns the resulting vector.
    ///
    /// <para>...</para>
    /// Author: Malachi Sanderson
    /// <para>...</para>
    /// Made: 11-12-22
    /// <para></para>
    /// </summary>
    /// <param name="otherP"></param>
    /// <returns></returns>
    point vectorAbsDistanceTo(point otherP)
    {
        point p;
        p.copy(*this);
        p.x = (float)sqrt(pow((p.x - otherP.x), 2));
        p.y = (float)sqrt(pow((p.y - otherP.y), 2));
        p.z = (float)sqrt(pow((p.z - otherP.z), 2));
        return p;
    }
};

/// <summary>
/// Prints a point's value's out according to Point.toString().
/// <para>...</para>
/// Author: Malachi Sanderson
/// <para>...</para>
/// Made: 11-12-22
/// <para></para>
/// </summary>
/// <param name="loc"></param>
static void printLocation(point loc)
{
    std::cout << loc.toString() << std::endl;
}


/// <summary>
/// calculates magnitude of distance between 2 points.
///
/// <para>...</para>
/// Author: Malachi Sanderson
/// <para></para>
/// Made: 11-5-22
/// <para></para>
///
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
static double calculateDistance(point a, point b)
{
    return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2) + pow((a.z - b.z), 2));
}

#pragma once
