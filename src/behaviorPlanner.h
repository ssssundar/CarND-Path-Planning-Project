#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER

#include <vector>
#include <string>

using namespace std;


/******************************************************************************
Constant definitions.
         Values used by the BehaviorPlanner computations. Keeping these values 
         as constants makes it easier to tune these values.
******************************************************************************/
const double MAX_SPEED = 49.5;
const double ACCEL_STEP = 0.224;
const double SAFE_DISTANCE = 30.0;


/******************************************************************************
Structure: otherCarPositions
           Used in the interface between main.cpp and BehaviorPlanner.
           If there is a car ahead, in the left lane and in the right lane, 
           within an unsafe distance, the corresponding value in this structure
           will be set to "true". 
******************************************************************************/
struct otherCarPositions {
    bool ahead;
    bool left;
    bool right;
};


/******************************************************************************
Class: BehaviorPlanner
       The BehaviorPlanner class is Algorithmic in nature.
       It doesn't have any data members and doesn't remember anything.
       The BehaviorPlanner class processes the Sensor Fusion data and
       analyzes the location of other vehicles in the ego vehicle's lane
       as well as in the other lanes.
       It is also responsible for making the lane change decisions and
       determing tehe reference velocity.
******************************************************************************/
class BehaviorPlanner
{
public:


  void areCarsPresentInLanes( double ego_s, int ego_lane, int prev_size, vector<vector<double>>sensor_fusion, otherCarPositions& others );

  void determineLaneAndVelocity( otherCarPositions& other_cars, int& lane, double& ref_velocity );

private:
  int getLaneFromD( double d);

};
#endif /* BEHAVIOR_PLANNER_H */
