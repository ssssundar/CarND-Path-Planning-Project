#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER

#include <vector>
#include <string>

using namespace std;

// If there is a car in any of the locations ahead, left and right,
// the corresponding value(s) will be set to true. Otherwise, teh value 
// is set to false.

struct otherCarPositions {
    bool ahead;
    bool left;
    bool right;
};
// The BehaviorPlanner class processes the Sensor Fusion data and
// analyzes the location of other vehicles in the ego vehicle's lane
// as well as in the other lanes.
class BehaviorPlanner
{
public:


  void decideBehavior( double ego_s, int ego_lane, int prev_size, vector<vector<double>>sensor_fusion, otherCarPositions& others );

private:
  int getLaneFromD( double d);

};
#endif /* BEHAVIOR_PLANNER_H */
