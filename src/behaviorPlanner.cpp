#include <math.h>
#include "behaviorPlanner.h"


/******************************************************************************
Function: areCarsPresentInLanes()
          Analyzes sensor fusion data.
          If there are cars ahead, in the left lane and the right lane
          within an unsafe distance, sets teh corresponding flag
          ahead, left and right to "true".
******************************************************************************/

void BehaviorPlanner::areCarsPresentInLanes(double ego_s, int ego_lane, int prev_size,
             vector<vector<double>> sensor_fusion, otherCarPositions& other_cars) {


  for(int i = 0; i < sensor_fusion.size(); i++ ) {

    // Find the lane of the 'i'th car.
    float other_d = sensor_fusion[i][6];
    int other_lane = getLaneFromD( other_d );

     if( other_lane < 0 ) {
       continue;
     }

    // Find the speed of the 'i'th car.
    double other_vx = sensor_fusion[i][3];
    double other_vy = sensor_fusion[i][4];
    double other_speed = sqrt( other_vx*other_vx + other_vy*other_vy );

    // Estimate where this car would be after executing the previous path.
    double other_s = sensor_fusion[i][5];
    other_s += ((double)prev_size * 0.02 * other_speed); // what if prev_size == 0 ??? Handle it.

    
    // Compute in which lane the 'i'th car would be.
    // Set the flag for the lane if the car is less than 30 meters 
    // to the ego car.
    if( other_lane == ego_lane ) {
      
      other_cars.ahead |= other_s > ego_s &&  other_s - ego_s < SAFE_DISTANCE;

    } else if( other_lane - ego_lane == -1 ) {

      other_cars.left |= ego_s - SAFE_DISTANCE < other_s &&  ego_s + SAFE_DISTANCE > other_s;

    } else if( other_lane - ego_lane == 1 ) {

      other_cars.right |= ego_s - SAFE_DISTANCE < other_s && ego_s + SAFE_DISTANCE > other_s;

    } 

  } /* for(sensor_fusion) */

} /* areCarsPresentInLanes() */

/******************************************************************************
Function: determineLaneAndVelocity()
          Based on the ego car's lane and the presence of other cars within
          an unsafe distance either in teh ego car's lane or in other lanes,
          decides whether to stay in the same lane or change lanes.
          If in leftmost or rightmost lane, if safe, decides to move to the
          center lane. 
          Either reduces or increases the reference velocity based on the
          current speed and the presense of car ahead.
******************************************************************************/
void BehaviorPlanner::determineLaneAndVelocity( otherCarPositions& other_cars, int& lane, double& ref_velocity) {

  // Decide whether to change lanes and to which lane.
  // Adjust the reference velocity as needed.
  
  if( other_cars.ahead ) {
    if( !other_cars.left && lane > 0 ) {

      // There is a car ahead. There is a left lane and
      // there is no car in the left lane.
      // Move to the left lane.
      lane--;

    } else if( !other_cars.right && lane != 2 ) {

      // There is a car ahead. There is a right lane and
      // there is no car in in the right lane.
      // Move to the right lane.
      lane++;

    } else {

      // There is a car ahead. There are cars in both the
      // left lane and the right lane. No where to go.
      // Reduce the speed.
      ref_velocity -= ACCEL_STEP; // 5 m/s/s (meter per secondsquare)

    }

  } /* if( other_cars.ahead ) */
  else {

    // There is no car ahead.
    // If the ego car is not in the center lane,
    // move to the center lane. But make sure there is no car
    // in the center lane.
    if( lane != 1 ) {
      if( ( lane == 0 && !other_cars.right ) || (lane == 2 &&!other_cars.left ) ) {
        // Safe to move to the center lane.
        lane = 1;
      }
     }

     // Check and adjsut the speed.
     if( ref_velocity < MAX_SPEED ) {
       ref_velocity += ACCEL_STEP;
     }
  } /* else */

} /* determineLaneAndVelocity() */

/******************************************************************************
Function: getLaneFromD()
          Given a Fernet coordinate value d, determines the lane number.
          Each lane is 4 meters in width.
******************************************************************************/
int BehaviorPlanner::getLaneFromD( double d) {

  int car_lane = -1;

  if( d > 0 && d < 4 ) {
    car_lane = 0;
  } else if( d > 4 && d < 8 ) {
    car_lane = 1;
  } else if ( d > 8 && d < 12 ) {
    car_lane = 2;
  }                     
  return car_lane;
                   
} /* getLaneFromD */
