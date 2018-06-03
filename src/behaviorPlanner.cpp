#include <math.h>
#include "behaviorPlanner.h"

void BehaviorPlanner::decideBehavior(double ego_s, int ego_lane, int prev_size,
             vector<vector<double>> sensor_fusion, otherCarPositions& others) {


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
      
      others.ahead |= other_s > ego_s &&  other_s - ego_s < 30;

    } else if( other_lane - ego_lane == -1 ) {

      others.left |= ego_s - 30 < other_s &&  ego_s + 30 > other_s;

    } else if( other_lane - ego_lane == 1 ) {

      others.right |= ego_s - 30 < other_s && ego_s + 30 > other_s;

    } 

  } /* for(sensor_fusion) */

} /* decideBehavior */


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
