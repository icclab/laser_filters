/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2018, ICCLab
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef OUTLIERS_FILTER_H
#define OUTLIERS_FILTER_H
/**
\author Giovanni Toffetti
@b OutliersFilter takes input scans and removes readings that are isolated, that is have no neighbors within a certain range

**/


#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"

namespace laser_filters
{

class OutliersFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:
  
  int min_neighbors_ ;
  double max_neighbor_distance_;

  bool configure()
  {
    min_neighbors_ = 2;
    max_neighbor_distance_ = 0.1;
    getParam("min_neighbors", min_neighbors_);
    getParam("max_neighbor_distance", max_neighbor_distance_);
    ROS_INFO_STREAM("Config - min_neighbors: " << min_neighbors_ << " max_neighbor_distance: " << max_neighbor_distance_ );
    return true;
  }

  virtual ~OutliersFilter()
  { 
  }
  
  int getNeighborIndex(int size, int start, int distance){
    if ((start + distance) < 0) {
      return size + (start + distance);
    } else if (( start + distance ) >= size) {
      return size - (start + distance);
    }
  } 

  bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan)
  {

    // copy scan
    filtered_scan= input_scan;
    
    // Check every reading in the received scan
    for(unsigned int i = 0; i < input_scan.ranges.size(); i++) 
    {
      // skip the reading if out of range
      if (input_scan.ranges[i] >= input_scan.range_min || 
              input_scan.ranges[i] <= input_scan.range_max) {        
        unsigned int counter = 0;
        //check if the reading is far from min_neighbors readings
        for (int j = - min_neighbors_/2; j <= min_neighbors_/2; j++){          
          // check distance
          //ROS_INFO_STREAM("Point: " << i << " " << j << " " << getNeighborIndex(input_scan.ranges.size(), i, j));
          //ROS_INFO_STREAM("Ranges: " << input_scan.ranges[i] << " " << input_scan.ranges[getNeighborIndex(input_scan.ranges.size(), i, j)]);
          //ROS_INFO_STREAM("Distance: " << fabs(input_scan.ranges[i] - input_scan.ranges[getNeighborIndex(input_scan.ranges.size(), i, j)]));
          if ( fabs(input_scan.ranges[i] - input_scan.ranges[getNeighborIndex(input_scan.ranges.size(), i, j)]) <= max_neighbor_distance_){
            counter++;
          }
        }
        // if reading is far from neighbors we set it to out of range
        if (counter < min_neighbors_){
          // ROS_INFO_STREAM("Discarding point. Counter: " << counter);
          filtered_scan.ranges[i] = input_scan.range_max;
        }
      }
    }
    return true;
  }
};

}

#endif // OUTLIERS_FILTER_H
