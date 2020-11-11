// This file is part of DVS-ROS - the RPG DVS ROS Package

#include "ynoise.hpp"
// #include "ynoise.h"



namespace ynoise {

YNoiseFilter::YNoiseFilter(ros::NodeHandle & nh, ros::NodeHandle nh_private) :
    nh_(nh)
{
  filtered_event_array_pub_ = nh.advertise<dvs_msgs::EventArray>("/filtered_events", 1);
  event_array_sub_ = nh.subscribe("/dvs/events", 1, &YNoiseFilter::runFilter, this);

  // load parameters
  //TODO: load parameters automatically
  sizeX = 346; 
  sizeY = 260;
  deltaT = 10000; //From 1 to 1000000
  lParam = 3; //From 3 to 15
  squareLParam = lParam * lParam;
	threshold = 2; //From 0 to 1000
	dividedLparam;
	modLparam;
  
  densityMatrix.resize(squareLParam);
	resetMatrix = densityMatrix;
	matrixMem.resize(sizeX * sizeY);

	regenerateDMLparam();
}



// generation of array of i/lParam and i%lParam for performance optimisation
void YNoiseFilter::regenerateDMLparam() {
	dividedLparam.resize(squareLParam);
	modLparam.resize(squareLParam);
	for (uint32_t i = 0; i < squareLParam; i++) {
		dividedLparam[i] = i / lParam;
		modLparam[i]     = i % lParam;
	}
}



void YNoiseFilter::updateMatrix(const dvs_msgs::Event &event) {
	auto address                 = event.x * sizeY + event.y;
	matrixMem[address].polarity  = event.polarity;
	matrixMem[address].timestamp = static_cast<int32_t>(event.ts.toNSec());
}



uint8_t YNoiseFilter::calculateDensity(const dvs_msgs::Event &event) {
	auto sub           = ((lParam - 1) / 2) - 1;
	auto addressX      = event.x - sub;
	auto addressY      = event.y - sub;
	auto timeToCompare = static_cast<uint32_t>(event.ts.toNSec() - deltaT);
	auto polarity      = event.polarity;
	uint8_t lInfNorm{0}; // event density performed with l infinity norm instead of l1 norm
	uint8_t sum{0};

	if (addressX >= 0 && addressY >= 0) {
		for (uint32_t i = 0; i < squareLParam; i++) {
			uint32_t newAddressY = addressY + modLparam[i];
			uint32_t newAddressX = addressX + dividedLparam[i];
			if (newAddressX < sizeX && newAddressY < sizeY) {
				auto &matrixElem = matrixMem[newAddressX * sizeY + newAddressY];
				if (polarity == matrixElem.polarity) {
					if (timeToCompare < matrixElem.timestamp) {
						if (modLparam[i] == 0) {
							lInfNorm = std::max(lInfNorm, sum);
							sum      = 0;
						}
						sum++;
					}
				}
			}
		}
	}

	return lInfNorm;
}

void YNoiseFilter::runFilter(const dvs_msgs::EventArray::ConstPtr& input_event_array)
{
  dvs_msgs::EventArray filtered_events;
  filtered_events.width = input_event_array->width;
  filtered_events.height = input_event_array->height;
  filtered_events.header = input_event_array->header;

  for (auto &evt : input_event_array->events)
  {
    if (calculateDensity(evt) >= threshold){
      filtered_events.events.push_back(evt);
    }
    updateMatrix(evt);
  }

  filtered_event_array_pub_.publish(filtered_events);

}



} // namespace
