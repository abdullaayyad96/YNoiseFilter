#include <ros/ros.h>
#include <string>

// boost
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>

// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

// camera info manager
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <ynoise_filter/ynoiseCfgConfig.h>

#include <vector>

namespace ynoise {

  struct matrixS {
	uint32_t timestamp; // keep only the lower part for performance
	bool polarity;
	matrixS() : timestamp{0}, polarity{false} {
    }
  };

  using densityMatrixType = bool;
  using densityMatrixT    = std::vector<densityMatrixType>;
  using matrixBufferT     = std::vector<matrixS>;
  using dividedT          = std::vector<uint32_t>;


class YNoiseFilter {
public:
  	static const char *initDescription();

	YNoiseFilter(ros::NodeHandle & nh, ros::NodeHandle nh_private);

	void runFilter(const dvs_msgs::EventArray::ConstPtr& input_event_array);

	void parameter_callback(ynoise_filter::ynoiseCfgConfig &config, uint32_t level);  

private:
	ros::NodeHandle & nh_;
  	densityMatrixT densityMatrix;
	densityMatrixT resetMatrix;
	matrixBufferT matrixMem;
	uint32_t deltaT;
	uint8_t lParam;
	uint32_t squareLParam;
	uint8_t threshold;
	uint32_t sizeX;
	uint32_t sizeY;
	dividedT dividedLparam;
	dividedT modLparam;

  	ros::Publisher filtered_event_array_pub_;
  	ros::Subscriber event_array_sub_;

	//dynamic config
	dynamic_reconfigure::Server<ynoise_filter::ynoiseCfgConfig> server_;
	dynamic_reconfigure::Server<ynoise_filter::ynoiseCfgConfig>::CallbackType f_;

	void updateMatrix(const dvs_msgs::Event &event);

	uint8_t calculateDensity(const dvs_msgs::Event &event);

	// generation of array of i/lParam and i%lParam for performance optimisation
	void regenerateDMLparam();
};

} // namespace
