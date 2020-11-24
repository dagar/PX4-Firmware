



#include <uORB/topics/vehicle_control_mode.h>

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>


struct mode_requirements_t {
	bool manual_control{false};

	bool angular_velocity{false};
	bool attitude{false};
	bool altitude{false};
	bool velocity{false};
	bool local_position{false};
	bool global_position{false};

};

class FlightMode : public px4::ScheduledWorkItem
{
public:
	FlightMode(const char *name, const wq_config_t &config) : ScheduledWorkItem(name, config) {}

	virtual ~FlightMode() = default;

	virtual vehicle_control_mode_s control_mode() = 0;
	virtual const char            *name() = 0;
	virtual mode_requirements_t    requirements() = 0;

	virtual bool Start(const vehicle_control_mode_s& previous_control_mode) = 0;
	virtual bool Stop() = 0;
	virtual bool Update() = 0;

	// TODO:
	//  - requirements
	//  - failsafes?
	//  - Scheduling
	//  - initialization

protected:


private:



};
