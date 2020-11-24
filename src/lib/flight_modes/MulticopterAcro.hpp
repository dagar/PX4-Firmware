

#include "FlightMode.hpp"

#include <px4_platform_common/module_params.h>
#include <lib/matrix/matrix/math.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_rates_setpoint.h>

class MulticopterAcro : public FlightMode, public ModuleParams
{
public:

	static constexpr mode_requirements_t REQUIREMENTS {
		.manual_control = true;
		.angular_velocity = true;
	};


	MulticopterAcro() :
		ModuleParams(nullptr),
		FlightMode("flight_mode_acro", px4::wq_configurations::hp_default)
	{

	}

	~MulticopterAcro() override = default;


	vehicle_control_mode_s control_mode() override
	{
		return {
			.flag_control_manual_enabled = true;
			.flag_control_rates_enabled = true;
		};
	};

	const char *name() const override { return "acro"; }

	bool Start(const vehicle_control_mode_s& previous_control_mode) override
	{
		return _manual_control_setpoint_sub.registerCallback();
	}

	bool Stop() override
	{
		return _manual_control_setpoint_sub.unregisterCallback();
	}

	bool Update() override
	{
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s param_update;
			_parameter_update_sub.copy(&param_update);

			updateParams();
			UpdateParameters();
		}

		manual_control_setpoint_s manual_control_setpoint;

		if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
			// manual rates control - ACRO mode
			const Vector3f man_rate_sp{
				math::superexpo(manual_control_setpoint.y, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
				math::superexpo(-manual_control_setpoint.x, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
				math::superexpo(manual_control_setpoint.r, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

			const matrix::Vector3f rates_sp = man_rate_sp.emult(_acro_rate_max);

			// publish rate setpoint
			vehicle_rates_setpoint_s v_rates_sp{};
			v_rates_sp.roll = rates_sp(0);
			v_rates_sp.pitch = rates_sp(1);
			v_rates_sp.yaw = rates_sp(2);
			v_rates_sp.thrust_body[2] = -manual_control_setpoint.z;
			v_rates_sp.timestamp = hrt_absolute_time();
			_v_rates_sp_pub.publish(v_rates_sp);

			return true;
		}

		return false;
	}

	void UpdateParameters()
	{
		// manual rate control acro mode rate limits
		_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
					  radians(_param_mc_acro_y_max.get()));

	}

private:
	uORB::Publication<vehicle_rates_setpoint_s> _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};

	uORB::SubscriptionCallbackWorkItem _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	matrix::Vector3f _acro_rate_max{}; /**< max attitude rates in acro mode */

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ACRO_R_MAX>) _param_mc_acro_r_max,
		(ParamFloat<px4::params::MC_ACRO_P_MAX>) _param_mc_acro_p_max,
		(ParamFloat<px4::params::MC_ACRO_Y_MAX>) _param_mc_acro_y_max,
		(ParamFloat<px4::params::MC_ACRO_EXPO>) _param_mc_acro_expo,			/**< expo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_EXPO_Y>) _param_mc_acro_expo_y,		/**< expo stick curve shape (yaw) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPO>) _param_mc_acro_supexpo,		/**< superexpo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPOY>) _param_mc_acro_supexpoy,		/**< superexpo stick curve shape (yaw) */

		(ParamFloat<px4::params::MC_RATT_TH>) _param_mc_ratt_th
	)

};
