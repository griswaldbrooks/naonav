// GODZILA library v0.1
//
// GODZILA is a potential fields style algorithm with a straight line planner
// and trap escape scheme.
// It is a lightweight memoryless algorithm used to avoid obstacles
// and bring the robot to some goal location.
// It is designed to be used with some other obstacle or occlusion
// detection method, such as a Lidar, though any system that provides
// a list of obstacles should be applicable.
//
// Author: Griswald Brooks
// Email: griswald.brooks@gmail.com
//

#ifndef _GODZILA_HPP
#define _GODZILA_HPP

// Standard includes.
#include <vector>
#include <string>
#include <ostream>

// Robotlib includes.
#include <rlib_sensor.hpp>
#include <rlib_state.hpp>

// Special includes.


#define ROBOT_RADIUS_IN_METERS 		0.3

/*! \brief GODZILA namespace. */
namespace godzila{

	/*! \brief Class for setting intermediate goals when straight line path to goal is available. */
	class StraightLinePlanner{
	public:
		StraightLinePlanner();
		bool isGoalInSight();
		void getGoal(float goalRange, float goalBearing);
	private:
	};

	/*! \brief Class for escaping from local minima. */
	class EscapeStrategy{
	public:
		EscapeStrategy();
		bool isStuck();
		void getGoal(float goalRange, float goalBearing);
	private:
	};

	/*! \brief Class for producing navigation commands using the GODZILA algorithm. */
	class Planner{
	public:
		/*!
		* \brief Default constructor.
		*/
		Planner();

		/*! 
		* \brief 	Method for setting the minimum and maximum linear and angular velocity of the vehicle.
		*		 	Defaults for these limits are zero.
		* \param[in] vmin 			Minimum linear velocity of the vehicle in meters/second.
		* \param[in] vmax 			Maximum linear velocity of the vehicle in meters/second.
		* \param[in] wmin 			Minimum angular velocity of the vehicle in radians/second.
		* \param[in] wmax 			Maximum angular velocity of the vehicle in radians/second.
		*/
		void setSpeedLimits(float vmin, float vmax, float wmin, float wmax);

		/*!
		* \brief 	Method for getting the velocity limits imposed on the planner.
		* \param[out] limits	Velocity limits in meters/second and radians/second.
		*						Format is [vmin, vmax, wmin, wmax].
		*/
		void getSpeedLimits(std::vector<float> & limits) const;

		/*! 
		* \brief 	Method for setting the minimum acceptable distance between the vehicle and an obstacle.
		*			Setting this distance does not guarantee that the robot will never violate this threshold.
		* \param[in] clearanceThreshold		Distance from the center of the robot to the center of an obstacle in meters. 
		*										This distance is center to center because all objects are modeled as points.
		*/
		void setClearanceDistance(float clearanceThreshold);

		/*! 
		* \brief 	Method for getting the minimum acceptable distance between the vehicle and an obstacle.
		* \return		Distance from the center of the robot to the center of an obstacle in meters. 
		*/
		float getClearanceDistance() const;

		/*! 
		* \brief 	Method for setting the obstacle range at which it is acceptable to treat obstacles
		* 			as attractive rather than repulsive.
		* \param[in] obstacleThreshold 		Obstacle range threshold measured in meters.
		*/
		void setObstacleRangeThreshold(float obstacleThreshold);

		/*! 
		* \brief 	Method for getting the obstacle range at which it is acceptable to treat obstacles
		* 			as attractive rather than repulsive.
		* \return 		Obstacle range threshold measured in meters.
		*/
		float getObstacleRangeThreshold() const;

		/*!
		* \brief 	Method for adding an obstacle set to the planner. This version uses only the current obstacle set.
		* \param[in] obstacles 		The set of obstacles represented as ranges and relative bearings.
		*/
		void addObstacles(rlib::SensorData* obstacles);

		/*!
		* \brief 	Method for setting the location of the goal with respect to the robot.
		* \param[in] range 			Distance to the goal measured in meters.
		* \param[in] angle			Relative bearing to the goal measured in radians.
		*/
		void updateGoal(float range, float angle);

		/*!
		* \brief 	Method for getting the location of the goal with respect to the robot.
		* \param[out] goal 	Range and bearing of the goal relative to the robot in meters.
		*					Format is [range, bearing].
		*/
		void returnGoal(std::vector<float> & goal) const;

		/*! 
		* \brief 	Method for tuning the planning parameters for angular velocity.
		* 			Default value for all parameters is one.
		* \param[in] goalAttraction				Tunes the strength of goal attraction.
		*											Larger values increase attraction strength.
		* \param[in] obstacleRepulsionTurning		Tunes the strength of obstacle repulsion for 
		*											obstacles closer than the obstacle range threshold.
		*											Larger values increase repulsion strength.
		* \param[in] obstacleAttraction 		Tunes the strength of obstacle attraction for 
		*											obstacles farther than the obstacle range threshold.
		*											Larger values increase attraction strength.
		* \param[in] obstacleGoalBearingRatio	Tunes the trade off between avoiding obstacles 
		*											which are in the vehicle's current direction 
		*											of travel versus avoiding objects which are 
		*											in the direction of the goal.
		*											This parameters ranges from 1 to 0. 
		*											Values closer to 1 amplify the avoidance of 
		*											obstacles in the direction of travel.
		*											Values closer to 0 amplify the avoidance of 
		*											obstacles in the direction of the goal.
		* \param[in] vehicleInertia				Tunes the strength of the vehicle's resistance 
		*											to turning. Larger values mean more resistance.
		*/
		void tuneAngular(	float goalAttraction, 
							float obstacleRepulsionTurning, 
							float obstacleAttraction, 
							float obstacleGoalBearingRatio, 
							float vehicleInertia);

		/*!
		* \brief 	Method for getting the values of the tuning parameters responsible for 
		*			steering the robot.
		* \param[out] params	Direction tuning parameters. Format is 
		*						[goalAttraction, obstacleRepulsionTurning, obstacleAttraction, obstacleBearingRatio, vehicleInertia].
		*/
		void getTurningParameters(std::vector<float> & params) const;

		/*!
		* \brief 	Method for tuning the planning parameters for linear velocity.
		*			Default value for all parameters in one.
		* \param[in] velocityGain 			Tunes the aggressiveness of the linear velocity. 
		*										Larger number produces more aggressiveness. 
		* \param[in] obstacleRepulsionForward		Tunes the strength of obstacle repulsion.
		*										Larger values increase repulsion strength.
		* \param[in] angularRateBraking 	Tunes the amount by which high turning rates 
		*										reduce linear velocity.
		* 										Larger values reduce linear velocity.
		*/
		void tuneLinear(float velocityGain, 
						float obstacleRepulsionForward, 
						float angularRateBraking);

		/*!
		* \brief 	Method for getting the values of the tuning parameters responsible for
		*			the forward velocity of the robot.
		* \param[out] params 	Forward velocity tuning parameters. Format is
		*						[velocityGain, obstacleRepulsionForward, angularRateBraking].
		*/
		void getForwardVelocityParameters(std::vector<float> & params) const;

		/*! 
		* \brief 	Method for generating velocity commands.
		* \param[in] currentTime 	Current time in seconds. Used for straight line planning
		*							and trap escaping. If zero, those features are disabled.
		*/
		void plan(float currentTime);

		/*! 
		* \brief 	Method for getting a velocity command from the planner. 
		* \return 		Velocity command for the vehicle.
		*/
		const rlib::Vel & getCommand() const;

		/*!
		* \brief Print operator for planner parameters.
		*/
		friend std::ostream& operator<<(std::ostream& os, Planner& p);


	private:
		/*!
		* \brief Method to compute the vector that points to the goal. 
		* \param[in] goalRange 		Distance to the goal in meters.
		* \param[in] goalBearing 	Relative bearing to the goal in radians.
		* \param[out] goalVector 	Vector that points to the goal.
		*/
		void computeGoalVector(float goalRange, float goalBearing, Eigen::Matrix<float, 2, 1> & goalVector);

		/*!
		* \brief Method to compute the vector that repels the robot from obstacles.
		* \param[in] goalRange 		Distance to the goal in meters.
		* \param[in] goalBearing 	Relative bearing to the goal in radians.
		* \param[out] repelVector 	Vector that repels the robot from obstacles.
		*/
		void computeRepulsionVector(float goalRange, float goalBearing, Eigen::Matrix<float, 2, 1> & repelVector);

		/*!
		* \brief Method to compute the vector that attracts the robot towards obstacles.
		* \param[in] goalRange 		Distance to the goal in meters.
		* \param[in] goalBearing 	Relative bearing to the goal in radians.
		* \param[out] attractVector 	Vector that attracts the robot to obstacles.
		*/
		void computeAttractionVector(float goalRange, float goalBearing, Eigen::Matrix<float, 2, 1> & attractVector);

		/*! 
		* \brief Method to compute the vehicle's resistance to turning.
		* \param[out] inertiaVector 	Vector that works to maintain the vehicle's current heading.
		*/
		void computeInertiaVector(Eigen::Matrix<float, 2, 1> & inertiaVector);

		/*!
		* \brief Method to compute the angular velocity command.
		* \param[in] steeringVector 	The desired heading.
		* \return 						The angular velocity command in radians/second.
		*/
		float computeAngularVelocity(Eigen::Matrix<float, 2, 1> & steeringVector);

		/*!
		* \brief Method to compute the linear velocity command.
		* \param[in] w 		The desired angular velocity.
		* \return 			The linear velocity command in meters/second.
		*/
		float computeLinearVelcoity(float w);

		float _currentTime; /**< Current time in seconds. */

		float _vmin; /**< Minimum linear velocity of vehicle in meters/second.*/
		float _vmax; /**< Maximum linear velocity of vehicle in meters/second.*/
		float _wmin; /**< Minimum angular velocity of vehicle in radians/second.*/
		float _wmax; /**< Maximum angular velocity of vehicle in radians/second.*/

		float _clearanceThreshold; 	/**< Minimum acceptable distance between the center of the 
											vehicle and the center of an obstacle in meters.*/
		float _obstacleThreshold; 	/**< Distance at which obstacles switch from being repulsive 
											to being attractive.*/

		float _goalRange; /**< 	Distance from the robot to the goal in meters.*/
		float _goalAngle; /**< 	Relative bearing of the goal from the robot in radians.*/

		float _goalAttraction; /**< 				Strength of goal attractiveness.*/
		float _obstacleRepulsionTurning; /**< 		Strength of obstacle repulsiveness.*/
		float _obstacleAttraction; /**< 			Strength of obstacle attraction.*/
		float _obstacleGoalBearingRatio; /**< 	Trade off between avoiding 
													obstacles in the direction of travel 
													and avoiding 
													obstacles in the direction of the goal.
													Ranges from 0 to 1. 
													1 prefers avoiding obstacles in the direction of travel.
													0 prefers avoiding obstacles in the direction of the goal.*/
		float _vehicleInertia; /**< 				Resistance to change in angle.*/

		float _velocityGain; /**< 				Aggressiveness of linear velocity.*/
		float _obstacleRepulsionForward; /**<	Strength of obstacle repulsiveness for computing forward velocity.*/
		float _angularRateBraking; /**< 			Linear velocity braking coefficient in response to high angular velocity.*/

		Eigen::Matrix<float, 2, Eigen::Dynamic> _obstacles; /**< 	Obstacle data in pair format. 
																	First element in pair corresponds to range in
																	meters. Second element in pair in the angle of 
																	that range in radians. */

		rlib::Vel _command; /**< 	Current vehicle velocity command. */

		StraightLinePlanner * _slp; /**< Straight Line Planner object for setting intermediate goals. */
		EscapeStrategy * _es; 		/**< Escape Strategy for escaping local minima. */
	};
	
}

#endif