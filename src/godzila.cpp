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

// Include header.
#include <godzila.hpp>

// Standard includes.
#include <math.h>

// Robotlib includes.
#include <rlib_sensor.hpp>
#include <rlib_state.hpp>

// Special includes.


/*! \brief GODZILA namespace. */
namespace godzila{

	StraightLinePlanner::StraightLinePlanner(){}
	bool StraightLinePlanner::isGoalInSight(){
		return false;
	}
	void StraightLinePlanner::getGoal(float goalRange, float goalBearing){}

	EscapeStrategy::EscapeStrategy(){}
	bool EscapeStrategy::isStuck(){
		return false;
	}
	void EscapeStrategy::getGoal(float goalRange, float goalBearing){}


	/*!
	* \brief Default constructor.
	*/
	Planner::Planner(){
		// Initialize the parameters.

		_currentTime = 0;

		// Velocity limits.
		_vmin = 0;
		_vmax = 0;
		_wmin = 0;
		_wmax = 0;

		// Safety distance from robot.
		_clearanceThreshold = 0;

		// Distance in meters when obstacles become attractive.
		_obstacleThreshold = 1e3;

		// Initial goal (origin).
		_goalRange = 0; 
		_goalAngle = 0; 

		// Steering parameters.
		_goalAttraction = 1;
		_obstacleRepulsionTurning = 1; 
		_obstacleAttraction = 1;
		_obstacleGoalBearingRatio = 1;

		// Resistance to turning.
		_vehicleInertia = 1;

		// Forward velocity parameters.
		_velocityGain = 1; 
		_obstacleRepulsionForward = 1; 
		_angularRateBraking = 1; 

		// Set initial velocity command.
		_command.vx = 0;
		_command.vy = 0;
		_command.vz = 0;
		_command.wx = 0;
		_command.wy = 0;
		_command.wz = 0;

		// Set additional planner modules.
		_slp = new StraightLinePlanner();
		_es = new EscapeStrategy();
	}

	/*! 
	* \brief 	Method for setting the minimum and maximum linear and angular velocity of the vehicle.
	*		 	Defaults for these limits are zero.
	* \param[in] vmin 			Minimum linear velocity of the vehicle in meters/second.
	* \param[in] vmax 			Maximum linear velocity of the vehicle in meters/second.
	* \param[in] wmin 			Minimum angular velocity of the vehicle in radians/second.
	* \param[in] wmax 			Maximum angular velocity of the vehicle in radians/second.
	*/
	void Planner::setSpeedLimits(float vmin, float vmax, float wmin, float wmax){
		// Velocity limits.
		_vmin = -fabs(vmin);
		_vmax =  fabs(vmax);
		_wmin = -fabs(wmin);
		_wmax =  fabs(wmax);
	}

	/*!
	* \brief 	Method for getting the velocity limits imposed on the planner.
	* \param[out] limits 	Velocity limits in meters/second and radians/second.
	*						Format is [vmin, vmax, wmin, wmax].
	*/
	void Planner::getSpeedLimits(std::vector<float> & limits)const{
		// Add velocity limits to vector.
		limits.push_back(_vmin);
		limits.push_back(_vmax);
		limits.push_back(_wmin);
		limits.push_back(_wmax);
	}

	/*! 
	* \brief 	Method for setting the minimum acceptable distance between the vehicle and an obstacle.
	*			Setting this distance does not guarantee that the robot will never violate this threshold.
	* \param[in] clearanceThreshold		Distance from the center of the robot to the center of an obstacle in meters. 
	*										This distance is center to center because all objects are modeled as points.
	*/
	void Planner::setClearanceDistance(float clearanceThreshold){
		_clearanceThreshold = clearanceThreshold;
	}

	/*! 
	* \brief 	Method for getting the minimum acceptable distance between the vehicle and an obstacle.
	* \return		Distance from the center of the robot to the center of an obstacle in meters. 
	*/
	float Planner::getClearanceDistance()const{
		return _clearanceThreshold;
	}

	/*! 
	* \brief 	Method for setting the obstacle range at which it is acceptable to treat obstacles
	* 			as attractive rather than repulsive.
	* \param[in] obstacleThreshold 		Obstacle range threshold measured in meters.
	*/
	void Planner::setObstacleRangeThreshold(float obstacleThreshold){
		_obstacleThreshold = obstacleThreshold;
	}

	/*! 
	* \brief 	Method for getting the obstacle range at which it is acceptable to treat obstacles
	* 			as attractive rather than repulsive.
	* \return 		Obstacle range threshold measured in meters.
	*/
	float Planner::getObstacleRangeThreshold()const{
		return _obstacleThreshold;
	}

	/*!
	* \brief 	Method for adding an obstacle set to the planner. This version uses only the current obstacle set.
	* \param[in] obstacles 		The set of obstacles represented as ranges and relative bearings.
	*/
	void Planner::addObstacles(rlib::SensorData* obstacles){
		// Need to add a check in the future to ensure this is not an odd data type like and image or something.
		// Copy the range and bearing data from the Lidar data.
		_obstacles = obstacles->getData();

		///*** Remove obstacles less than a certain distance because it's the robot, not an obstacle. ***///

		// Generate near obstacle set.
		std::vector<float> filteredObstacles_ranges, filteredObstacles_angles;

		// Iterate through the obstacle set and retain the obstacles that aren't the robot.
		for(int i = 0; i < _obstacles.cols(); i++){
			// Is the obstacle farther than the threshold?
			if(_obstacles(0,i) > ROBOT_RADIUS_IN_METERS){
				// Add it to the vector
				filteredObstacles_ranges.push_back(_obstacles(0,i));
				filteredObstacles_angles.push_back(_obstacles(1,i));
			}
		}

		// Populate the matrix with the filtered obstacles.
		_obstacles.resize(2, filteredObstacles_ranges.size());
		for(int i = 0; i < filteredObstacles_ranges.size(); i++){
			_obstacles(0, i) = filteredObstacles_ranges[i];
			_obstacles(1, i) = filteredObstacles_angles[i];
		}

		// Modify ranges according to safety clearance.
		// Remember: in Eigen arrays and matrices cannot be mixed.
		_obstacles.row(0) = _obstacles.row(0).array() - _clearanceThreshold*exp(-_obstacles.row(0).array());
	}

	/*!
	* \brief 	Method for setting the location of the goal with respect to the robot.
	* \param[in] range 			Distance to the goal measured in meters.
	* \param[in] angle			Relative bearing to the goal measured in radians.
	*/
	void Planner::updateGoal(float range, float angle){
		// Update goal location.
		_goalRange = range;
		_goalAngle = angle;
	}

	/*!
	* \brief 	Method for getting the location of the goal with respect to the robot.
	* \param[out] goal	Range and bearing of the goal relative to the robot in meters.
	*					Format is [range, bearing].
	*/
	void Planner::returnGoal(std::vector<float> & goal)const{
		// Add goal information to vector.
		goal.push_back(_goalRange);
		goal.push_back(_goalAngle);
	}

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
	void Planner::tuneAngular(	float goalAttraction,
						float obstacleRepulsionTurning, 
						float obstacleAttraction, 
						float obstacleGoalBearingRatio, 
						float vehicleInertia)
	{
		// Set parameters.
		_goalAttraction = goalAttraction;
		_obstacleRepulsionTurning = obstacleRepulsionTurning;
		_obstacleAttraction = obstacleAttraction;
		_vehicleInertia = vehicleInertia;

		// Ensure that the obstacle goal bearing ratio is between zero and one.
		if(obstacleGoalBearingRatio > 1) obstacleGoalBearingRatio = 1;
		if(obstacleGoalBearingRatio < 0) obstacleGoalBearingRatio = 0;

		_obstacleGoalBearingRatio = obstacleGoalBearingRatio;
	}

	/*!
	* \brief 	Method for getting the values of the tuning parameters responsible for 
	*			steering the robot.
	* \param[out] params 	Direction tuning parameters. Format is 
	*						[goalAttraction, obstacleRepulsionTurning, obstacleAttraction, obstacleBearingRatio, vehicleInertia].
	*/
	void Planner::getTurningParameters(std::vector<float> & params)const{
		// Add parameters to vector.
		params.push_back(_goalAttraction);
		params.push_back(_obstacleRepulsionTurning);
		params.push_back(_obstacleAttraction);
		params.push_back(_obstacleGoalBearingRatio);
		params.push_back(_vehicleInertia);
	}

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
	void Planner::tuneLinear(float velocityGain,
					float obstacleRepulsionForward, 
					float angularRateBraking){
		// Set parameters.
		_velocityGain = velocityGain;
		_obstacleRepulsionForward = obstacleRepulsionForward;
		_angularRateBraking = angularRateBraking;
	}

	/*!
	* \brief 	Method for getting the values of the tuning parameters responsible for
	*			the forward velocity of the robot.
	* \param[out] params 	Forward velocity tuning parameters. Format is
	*						[velocityGain, obstacleRepulsionForward, angularRateBraking].
	*/
	void Planner::getForwardVelocityParameters(std::vector<float> & params)const{
		// Add parameters to vector.
		params.push_back(_velocityGain);
		params.push_back(_obstacleRepulsionForward);
		params.push_back(_angularRateBraking);
	}

	/*! 
	* \brief 	Method for generating velocity commands.
	*/
	void Planner::plan(float currentTime){
		// Be sure that there are obstacles to work with.
		if(!_obstacles.cols()) return;

		// Do the heavy lifting.
		// Set time.
		_currentTime = currentTime;

		// Update the straight line planner.

		// Update the escape strategy.

		// Goal pose.
		float goalRange, goalBearing;
		// Use actual goal location
		goalRange = _goalRange;
		goalBearing = _goalAngle;

		// Modify goal if necessary.
		if(_slp->isGoalInSight()){ // Check straight line planner.
			// Set goal to intermediate goal.
			_slp->getGoal(goalRange, goalBearing);
		}
		else if(_es->isStuck()){ // Check stuck condition.
			// Set random goal.
			_es->getGoal(goalRange, goalBearing);
		}

		//std::cout << "Goal: (" << goalRange << ", " << goalBearing << ")" << std::endl;

		// If we're at the goal, stay here.
		if(goalRange < 0.4){
			//std::cout << "At goal!" << std::endl;
			// Set command.
			_command.vx = 0;
			_command.wz = 0;
			return;
		}

		// Compute velocity commands.

		// Angle terms.
		Eigen::Matrix<float, 2, 1> goalVector, repelVector, attractVector, inertiaVector;
		Eigen::Matrix<float, 2, 1> steeringVector;

		// Get their values.
		computeGoalVector(goalRange, goalBearing, goalVector);
		computeRepulsionVector(goalRange, goalBearing, repelVector);
		computeAttractionVector(goalRange, goalBearing, attractVector);
		computeInertiaVector(inertiaVector);

		// // Compute direction.
		// std::cout << "Goal Vector: ";
		// std::cout << "(";
		// std::cout << goalVector(0,0); std::cout << ", ";
		// std::cout << goalVector(1,0); std::cout << ")";
		// std::cout << std::endl;

		// std::cout << "Repel Vector: ";
		// std::cout << "(";
		// std::cout << repelVector(0,0); std::cout << ", ";
		// std::cout << repelVector(1,0); std::cout << ")";
		// std::cout << std::endl;

		// std::cout << "Attract Vector: ";
		// std::cout << "(";
		// std::cout << attractVector(0,0); std::cout << ", ";
		// std::cout << attractVector(1,0); std::cout << ")";
		// std::cout << std::endl;

		// std::cout << "Inertia Vector: ";
		// std::cout << "(";
		// std::cout << inertiaVector(0,0); std::cout << ", ";
		// std::cout << inertiaVector(1,0); std::cout << ")";
		// std::cout << std::endl;

		steeringVector = goalVector - repelVector + attractVector + inertiaVector;

		// std::cout << "Steering Vector: ";
		// std::cout << "(";
		// std::cout << steeringVector(0,0); std::cout << ", ";
		// std::cout << steeringVector(1,0); std::cout << ")";
		// std::cout << std::endl;

		// steeringVector.normalize();
		// std::cout << "Steering Vector(Normalized): ";
		// std::cout << "(";
		// std::cout << steeringVector(0,0); std::cout << ", ";
		// std::cout << steeringVector(1,0); std::cout << ")";
		// std::cout << std::endl;

		// Compute angular velocity.
		float w = computeAngularVelocity(steeringVector);
		// Compute linear velocity.
		float v = computeLinearVelcoity(w);

		// Set command.
		_command.vx = v;
		_command.wz = w;

	}

	/*! 
	* \brief 	Method for getting a velocity command from the planner. 
	* \return 		Velocity command for the vehicle.
	*/
	const rlib::Vel & Planner::getCommand()const{
		return _command;
	}

	/*!
	* \brief Print operator for planner parameters.
	*/
	std::ostream& operator<<(std::ostream& os, Planner& p){
		// Velocity limits.
		os << "Velocity limits: [";
		os << p._vmin; os << ", ";
		os << p._vmax; os << ", ";
		os << p._wmin; os << ", ";
		os << p._wmax; os << "]"; os << std::endl;

		// Safety distance from robot.
		os << "Clearance threshold: ";
		os << p._clearanceThreshold; os << std::endl; 

		// Distance in meters when obstacles become attractive.
		os << "Obstacle threshold: ";
		os << p._obstacleThreshold; os << std::endl;

		// Initial goal (origin).
		os << "Goal: [";
		os << p._goalRange; os << ", ";
		os << p._goalAngle; os << "]"; os << std::endl;

		// Steering parameters.
		os << "Steering parameters: [";
		os << p._goalAttraction;  os << ", ";
		os << p._obstacleRepulsionTurning;  os << ", ";
		os << p._obstacleAttraction;  os << ", ";
		os << p._obstacleGoalBearingRatio;  os << "]"; os << std::endl;

		// Resistance to turning.
		os << "Inertia: ";
		os << p._vehicleInertia; os << std::endl;

		// Forward velocity parameters.
		os << "Forward velocity parameters: [";
		os << p._velocityGain; os << ", ";
		os << p._obstacleRepulsionForward; os << ", ";
		os << p._angularRateBraking; os << "]"; os << std::endl;

		// Set initial velocity command.
		os << "Velocity command: [";
		os << p._command.vx; os << ", ";
		os << p._command.wz; os << "]"; os << std::endl;
	}


	/*!
	* \brief Method to compute the vector that points to the goal. 
	* \param[in] goalRange 		Distance to the goal in meters.
	* \param[in] goalBearing 	Relative bearing to the goal in radians.
	* \param[out] goalVector 	Vector that points to the goal.
	*/
	void Planner::computeGoalVector(float goalRange, float goalBearing, Eigen::Matrix<float, 2, 1> & goalVector){
		// Check for degenerate case.
		if(goalRange == 0.0) goalRange = 1e-3;
		// Compute goal range gain.
		float rangeGain = pow(goalRange, -2.0);

		// Compute goal vector.
		goalVector(0,0) = cos(goalBearing);
		goalVector(1,0) = sin(goalBearing);

		goalVector *= _goalAttraction*rangeGain;

	}

	/*!
	* \brief Method to compute the vector that repels the robot from obstacles.
	* \param[out] repelVector 	Vector that repels the robot from obstacles.
	*/
	void Planner::computeRepulsionVector(float goalRange, float goalBearing, Eigen::Matrix<float, 2, 1> & repelVector){
		// Generate near obstacle set.
		std::vector<float> ranges, angles;

		// Iterate through the obstacle set and pull out the near obstacles.
		for(int i = 0; i < _obstacles.cols(); i++){
			// Is the obstacle closer than the threshold?
			if(_obstacles(0,i) < _obstacleThreshold){
				// Add it to the vector
				ranges.push_back(_obstacles(0,i));
				angles.push_back(_obstacles(1,i));
			}
		}

		// Check for empty set.
		if(ranges.empty()){
			// Set the attract vector.
			repelVector(0,0) = 0;
			repelVector(1,0) = 0;
			// Get out. Our job is done.
			return;
		}

		// Generate near obstacle matrix.
		Eigen::Matrix<float, 2, Eigen::Dynamic> nearObstacles;
		nearObstacles.resize(2, ranges.size());
		for(int i = 0; i < ranges.size(); i++){
			nearObstacles(0, i) = ranges[i];
			nearObstacles(1, i) = angles[i];
		}

		// Generate matrix of unit vectors for sensor directions.
		Eigen::Matrix<float, 2, Eigen::Dynamic> alpha;
		alpha.resize(2,nearObstacles.cols());
		alpha.row(0) = cos(nearObstacles.row(1).array());
		alpha.row(1) = sin(nearObstacles.row(1).array());

		// Generate obstacle range gains.
		Eigen::Matrix<float, Eigen::Dynamic, 1> g_r;
		g_r.resize(nearObstacles.cols(),1);
		g_r = nearObstacles.row(0).array().pow(-2.0).transpose();

		// Generate obstacle vehicle direction gains.
		Eigen::Matrix<float, Eigen::Dynamic, 1> g_alpha_i;
		g_alpha_i.resize(nearObstacles.cols(),1);
		g_alpha_i = nearObstacles.row(1).array().pow(-2.0).transpose();
		// Adjust gains.
		g_alpha_i *= _obstacleGoalBearingRatio;

		// Generate obstacle goal direction gains.
		Eigen::Matrix<float, Eigen::Dynamic, 1> g_alpha_g;
		g_alpha_g.resize(nearObstacles.cols(),1);
		g_alpha_g = (nearObstacles.row(1).array() - goalBearing).pow(-2.0).transpose();
		// Adjust gains.
		g_alpha_g *= 1 - _obstacleGoalBearingRatio;

		// Generate repulsion gains.
		Eigen::Matrix<float, Eigen::Dynamic, 1> g_repel;
		g_repel.resize(nearObstacles.cols(),1);
		g_repel = g_r.cwiseProduct(g_alpha_g + g_alpha_i);

		// Compute repulsion vector.
		repelVector = _obstacleRepulsionTurning*alpha*g_repel;
	}

	/*!
	* \brief Method to compute the vector that attracts the robot towards obstacles.
	* \param[out] attractVector 	Vector that attracts the robot to obstacles.
	*/
	void Planner::computeAttractionVector(float goalRange, float goalBearing, Eigen::Matrix<float, 2, 1> & attractVector){
		// Generate far obstacle set.
		std::vector<float> ranges, angles;

		// Iterate through the obstacle set and pull out the far obstacles.
		for(int i = 0; i < _obstacles.cols(); i++){
			// Is the obstacle farther than the threshold?
			if(_obstacles(0,i) >= _obstacleThreshold){
				// Add it to the vector
				ranges.push_back(_obstacles(0,i));
				angles.push_back(_obstacles(1,i));
			}
		}

		// Check for empty set.
		if(ranges.empty()){
			// Set the attract vector.
			attractVector(0,0) = 0;
			attractVector(1,0) = 0;
			// Get out. Our job is done.
			return;
		}

		// Generate far obstacle matrix.
		Eigen::Matrix<float, 2, Eigen::Dynamic> farObstacles;
		farObstacles.resize(2, ranges.size());
		for(int i = 0; i < ranges.size(); i++){
			farObstacles(0, i) = ranges[i];
			farObstacles(1, i) = angles[i];
		}

		// Generate matrix of unit vectors for sensor directions.
		Eigen::Matrix<float, 2, Eigen::Dynamic> alpha;
		alpha.resize(2,farObstacles.cols());
		alpha.row(0) = cos(farObstacles.row(1).array());
		alpha.row(1) = sin(farObstacles.row(1).array());

		// Generate obstacle range gains.
		Eigen::Matrix<float, Eigen::Dynamic, 1> g_r;
		g_r.resize(farObstacles.cols(),1);
		g_r = farObstacles.row(0).array().pow(2.0);

		// Generate obstacle goal direction gains.
		Eigen::Matrix<float, Eigen::Dynamic, 1> g_alpha_g;
		g_alpha_g.resize(farObstacles.cols(),1);
		g_alpha_g = (farObstacles.row(1).array() - goalBearing).pow(-2.0);

		// Generate attraction gains.
		Eigen::Matrix<float, Eigen::Dynamic, 1> g_attract;
		g_attract.resize(farObstacles.cols(),1);
		g_attract = g_r.cwiseProduct(g_alpha_g);

		// Compute repulsion vector.
		attractVector = _obstacleAttraction*alpha*g_attract;
	}

	/*! 
	* \brief Method to compute the vehicle's resistance to turning.
	* \param[out] inertiaVector 	Vector that works to maintain the vehicle's current heading.
	*/
	void Planner::computeInertiaVector(Eigen::Matrix<float, 2, 1> & inertiaVector){
		inertiaVector(0,0) = _vehicleInertia;
		inertiaVector(1,0) = 0;
	}

	/*!
	* \brief Method to compute the angular velocity command.
	* \param[in] steeringVector 	The desired heading.
	* \return 						The angular velocity command in radians/second.
	*/
	float Planner::computeAngularVelocity(Eigen::Matrix<float, 2, 1> & steeringVector){
		// std::cout << "steeringVector: [";
		// std::cout << steeringVector(1,0) << ", ";
		// std::cout << steeringVector(0,0) << "] ";

		// Get desired heading angle.
		float headingAngle = atan2(steeringVector(1,0), steeringVector(0,0));

		// std::cout << "headingAngle: [" << headingAngle << "] ";

		// Normalize the heading angle.
		float headingMag = headingAngle/M_PI;
		// std::cout << "headingMag: [" << headingMag << "] ";
		// std::cout << std::endl;

		// Use angular velocity limits.
		if(headingMag > 0) return headingMag*fabs(_wmax);
		else return headingMag*fabs(_wmin);
	}

	/*!
	* \brief Method to compute the linear velocity command.
	* \param[in] w 		The desired angular velocity.
	* \return 			The linear velocity command in meters/second.
	*/
	float Planner::computeLinearVelcoity(float w){
		// Grab the minimum range obstacle.
		// Possibly change the obstacle range. Scale ranges and then find min.
		float minRange = _obstacles.row(0).minCoeff();
		//std::cout << "Min range: " << minRange << std::endl;

		// Modify range according to gain parameter.
		float adjustedRange = pow(_obstacleRepulsionForward, -1)*minRange;
		
		// Compute linear velocity.
		float v = _velocityGain*pow(adjustedRange, 2)*pow(w, -2);

		// Limit velocity.
		if(v > _vmax) v = _vmax;
		else if(v < _vmin) v = _vmin;

		return v;
	}

}

