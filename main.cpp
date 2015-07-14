// Standard Libraries
#include <iostream>
#include <math.h>
#include <string>
#include <stdlib.h>
#include <pthread.h>
#include <limits>
#include <fstream>
#include <ostream>
#include <istream>
#include <boost/program_options.hpp>

// AL Libraries
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alsensorsproxy.h>
#include <alproxies/dcmproxy.h>
#include <alproxies/albasicawarenessproxy.h>
#include <qi/os.hpp>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/alredballtrackerproxy.h>

// Custom Libraries

// Robotlib Libraries
#include <rlib_sensor.hpp>
#include <godzila.hpp>
#include <naopp.hpp>

// Namespace Variables
namespace po = boost::program_options;

///*** Functions to tune GODZILA gains through the command line.***///
void* tuneGains(void* voidPlanner);
void printMainMenu();
void editValue(std::vector<float> & values, int index);
void configureSpeedLimits(godzila::Planner * planner);
void configureRangeThresholds(godzila::Planner * planner);
void configureGoal(godzila::Planner * planner);
void configureSteering(godzila::Planner * planner);
void configureForwardMotion(godzila::Planner * planner);
void streamData(godzila::Planner * planner);
void recordData(godzila::Planner * planner);

///*** Functions for doing config file operations. ***///
#define GODZILA_CONFIG_FILENAME "godzila.config"
void writeGodzilaConfig(const godzila::Planner* planner);
bool readGodzilaConfig(po::options_description& desc, po::variables_map& vm);

#define GODZILA_LOG_FILENAME "godzila.log"

///*** Main Function ***///
int main(int argc, char* argv[]) {
	if(argc != 2){
	    std::cerr << "Wrong number of arguments!" << std::endl;
	    std::cerr << "Usage: naonav NAO_IP" << std::endl;
	    exit(2);
  	}

	///*** Initialize Program Control Global Variables ***///
	bool robotHalted = false; // Controls if robot should run or stop.


	///*** Initialize rlib Objects ***///
	// Create Lidar.
	rlib::Sensor* Lidar = new rlib::Sensor("hokuyo");
	// Create Lidar Handler.
	rlib::SensorHandler* LidarHandler = new rlib::HokuyoLidarHandlerIP(argv[1], "9002");
	// Set Handler.
	Lidar->attachHandler(LidarHandler);
	// Start Lidar.
	Lidar->start();

	// Create Navigator.
	godzila::Planner* planner = new godzila::Planner();
	// Read configuration from file. 
	std::vector<float> params(14);
	po::options_description godzilaConfig("Godzila Config");
	godzilaConfig.add_options()
		("vmin", po::value<float>(&(params[0])),"vmin")
		("vmax", po::value<float>(&(params[1])),"vmax")
		("wmin", po::value<float>(&(params[2])),"wmin")
		("wmax", po::value<float>(&(params[3])),"wmax")
		("clearanceThreshold", po::value<float>(&(params[4])),"clearanceThreshold")
		("obstacleThreshold", po::value<float>(&(params[5])),"obstacleThreshold")
		("goalAttraction", po::value<float>(&(params[6])),"goalAttraction")
		("obstacleRepulsionTurning", po::value<float>(&(params[7])),"obstacleRepulsionTurning")
		("obstacleAttraction", po::value<float>(&(params[8])),"obstacleAttraction")
		("obstacleGoalBearingRatio", po::value<float>(&(params[9])),"obstacleGoalBearingRatio")
		("vehicleInertia", po::value<float>(&(params[10])),"vehicleInertia")
		("velocityGain", po::value<float>(&(params[11])),"velocityGain")
		("obstacleRepulsionForward", po::value<float>(&(params[12])),"obstacleRepulsionForward")
		("angularRateBraking", po::value<float>(&(params[13])),"angularRateBraking");

	po::variables_map godzilaParams;
	if(readGodzilaConfig(godzilaConfig, godzilaParams)){
		// Load the parameters.
		planner->setSpeedLimits(godzilaParams["vmin"].as<float>(),
								godzilaParams["vmax"].as<float>(), 
								godzilaParams["wmin"].as<float>(), 
								godzilaParams["wmax"].as<float>());
		planner->setClearanceDistance(godzilaParams["clearanceThreshold"].as<float>());
		planner->setObstacleRangeThreshold(godzilaParams["obstacleThreshold"].as<float>());
		planner->tuneAngular(	godzilaParams["goalAttraction"].as<float>(),
								godzilaParams["obstacleRepulsionTurning"].as<float>(), 
								godzilaParams["obstacleAttraction"].as<float>(), 
								godzilaParams["obstacleGoalBearingRatio"].as<float>(),
								godzilaParams["vehicleInertia"].as<float>());
		planner->tuneLinear(	godzilaParams["velocityGain"].as<float>(), 
								godzilaParams["obstacleRepulsionForward"].as<float>(), 
								godzilaParams["angularRateBraking"].as<float>());
	}
	else{
		// Set defaults.
		// Set speed limits.
		planner->setSpeedLimits(-0.4, 0.4, -0.2, 0.2);
		// Set configuration distances.
		planner->setObstacleRangeThreshold(2.0);
		planner->setClearanceDistance(0.6);
	}
	// Print parameters.
	std::cout << *planner << std::endl;

	// Create thread for tuning planner gains.
	pthread_t tuningThread;
	pthread_create(&tuningThread, NULL, tuneGains, planner);

	///*** Initialize NaoQi String Names ***///
	///*** Package this into the implementation object for the Robot Actuator Subclass ***///

	// String for Button Values
	const std::string frontButton = "Device/SubDeviceList/Head/Touch/Front/Sensor/Value";
	const std::string middleButton = "Device/SubDeviceList/Head/Touch/Middle/Sensor/Value";	
	const std::string rearButton = "Device/SubDeviceList/Head/Touch/Rear/Sensor/Value";	

  	///*** Initialize NaoQi Proxies ***///

	// Initialize Memory Proxy to grab memory values.
	AL::ALMemoryProxy memPrx(argv[1], 9559);
	// Initialize Motion Proxy for walking.
	AL::ALMotionProxy motionPrx(argv[1], 9559);
	// Initialize Posture Proxy for going to predefined poses.
	AL::ALRobotPostureProxy pstrPrx(argv[1], 9559);
	// Initialize Redball Tracking Proxy.
	AL::ALRedBallTrackerProxy redBallPrx(argv[1], 9559);
	// Initialize DCM Proxy, used to get time.
	AL::DCMProxy dcm_proxy(argv[1], 9559);
  
	///*** Configure NaoQi Proxies ***///
  
  
	// Disable Collision Protection

	naopp::testArmPose(argc, argv);
	Lidar->stop();
	exit(0);

	// Head Pressed? Then stand.
	while(memPrx.getData(frontButton) != AL::ALValue(1.0));

	// Stand Nao.
	pstrPrx.goToPosture("Stand", 1.0f);
	// Start tracking.
	redBallPrx.startTracker();

	// Initialize Walk

	try {

    	std::cout << "Loop started." << std::endl;

    	// Should the robot stop?
    	while(!robotHalted){

			// Head Pressed? Then stand.
			if(memPrx.getData(frontButton) == AL::ALValue(1.0)){
				// Stand Nao.
				pstrPrx.goToPosture("Stand", 1.0f);
			}
			if(memPrx.getData(rearButton) == AL::ALValue(1.0)){
				// Stand Nao.
				pstrPrx.goToPosture("Crouch", 1.0f);
			}


    		///*** Update System State ***///
			// Grab scan
			Lidar->update();
			rlib::SensorData* scan = Lidar->getData();

			// Head Pressed?
			if(memPrx.getData(middleButton) == AL::ALValue(1.0)) robotHalted = true;

			
			// Update Motion Planner.
			planner->addObstacles(scan);
			//planner->updateGoal(5,0);
			// Update Ball Pose
		    if(redBallPrx.isNewData()){

				std::vector<float> ballPose = redBallPrx.getPosition();
				float ballRange = sqrt(pow(ballPose[0],2) + pow(ballPose[1],2));
				float ballBearing = atan2(ballPose[1], ballPose[0]);
				planner->updateGoal(ballRange, ballBearing);
		    }
			planner->plan(0);			
			// Get movement command.
			rlib::Vel speed = planner->getCommand();
			
			// Update Walk
			motionPrx.moveToward(speed.vx, speed.vy, speed.wz);

    	}

    	///*** Shut things down ***///
		std::cout << "Robot Halted." << std::endl;
		
		// Stop robot.
		motionPrx.moveToward(0.0f, 0.0f, 0.0f);
    	pstrPrx.goToPosture("Crouch", 1.0f);	
    	// Stop tracking.
		redBallPrx.stopTracker();

    	// Terminate Lidar.
    	Lidar->stop();

		// Wait for thread to terminate.
		pthread_join(tuningThread, NULL);

  	}
	catch (const AL::ALError& e) {
  		// Stop tracking.
		redBallPrx.stopTracker();
    	// Stop Lidar.
    	Lidar->stop();
    	exit(1);
  	}

  	exit(0);
}

void writeGodzilaConfig(const godzila::Planner* planner){
  	// Open config file.
	std::ofstream godzilaConfig(GODZILA_CONFIG_FILENAME);
  	// Parameters.
  	std::vector<float> temp, params;
	// Param names.
	std::string names[] = {	"vmin", "vmax", "wmin", "wmax", 
						"clearanceThreshold","obstacleThreshold",
						"goalAttraction", "obstacleRepulsionTurning", "obstacleAttraction", "obstacleGoalBearingRatio", "vehicleInertia",
						"velocityGain", "obstacleRepulsionForward", "angularRateBraking"};

	
	// Speed Limits.
	// Clear the vector.
	temp.clear();
	// Get the current settings.
	planner->getSpeedLimits(temp);
	// Add to parameter vector.
	params.insert(params.end(), temp.begin(), temp.end());

	// Range Thresholds.
	params.push_back(planner->getClearanceDistance());
	params.push_back(planner->getObstacleRangeThreshold());

	// Turing Parameters.
	// Clear the vector.
	temp.clear();
	// Get the current settings.
	planner->getTurningParameters(temp);
	// Add to parameter vector.
	params.insert(params.end(), temp.begin(), temp.end());

	// Forward Parameters.
	// Clear the vector.
	temp.clear();
	// Get the current settings.
	planner->getForwardVelocityParameters(temp);
	// Add to parameter vector.
	params.insert(params.end(), temp.begin(), temp.end());	

	// Write the settings.
	for (int i = 0; i < params.size(); ++i){
		// Write config.
		godzilaConfig << names[i] << "=" << params[i] << std::endl;
	}

	godzilaConfig.close();
  	
}

bool readGodzilaConfig(po::options_description& desc, po::variables_map& vm){
  
	std::ifstream godzilaConfig(GODZILA_CONFIG_FILENAME);
	if(godzilaConfig.is_open()){

		// Clear the map.
		vm = po::variables_map();

		po::store(po::parse_config_file(godzilaConfig , desc), vm);
		po::notify(vm);    
		return true;
	}
	else return false;
}

void* tuneGains(void* voidPlanner){
	
	// Cast planner to the appropriate type.
	godzila::Planner* planner = static_cast<godzila::Planner*>(voidPlanner);

	// Command for the terminal.
	char option = '0';

	// Wait for the exit command.
	while(option != 'x'){
		switch(option){
			case '1':
				// Do things with Goal.
				configureGoal(planner);
				break;
			case '2':
				// Do things with Speed Limits.
				configureSpeedLimits(planner);
				break;
			case '3':
				// Do things with Range Thresholds.
				configureRangeThresholds(planner);
				break;
			case '4':
				// Do things with Steering Parameters.
				configureSteering(planner);
				break;
			case '5':
				// Do things with Forward Motion Parameters.
				configureForwardMotion(planner);
				break;
			case '6':
				// Write parameters to file.
				writeGodzilaConfig(planner);
				break;
			case '7':	
				// Stream data about robot.
				streamData(planner);
				break;
			case '8':	
				// Stream data about robot.
				recordData(planner);
				break;
			default:
				break;
		}
		
		// Print menu.
		printMainMenu();	
		// Get the user option.
		std::cin >> option;
	}

	// Shut down thread.
	pthread_exit(NULL);
}

void printMainMenu(){
	std::cout << std::endl;
	std::cout << "1. Set Goal" << std::endl;
	std::cout << "2. Configure Speed Limits" << std::endl;
	std::cout << "3. Configure Range Thresholds" << std::endl;
	std::cout << "4. Configure Steering Parameters" << std::endl;
	std::cout << "5. Configure Forward Motion Parameters" << std::endl;
	std::cout << "6. Write Parameters to File" << std::endl;
	std::cout << "7. Stream Data" << std::endl;
	std::cout << "8. Record Data" << std::endl;
	std::cout << "x. Exit" << std::endl;
	std::cout << "Choose option:" ;
}

void editValue(std::vector<float> & values, int index){
	// New value.
	float value;

	// Print message.
	std::cout << "Enter new value [" << values[index] << "]:";
	// Get user input.
	if(std::cin >> value){
		// If the input is good then set the parameter.
		values[index] = value;	
	}
	else{
		// If not ignore it and move on.
		std::cin.clear();
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	}
}

void configureSpeedLimits(godzila::Planner * planner){
	// Get current limits.
	std::vector<float> limits;

	// Menu variables.
	char option = '0';

	// Decisions, decisions...
	while(option != 'x'){

		// Update.
		limits.clear();
		planner->getSpeedLimits(limits);	

		switch(option){
			case '1':
				editValue(limits, 0);
				break;
			case '2':
				editValue(limits, 1);
				break;
			case '3':
				editValue(limits, 2);
				break;
			case '4':
				editValue(limits, 3);
				break;
			default:
				break;
		}

		// Print menu.
		std::cout << std::endl;
		std::cout << "1. Vmin[" << limits[0] << "]" << std::endl;
		std::cout << "2. Vmax[" << limits[1] << "]" << std::endl;
		std::cout << "3. Wmin[" << limits[2] << "]" << std::endl;
		std::cout << "4. Vmax[" << limits[3] << "]" << std::endl;
		std::cout << "x. Exit" << std::endl;
		std::cout << "Choose option:";

		// Set limits. 	[vmin, vmax, wmin, wmax]
		planner->setSpeedLimits(limits[0], limits[1], limits[2], limits[3]);

		// Get user option.
		std::cin >> option;
	}
}

void configureRangeThresholds(godzila::Planner * planner){
	// Get current limits.	
	std::vector<float> ranges;

	// Menu variables.
	char option = '0';

	// Decisions, decisions...
	while(option != 'x'){

		// Update.
		ranges.clear();
		ranges.push_back(planner->getClearanceDistance());	
		ranges.push_back(planner->getObstacleRangeThreshold());	

		switch(option){
			case '1':
				editValue(ranges, 0);
				break;
			case '2':
				editValue(ranges, 1);
				break;
			default:
				break;
		}

		// Print menu.
		std::cout << std::endl;
		std::cout << "1. Clearance Distance[" << ranges[0] << "]" << std::endl;
		std::cout << "2. Range Threshold[" << ranges[1] << "]" << std::endl;
		std::cout << "x. Exit" << std::endl;
		std::cout << "Choose option:";

		// Set limits.
		planner->setClearanceDistance(ranges[0]);
		planner->setObstacleRangeThreshold(ranges[1]);

		// Get user option.
		std::cin >> option;
	}
}

void configureGoal(godzila::Planner * planner){

	// Get current parameters.
	std::vector<float> params;

	// Menu variables.
	char option = '0';

	// Decisions, decisions...
	while(option != 'x'){
		
		// Update parameters
		params.clear();
		planner->returnGoal(params);

		switch(option){
			case '1':
				editValue(params, 0);
				break;
			case '2':
				editValue(params, 1);
				break;
			default:
				break;
		}

		// Print menu.
		std::cout << std::endl;
		std::cout << "1. Goal Range[" << params[0] << "]" << std::endl;
		std::cout << "2. Goal Bearing[" << params[1] << "]" << std::endl;
		std::cout << "x. Exit" << std::endl;
		std::cout << "Choose option:";

		// Set limits.
		planner->updateGoal(params[0], params[1]);

		// Get user option.
		std::cin >> option;
	}
}

void configureSteering(godzila::Planner * planner){
	// Get current parameters.
	std::vector<float> params;

	// Menu variables.
	char option = '0';

	// Decisions, decisions...
	while(option != 'x'){
		
		// Update parameters
		params.clear();
		planner->getTurningParameters(params);

		switch(option){
			case '1':
				editValue(params, 0);
				break;
			case '2':
				editValue(params, 1);
				break;
			case '3':
				editValue(params, 2);
				break;
			case '4':
				editValue(params, 3);
				break;
			case '5':
				editValue(params, 4);
				break;
			default:
				break;
		}

		// Print menu.
		std::cout << std::endl;
		std::cout << "1. Goal Attraction[" << params[0] << "]" << std::endl;
		std::cout << "2. Obstacle Repulsion[" << params[1] << "]" << std::endl;
		std::cout << "3. Obstacle Attraction[" << params[2] << "]" << std::endl;
		std::cout << "4. Obstacle Goal Bearing Ratio[" << params[3] << "]" << std::endl;
		std::cout << "5. Vehicle Inertia[" << params[4] << "]" << std::endl;
		std::cout << "x. Exit" << std::endl;
		std::cout << "Choose option:";

		// Set limits.
		planner->tuneAngular(params[0], params[1], params[2], params[3], params[4]);

		// Get user option.
		std::cin >> option;
	}
}

void configureForwardMotion(godzila::Planner * planner){

	// Get current parameters.
	std::vector<float> params;

	// Menu variables.
	char option = '0';

	// Decisions, decisions...
	while(option != 'x'){
		
		// Update parameters
		params.clear();
		planner->getForwardVelocityParameters(params);

		switch(option){
			case '1':
				editValue(params, 0);
				break;
			case '2':
				editValue(params, 1);
				break;
			case '3':
				editValue(params, 2);
				break;
			default:
				break;
		}

		// Print menu.
		std::cout << std::endl;
		std::cout << "1. Velocity Gain[" << params[0] << "]" << std::endl;
		std::cout << "2. Obstacle Repulsion[" << params[1] << "]" << std::endl;
		std::cout << "3. Angular Rate Braking[" << params[2] << "]" << std::endl;
		std::cout << "x. Exit" << std::endl;
		std::cout << "Choose option:";

		// Set limits.
		planner->tuneLinear(params[0], params[1], params[2]);

		// Get user option.
		std::cin >> option;
	}
}

// Flag to know when to quit printing.
bool flag_printData;

void * printData(void * voidPlanner){
	// Cast planner to the appropriate type.
	godzila::Planner* planner = static_cast<godzila::Planner*>(voidPlanner);	

	while(flag_printData){
		// Print the velocity command.
		rlib::Vel speed = planner->getCommand();
		std::cout << "Command: [" << speed.vx << ", " << speed.wz << "] ";
		// Print the goal location.
		std::vector<float> goal;
		planner->returnGoal(goal);
		std::cout << "Goal: [" << goal[0] << ", " << goal[1] << "] ";

		std::cout << std::endl;

		// Sleep the thread.
		sleep(1);
	}

	// Shut down thread.
	pthread_exit(NULL);
}

void streamData(godzila::Planner * planner){
	// Set printing flag.
	flag_printData = true;

	// Create thread for printing planner data.
	pthread_t thread_printData;
	pthread_create(&thread_printData, NULL, printData, planner);

	// Dummy char for accepting keyboard input.
	char kbEntry;
	std::cin >> kbEntry;

	// Indicate to the thread that it should end.
	flag_printData = false;

	// Wait for thread to terminate.
	pthread_join(thread_printData, NULL);
	
}

// Flag to know when to quit recording.
bool flag_recordData;

void * recordDataToFile(void * voidPlanner){
	// Cast planner to the appropriate type.
	godzila::Planner* planner = static_cast<godzila::Planner*>(voidPlanner);	

	// Open log file.
	std::ofstream godzilaLog(GODZILA_LOG_FILENAME);

	while(flag_recordData){
		// Print the velocity command.
		rlib::Vel speed = planner->getCommand();
		std::cout << "Command: [" << speed.vx << ", " << speed.wz << "] ";

		// Print the goal location.
		std::vector<float> goal;
		planner->returnGoal(goal);
		std::cout << "Goal: [" << goal[0] << ", " << goal[1] << "] ";

		std::cout << std::endl;

		// Record the data to file.
		godzilaLog << speed.vx 	<< ", ";
		godzilaLog << speed.wz 	<< ", ";
		godzilaLog << goal[0] 	<< ", ";
		godzilaLog << goal[1] 	<< std::endl;

		// Sleep the thread.
		sleep(1);
	}

	godzilaLog.close();

	// Shut down thread.
	pthread_exit(NULL);
}

void recordData(godzila::Planner * planner){
	// Set printing flag.
	flag_recordData = true;

	// Create thread for printing planner data.
	pthread_t thread_recordData;
	pthread_create(&thread_recordData, NULL, recordDataToFile, planner);

	// Dummy char for accepting keyboard input.
	char kbEntry;
	std::cin >> kbEntry;

	// Indicate to the thread that it should end.
	flag_recordData = false;

	// Wait for thread to terminate.
	pthread_join(thread_recordData, NULL);
	
}