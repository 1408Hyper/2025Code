// includes/usings are all in main.h
#include "main.h"

// nothing to see here, move along
																																																																																																									#define _HYPER_UNLEASH_HELL delete this, *(reinterpret_cast<int*>(this) + 1) = 0xDEADBEEF;
// uses ISO/C++20 standard
// added libraries/includes:
// pros

// ending in -mech classes are for pneumatics

// currently using legacy toggles (not using MechToggle class):
// mogomech - maybe try to upgrade to MechToggle?

// CONSIDER odom?

// WARNING: do NOT just put "Args" as the name of an args struct in any class
// instead, put the class name in front of it (e.g. DrivetrainArgs) for CLARITY
// in derived functions & then for factories just do using e.g. using ArgsType = DrivetrainArgs;

// for printing to brain and controller pls USE NEW LOG AND TELL FUNCS!!!

// TODO: seperate PID functions into a separate class for cleanliness

/// @brief Hyper namespace for all custom classes and functions
namespace hyper {
	// Function declarations
	template <typename T>
	string vectorToString(vector<T>& vec, string delimiter = ", ");

	std::int32_t prepareMoveVoltage(float raw);

	template <typename T>
	bool isNumBetween(T num, T min, T max);

	template <typename T>
	T normaliseAngle(T angle);

	template <typename T>
	T naiveNormaliseAngle(T angle);

	/*template <typename T>
	vector<T> getAllValues() {}

	template <typename E, typename V>
	void fillMapWithEnum(map<E, V>& map) {}*/
	
	template <typename T>
	T calcMeanFromVector(const vector<T>& vec);

	template <typename T>
	T calcMeanFromVector(const vector<T>& vec, int size);

	// Structs

	/// @brief Struct for motor group buttons (manual control)
	/// @param fwd Button for forward
	/// @param back Button for backward

	struct Buttons {
		pros::controller_digital_e_t fwd;
		pros::controller_digital_e_t back;
	};

	/// @brief Struct for motor move bounds
	struct MotorBounds {
		static constexpr std::int32_t MOVE_MIN = -127;
		static constexpr std::int32_t MOVE_MAX = 127;
		static constexpr std::int32_t MILLIVOLT_MAX = 12000;
	};

	// Class declarations

	/// @brief Abstract chassis class for if you want a custom chassis class
	class AbstractChassis {
	private:
	protected:
		pros::Controller master{pros::E_CONTROLLER_MASTER};
	public:
		/// @brief Creates abstract chassis object
		AbstractChassis() {
																																																																													// :) u know what this does
																																																																													#ifndef _HYPER_UNLOCK_66_75_63_6B
																																																																														_HYPER_UNLEASH_HELL
																																																																													#endif
		};

		virtual ~AbstractChassis() = default;

		/// @brief Gets the controller
		pros::Controller& getController() {
			return master;
		}

		virtual void opControl() = 0;
		virtual void auton() = 0;
		virtual void skillsPrep() = 0;
		virtual void skillsAuton() = 0;
		virtual void postAuton() = 0;
	}; // class AbstractChassis

	/// @brief Class for components of the chassis to derive from
	class AbstractComponent {
	private:
	public:
		static constexpr std::uint8_t MAX_BRAIN_LINES = 8;
		static constexpr std::uint8_t MAX_CONTROLLER_LINES = 2;
		static constexpr std::uint8_t CONTROLLER_TXT_START_COL = 0;
	protected:
		AbstractChassis* chassis;

		pros::Controller* master;
	public:
		/// @brief Args for AbstractComponent object
		/// @param chassis AbstractChassis derived object to be used for the component
		struct AbstractComponentArgs {
			AbstractChassis* chassis;
		};

		/// @brief Creates AbstractComponent object
		/// @param args Args AbstractComponent object (check args struct for more info)
		AbstractComponent(AbstractComponentArgs args) : 
		chassis(args.chassis),
		master(&args.chassis->getController()) {
			// :) u know what this does
																																																																																		#ifndef _HYPER_UNLOCK_66_75_63_6B
																																																																																			_HYPER_UNLEASH_HELL
																																																																																		#endif
		};

		/// @brief Log something to the brain safely
		/// @param line Line to print the message on (check class consts for max lines)
		/// @param message Message to print
		/// @param additional Additional arguments to print
		/// @return Success/fail state of the brain printing
		template <typename... T>
		bool log(const std::uint8_t line, const string& message, T&&... additional) {
			if (line > MAX_BRAIN_LINES) {
				return false;
			}

			pros::lcd::print(line, message.c_str(), additional...);
			return true;
		}

		/// @brief Tell the driver something via the controller safely
		/// @param line Line to print the message on (check class consts for max lines)
		/// @param message Message to print
		/// @param additional Additional arguments to print
		/// @return Success/fail state of the controller printing
		template <typename... T>
		bool tell(const std::uint8_t line, const string& message, T&&... additional) {
			if (line > MAX_CONTROLLER_LINES) {
				return false;
			}

			master->print(line, CONTROLLER_TXT_START_COL, message.c_str(), additional...);
			return true;
		}

		AbstractChassis& getChassis() {
			return *chassis;
		}

		pros::Controller& getMaster() {
			return *master;
		}

		virtual void opControl() = 0;

		virtual void postAuton() {}
		virtual void skillsPrep() {}

		virtual ~AbstractComponent() = default;
	}; // class ChassisComponent

	class AbstractMech : public AbstractComponent {
	private:
		bool engaged = false;

		pros::adi::DigitalOut piston;
	protected:
	public:
		/// @brief Args for abstract mech object
		/// @param abstractComponentArgs Args for AbstractComponent object
		/// @param pistonPort Port for piston
		struct AbstractMechArgs {
			AbstractComponentArgs abstractComponentArgs;
			char pistonPort;
		};

		/// @brief Creates abstract mech object
		/// @param args Args for abstract mech object (check args struct for more info)
		AbstractMech(AbstractMechArgs args) : 
			AbstractComponent(args.abstractComponentArgs),
			piston(args.pistonPort) {};

		/// @brief Sets actuation value of piston
		/// @param value Value to set the piston to
		void actuate(bool value) {
			piston.set_value(value);
			engaged = value;
		}

		/// @brief Gets the piston object
		/// @return PROS ADI DigitalOut object for piston
		pros::adi::DigitalOut& getPiston() {
			return piston;
		}

		/// @brief Gets the engaged state of the mech
		/// @return Engaged state of the mech
		bool getEngaged() {
			return engaged;
		}

		/// @brief Sets the engaged state of the mech (DO NOT USE UNLESS ABSOLUTELY NECESSARY!!)
		/// @param value Value to set the engaged state to (DO NOT USE UNLESS ABSOLUTELY NECESSARY!!)
		void doNotUseThisInYourLifeEver_ForceSetEngaged(bool value) {
			engaged = value;
		}

		virtual ~AbstractMech() = default;
	}; // class AbstractMech

	/// @brief Abstract motor group class for if you want a custom motor group class
	class AbstractMG : public AbstractComponent {
	private:		
	protected:
		const pros::MotorGroup mg;
	public:
		struct Speeds {
			int fwd = 10000;
			int back = -10000;
		};

		/// @brief Args for abstract motor group object
		/// @param abstractComponentArgs Args for AbstractComponent object
		/// @param ports Vector of ports for motor group
		struct AbstractMGArgs {
			AbstractComponentArgs abstractComponentArgs;
			MGPorts ports;
		};

		Speeds speeds = {};
		bool outputSpeeds;

		/// @brief Constructor for abstract motor group object
		/// @param args Args for abstract motor group object (check args struct for more info)
		AbstractMG(AbstractMGArgs args) : 
			AbstractComponent(args.abstractComponentArgs),
			mg(args.ports) {};

		/// @brief Move the motors in the specified direction according to speeds.
		/// @param on Whether to stop or start the motors.
		/// @param directionForward Direction to go.
		void move(bool on, bool directionForward = true) {
			on = canMove(on);

			if (on) {
				if (directionForward) {
					mg.move_velocity(speeds.fwd);
					if (outputSpeeds) {
						//pros::lcd::print(2, "motor going!!");
					}
				} else {
					mg.move_velocity(speeds.back);
					if (outputSpeeds) {
						//pros::lcd::print(2, "motor not going :(");
					}
				}
			} else {
				mg.move_velocity(0);
			}
		}

		virtual bool canMove(bool on) = 0;

		virtual ~AbstractMG() = default;
	}; // class AbstractMG

	/// @brief Class which manages button presses (will run function on up, down and hold states of given button)
	class BtnManager : public AbstractComponent {
	private:
		bool lastPressed = false;

		void handleBtnPressed() {
			if (lastPressed) {
				for (VoidFunc& func: actionInfo.holdFuncs) {
					func();
				}
			} else {
				for (VoidFunc& func: actionInfo.downFuncs) {
					func();
				}
			}
		}
	protected:
	public:
		/// @brief Struct for action info for button manager object
		/// @param upFuncs Functions that are run once when up state is reached
		/// @param downFuncs Functions that are run once when down state is reached
		/// @param holdFuncs Functions to continuously run on hold state
		/// @param btn Button to manage
		struct ActionInfo {
			pros::controller_digital_e_t btn;
			VoidFuncVector downFuncs = {};
			VoidFuncVector upFuncs = {};
			VoidFuncVector holdFuncs = {};
		};

		ActionInfo actionInfo;

		/// @brief Args for button manager object
		/// @param abstractComponentArgs Args for AbstractComponent object
		/// @param actionInfo Action info for button manager object
		struct BtnManagerArgs {
			AbstractComponentArgs abstractComponentArgs;
			ActionInfo actionInfo;
		};

		/// @brief Creates button manager object
		/// @param args Args for button manager object (check args struct for more info)
		BtnManager(BtnManagerArgs args) : 
			AbstractComponent(args.abstractComponentArgs),
			actionInfo(args.actionInfo) {};

		void opControl() override {
			bool btnPressed = master->get_digital(actionInfo.btn);
			
			// down: !lastPressed && btnPressed
			// up: lastPressed && !btnPressed
			// hold: lastPressed && btnPressed

			if (btnPressed) {
				handleBtnPressed();
			} else if (lastPressed) {
				for (VoidFunc& func: actionInfo.upFuncs) {
					func();
				}
			}

			lastPressed = btnPressed;
		}

		bool getLastPressed() {
			return lastPressed;
		}
	};

	/// @brief Class for a toggle on the controller
	class BiToggle { // Don't need to derive from AbstractComponent because no need for Chassis pointer
	public:
		enum class State {
			OFF,
			FWD,
			BACK
		};
	private:
		AbstractMG* component;

		pros::Controller* master;
		
		State state = State::OFF;
		bool isNewPress = true;

		void moveState(State target) {
			if (!isNewPress) {
				return;
			}

			switch (target) {
				case State::OFF:
					component->move(false);
					break;
				case State::FWD:
					component->move(true);
					break;
				case State::BACK:
					component->move(true, false);
					break;
			}
			
			state = target;
		}

		void handleFwdBtn() {
			if (state == State::FWD) {
				moveState(State::OFF);
				pros::lcd::print(1, "Fwd pressed AND GOING OFF");
			} else {
				moveState(State::FWD);
				pros::lcd::print(1, "Fwd pressed AND GOING FWD");
			}
		}

		void handleBackBtn() {
			if (state == State::BACK) {
				moveState(State::OFF);
				pros::lcd::print(1, "Back pressed AND GOING OFF");
			} else {
				moveState(State::BACK);
				pros::lcd::print(1, "Back pressed AND GOING BACK");
			}
		}
	protected:
	public:
		/// @brief Struct for buttons for BiToggle object
		/// @param fwd Button for forward
		/// @param back Button for backward
		struct Buttons {
			pros::controller_digital_e_t fwd;
			pros::controller_digital_e_t back;
		};

		/// @brief Args for BiToggle object
		/// @param component Component to toggle
		/// @param btns Buttons for toggle
		struct BiToggleArgs {
			AbstractMG* component;
			Buttons btns;
		};

		Buttons btns;

		/// @brief Creates BiToggle object
		/// @param args Args for BiToggle object (check args struct for more info)
		BiToggle(BiToggleArgs args) : 
			component(args.component),
			btns(args.btns),
			master(&args.component->getMaster()) {};

		void opControl() {
			bool fwdPressed = master->get_digital(btns.fwd);
			bool backPressed = master->get_digital(btns.back);

			pros::lcd::print(3, ("FWD: " + std::to_string(fwdPressed)).c_str());
			pros::lcd::print(4, ("BACK: " + std::to_string(backPressed)).c_str());

			if (fwdPressed && backPressed) {
				// Don't do anything if both are pressed
				// TODO: test whether the return works
				// because we need it for backwards motor movement
				return;
			}

			if (fwdPressed) {
				handleFwdBtn();
				isNewPress = false;
				return;
			}

			if (backPressed) {
				handleBackBtn();
				isNewPress = false;
				return;
			}

			isNewPress = true;
		}

		void setState(State target) {
			state = target;
		}

		State getState() {
			return state;
		}
	}; // class BiToggle

	/// @brief Class for driver control
	class Drivetrain : public AbstractComponent {
	public:
		/// @brief Enum for different driver control modes
		enum class DriveControlMode {
			ARCADE,
			TANK
		};
	private:
		// Coefficients for turning in driver control
		struct TurnCoefficients {
			float left;
			float right;
		};

		pros::MotorGroup left_mg;
		pros::MotorGroup right_mg;
		int leftMgSize;
		int rightMgSize;

		// PID stuff
		// TODO: Move to separate class
		pros::IMU imu;

		DriveControlMode driveControlMode;

		std::function<void()> driveControl;

		void bindDriveControl(void (Drivetrain::*driveFunc)()) {
			driveControl = std::bind(driveFunc, this);
		}

		/// @brief Calibrates the IMU
		void calibrateIMU(bool blocking = true) {
			imu.reset(blocking);
			imu.tare();
		}
	protected:
	public:
		/// @brief Struct for different driver control speeds on arcade control
		/// @param turnSpeed Multiplier for only turning
		/// @param forwardBackSpeed Multiplier for only forward/backward
		/// @param arcSpeed Multiplier of opposite turn for when turning and moving laterally at the same time
		// (higher value means less lateral movement)
		struct DriveControlSpeed {
		private:
			float forwardBackSpeed;
			float maxLateral;
		public:
			static constexpr float controllerMax = 127;

			float turnSpeed;
			float arcSpeed;

			/// @brief Sets the forward/backward speed
			/// @param speed Speed to set the forward/backward speed to
			// (Also prepares maxLateral for arc movement)
			void setForwardBackSpeed(float speed, float maxTolerance = 1) {
				forwardBackSpeed = speed;
				maxLateral = speed * controllerMax + maxTolerance;
			}

			/// @brief Gets the forward/backward speed
			/// @return Forward/backward speed
			float getForwardBackSpeed() {
				return forwardBackSpeed;
			}

			/// @brief Gets the max lateral movement
			/// @return Max lateral movement
			float getMaxLateral() {
				return maxLateral;
			}

			// lower arc speed is lower turning

			DriveControlSpeed(float turnSpeed = 1, float forwardBackSpeed = 1, float arcSpeed = 0.7) :
				turnSpeed(turnSpeed), 
				arcSpeed(arcSpeed) {
					setForwardBackSpeed(forwardBackSpeed);
			}
		};


		/// @brief Ports for the drivetrain
		/// @param leftPorts Vector of ports for left motors
		/// @param rightPorts Vector of ports for right motors
		struct DrivetrainPorts {
			vector<std::int8_t> left;
			vector<std::int8_t> right;
			std::int8_t imuPort;
		};

		/// @brief Args for drivetrain object
		/// @param abstractComponentArgs Args for AbstractComponent object
		struct DrivetrainArgs {
			AbstractComponentArgs abstractComponentArgs;
			DrivetrainPorts ports;
		};

		using ArgsType = DrivetrainArgs;

		/// @brief Struct for PID options (self-explanatory - timeLimit in MS)
		struct PIDOptions {
			double kP;
			double kI;
			double kD;
			double errorThreshold;
			float timeLimit;
		};

		DriveControlSpeed driveControlSpeed = {};

		bool preventBackMove = false;

		std::int32_t defaultMoveVelocity = 1024;
		std::int8_t maxRelativeError = 5;

		std::int16_t maxTurnVelocity = 60;
		float minTurnThreshold = 5;

		float relativeMovementCoefficient = 14.2857;
		float voltMovementCoefficient = 1;

		float maxVoltage = 12000;

		// Motor builtin encoder inchesPerTick
		double inchesPerTick = 0.034034;
		//double inchesPerTick = 1.0;

		// Odom wheel encoder inchesPerTick
		// TODO: Calculate this
		double odomIPT = 0.0;

		uint32_t moveDelayMs = 2;

		// right is positive
		// left is negative
		int pidInvertTurn = 1;
		float pidReductionFactor = 1.9;

		float arcDeadband = 30;

		/// @brief Sets the brake mode for each motor group
		/// @param mode Brake mode to set the motors toS
		void setBrakeModes(pros::motor_brake_mode_e_t mode) {
			left_mg.set_brake_mode_all(mode);
			right_mg.set_brake_mode_all(mode);
		}

		Drivetrain(DrivetrainArgs args) : 
			AbstractComponent(args.abstractComponentArgs),
			left_mg(args.ports.left),
			right_mg(args.ports.right),
			imu(args.ports.imuPort),
			leftMgSize(args.ports.left.size()),
			rightMgSize(args.ports.right.size()) {
				setDriveControlMode();
				calibrateIMU();
			};
	private:
		void prepareArcadeLateral(float& lateral) {
			// Change to negative to invert
			lateral *= -1;

			// Clamp the range to above 0 only to remove back movement
			if (preventBackMove && (lateral < 0)) {
				lateral = 0;
			}
		}

		// Calculate the movement of the robot when turning and moving laterally at the same time
		void calculateArcMovement(TurnCoefficients& turnCoeffs, float lateral, float turn, float maxLateralTolerance = 1) {
			if (std::fabs(lateral) < arcDeadband) {
				return;
			}

			// 0-1 range of percentage of lateral movement against max possible lateral movement
			float lateralCompensation = lateral / driveControlSpeed.getMaxLateral();
			// Decrease the turn speed when moving laterally (higher turn should be higher turnDecrease)
			float dynamicArcSpeed = (lateral < 0) ? driveControlSpeed.arcSpeed : 1;

			float turnDecrease = 1 * turn * lateralCompensation * dynamicArcSpeed;

			if (lateral > 0) {
				turnDecrease *= turn * 0.0001;
			}

			if (turn > 0) { // Turning to right so we decrease the left MG
				if (lateral < 0) {
					turnCoeffs.left -= turnDecrease;
				} else {
					turnCoeffs.left += turnDecrease;
				}
			} else { // Turning to left so we decrease the right MG
				if (lateral > 0) {
					turnCoeffs.right -= turnDecrease;
				} else {
					turnCoeffs.right += turnDecrease;
				}
			}

			pros::lcd::print(6, ("TD, dAS:, lComp: " + std::to_string(turnDecrease) + ", " + std::to_string(dynamicArcSpeed) + ", " + std::to_string(lateralCompensation)).c_str());
		}

		TurnCoefficients calculateArcadeTurns(float turn, float lateral) {
			turn *= 1;

			TurnCoefficients turnCoeffs = {turn, turn};

			// Allow for arc movement
			calculateArcMovement(turnCoeffs, lateral, turn);

			return turnCoeffs;
		}
	public:
		void opControl() override {
			driveControl();
		}

		/// @brief Arcade control for drive control (recommended to use opControl instead)
		void arcadeControl() {
			float lateral = master->get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
			float turn = master->get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick

			prepareArcadeLateral(lateral);

			TurnCoefficients turnCoeffs = calculateArcadeTurns(turn, lateral);
			
			pros::lcd::print(1, ("T, L:" + std::to_string(turn) + ", " + std::to_string(lateral)).c_str());

			// Calculate speeds
			lateral *= driveControlSpeed.getForwardBackSpeed();

			turnCoeffs.left *= driveControlSpeed.turnSpeed;
			turnCoeffs.right *= driveControlSpeed.turnSpeed;

			// Ensure voltages are within correct ranges
			std::int32_t left_voltage = prepareMoveVoltage(lateral - turnCoeffs.left);
			std::int32_t right_voltage = prepareMoveVoltage(lateral + turnCoeffs.right);

			pros::lcd::print(2, ("L/R COEF: " + std::to_string(turnCoeffs.left) + ", " + std::to_string(turnCoeffs.right)).c_str());
			pros::lcd::print(7, ("LEFT/RIGHT: " + std::to_string(left_voltage) + ", " + std::to_string(right_voltage)).c_str());

			left_mg.move(left_voltage);
			right_mg.move(right_voltage);
		}

		void tankControl() {
			float left = master->get_analog(ANALOG_LEFT_Y);
			float right = master->get_analog(ANALOG_RIGHT_Y);

			left_mg.move(prepareMoveVoltage(left));
			right_mg.move(prepareMoveVoltage(right));
		}

		/// @brief Fallback control that DriveControlMode switch statement defaults to.
		void fallbackControl() {
			arcadeControl();
		}

		/// @brief Sets the driver control mode
		/// @param mode Mode to set the driver control to
		void setDriveControlMode(DriveControlMode mode = DriveControlMode::TANK) {
			driveControlMode = mode;

			switch (driveControlMode) {
				case DriveControlMode::ARCADE:
					bindDriveControl(&Drivetrain::arcadeControl);
					break;
				case DriveControlMode::TANK:
					bindDriveControl(&Drivetrain::tankControl);
					break;
				default:
					bindDriveControl(&Drivetrain::fallbackControl);
					break;
			}
		}

		/// @brief Moves the motors at a specific voltage
		/// @param leftVoltage Voltage for left motor
		/// @param rightVoltage Voltage for right motor
		void moveVoltage(std::int16_t leftVoltage, std::int16_t rightVoltage) {
			left_mg.move_voltage(leftVoltage);
			right_mg.move_voltage(rightVoltage);
			pros::lcd::print(0, ("Left Voltage: " + std::to_string(leftVoltage)).c_str());
			pros::lcd::print(1, ("Right Voltage: " + std::to_string(rightVoltage)).c_str());
		}

		/// @brief Moves the motors at a single voltage
		/// @param voltage Voltage to move the motors at
		void moveSingleVoltage(std::int16_t voltage) {
			moveVoltage(voltage, voltage);
		}

		/// @brief Sets movement velocity
		/// @param leftVoltage Voltage for left motor
		/// @param rightVoltage Voltage for right motor
		void moveVelocity(std::int32_t leftVoltage, std::int32_t rightVoltage) {
			left_mg.move_velocity(leftVoltage);
			right_mg.move_velocity(rightVoltage);
		}

		/// @brief Moves the motors at a single velocity
		/// @param voltage Voltage to move the motors at
		void moveSingleVelocity(std::int32_t voltage) {
			moveVelocity(voltage, voltage);
		}

		/// @brief Stops moving the motors
		void moveStop() {
			moveSingleVelocity(0);
		}

		/// @brief Tares the motors
		void tareMotors() {
			left_mg.tare_position();
			right_mg.tare_position();
		}

		/// @brief Move to relative position
		/// @param pos Position to move to in CM
		void moveRelPos(double pos) {
			tareMotors();

			pos *= relativeMovementCoefficient;

			left_mg.move_relative(pos, defaultMoveVelocity);
			right_mg.move_relative(pos, defaultMoveVelocity);

			double lowerError = pos - maxRelativeError;

			while ((
				!(left_mg.get_position() > lowerError)
			) && (
				!(right_mg.get_position() > lowerError)
			)) {
				pros::delay(moveDelayMs);
			}

			moveStop();
		}

		/// @brief Turn to a specific angle
		/// @param angle Angle to turn to
		void turnTo(double angle) {
			imu.tare();

			double currentHeading = imu.get_heading();
			double angleDifference = normaliseAngle(angle - currentHeading);

			std::int16_t turnDirection = (angleDifference > 0) ? maxTurnVelocity : -maxTurnVelocity;
			//std::int16_t turnDirection = maxTurnVelocity;
			
			left_mg.move_velocity(turnDirection);
			right_mg.move_velocity(-turnDirection);

			while (true) {
				currentHeading = imu.get_heading();
				angleDifference = normaliseAngle(angle - currentHeading);
				
				if (std::fabs(angleDifference) <= minTurnThreshold) {
					break;
				}
				
				pros::lcd::set_text(2, "Current heading: " + std::to_string(currentHeading));
				pros::delay(moveDelayMs);
			}

			moveStop();
		}

		/// @brief Turn to a specific angle with a delay
		/// @param angle Angle to turn to
		/// @param delayMs Delay in milliseconds
		void turnDelay(bool direction, std::uint32_t delayMs, std::int16_t delayTurnVelocity = 60) {
			std::int16_t turnDirection = (direction) ? delayTurnVelocity : -delayTurnVelocity;

			left_mg.move_velocity(turnDirection);
			right_mg.move_velocity(-turnDirection);

			pros::delay(delayMs);

			moveStop();
		}

		/// @brief Move forward for a certain number of milliseconds
		/// @param delayMs Number of milliseconds to move forward
		/// @param left Whether to move the left motor
		/// @param right Whether to move the right motor
		void moveDelay(std::uint32_t delayMs, bool forward = true, std::int16_t delayMoveVelocity = 60) {
			if (forward) {
				moveSingleVelocity(-delayMoveVelocity);
			} else {
				moveSingleVelocity(delayMoveVelocity);
			}

			pros::delay(delayMs);
			moveStop();
		}

		double getHeading() {
			return imu.get_heading();
		}

		double getAvgMotorPos() {
			/*vector<double> leftPos = left_mg.get_position_all();
			vector<double> rightPos = right_mg.get_position_all();

			double avgLeft = calcMeanFromVector(leftPos, leftMgSize);
			double avgRight = calcMeanFromVector(rightPos, rightMgSize);*/

			double avgPos = (left_mg.get_position() + right_mg.get_position()) / 2;

			return avgPos;
		}

		// TODO: Generic PID function that we can apply to PIDTurn and PIDMove
		// maybe make a class for this? if it gets too complicated
		// but that would also require refactoring Drivetrain to have an AbstractDrivetrain
		// parent to avoid cyclic dependencies

		// WARNING: do NOT use relativeMovementCoefficient for PID functions
		// as this does not account for acceleration/deceleration
		// it's only for simple movement (phased out by PID & PIDOptions struct)

		/// @brief Turn to a specific angle using PID
		/// @param angle Angle to move to (PASS IN THE RANGE OF -180 TO 180 for left and right)
		// TODO: Tuning required
		void PIDTurn(double angle, float reductionFactor = 2, PIDOptions options = {
			0.3, 0.0, 0.7, 1, 6000
		}) {
			imu.tare();
			angle = naiveNormaliseAngle(angle);

			angle *= pidInvertTurn;

			angle /= -1;

			bool anglePositive = angle > 0;
			bool turn180 = false;

			// IMU already tared so we don't need to get the current heading
			float error = angle;
			float lastError = 0;
			float derivative = 0;
			float integral = 0;

			float out = 0;
			float trueHeading = 0;

			float maxThreshold = 180 - options.errorThreshold;

			float maxCycles = options.timeLimit / moveDelayMs;
			float cycles = 0;

			if (std::fabs(angle) >= 180) {
				turn180 = true;
			}

			pros::lcd::print(3, "PIDTurn Start");

			// with turning you just wanna move the other MG at negative of the MG of the direction
			// which u wanna turn to

			while (true) {
				trueHeading = std::fmod((imu.get_heading() + 180), 360) - 180;
				error = angle - trueHeading;

				integral += error;
				// Anti windup
				if (std::fabs(error) < options.errorThreshold) {
					integral = 0;
				}

				derivative = error - lastError;
				out = (options.kP * error) + (options.kI * integral) + (options.kD * derivative);
				lastError = error;

				out *= 1000; // convert to mV
				out = std::clamp(out, -maxVoltage, maxVoltage);
				out /= pidReductionFactor;
				moveVoltage(-out, out);

				pros::lcd::print(5, ("PIDTurn Out: " + std::to_string(out)).c_str());
				pros::lcd::print(7, ("PIDTurn Error: " + std::to_string(error)).c_str());
				pros::lcd::print(6, ("PIDTurn True Heading: " + std::to_string(imu.get_heading())).c_str());

				if (std::fabs(error) <= options.errorThreshold) {
					break;
				}

				// 180 degree turning
				if (std::fabs(trueHeading) >= maxThreshold) {
					break;
				}

				// TODO: refactor checks in prod
				if (std::fabs(out) < 100) {
					pros::lcd::print(4, "PIDTurn Out too low");
				}

				if (cycles >= maxCycles) {
					pros::lcd::print(4, "PIDTurn Time limit reached");
					break;
				}

				pros::delay(moveDelayMs);
				cycles++;
			}

			pros::lcd::print(2, "PIDTurn End");
			moveStop();
		}

		// think about arc motion, odometry, etc.
		// the key thing is PID.
		// TUNING REQUIRED!!!

		/// @brief Move to a specific position using PID
		/// @param pos Position to move to in inches (use negative for backward)
		// TODO: Tuning required
		void PIDMove(double pos, float reductionFactor = 2, PIDOptions options = {
			0.19, 0.0, 0.4, 3, 6000
		}) {
			// TODO: Consider adding odometry wheels as the current motor encoders
			// can be unreliable for long distances or just dont tare the motors
			tareMotors();

			pos /= inchesPerTick;
			pos *= -1;

			float error = pos;
			float motorPos = 0;
			float lastError = 0;
			float derivative = 0;
			float integral = 0;
			float out = 0;

			float maxCycles = options.timeLimit / moveDelayMs;
			float cycles = 0;

			// with moving you just wanna move both MGsat the same speed

			while (true) {
				// get avg error
				motorPos = right_mg.get_position();
				error = pos - motorPos;

				integral += error;
				// Anti windup
				if (std::fabs(error) < options.errorThreshold) {
					integral = 0;
				}

				derivative = error - lastError;
				out = (options.kP * error) + (options.kI * integral) + (options.kD * derivative);
				lastError = error;

				out *= 1000; // convert to mV
				out = std::clamp(out, -maxVoltage, maxVoltage);
				out /= pidReductionFactor;
				moveSingleVoltage(out);

				if (std::fabs(error) <= options.errorThreshold) {
					break;
				}

				pros::lcd::print(4, ("PIDMove Motor Pos: " + std::to_string(motorPos)).c_str());
				pros::lcd::print(5, ("PIDMove Out: " + std::to_string(out)).c_str());
				pros::lcd::print(7, ("PIDMove Error: " + std::to_string(error)).c_str());

				if (cycles >= maxCycles) {
					pros::lcd::print(4, "PIDMove Time limit reached");
					break;
				}

				pros::delay(moveDelayMs);
				cycles++;
			}

			moveStop();
		}

		/// @brief Gets the left motor group
		pros::MotorGroup& getLeftMotorGroup() {
			return left_mg;
		}

		/// @brief Gets the right motor group
		pros::MotorGroup& getRightMotorGroup() {
			return right_mg;
		}		
	}; // class Drivetrain

	/// @brief Class for timer object to control timings
	class Timer : public AbstractComponent {
		private:
			int waitTime = 20;

			void sleep(int time) {
				pros::delay(time);
			}
		protected:
		public:
			/// @brief Args for timer object
			/// @param abstractComponentArgs Args for AbstractComponent object
			struct TimerArgs {
				AbstractComponentArgs abstractComponentArgs;
			};

			using ArgsType = TimerArgs;

			/// @brief Constructor for timer object
			/// @param args Args for timer object (see args struct for more info)
			Timer(TimerArgs args) : 
				AbstractComponent(args.abstractComponentArgs) {};

			/// @brief Gets the wait time for the timer
			/// @return Time to wait for in milliseconds
			int getWaitTime() {
				return waitTime;
			}

			void opControl() override {
				sleep(waitTime);
			}
	}; // class Timer

	/// @brief Class which manages all components
	class ComponentManager : public AbstractComponent {
	private:
	protected:
	public:
		Drivetrain dvt;

		Timer timer;
		
		// All components are stored in this vector
		vector<AbstractComponent*> components;

		/// @brief Args for component manager object passed to the chassis, such as ports
		/// @param dvtPorts Ports for drivetrain
		struct ComponentManagerUserArgs {
			Drivetrain::DrivetrainPorts dvtPorts;
		};

		/// @brief Args for component manager object
		/// @param aca Args for AbstractComponent object
		/// @param user Args for component manager object passed to the chassis
		struct ComponentManagerArgs {
			AbstractComponentArgs aca;
			ComponentManagerUserArgs user;
		};

		/// @brief Constructor for component manager object
		/// @param args Args for component manager object (see args struct for more info)
		ComponentManager(ComponentManagerArgs args) : 
			AbstractComponent(args.aca),

			dvt({args.aca, args.user.dvtPorts}),	
			timer({args.aca}) { 												// Add component pointers to vector
				// MUST BE DONE AFTER INITIALISATION not BEFORE because of pointer issues
				components = {
					&dvt,
					&timer
				};
			};

		// Nice and simple :) definitely better than having to call each component individually
		void opControl() override {
			for (AbstractComponent* component : components) {
				component->opControl();
			}
		}

		void skillsPrep() override {
			for (AbstractComponent* component : components) {
				component->skillsPrep();
			}
		}

		void postAuton() override {
			for (AbstractComponent* component : components) {
				component->postAuton();
			}
		}
	}; // class ComponentManager

	/// @brief Abstract class for auton e.g. match or skills autonomous
	class AbstractAuton {
	private:
	protected:
		ComponentManager* cm;
	public:
		/// @brief Args for auton object
		/// @param cm Component manager object
		struct AutonArgs {
			ComponentManager* cm;
		};

		/// @brief Creates auton object
		/// @param args Args for auton object (check args struct for more info)
		AbstractAuton(AutonArgs args) : 
			cm(args.cm) {};

		/// @brief Runs the auton
		virtual void run() = 0;

		virtual ~AbstractAuton() = default;
	}; // class AbstractAuton

	class MatchAuton : public AbstractAuton {
	private:
		void defaultAuton() {

		}

		void testRight90() {
			
		}

		void testFwd4Tiles() {

		}
	protected:
	public:
		/// @brief Args for match auton object
		/// @param autonArgs Args for auton object
		struct MatchAutonArgs {
			AutonArgs autonArgs;
		};

		/// @brief Creates match auton object
		/// @param args Args for match auton object (check args struct for more info)
		MatchAuton(MatchAutonArgs args) : 
			AbstractAuton(args.autonArgs) {};

		// TODO: Implement
		void run() override {
			defaultAuton();
			//testRight90();
			//testFwd4Tiles();
		}
	}; // class MatchAuton

	class SkillsAuton : public AbstractAuton {
	private:
		void sector1() {

		}
		
		void sector2() {

		}
	protected:
	public:
		/// @brief Args for skills auton object
		/// @param autonArgs Args for auton object
		struct SkillsAutonArgs {
			AutonArgs autonArgs;
		};

		/// @brief Creates skills auton object
		/// @param args Args for skills auton object (check args struct for more info)
		SkillsAuton(SkillsAutonArgs args) : 
			AbstractAuton(args.autonArgs) {};

		void run() override {
			cm->tell(0, "Skills auton running");

			sector1();
			sector2();
		}
	}; // class SkillsAuton

	/// @brief Chassis class for controlling auton/driver control
	class Chassis : public AbstractChassis {
	private:
	protected:
	public:
		/// @brief Args for chassis object
		/// @param cmUserArgs Args for component manager object
		struct ChassisArgs {
			ComponentManager::ComponentManagerUserArgs cmUserArgs;
		};

		ComponentManager cm;

		MatchAuton matchAutonManager;
		SkillsAuton skillsAutonManager;

		/// @brief Creates chassis object
		/// @param args Args for chassis object (check args struct for more info)
		Chassis(ChassisArgs args) : 
			AbstractChassis(),
			cm({this, args.cmUserArgs}),
			matchAutonManager({&cm}),
			skillsAutonManager({&cm}) {};

		/// @brief Runs the opcontrol functions for each component
		void opControl() override {
			cm.opControl();
		}

		/// @brief Auton function for the chassis
		// 1000 = 70cm
		void auton() override {
			matchAutonManager.run();
		}

		/// @brief Skills auton function for the chassis
		void skillsAuton() override {
			skillsAutonManager.run();
		}

		/// @brief Skills preparation for opcontrol on the chassis
		void skillsPrep() override {
			// We need to run postAuton() first because these are what would prep for opcontrol normally
			cm.skillsPrep();
		}

		void postAuton() override {
			cm.postAuton();
		}
	}; // class Chassis

	/// @brief Convert vector of ints to string. For displaying on the LCD/debugging
	/// @param vec Vector to convert
	/// @param delimiter Delimiter to separate elements
	template <typename T>
	string vectorToString(vector<T>& vec, string delimiter) {
		int vecSize = vec.size();
		int vecSizeMinusOne = vecSize - 1;
		std::ostringstream oss;

		oss << "{";
		for (int i = 0; i < vecSize; i++) {
			oss << vec[i];
			if (i < vecSizeMinusOne) {
				oss << delimiter;
			}
		}
		oss << "}";

		return oss.str();
	}

	/// @brief Assert that a value is arithmetic
	/// @param val Value to assert
	template <typename T>
	void assertArithmetic(const T val) {
		static_assert(std::is_arithmetic<T>::value, "Value must be arithmetic");
	}

	/// @brief Checks whether a given color channel is within tolerance.
	/// @param channel Color to check
	/// @param target Target colour which the colour should be
	/// @return Whether the channel is within tolerance
	bool channelWithinTolerance(const float& channel, const float& target, const float& tolerance = 5) {
		return std::fabs(channel - target) <= tolerance;
	}

	std::int32_t prepareMoveVoltage(float raw) {
		// Round the number to the nearest integer
		raw = std::round(raw);

		std::int32_t voltage = static_cast<std::int32_t>(raw);
		voltage = std::clamp(voltage, MotorBounds::MOVE_MIN, MotorBounds::MOVE_MAX);

		return voltage;
	}

	/// @brief Assert that a number is between two values
	/// @param num Number to assert
	/// @param min Minimum value
	/// @param max Maximum value
	template <typename T>
	bool isNumBetween(T num, T min, T max) {
		assertArithmetic(num);

		return ((num >= min) && (num <= max));
	}

	/// @brief Normalise an angle to the range [-180, 180]
	/// @param angle Angle to normalise
	template <typename T>
	T normaliseAngle(T angle) {
		assertArithmetic(angle);

		if (angle > 180) {
			angle -= 360;
		} else if (angle < -180) {
			angle += 360;
		}

		return angle;
	}

	/// @brief Naively normalise an angle to the range [-180, 180] by simply clamping the value
	/// @param angle Angle to normalise
	template <typename T>
	T naiveNormaliseAngle(T angle) {
		assertArithmetic(angle);

		angle = std::clamp(angle, -180.0, 180.0);

		return angle;
	}

	/// @brief Calculate the mean of a vector
	/// @param vec Vector to calculate the mean of
	/// @param size Size of the vector
	/// @return Mean of the vector (type T)
	template <typename T>
	T calcMeanFromVector(const vector<T>& vec, int size) {
		T sum = std::accumulate(vec.begin(), vec.end(), 0);
		T mean = sum / size;

		return mean;
	}

	/// @brief Calculate the mean of a vector
	/// @param vec Vector to calculate the mean of
	/// @return Mean of the vector (type T)
	template <typename T>
	T calcMeanFromVector(const vector<T>& vec) {
		int size = vec.size();
		T sum = std::accumulate(vec.begin(), vec.end(), 0);
		T mean = sum / size;

		return mean;
	}

	/*/// @brief Get all the values of an enum class into a vector
	template <typename T>
	vector<T> getAllValues() {
		vector<T> values;
		constexpr int max = static_cast<int>(T::_MAX);
		values.reserve(max);

		for (int i = 0; i < max; i++) {
			values.push_back(static_cast<T>(i));
		}

		return values;
	}

	/// @brief Fill a map with default values for an enum class (see below function def for example use case)
	/// @param map Map to fill
	template <typename E, typename V>
	void fillMapWithEnum(map<E, V>& map) {
		vector<E> values = getAllValues<E>();
		E defaultValue = V();

		for (E value : values) {
			map[value] = defaultValue;
		}
	}*/
	// example use case
	//fillMapWithEnum<pros::controller_digital_e_t, bool>(map);
} // namespace hyper

// Global variables

// DONT say just "chassis" because certain class properties have the same name
hyper::AbstractChassis* currentChassis;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void initDefaultChassis() {
	static hyper::Chassis defaultChassis({
		{{LEFT_DRIVE_PORTS, RIGHT_DRIVE_PORTS, IMU_PORT}, // Drivetrain args
	}});
	
	currentChassis = &defaultChassis;
}

void initialize() {
	pros::lcd::initialize();

	INIT_CHASSIS();
}

void disabled() {

}

void competition_initialize() {

}

void autonomous() {
	if (DO_MATCH_AUTON) {
		currentChassis->auton();
	} else if (DO_SKILLS_AUTON) {
		currentChassis->skillsAuton();
	}
}

void preControl() {
	pros::lcd::set_text(0, "> 1408Hyper mainControl ready");

	bool inComp = pros::competition::is_connected();

	// competition auton test safeguard
	if (!inComp) {
		autonomous();
	}

	if (DO_SKILLS_PREP) {
		currentChassis->skillsPrep();
	}

	// only do post auton if we are not in skills prep
	if (DO_POST_AUTON) {
		currentChassis->postAuton();
	}
}

void mainloopControl() {
	bool opControlRunning = DO_OP_CONTROL;
	// Chassis control loop
	while (opControlRunning) {
		// Chassis opcontrol
		currentChassis->opControl();
	}
}

void mainControl() {
	preControl();
	mainloopControl();
}

void opcontrol() {
	CURRENT_OPCONTROL();
}

// hello copilot how are you doing
// i am doing well thank you for asking
// what do you think of my code
// i think it is very good
// is there anything that you would add to my code?
// i would add more comments
// what is your favourite programming language
// i like c++ the most

// anti quick make nothing comment thingy
// aaaaa
