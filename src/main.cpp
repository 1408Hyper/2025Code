// includes/usings are all in main.h
#include "main.h"

// uses ISO/C++20 standard
// added libraries/includes:
// pros

/// @brief Hyper namespace for all custom classes and functions
namespace hyper {
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
		static constexpr float SCALE_MIN = 0;
		static constexpr float SCALE_MAX = 1;

		static constexpr std::int32_t MOVE_MIN = -127;
		static constexpr std::int32_t MOVE_MAX = 127;

		static constexpr float MILLIVOLT_MAX = 12000;
	};

	/// @brief Base struct for any values on the horizontal axis
	/// @param left Left side value
	/// @param right Right side value
	struct Horizontal {
		float left;
		float right;
	};

	/// @brief Vertical axis struct
	/// @param low Low value
	/// @param high High value
	struct Vertical {
		float low;
		float high;
	};

	// Helper functions

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

	std::int32_t prepareMoveSpeed(float raw) {
		// Round the number to the nearest integer
		raw = std::round(raw);

		std::int32_t speed = static_cast<std::int32_t>(raw);
		speed = std::clamp(speed, MotorBounds::MOVE_MIN, MotorBounds::MOVE_MAX);

		return speed;
	}

	/// @brief Assert that a number is between two values
	/// @param num Number to assert
	/// @param min Minimum value
	/// @param max Maximum value
	template <typename T>
	bool isNumBetween(T num, T min, T max) {
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

	// Class declarations

	/// @brief Abstract chassis class for if you want a custom chassis class
	class AbstractChassis {
	private:
	protected:
		pros::Controller master{pros::E_CONTROLLER_MASTER};
	public:
		/// @brief Creates abstract chassis object
		AbstractChassis() {};

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
	protected:
		AbstractChassis* chassis;

		pros::Controller* master;
	public:
		static constexpr std::uint8_t MAX_BRAIN_LINES = 8;
		static constexpr std::uint8_t MAX_CONTROLLER_LINES = 2;
		static constexpr std::uint8_t CONTROLLER_TXT_START_COL = 0;

		/// @brief Args for AbstractComponent object
		/// @param chassis AbstractChassis derived object to be used for the component
		struct AbstractComponentArgs {
			AbstractChassis* chassis;
		};

		/// @brief Creates AbstractComponent object
		/// @param args Args AbstractComponent object (check args struct for more info)
		AbstractComponent(AbstractComponentArgs args) : 
		chassis(args.chassis),
		master(&args.chassis->getController()) {};

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
	class BiToggle : public AbstractComponent {
	public:
		enum class State {
			OFF,
			FWD,
			BACK
		};
	private:
		AbstractMG* component;
		
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
		/// @param abstractComponentArgs Args for AbstractComponent object
		/// @param component Component to toggle
		/// @param btns Buttons for toggle
		struct BiToggleArgs {
			AbstractComponentArgs abstractComponentArgs;
			AbstractMG* component;
			Buttons btns;
		};

		Buttons btns;

		/// @brief Creates BiToggle object
		/// @param args Args for BiToggle object (check args struct for more info)
		BiToggle(BiToggleArgs args) : 
			AbstractComponent(args.abstractComponentArgs),
			component(args.component),
			btns(args.btns) {};

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

	/// @brief Advanced task scheduling class with thread-safe timing capabilities
	class Timer : public AbstractComponent {
		private:
			// Time passed in milliseconds
			std::uint32_t ms = 0;
			int tickMs = 20;
		protected:
		public:
			// Assuming 1 tick is at least 1 milliseconds, this is at least 24 hours worth of runs
			static constexpr int INFINITE_RUNS = 86400000;
			static constexpr int NO_RUNS = 0;

			/// @brief Basic structure for a scheduled task
			/// @param func Function to run
			/// @param ms Time to wait for in milliseconds
			/// @param runs Number of times to run the function (use INFINITE_RUNS for infinite)
			struct Timeout {
				std::function<void(Timeout& timeout)> func;
				int ms;
				int runs = 1;
			};
		private:
			/// @brief Internal structure for a scheduled task
			/// @param timeout Timeout object
			/// @param nextRun Next ms time to run timeout
			struct InternalTimeout {
				Timeout timeout;
				std::uint32_t nextRun;
			};

			vector<InternalTimeout> timeouts = {};

			void removeExpiredTimeouts() {
				timeouts.erase(
					std::remove_if(timeouts.begin(), timeouts.end(),
						[](const InternalTimeout& ito) {
							return ito.timeout.runs <= NO_RUNS;
						}
					),
					timeouts.end()
				);
			}

			void processTimeouts() {
				removeExpiredTimeouts();

				for (InternalTimeout& ito : timeouts) {
					ito.timeout.runs--;

					if (ito.nextRun <= ms) {
						ito.timeout.func(ito.timeout);
					}
					
					ito.nextRun += ito.timeout.ms;
				}
			}
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
			
			/// @brief Sleep function to be used instead of pros::delay to track time for task scheduling
			/// @param time Time in milliseconds to wait for
			void sleep(int time) {
				pros::delay(time);
				ms += time;
			}

			/// @brief Gets the tick wait time in milliseconds
			/// @return Time to wait for in milliseconds
			int getTickMs() {
				return tickMs;
			}

			/// @brief Gets the current time in milliseconds
			/// @return Current time in milliseconds
			std::uint32_t getTimeMs() {
				return ms;
			}

			/// @brief Setup a new timeout
			/// @param timeout Timeout object to setup
			void setupTimeout(Timeout timeout) {
				std::uint32_t nextRun = ms + timeout.ms;
				InternalTimeout ito = {timeout, nextRun};
				timeouts.push_back(ito);
			}

			void opControl() override {
				sleep(tickMs);
				processTimeouts();
			}
	}; // class Timer

	namespace Drivetrain {
		/// @brief Struct for drivetrain motor groups
		/// @param left Left motor group
		/// @param right Right motor group
		struct DriveMGs {
			pros::MotorGroup left;
			pros::MotorGroup right;

			/// @brief Struct for drive ports
			/// @param leftPorts Ports for left motor group
			/// @param rightPorts Ports for right motor group
			struct DrivePorts {
				MGPorts leftPorts;
				MGPorts rightPorts;
			};

			/// @brief Constructor for DriveMGs object
			/// @param leftPorts Ports for left motor group
			/// @param rightPorts Ports for right motor group
			DriveMGs(DrivePorts drivePorts) : 
				left(drivePorts.leftPorts), right(drivePorts.rightPorts) {};

			/// @brief Set the voltage of the motor groups
			/// @param leftVoltage Voltage to set the left motor group to
			/// @param rightVoltage Voltage to set the right motor group to
			void voltage(int leftVoltage, int rightVoltage) {
				left.move_voltage(leftVoltage);
				right.move_voltage(rightVoltage);
			}

			/// @brief Stop the motor groups by setting their voltage to 0
			void stop() {
				voltage(0, 0);
			}

			/// @brief Set the velocity of the motor groups
			/// @param leftVel Velocity to set the left motor group to
			/// @param rightVel Velocity to set the right motor group to
			void velocity(int leftVel, int rightVel) {
				left.move_velocity(leftVel);
				right.move_velocity(rightVel);
			}

			/// @brief Move the motor groups using the motor.move() function
			/// @param leftSpeed Speed to move the left motor group at
			/// @param rightSpeed Speed to move the right motor group at
			void move(int leftSpeed, int rightSpeed) {
				left.move(leftSpeed);
				right.move(rightSpeed);
			}

			/// @brief Tare the motor groups
			void tare() {
				left.tare_position();
				right.tare_position();
			}

			/// @brief Get the average position of the motor groups (certain wires on our motor are broken so you MUST use this if you want a reliable position)
			/// @return Average position of the motor groups
			double position() {
				return (left.get_position() + right.get_position()) / 2;
			}
		};

		/// @brief Class to control autonomous PID routines for driving
		class DrivePID {
		public:
			pros::Rotation rotary;
		private:
			struct KValues {
				float kP;
				float kI;
				float kD;
				float threshold;
			};

			KValues kTurn = {
				0.0, 0.0, 0.0, 1.0
			};

			KValues kMove = {
				0.0, 0.0, 0.0, 3.0
			};

			float inchesPerTick = 0.0002836;

			DriveMGs* mgs;
			pros::IMU imu;

			int delayMs = 20;

			void calibrateIMU(bool blocking = true) {
				imu.reset(blocking);
				imu.tare();
			}

			double position() {
				// Replace this with odometry wheel tracking later on
				//return mgs->position();
				return rotary.get_position();
			}
		protected:
		public:
			struct DrivePIDArgs {
				DriveMGs* mgs;
				std::int8_t imuPort;
				uint8_t rotaryPort;
			};

			DrivePID(DrivePIDArgs args) : 
				mgs(args.mgs),
				imu(args.imuPort),
				rotary(args.rotaryPort) {
					calibrateIMU();
					rotary.reset_position();
				};

			pros::IMU& getIMU() {
				return imu;
			}

			// TODO: Implement PID functions (and copy over legacy code)

			/// @brief Move to a specific position using PID
			/// @param pos Position to move to in inches (use negative for backward)
			// TODO: Tuning required
			void lateral(double pos, float reductionFactor = 2, float timeLimit = 5000) {
				if (pos <= 0.01) { return; }

				mgs->tare();

				pos /= inchesPerTick;

				float error = pos;
				float motorPos = 0;
				float lastError = 0;
				float derivative = 0;
				float integral = 0;
				float out = 0;

				float maxCycles = timeLimit / delayMs;
				float cycles = 0;

				// with moving you just wanna move both MGsat the same speed

				while (true) {
					// get avg error
					motorPos = position();
					error = pos - motorPos;

					integral += error;
					// Anti windup
					if (std::fabs(error) < kMove.threshold) {
						integral = 0;
					}

					derivative = error - lastError;
					out = (kMove.kP * error) + (kMove.kI * integral) + (kMove.kD * derivative);
					lastError = error;

					out *= 1000; // convert to mV
					out = std::clamp(out, -MotorBounds::MILLIVOLT_MAX, MotorBounds::MILLIVOLT_MAX);

					out /= reductionFactor;
					mgs->voltage(out, out);

					if (std::fabs(error) <= kMove.threshold) {
						break;
					}

					pros::lcd::print(4, ("PIDMove Motor Pos: " + std::to_string(motorPos)).c_str());
					pros::lcd::print(5, ("PIDMove Out: " + std::to_string(out)).c_str());
					pros::lcd::print(7, ("PIDMove Error: " + std::to_string(error)).c_str());

					if (cycles >= maxCycles) {
						pros::lcd::print(4, "PIDMove Time limit reached");
						break;
					}

					pros::delay(delayMs);
					cycles++;
				}

				mgs->stop();
			}

			/// @brief Turn to a specific angle using PID
			/// @param angle Angle to move to (PASS IN THE RANGE OF -180 TO 180 for left and right)
			/// @param reductionFactor Factor to reduce the output by (higher value means lower speed)
			/// @param timeLimit Time limit for the turn in milliseconds
			void turn(double angle, float reductionFactor = 2, float timeLimit = 5000) {
				if (angle <= 0.01) { return; }

				imu.tare();
				angle = naiveNormaliseAngle(angle);

				bool anglePositive = angle > 0;
				bool turn180 = false;

				// IMU already tared so we don't need to get the current heading
				float error = angle;
				float lastError = 0;
				float derivative = 0;
				float integral = 0;

				float out = 0;
				float trueHeading = 0;

				float maxThreshold = 180 - kTurn.threshold;

				float maxCycles = timeLimit / delayMs;
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
					if (std::fabs(error) < kTurn.threshold) {
						integral = 0;
					}

					derivative = error - lastError;
					out = (kTurn.kP * error) + (kTurn.kI * integral) + (kTurn.kD * derivative);
					lastError = error;

					out *= 1000; // convert to mV
					out = std::clamp(out, -MotorBounds::MILLIVOLT_MAX, MotorBounds::MILLIVOLT_MAX);
					out /= reductionFactor;

					mgs->voltage(out, -out);

					pros::lcd::print(5, ("PIDTurn Out: " + std::to_string(out)).c_str());
					pros::lcd::print(7, ("PIDTurn Error: " + std::to_string(error)).c_str());
					pros::lcd::print(6, ("PIDTurn True Heading: " + std::to_string(imu.get_heading())).c_str());

					if (std::fabs(error) <= kTurn.threshold) {
						break;
					}

					// 180 degree turning
					if (std::fabs(trueHeading) >= maxThreshold) {
						break;
					}

					if (std::fabs(out) < 100) {
						pros::lcd::print(4, "PIDTurn Out too low");
					}

					if (cycles >= maxCycles) {
						pros::lcd::print(4, "PIDTurn Time limit reached");
						break;
					}

					pros::delay(delayMs);
					cycles++;
				}

				pros::lcd::print(2, "PIDTurn End");
				mgs->stop();
			}

			// New unified PID movement function (under development)
			void move(float rotation, float position, float reductionFactor = 2) {
				turn(rotation, reductionFactor);
				lateral(position, reductionFactor);
			}
		}; // class DrivePID

		/// @brief Class to manage drivetrain operator control
		class DriveControl : public AbstractComponent {
		public:
			enum class DriveControlMode {
				ARCADE,
				TANK,
				ATAC
			};

			std::function<Horizontal()> driveControl;

			DriveControlMode driveControlMode;

			DriveMGs* mgs;

			/// @brief Args for DriveControl object
			/// @param abstractComponentArgs Args for AbstractComponent object
			struct DriveControlArgs {
				AbstractComponentArgs abstractComponentArgs;
				DriveMGs* mgs;
			};

			/// @brief Struct for different driver control speeds on arcade control
			/// @param turnSpeed Multiplier for only turning
			/// @param forwardBackSpeed Multiplier for only forward/backward
			/// @param arcSpeed Multiplier of opposite turn for when turning and moving laterally at the same time
			// (higher value means less lateral movement)
			struct ArcadeControlSpeed {
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

				ArcadeControlSpeed(float turnSpeed = 1, float forwardBackSpeed = 1, float arcSpeed = 0.7) :
					turnSpeed(turnSpeed), 
					arcSpeed(arcSpeed) {
						setForwardBackSpeed(forwardBackSpeed);
				}
			};

			ArcadeControlSpeed arcadeSpeed = {};

			/// @brief Speed for tank control on single side
			/// @param base Base speed for the side
			/// @param deadband Absolute deadband for the side
			/// @param sigmoid Sigmoid for the side
			struct TankSpeed {
				float base = 1.0;
				float deadband = 0.0;
				Vertical sigmoid = {1.0, 1.0};
			};

			// Tank speeds for left and right sides
			TankSpeed tankSpeeds[2] = {{}, {}};
		private:
			// Coefficients for turning in driver control
			struct TurnCoefficients {
				float left;
				float right;
			};

			void bindDriveControl(Horizontal (DriveControl::*driveFunc)()) {
				driveControl = std::bind(driveFunc, this);
			}

			void prepareArcadeLateral(float& lateral) {
				// Change to negative to invert
				lateral *= -1;
			}

			// Calculate the movement of the robot when turning and moving laterally at the same time
			void calculateArcMovement(TurnCoefficients& turnCoeffs, float lateral, float turn, float maxLateralTolerance = 1, float arcDeadband = 30) {
				if (std::fabs(lateral) < arcDeadband) {
					return;
				}

				// 0-1 range of percentage of lateral movement against max possible lateral movement
				float lateralCompensation = lateral / arcadeSpeed.getMaxLateral();
				// Decrease the turn speed when moving laterally (higher turn should be higher turnDecrease)
				float dynamicArcSpeed = (lateral < 0) ? arcadeSpeed.arcSpeed : 1;

				float turnDecrease = 1 * turn * lateralCompensation * dynamicArcSpeed;

				if (lateral > 0) {
					turnDecrease *= turn * 0.0001;
				}

				if (turn > 0) { // Turning to right so we decrease the left MG
					turnCoeffs.left += (lateral < 0) ? -turnDecrease : turnDecrease;
				} else { // Turning to left so we decrease the right MG
					turnCoeffs.right += (lateral > 0) ? -turnDecrease : turnDecrease;
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

			Horizontal arcadeControl() {
				float lateral = master->get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
				float turn = master->get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick

				prepareArcadeLateral(lateral);

				TurnCoefficients turnCoeffs = calculateArcadeTurns(turn, lateral);
				
				pros::lcd::print(1, ("T, L:" + std::to_string(turn) + ", " + std::to_string(lateral)).c_str());

				// Calculate speeds
				lateral *= arcadeSpeed.getForwardBackSpeed();

				turnCoeffs.left *= arcadeSpeed.turnSpeed;
				turnCoeffs.right *= arcadeSpeed.turnSpeed;

				// Ensure voltages are within correct ranges
				float left_speed = lateral - turnCoeffs.left;
				float right_speed = lateral + turnCoeffs.right;

				pros::lcd::print(2, ("L/R COEF: " + std::to_string(turnCoeffs.left) + ", " + std::to_string(turnCoeffs.right)).c_str());
				pros::lcd::print(7, ("LEFT/RIGHT: " + std::to_string(left_speed) + ", " + std::to_string(right_speed)).c_str());

				return {left_speed, right_speed};
			}

			// Basic tank control
			Horizontal tankControl() {
				float left = master->get_analog(ANALOG_LEFT_Y) * tankSpeeds[0].base;
				float right = master->get_analog(ANALOG_RIGHT_Y) * tankSpeeds[1].base;

				return {left, right};
			}

			float atacSigmoid(float absSpeed, const Vertical& sigmoid) {
				if (absSpeed < 0.5) {
					return 0.5 * std::pow(2 * absSpeed, sigmoid.low);
				} else {
					return 1 - 0.5 * std::pow(2 - (2 * absSpeed), sigmoid.high);
				}
			}

			// ATAC on individual axis (ran for each axis)
			float atacAxis(float speed, const TankSpeed& tankSpeed) {
				float absSpeed = std::fabs(speed);
				// Process deadbands
				if (absSpeed < tankSpeed.deadband) {
					return 0;
				}

				float sign = (speed < 0) ? -1 : 1;
				speed *= tankSpeed.base;
				speed = atacSigmoid(absSpeed, tankSpeed.sigmoid);
				speed *= sign;

				return speed;
			}

			// Advanced Tank Action Control: Implementing all features we've ever wanted
			Horizontal atac() {
				// Must use static_cast to avoid narrowing conversion warning as we are working with arrays
				float speeds[2] = {
					static_cast<float>(master->get_analog(ANALOG_LEFT_Y)),
					static_cast<float>(master->get_analog(ANALOG_RIGHT_Y))
				};
			
				int index = 0;
				for (float& speed : speeds) {
					// Rescale to -1 to 1 value
					speed /= MotorBounds::MOVE_MAX;

					// Process speed on one axis
					speed = atacAxis(speed, tankSpeeds[index]);

					// Rescale to -127 to 127 value
					speed *= MotorBounds::MOVE_MAX;

					index++;
				}

				//tell(0, "ROT LAT POS: " + std::to_string());

				return {speeds[0], speeds[1]};
			}

			// Final fallback driver control to default back to final working mode
			Horizontal fallbackControl() {
				return tankControl();
			}

			// REMEMBER: preparing the move speed must be done in Drivetrain class, NOT in DriveControl class
		public:
			/// @brief Sets the driver control mode
			/// @param mode Mode to set the driver control to
			void setDriveControlMode(DriveControlMode mode = DriveControlMode::TANK) {
				driveControlMode = mode;

				switch (driveControlMode) {
					case DriveControlMode::ARCADE:
						bindDriveControl(&DriveControl::arcadeControl);
						break;
					case DriveControlMode::TANK:
						bindDriveControl(&DriveControl::tankControl);
						break;
					case DriveControlMode::ATAC:
						bindDriveControl(&DriveControl::atac);
					default:
						bindDriveControl(&DriveControl::fallbackControl);
						break;
				}
			}

			/// @brief Creates DriveControl object
			/// @param args Args for DriveControl object (check args struct for more info)
			DriveControl(DriveControlArgs args) : 
				AbstractComponent(args.abstractComponentArgs),
				mgs(args.mgs) {
					setDriveControlMode();
				};

			void opControl() override {
				Horizontal speeds = driveControl();

				speeds.left = prepareMoveSpeed(speeds.left);
				speeds.right = prepareMoveSpeed(speeds.right);

				mgs->move(speeds.left, speeds.right);
			}
		}; // class DriveControl

		class DriveManager : public AbstractComponent {
		private:
		protected:
		public:
			/// @brief Structure for DriveManager options provided by user
			/// @param drivePorts Ports for drivetrain
			/// @param imuPort Port for IMU
			struct DriveManagerUserArgs {
				DriveMGs::DrivePorts drivePorts;
				DigiPort imuPort;
				uint8_t rotaryPort;
			};

			/// @brief Args for DriveManager object
			/// @param abstractComponentArgs Args for AbstractComponent object
			/// @param user Args for DriveManager object provideed by user
			struct DriveManagerArgs {
				AbstractComponentArgs abstractComponentArgs;
				DriveManagerUserArgs user;
			};

			DriveMGs mgs;
			DrivePID pid;
			DriveControl control;

			/// @brief Creates DriveManager object
			/// @param args Args for DriveManager object (check args struct for more info)
			DriveManager(DriveManagerArgs args) : 
				AbstractComponent(args.abstractComponentArgs),
				mgs({args.user.drivePorts}),
				pid({&mgs, args.user.imuPort, args.user.rotaryPort}),
				control({args.abstractComponentArgs, &mgs}) {};

			void opControl() override {
				control.opControl();
			}
		}; // class DriveManager
	} // namespace Drivetrain

	/// @brief Class for GPS diagnostic
	class GPSDiagnostic : public AbstractComponent {
		private:
		protected:
		public:
			/// @brief Args for GPS diagnostic object
			/// @param abstractComponentArgs Args for AbstractComponent object
			/// @param gpsPort Port for GPS
			struct GPSDiagnosticArgs {
				AbstractComponentArgs abstractComponentArgs;
				uint8_t gpsPort;
			};

			pros::Gps gps;

			/// @brief Creates GPS diagnostic object
			/// @param args Args for GPS diagnostic object (check args struct for more info)
			GPSDiagnostic(GPSDiagnosticArgs args) : 
				AbstractComponent(args.abstractComponentArgs),
				gps(args.gpsPort) {}

			void opControl() override {

			}
	}; // class GPSDiagnostic

	/// @brief Class which manages all components
	class ComponentManager : public AbstractComponent {
	private:
	protected:
	public:
		Drivetrain::DriveManager drive;

		Timer timer;
		
		// All components are stored in this vector
		vector<AbstractComponent*> components;

		/// @brief Args for component manager object passed to the chassis, such as ports
		/// @param dvtPorts Ports for drivetrain
		struct ComponentManagerUserArgs {
			Drivetrain::DriveManager::DriveManagerUserArgs driveArgs;
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

			drive({args.aca, args.user.driveArgs}),	
			timer({args.aca}) {
				// Add component pointers to vector
				// MUST BE DONE AFTER INITIALISATION not BEFORE because of pointer issues
				components = {
					&drive,
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
			cm->drive.pid.lateral(48);
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

		/// @brief Post auton function for the chassis
		void postAuton() override {
			cm.postAuton();
		}
	}; // class Chassis
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
		{{{LEFT_DRIVE_PORTS, RIGHT_DRIVE_PORTS}, IMU_PORT, ROT_DRIVE_PORT}} // Drivetrain MGs and IMU ports
	});
	
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
	#if DO_MATCH_AUTON
		currentChassis->auton();
	#elif DO_SKILLS_AUTON
		currentChassis->skillsAuton();
	#endif
}

void preControl() {
	pros::lcd::set_text(0, "> 1408Hyper mainControl ready");

	bool inComp = pros::competition::is_connected();

	// Run autonomous even if we are NOT in the compeition
	if (!inComp) {
		autonomous();
	}

	#if DO_SKILLS_PREP
		currentChassis->skillsPrep();
	#endif

	// only do post auton if we are not in skills prep
	// 2025: What the hell does this comment even mean?
	#if DO_POST_AUTON
		currentChassis->postAuton();
	#endif
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
