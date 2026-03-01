// includes/usings are all in main.h
#include "main.h"

// uses ISO/C++20 standard
// added libraries/includes:
// pros

/// @brief Hyper namespace for all custom classes and functions
namespace hyper
{
	// 2nd reminder: all usings should be in main.h!

	// User defined literals

	namespace literals {
		namespace dist {
			// TODO: Implement inches literalss
			struct InchesPerTick {
				static constexpr float L_ROT_DIV_THEORETICAL = 0.0001096386338;
				static constexpr float L_ROT_MUL_THEORETICAL = 9120.872500328438;
				static constexpr float L_ROT_MUL_PRACTICAL = 5753.54167;
			};

			// Unified single source of truth for tick-inches conversion
			
		} // namespace dist
	} // namespace literals

	// Structs

	/// @brief Struct for motor group buttons (manual control)
	/// @param fwd Button for forward
	/// @param back Button for backward

	struct Buttons
	{
		pros::controller_digital_e_t fwd;
		pros::controller_digital_e_t back;
	};

	/// @brief Struct for motor move bounds
	struct MotorBounds
	{
		static constexpr float SCALE_MIN = 0;
		static constexpr float SCALE_MAX = 1;

		static constexpr std::int32_t MOVE_MIN = -127;
		static constexpr std::int32_t MOVE_MAX = 127;

		static constexpr float MILLIVOLT_MAX = 12000;
	};

	/// @brief Base struct for any values on the horizontal axis
	/// @param left Left side value
	/// @param right Right side value
	struct Horizontal
	{
		float left;
		float right;
	};

	/// @brief Vertical axis struct
	/// @param low Low value
	/// @param high High value
	struct Vertical
	{
		float low;
		float high;
	};

	// Helper functions

	/// @brief Convert vector of ints to string. For displaying on the LCD/debugging
	/// @param vec Vector to convert
	/// @param delimiter Delimiter to separate elements
	template <typename T>
	string vectorToString(vector<T> &vec, string delimiter)
	{
		int vecSize = vec.size();
		int vecSizeMinusOne = vecSize - 1;
		std::ostringstream oss;

		oss << "{";
		for (int i = 0; i < vecSize; i++)
		{
			oss << vec[i];
			if (i < vecSizeMinusOne)
			{
				oss << delimiter;
			}
		}
		oss << "}";

		return oss.str();
	}

	/// @brief Assert that a value is arithmetic
	/// @param val Value to assert
	template <typename T>
	void assertArithmetic(const T val)
	{
		static_assert(std::is_arithmetic<T>::value, "Value must be arithmetic");
	}

	/// @brief Checks whether a given color channel is within tolerance.
	/// @param channel Color to check
	/// @param target Target colour which the colour should be
	/// @return Whether the channel is within tolerance
	bool channelWithinTolerance(const float &channel, const float &target, const float &tolerance = 5)
	{
		return std::fabs(channel - target) <= tolerance;
	}

	std::int32_t prepareMoveSpeed(float raw)
	{
		// Round the number to the nearest integer
		raw = std::round(raw);

		std::int32_t speed = static_cast<std::int32_t>(raw);
		speed = std::clamp(speed, MotorBounds::MOVE_MIN, MotorBounds::MOVE_MAX);

		return speed;
	}

	/// @brief Always return true
	bool sayYes() 
	{
		return true;
	}

	/// @brief Always return false
	bool sayNo()
	{
		return false;
	}

	/// @brief Assert that a number is between two values
	/// @param num Number to assert
	/// @param min Minimum value
	/// @param max Maximum value
	template <typename T>
	bool isNumBetween(T num, T min, T max)
	{
		return ((num >= min) && (num <= max));
	}

	/// @brief Normalise an angle to the range [-180, 180]
	/// @param angle Angle to normalise
	template <typename T>
	T normaliseAngle(T angle)
	{
		assertArithmetic(angle);

		if (angle > 180)
		{
			angle -= 360;
		}
		else if (angle < -180)
		{
			angle += 360;
		}

		return angle;
	}

	/// @brief Naively normalise an angle to the range [-180, 180] by simply clamping the value
	/// @param angle Angle to normalise
	template <typename T>
	T naiveNormaliseAngle(T angle)
	{
		assertArithmetic(angle);

		angle = std::clamp(angle, -180.0, 180.0);

		return angle;
	}

	/// @brief Calculate the mean of a vector
	/// @param vec Vector to calculate the mean of
	/// @param size Size of the vector
	/// @return Mean of the vector (type T)
	template <typename T>
	T calcMeanFromVector(const vector<T> &vec, int size)
	{
		T sum = std::accumulate(vec.begin(), vec.end(), 0);
		T mean = sum / size;

		return mean;
	}

	/// @brief Calculate the mean of a vector
	/// @param vec Vector to calculate the mean of
	/// @return Mean of the vector (type T)
	template <typename T>
	T calcMeanFromVector(const vector<T> &vec)
	{
		int size = vec.size();
		T sum = std::accumulate(vec.begin(), vec.end(), 0);
		T mean = sum / size;

		return mean;
	}

	// Class declarations

	/// @brief Abstract chassis class for if you want a custom chassis class
	class AbstractChassis
	{
	private:
	protected:
		pros::Controller master{pros::E_CONTROLLER_MASTER};

	public:
		/// @brief Creates abstract chassis object
		AbstractChassis() {};

		virtual ~AbstractChassis() = default;

		/// @brief Gets the controller
		pros::Controller &getController()
		{
			return master;
		}

		virtual void opControl() = 0;
		virtual void auton() = 0;
		virtual void skillsPrep() = 0;
		virtual void skillsAuton() = 0;
		virtual void postAuton() = 0;
	}; // class AbstractChassis

	/// @brief Class for components of the chassis to derive from
	class AbstractComponent
	{
	private:
	protected:
		AbstractChassis *chassis;

		pros::Controller *master;

	public:
		static constexpr std::uint8_t MAX_BRAIN_LINES = 8;
		static constexpr std::uint8_t MAX_CONTROLLER_LINES = 2;
		static constexpr std::uint8_t CONTROLLER_TXT_START_COL = 0;

		/// @brief Args for AbstractComponent object
		/// @param chassis AbstractChassis derived object to be used for the component
		struct AbstractComponentArgs
		{
			AbstractChassis *chassis;
		};

		/// @brief Creates AbstractComponent object
		/// @param args Args AbstractComponent object (check args struct for more info)
		AbstractComponent(AbstractComponentArgs args) : chassis(args.chassis),
														master(&args.chassis->getController()) {};

		/// @brief Log something to the brain safely
		/// @param line Line to print the message on (check class consts for max lines)
		/// @param message Message to print
		/// @param additional Additional arguments to print
		/// @return Success/fail state of the brain printing
		template <typename... T>
		bool log(const std::uint8_t line, const string &message, T &&...additional)
		{
			if (line > MAX_BRAIN_LINES)
			{
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
		bool tell(const std::uint8_t line, const string &message, T &&...additional)
		{
			if (line > MAX_CONTROLLER_LINES)
			{
				return false;
			}

			master->print(line, CONTROLLER_TXT_START_COL, message.c_str(), additional...);
			return true;
		}

		AbstractChassis &getChassis()
		{
			return *chassis;
		}

		pros::Controller &getMaster()
		{
			return *master;
		}

		virtual void opControl() = 0;

		virtual void postAuton() {}
		virtual void skillsPrep() {}

		virtual ~AbstractComponent() = default;
	}; // class ChassisComponent

	/// @brief Abstract pneumatic mechanism class for custom mech classes
	class AbstractMech : public AbstractComponent
	{
	private:
		bool engaged = false;

		pros::adi::DigitalOut piston;

	protected:
	public:
		/// @brief Args for abstract mech object
		/// @param abstractComponentArgs Args for AbstractComponent object
		/// @param pistonPort Port for piston
		struct AbstractMechArgs
		{
			AbstractComponentArgs abstractComponentArgs;
			char pistonPort;
		};

		/// @brief Creates abstract mech object
		/// @param args Args for abstract mech object (check args struct for more info)
		AbstractMech(AbstractMechArgs args) : AbstractComponent(args.abstractComponentArgs),
											  piston(args.pistonPort) {};

		/// @brief Sets actuation value of piston
		/// @param value Value to set the piston to
		void actuate(bool value)
		{
			piston.set_value(value);
			engaged = value;
		}

		/// @brief Toggles the piston state
		void toggle()
		{
			actuate(!engaged);
		}

		/// @brief Gets the piston object
		/// @return PROS ADI DigitalOut object for piston
		pros::adi::DigitalOut &getPiston()
		{
			return piston;
		}

		/// @brief Gets the engaged state of the mech
		/// @return Engaged state of the mech
		bool getEngaged()
		{
			return engaged;
		}

		virtual ~AbstractMech() = default;
	}; // class AbstractMech

	/// @brief Abstract motor group class for if you want a custom motor group class
	class AbstractMG : public AbstractComponent
	{
	private:
	protected:
		const pros::MotorGroup mg;

	public:
		struct Speeds
		{
			int fwd = 10000;
			int back = -10000;
		};

		/// @brief Args for abstract motor group object
		/// @param abstractComponentArgs Args for AbstractComponent object
		/// @param ports Vector of ports for motor group
		struct AbstractMGArgs
		{
			AbstractComponentArgs abstractComponentArgs;
			MGPorts ports;
		};

		Speeds speeds = {};
		bool outputSpeeds;

		/// @brief Constructor for abstract motor group object
		/// @param args Args for abstract motor group object (check args struct for more info)
		AbstractMG(AbstractMGArgs args) : AbstractComponent(args.abstractComponentArgs),
										  mg(args.ports) {};

		/// @brief Move the motors in the specified direction according to speeds.
		/// @param on Whether to stop or start the motors.
		/// @param directionForward Direction to go.
		void move(bool on, bool directionForward = true)
		{
			on = canMove(on);

			if (on)
			{
				if (directionForward)
				{
					mg.move_velocity(speeds.fwd);
					if (outputSpeeds)
					{
						// pros::lcd::print(2, "motor going!!");
					}
				}
				else
				{
					mg.move_velocity(speeds.back);
					if (outputSpeeds)
					{
						// pros::lcd::print(2, "motor not going :(");
					}
				}
			}
			else
			{
				mg.move_velocity(0);
			}
		}

		virtual bool canMove(bool on) = 0;

		virtual ~AbstractMG() = default;
	}; // class AbstractMG

	/// @brief Class which manages button presses (will run function on up, down and hold states of given button)
	class BtnManager : public AbstractComponent
	{
	private:
		bool lastPressed = false;

		void handleBtnPressed()
		{
			if (lastPressed)
			{
				for (VoidFunc &func : actionInfo.holdFuncs)
				{
					func();
				}
			}
			else
			{
				for (VoidFunc &func : actionInfo.downFuncs)
				{
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
		struct ActionInfo
		{
			pros::controller_digital_e_t btn;
			VoidFuncVector downFuncs = {};
			VoidFuncVector upFuncs = {};
			VoidFuncVector holdFuncs = {};
		};

		ActionInfo actionInfo;

		/// @brief Args for button manager object
		/// @param abstractComponentArgs Args for AbstractComponent object
		/// @param actionInfo Action info for button manager object
		struct BtnManagerArgs
		{
			AbstractComponentArgs abstractComponentArgs;
			ActionInfo actionInfo;
		};

		/// @brief Creates button manager object
		/// @param args Args for button manager object (check args struct for more info)
		BtnManager(BtnManagerArgs args) : AbstractComponent(args.abstractComponentArgs),
										  actionInfo(args.actionInfo) {};

		void opControl() override
		{
			bool btnPressed = master->get_digital(actionInfo.btn);

			// down: !lastPressed && btnPressed
			// up: lastPressed && !btnPressed
			// hold: lastPressed && btnPressed

			if (btnPressed)
			{
				handleBtnPressed();
			}
			else if (lastPressed)
			{
				for (VoidFunc &func : actionInfo.upFuncs)
				{
					func();
				}
			}

			lastPressed = btnPressed;
		}

		bool getLastPressed()
		{
			return lastPressed;
		}
	};

	/// @brief Class for a toggle on the controller
	class BiToggle : public AbstractComponent
	{
	public:
		enum class State
		{
			OFF,
			FWD,
			BACK
		};

	private:
		AbstractMG *component;

		State state = State::OFF;
		bool isNewPress = true;

		void moveState(State target)
		{
			if (!isNewPress)
			{
				return;
			}

			switch (target)
			{
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

		void handleFwdBtn()
		{
			if (state == State::FWD)
			{
				moveState(State::OFF);
				pros::lcd::print(1, "Fwd pressed AND GOING OFF");
			}
			else
			{
				moveState(State::FWD);
				pros::lcd::print(1, "Fwd pressed AND GOING FWD");
			}
		}

		void handleBackBtn()
		{
			if (state == State::BACK)
			{
				moveState(State::OFF);
				pros::lcd::print(1, "Back pressed AND GOING OFF");
			}
			else
			{
				moveState(State::BACK);
				pros::lcd::print(1, "Back pressed AND GOING BACK");
			}
		}

	protected:
	public:
		/// @brief Struct for buttons for BiToggle object
		/// @param fwd Button for forward
		/// @param back Button for backward
		struct Buttons
		{
			pros::controller_digital_e_t fwd;
			pros::controller_digital_e_t back;
		};

		/// @brief Args for BiToggle object
		/// @param abstractComponentArgs Args for AbstractComponent object
		/// @param component Component to toggle
		/// @param btns Buttons for toggle
		struct BiToggleArgs
		{
			AbstractComponentArgs abstractComponentArgs;
			AbstractMG *component;
			Buttons btns;
		};

		Buttons btns;

		/// @brief Creates BiToggle object
		/// @param args Args for BiToggle object (check args struct for more info)
		BiToggle(BiToggleArgs args) : AbstractComponent(args.abstractComponentArgs),
									  component(args.component),
									  btns(args.btns) {};

		void opControl()
		{
			bool fwdPressed = master->get_digital(btns.fwd);
			bool backPressed = master->get_digital(btns.back);

			pros::lcd::print(3, ("FWD: " + std::to_string(fwdPressed)).c_str());
			pros::lcd::print(4, ("BACK: " + std::to_string(backPressed)).c_str());

			if (fwdPressed && backPressed)
			{
				// Don't do anything if both are pressed
				// TODO: test whether the return works
				// because we need it for backwards motor movement
				return;
			}

			if (fwdPressed)
			{
				handleFwdBtn();
				isNewPress = false;
				return;
			}

			if (backPressed)
			{
				handleBackBtn();
				isNewPress = false;
				return;
			}

			isNewPress = true;
		}

		void setState(State target)
		{
			state = target;
		}

		State getState()
		{
			return state;
		}
	}; // class BiToggle

	/// @brief Advanced task scheduling class with thread-safe timing capabilities
	class Timer : public AbstractComponent
	{
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
		struct Timeout
		{
			std::function<void(Timeout &timeout)> func;
			int ms;
			int runs = 1;
		};

	private:
		/// @brief Internal structure for a scheduled task
		/// @param timeout Timeout object
		/// @param nextRun Next ms time to run timeout
		struct InternalTimeout
		{
			Timeout timeout;
			std::uint32_t nextRun;
		};

		vector<InternalTimeout> timeouts = {};

		void removeExpiredTimeouts()
		{
			timeouts.erase(
				std::remove_if(timeouts.begin(), timeouts.end(),
							   [](const InternalTimeout &ito)
							   {
								   return ito.timeout.runs <= NO_RUNS;
							   }),
				timeouts.end());
		}

		void processTimeouts()
		{
			removeExpiredTimeouts();

			for (InternalTimeout &ito : timeouts)
			{
				ito.timeout.runs--;

				if (ito.nextRun <= ms)
				{
					ito.timeout.func(ito.timeout);
				}

				ito.nextRun += ito.timeout.ms;
			}
		}

	public:
		/// @brief Args for timer object
		/// @param abstractComponentArgs Args for AbstractComponent object
		struct TimerArgs
		{
			AbstractComponentArgs abstractComponentArgs;
		};

		using ArgsType = TimerArgs;

		/// @brief Constructor for timer object
		/// @param args Args for timer object (see args struct for more info)
		Timer(TimerArgs args) : AbstractComponent(args.abstractComponentArgs) {};

		/// @brief Sleep function to be used instead of pros::delay to track time for task scheduling
		/// @param time Time in milliseconds to wait for
		void sleep(int time)
		{
			pros::delay(time);
			ms += time;
		}

		/// @brief Gets the tick wait time in milliseconds
		/// @return Time to wait for in milliseconds
		int getTickMs()
		{
			return tickMs;
		}

		/// @brief Gets the current time in milliseconds
		/// @return Current time in milliseconds
		std::uint32_t getTimeMs()
		{
			return ms;
		}

		/// @brief Setup a new timeout
		/// @param timeout Timeout object to setup
		void setupTimeout(Timeout timeout)
		{
			std::uint32_t nextRun = ms + timeout.ms;
			InternalTimeout ito = {timeout, nextRun};
			timeouts.push_back(ito);
		}

		void opControl() override
		{
			sleep(tickMs);
			processTimeouts();
		}
	}; // class Timer

	namespace Drivetrain
	{
		/// @brief Struct for drivetrain IO
		struct DriveIO
		{
			// Drivetrain motor groups lef/right
			pros::MotorGroup left;
			pros::MotorGroup right;

			// Lateral and turning rotary sensors
			pros::Rotation lRot;
			pros::Rotation tRot;

			// IMU
			pros::IMU imu;

			/// @brief Struct for drive ports
			/// @param leftPorts Ports for left motor group
			/// @param rightPorts Ports for right motor group
			struct DrivePorts
			{
				MGPorts leftPorts;
				MGPorts rightPorts;
				int8_t imuPort;
				uint8_t lRotPort;
				uint8_t tRotPort;
			};

			/// @brief Tare the motor groups
			void tare()
			{
				left.tare_position();
				right.tare_position();
			}

			void resetEncoders()
			{
				// Reset encoders
				tRot.reset();
				lRot.reset();
			}
			

			void calibrateAll(bool blocking = true)
			{
				// Calibrate/tare IMU
				imu.reset(blocking);
				imu.tare();

				resetEncoders();
			}

			/// @brief Constructor for DriveMGs object
			/// @param leftPorts Ports for left motor group
			/// @param rightPorts Ports for right motor group
			DriveIO(DrivePorts drivePorts) : left(drivePorts.leftPorts), right(drivePorts.rightPorts),
											 imu(drivePorts.imuPort),
											 lRot(drivePorts.lRotPort), tRot(drivePorts.tRotPort)
			{
				calibrateAll();
			};

			/// @brief Set the voltage of the motor groups
			/// @param leftVoltage Voltage to set the left motor group to
			/// @param rightVoltage Voltage to set the right motor group to
			void voltage(int leftVoltage, int rightVoltage)
			{
				left.move_voltage(leftVoltage);
				right.move_voltage(rightVoltage);
			}

			/// @brief Set the same voltage to both motor groups
			/// @param volt Voltage to set both motor groups to
			void voltage(int volt)
			{
				voltage(volt, volt);
			}

			/// @brief Stop the motor groups by setting their voltage to 0
			void stop()
			{
				voltage(0, 0);
			}

			/// @brief Set the velocity of the motor groups
			/// @param leftVel Velocity to set the left motor group to
			/// @param rightVel Velocity to set the right motor group to
			void velocity(int leftVel, int rightVel)
			{
				left.move_velocity(leftVel);
				right.move_velocity(rightVel);
			}

			/// @brief Set the same velocity to both motor groups
			void velocity(int vel)
			{
				velocity(vel, vel);
			}

			/// @brief Move the motor groups using the motor.move() function
			/// @param leftSpeed Speed to move the left motor group at
			/// @param rightSpeed Speed to move the right motor group at
			void move(int leftSpeed, int rightSpeed)
			{
				left.move(leftSpeed);
				right.move(rightSpeed);
			}

			/// @brief Move both motor groups at the same speed
			/// @param speed Speed to move motor groups at
			void move(int speed)
			{
				move(speed, speed);
			}

			/// @brief Get the average position of the motor groups (certain wires on our motor are broken so you MUST use this if you want a reliable position)
			/// @return Average position of the motor groups
			double positionMG()
			{
				return (left.get_position() + right.get_position()) / 2;
			}

			/// @brief Get the average velocity of the motor groups (certain wires on our motor are broken so you MUST use this if you want a reliable velocity)
			/// @param blocking Whether to block IMU during calibration
			void calibrateIMU(bool blocking = true)
			{
				imu.reset(blocking);
				imu.tare();
			}
		};

		/// @brief Class to control autonomous routines for driving
		class DrivePID
		{
		public:
		private:
			DriveIO *dio;

			int delayMs = 20;

		protected:
		public:
			struct KValues
			{
				float kP;
				float kI;
				float kD;
				float threshold;
			};

			static inline const KValues kTurn = {
				0.3, 0.0, 0.7, 1.0};

			static inline const KValues kMove = {
				1.0, 0.0, 0.4, 3.0};

			struct InchesPerTick
			{
				static constexpr float L_ROT_DIV_THEORETICAL = 0.0001096386338;
				static constexpr float L_ROT_MUL_THEORETICAL = 9120.872500328438;
				static constexpr float L_ROT_MUL_PRACTICAL = 5753.54167;
			};

			/// @brief Manages checkpoint triggering during lateral PID movements
			struct CheckpointManager
			{
			public:
				/// @brief Checkpoint structure for triggering functions at specific distances during lateral movement
				/// @param distanceInches Distance in inches (from start position) at which to trigger the action
				/// @param action Function to execute when checkpoint is reached
				struct Checkpoint
				{
					float distanceInches;
					VoidFunc action;
				};

				using Checkpoints = vector<Checkpoint>;

			private:
				vector<float> checkpointDistancesTicks;
				vector<bool> available;
				const Checkpoints &checkpoints;
				bool movingForward;

			protected:
			public:
				/// @brief Initializes checkpoint manager with converted distances
				/// @param checkpoints Reference to checkpoint vector
				/// @param direction True if moving forward, false if moving backward
				CheckpointManager(const Checkpoints &checkpoints, bool direction) : checkpoints(checkpoints),
																					movingForward(direction),
																					available(checkpoints.size(), true)
				{

					// Convert all checkpoint distances from inches to encoder ticks
					checkpointDistancesTicks.reserve(checkpoints.size());
					for (const Checkpoint &checkpoint : checkpoints)
					{
						checkpointDistancesTicks.push_back(checkpoint.distanceInches * InchesPerTick::L_ROT_MUL_PRACTICAL);
					}
				}

				/// @brief Checks and triggers any checkpoints that have been reached
				/// @param currentPositionTicks Current position in encoder ticks
				void check(float currentPositionTicks)
				{
					for (size_t i = 0; i < checkpoints.size(); i++)
					{
						if (available[i])
						{
							bool shouldTrigger = false;
							if (movingForward)
							{
								shouldTrigger = currentPositionTicks >= checkpointDistancesTicks[i];
							}
							else
							{
								shouldTrigger = currentPositionTicks <= checkpointDistancesTicks[i];
							}

							if (shouldTrigger)
							{
								checkpoints[i].action();
								available[i] = false;
							}
						}
					}
				}
			}; // struct CheckpointManager

			struct DrivePIDArgs
			{
				DriveIO *dio;
			};

			DrivePID(DrivePIDArgs args) : dio(args.dio) {};

			// TODO: Implement PID functions (and copy over legacy code)

			/// @brief Basic information for any PID maneuver
			/// @param pos Target position
			/// @param reductionFactor Reduction factor for output (higher is slower)
			/// @param timeLimit Time limit for maneuver in milliseconds
			struct Maneuver
			{
				double pos;
				float reductionFactor;
				float timeLimit;
			};
			// TODO: Change PID funcs to use this new struct

			/// @brief Move to a specific position using PID
			/// @param pos Position to move to in inches (use negative for backward)
			/// @param reductionFactor Factor to reduce the output by (higher value means lower speed)
			/// @param timeLimit Time limit for the movement in milliseconds
			/// @param stopCheck Optional function to pass to stop the loop if it returns true
			/// @param kv PID tuning values
			/// @param checkpoints Vector of checkpoints to trigger functions at specific distances
			// TODO: Tuning required
			void lateral(
				double pos, float reductionFactor = 2, float timeLimit = 5000,
				std::function<bool()> stopCheck = []() { return false; },
				const CheckpointManager::Checkpoints &checkpoints = {},
				const KValues &kv = DrivePID::kMove)
			{
				if (std::fabs(pos) <= 0.01)
				{
					return;
				}

				dio->tare();
				dio->resetEncoders();

				// Convert target position to encoder ticks
				pos *= InchesPerTick::L_ROT_MUL_PRACTICAL;

				// Initialize checkpoint manager
				bool movingForward = pos > 0;
				CheckpointManager checkpointMgr(checkpoints, movingForward);

				float error = pos;
				float motorPos = 0;
				float lastError = 0;
				float derivative = 0;
				float integral = 0;
				float out = 0;

				float maxCycles = timeLimit / delayMs;
				float cycles = 0;

				bool lastOutPositive = pos > 0;
				bool curOutPositive = pos > 0;
				int outCycles = 0;

				// with moving you just wanna move both MGs at the same speed

				while (true)
				{
					// Get current position
					motorPos = -dio->lRot.get_position();
					error = pos - motorPos;

					// Check and trigger checkpoints
					checkpointMgr.check(motorPos);

					// PID calculations
					integral += error;
					// Anti windup
					if (std::fabs(error) < kv.threshold)
					{
						integral = 0;
					}

					if (stopCheck()) {
						return;
					}

					derivative = error - lastError;
					out = (kv.kP * error) + (kv.kI * integral) + (kv.kD * derivative);
					lastError = error;

					out *= 1000; // convert to mV
					out = std::clamp(out, -MotorBounds::MILLIVOLT_MAX, MotorBounds::MILLIVOLT_MAX);

					out /= reductionFactor;
					dio->voltage(out, out);

					curOutPositive = out > 0;
					if (lastOutPositive != curOutPositive)
					{
						outCycles++;
					}

					if (outCycles > 3)
					{
						pros::lcd::print(4, "PIDMove Out oscillating STOP");
						break;
					}

					if (std::fabs(error) <= kv.threshold)
					{
						break;
					}

					pros::lcd::print(4, ("PIDMove Motor Pos: " + std::to_string(motorPos)).c_str());
					pros::lcd::print(5, ("PIDMove Out: " + std::to_string(out)).c_str());
					pros::lcd::print(7, ("PIDMove Error: " + std::to_string(error)).c_str());

					if (cycles >= maxCycles)
					{
						pros::lcd::print(4, "PIDMove Time limit reached");
						break;
					}

					pros::delay(delayMs);
					cycles++;
				}

				dio->stop();
			}

			/// @brief Turn to a specific angle using PID
			/// @param angle Angle to move to (PASS IN THE RANGE OF -180 TO 180 for left and right)
			/// @param reductionFactor Factor to reduce the output by (higher value means lower speed)
			/// @param timeLimit Time limit for the turn in milliseconds
			/// @param kv PID tuning values
			void turn(double angle, float reductionFactor = 2, float timeLimit = 5000, const KValues &kv = DrivePID::kTurn)
			{
				float absAngle = std::fabs(angle);

				if (absAngle <= 0.01)
				{
					return;
				}

				dio->imu.tare();
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

				float maxThreshold = 180 - kv.threshold;

				float maxCycles = timeLimit / delayMs;
				float cycles = 0;

				if (std::fabs(absAngle) >= 180)
				{
					turn180 = true;
				}

				pros::lcd::print(3, "PIDTurn Start");

				// with turning you just wanna move the other MG at negative of the MG of the direction
				// which u wanna turn to

				while (true)
				{
					trueHeading = std::fmod((dio->imu.get_heading() + 180), 360) - 180;
					error = angle - trueHeading;

					integral += error;
					// Anti windup
					if (std::fabs(error) < kv.threshold)
					{
						integral = 0;
					}

					derivative = error - lastError;
					out = (kv.kP * error) + (kv.kI * integral) + (kv.kD * derivative);
					lastError = error;

					out *= 1000; // convert to mV
					out = std::clamp(out, -MotorBounds::MILLIVOLT_MAX, MotorBounds::MILLIVOLT_MAX);
					out /= reductionFactor;

					dio->voltage(out, -out);

					pros::lcd::print(5, ("PIDTurn Out: " + std::to_string(out)).c_str());
					pros::lcd::print(7, ("PIDTurn Error: " + std::to_string(error)).c_str());
					pros::lcd::print(6, ("PIDTurn True Heading: " + std::to_string(dio->imu.get_heading())).c_str());

					if (std::fabs(error) <= kv.threshold)
					{
						break;
					}

					// 180 degree turning
					if (std::fabs(trueHeading) >= maxThreshold)
					{
						break;
					}

					if (std::fabs(out) < 100)
					{
						pros::lcd::print(4, "PIDTurn Out too low");
					}

					if (cycles >= maxCycles)
					{
						pros::lcd::print(4, "PIDTurn Time limit reached");
						break;
					}

					pros::delay(delayMs);
					cycles++;
				}

				pros::lcd::print(2, "PIDTurn End");
				dio->stop();
			}

			/// @brief 180 deg turning - turn(180) can be buggy, this is ALWAYS the latest practical solution to achieve this.
			void uTurn()
			{
				turn(90);
				turn(90);
			}

			// New unified PID movement function (under development)
			void move(float rotation, float position, float reductionFactor = 2)
			{
				turn(rotation, reductionFactor);
				lateral(position, reductionFactor);
			}
		}; // class DrivePID

		/// @brief Class to manage drivetrain operator control
		class DriveControl : public AbstractComponent
		{
		public:
			enum class DriveControlMode
			{
				ARCADE,
				TANK,
				ATAC
			};

			std::function<Horizontal()> driveControl;

			DriveControlMode driveControlMode;

			DriveIO *dio;

			/// @brief Args for DriveControl object
			/// @param abstractComponentArgs Args for AbstractComponent object
			struct DriveControlArgs
			{
				AbstractComponentArgs abstractComponentArgs;
				DriveIO *dio;
			};

			/// @brief Struct for different driver control speeds on arcade control
			/// @param turnSpeed Multiplier for only turning
			/// @param forwardBackSpeed Multiplier for only forward/backward
			/// @param arcSpeed Multiplier of opposite turn for when turning and moving laterally at the same time
			// (higher value means less lateral movement)
			struct ArcadeControlSpeed
			{
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
				void setForwardBackSpeed(float speed, float maxTolerance = 1)
				{
					forwardBackSpeed = speed;
					maxLateral = speed * controllerMax + maxTolerance;
				}

				/// @brief Gets the forward/backward speed
				/// @return Forward/backward speed
				float getForwardBackSpeed()
				{
					return forwardBackSpeed;
				}

				/// @brief Gets the max lateral movement
				/// @return Max lateral movement
				float getMaxLateral()
				{
					return maxLateral;
				}

				// lower arc speed is lower turning

				ArcadeControlSpeed(float turnSpeed = 1, float forwardBackSpeed = 1, float arcSpeed = 0.7) : turnSpeed(turnSpeed),
																											arcSpeed(arcSpeed)
				{
					setForwardBackSpeed(forwardBackSpeed);
				}
			};

			ArcadeControlSpeed arcadeSpeed = {};

			/// @brief Speed for tank control on single side
			/// @param base Base speed for the side
			/// @param deadband Absolute deadband for the side
			/// @param sigmoid Sigmoid for the side
			struct TankSpeed
			{
				float base = 1.0;
				float deadband = 0.0;
				Vertical sigmoid = {1.0, 1.0};
			};

			// Tank speeds for left and right sides
			TankSpeed tankSpeeds[2] = {{}, {}};

		private:
			// Coefficients for turning in driver control
			struct TurnCoefficients
			{
				float left;
				float right;
			};

			void bindDriveControl(Horizontal (DriveControl::*driveFunc)())
			{
				driveControl = std::bind(driveFunc, this);
			}

			void prepareArcadeLateral(float &lateral)
			{
				// Change to negative to invert
				lateral *= 1;
			}

			// Calculate the movement of the robot when turning and moving laterally at the same time
			void calculateArcMovement(TurnCoefficients &turnCoeffs, float lateral, float turn, float maxLateralTolerance = 1, float arcDeadband = 30)
			{
				if (std::fabs(lateral) < arcDeadband)
				{
					return;
				}

				// 0-1 range of percentage of lateral movement against max possible lateral movement
				float lateralCompensation = lateral / arcadeSpeed.getMaxLateral();
				// Decrease the turn speed when moving laterally (higher turn should be higher turnDecrease)
				float dynamicArcSpeed = (lateral < 0) ? arcadeSpeed.arcSpeed : 1;

				float turnDecrease = 1 * turn * lateralCompensation * dynamicArcSpeed;

				if (lateral > 0)
				{
					turnDecrease *= turn * 0.0001;
				}

				if (turn > 0)
				{ // Turning to right so we decrease the left MG
					turnCoeffs.left += (lateral < 0) ? -turnDecrease : turnDecrease;
				}
				else
				{ // Turning to left so we decrease the right MG
					turnCoeffs.right += (lateral > 0) ? -turnDecrease : turnDecrease;
				}

				pros::lcd::print(6, ("TD, dAS:, lComp: " + std::to_string(turnDecrease) + ", " + std::to_string(dynamicArcSpeed) + ", " + std::to_string(lateralCompensation)).c_str());
			}

			TurnCoefficients calculateArcadeTurns(float turn, float lateral)
			{
				turn *= -1;

				TurnCoefficients turnCoeffs = {turn, turn};

				// Allow for arc movement
				calculateArcMovement(turnCoeffs, lateral, turn);

				return turnCoeffs;
			}

			Horizontal arcadeControl()
			{
				float lateral = master->get_analog(ANALOG_LEFT_Y); // Gets amount forward/backward from left joystick
				float turn = master->get_analog(ANALOG_RIGHT_X);   // Gets the turn left/right from right joystick

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
			Horizontal tankControl()
			{
				float left = master->get_analog(ANALOG_LEFT_Y) * tankSpeeds[0].base;
				float right = master->get_analog(ANALOG_RIGHT_Y) * tankSpeeds[1].base;

				return {left, right};
			}

			float atacSigmoid(float absSpeed, const Vertical &sigmoid)
			{
				if (absSpeed < 0.5)
				{
					return 0.5 * std::pow(2 * absSpeed, sigmoid.low);
				}
				else
				{
					return 1 - 0.5 * std::pow(2 - (2 * absSpeed), sigmoid.high);
				}
			}

			// ATAC on individual axis (ran for each axis)
			float atacAxis(float speed, const TankSpeed &tankSpeed)
			{
				float absSpeed = std::fabs(speed);
				// Process deadbands
				if (absSpeed < tankSpeed.deadband)
				{
					return 0;
				}

				float sign = (speed < 0) ? -1 : 1;
				speed *= tankSpeed.base;
				speed = atacSigmoid(absSpeed, tankSpeed.sigmoid);
				speed *= sign;

				return speed;
			}

			// Advanced Tank Action Control: Implementing all features we've ever wanted
			Horizontal atac()
			{
				// Must use static_cast to avoid narrowing conversion warning as we are working with arrays
				float speeds[2] = {
					static_cast<float>(master->get_analog(ANALOG_LEFT_Y)),
					static_cast<float>(master->get_analog(ANALOG_RIGHT_Y))};

				int index = 0;
				for (float &speed : speeds)
				{
					// Rescale to -1 to 1 value
					speed /= MotorBounds::MOVE_MAX;

					// Process speed on one axis
					speed = atacAxis(speed, tankSpeeds[index]);

					// Rescale to -127 to 127 value
					speed *= MotorBounds::MOVE_MAX;

					index++;
				}

				// tell(0, "ROT LAT POS: " + std::to_string());

				return {speeds[0], speeds[1]};
			}

			// Final fallback driver control to default back to final working mode
			Horizontal fallbackControl()
			{
				return tankControl();
			}

			// REMEMBER: preparing the move speed must be done in Drivetrain class, NOT in DriveControl class
		public:
			/// @brief Sets the driver control mode
			/// @param mode Mode to set the driver control to
			void setDriveControlMode(DriveControlMode mode = DriveControlMode::ATAC)
			{
				driveControlMode = mode;

				switch (driveControlMode)
				{
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
			DriveControl(DriveControlArgs args) : AbstractComponent(args.abstractComponentArgs),
												  dio(args.dio)
			{
				setDriveControlMode();
			};

			void opControl() override
			{
				Horizontal speeds = driveControl();

				speeds.left = prepareMoveSpeed(speeds.left);
				speeds.right = prepareMoveSpeed(speeds.right);

				dio->move(speeds.left, speeds.right);
			}
		}; // class DriveControl

		class DriveManager : public AbstractComponent
		{
		private:
		protected:
		public:
			/// @brief Structure for DriveManager options provided by user
			/// @param drivePorts Ports for drivetrain
			/// @param imuPort Port for IMU
			struct DriveManagerUserArgs
			{
				DriveIO::DrivePorts drivePorts;
			};

			/// @brief Args for DriveManager object
			/// @param abstractComponentArgs Args for AbstractComponent object
			/// @param user Args for DriveManager object provideed by user
			struct DriveManagerArgs
			{
				AbstractComponentArgs abstractComponentArgs;
				DriveManagerUserArgs user;
			};

			DriveIO dio;
			DrivePID pid;
			DriveControl control;

			/// @brief Creates DriveManager object
			/// @param args Args for DriveManager object (check args struct for more info)
			DriveManager(DriveManagerArgs args) : AbstractComponent(args.abstractComponentArgs),
												  dio({args.user.drivePorts}),
												  pid({&dio}),
												  control({args.abstractComponentArgs, &dio}) {};

			void opControl() override
			{
				control.opControl();
			}
		}; // class DriveManager
	} // namespace Drivetrain

	/// @brief Class for GPS diagnostic
	class GPSDiagnostic : public AbstractComponent
	{
	private:
	protected:
	public:
		/// @brief Args for GPS diagnostic object
		/// @param abstractComponentArgs Args for AbstractComponent object
		/// @param gpsPort Port for GPS
		struct GPSDiagnosticArgs
		{
			AbstractComponentArgs abstractComponentArgs;
			uint8_t gpsPort;
		};

		pros::Gps gps;

		/// @brief Creates GPS diagnostic object
		/// @param args Args for GPS diagnostic object (check args struct for more info)
		GPSDiagnostic(GPSDiagnosticArgs args) : AbstractComponent(args.abstractComponentArgs),
												gps(args.gpsPort) {}

		void opControl() override
		{
		}
	}; // class GPSDiagnostic

	/// @brief Screens and reports state of game elements being held by robot
	class DynamicScreen : public AbstractComponent
	{
	private:
	protected:
	public:
		enum class State
		{
			DETECTED,
			UNDETECTED
		};

	private:
		State state = State::UNDETECTED;

		pros::Optical sensor;

	public:
		int proximityThreshold = 45; // Occlusion threshold

		struct SensorPorts
		{
			DigiPort optical;
		};

		/// @brief Args for dynamic screen object
		/// @param abstractComponentArgs Args for AbstractComponent object
		struct DynamicScreenArgs
		{
			AbstractComponentArgs abstractComponentArgs;
			SensorPorts sensorPorts;
		};

		/// @brief Creates dynamic screen object
		/// @param args Args for dynamic screen object (check args struct for more info)
		DynamicScreen(DynamicScreenArgs args) : AbstractComponent(args.abstractComponentArgs),
												sensor(args.sensorPorts.optical) {};

		void opControl()
		{
			// Detect for game element using proximity
			// TODO: Implement multi-colour sorting

			if (sensor.get_proximity() >= proximityThreshold)
			{
				state = State::DETECTED;
			}
			else
			{
				state = State::UNDETECTED;
			}

			pros::lcd::print(2, ("Proximity: " + std::to_string(sensor.get_proximity())).c_str());
		}

		/// @brief Retrieve state of game elements
		/// @return State of game elements
		State getState()
		{
			return state;
		}

	}; // class DynamicScreen

	// Basic pneumatic fork mechanism. Flexible to a variety of similar pneumatic mechanisms.
	class ForkMech : public AbstractMech
	{
	private:
		BtnManager btnMgr;

	protected:
	public:
		/// @brief Args for fork mechanism object
		/// @param abstractComponentArgs Args for AbstractComponent object
		struct ForkMechArgs
		{
			AbstractMechArgs abstractMechArgs;
			pros::controller_digital_e_t btn = pros::E_CONTROLLER_DIGITAL_LEFT;
		};

		/// @brief Creates fork mechanism object
		/// @param args Args for fork mechanism object (check args struct for more info)
		ForkMech(ForkMechArgs args) : AbstractMech(args.abstractMechArgs),
									  btnMgr({{args.abstractMechArgs.abstractComponentArgs},
											  {args.btn, {std::bind(&ForkMech::toggle, this)}, {}, {}}})
		{
		}

	
		void opControl() override
		{
			btnMgr.opControl();
		}
	}; // class ForkMech

	class Disperser : public AbstractComponent
	{
	private:
	protected:
	public:
		// Old struct MotorID gone because we now only have 1 Unified MG
		// and also: no more dynamic screen! Class still here but not used rn.	

		struct DisperserPorts
		{
			MGPorts score;
		};

		struct DisperserMechs {
			ForkMech* ballBlocker;
			ForkMech* descore;
		};

		pros::MotorGroup mg;

		DisperserMechs mechs;

		// TODO: Refactor holdPriority/xBlock flags into unified priority control system.

		// Whether HOLD actions are currently active and should take priority over TOGGLE actions.
		bool holdPriority = true;

		// Block the main command selection loop from running if we have the X timer active
		bool xBlock = false;

		int timer = 0;
		int timerDuration = 50;

		/// @brief Args for disperser object
		/// @param abstractComponentArgs Args for AbstractComponent object
		struct DisperserArgs
		{
			AbstractComponentArgs abstractComponentArgs;
			DisperserPorts ports;
			DisperserMechs mechs;
		};

		struct Command {
			std::optional<bool> holdPriority; // true for HOLD priority, false for TOGGLE priority, nullopt for no change
			std::optional<int> mgSpeed; // Speed to set the motor group to, nullopt for no change
			std::optional<bool> ballBlocker; // true for block, false for unblock, nullopt for no change
			std::optional<bool> descore; // true for descore, false for no descore, nullopt for no change
		};

		/// @brief Creates disperser object
		/// @param args Args for disperser object (check args struct for more info)
		Disperser(DisperserArgs args) : AbstractComponent(args.abstractComponentArgs),
										// Initialize array elements with the provided port groups
										mg(args.ports.score),
										mechs(args.mechs) {};

		void processCmd(Command cmd) {
			xBlock = false; // Any new command should unblock the X timer block

			if (cmd.holdPriority.has_value()) {
				holdPriority = cmd.holdPriority.value();
			}

			if (cmd.mgSpeed.has_value()) {
				mg.move(cmd.mgSpeed.value());
			}

			if (cmd.ballBlocker.has_value()) {
				mechs.ballBlocker->actuate(cmd.ballBlocker.value());
			}

			if (cmd.descore.has_value()) {
				mechs.descore->actuate(cmd.descore.value());
			}
		}

		void stopCmd() {
			processCmd({.mgSpeed = 0});
		}

		void handleR2() {
			// holdPriority should DOUBLE AS a flag for whether R2 is being toggled or not
			// so we turn it OFF when start toggle, ON when end toggle. 
			// and also: if a HOLD button is triggered, this INTERRUPTS the toggle and sets holdPriority to TRUE so that the toggle isnt being considered active anymore.
			if (holdPriority) 
			{
				// toggle START
				processCmd({.holdPriority = false, .mgSpeed = 127, .ballBlocker = false});
			} else {
				// toggle END
				processCmd({.holdPriority = true, .mgSpeed = 0});
			}
		}

		void handleXTimer() {
			if (timer <= 0) 
			{
				xBlock = false;
				timer = 0;
				stopCmd();
				return;
			}

			timer -= 1;
		}

		void handleR1() 
		{
			processCmd({.holdPriority = true, .mgSpeed = -127, .ballBlocker = false});
		}

		void handleL1(int mgSpeed = 127)
		{
			processCmd({.holdPriority = true, .mgSpeed = mgSpeed, .ballBlocker = true, .descore = false});
		}

		void handleL2()
		{
			processCmd({.holdPriority = true, .mgSpeed = 127, .ballBlocker = true, .descore = true});
		}

		void handleB() 
		{
			processCmd({.holdPriority = true, .mgSpeed = -127, .ballBlocker = true, .descore = false});
		}

		void handleX()
		{
			processCmd({.mgSpeed = -60});
			xBlock = true;
			master->print(0, 0, "X TIMER NOW");
			timer = timerDuration; // roughly 1 second - 50 opcontrol cycles at ~ 25-30 ms each
		}

		void stopCheck()
		{
			if (holdPriority && !xBlock) // Only stop if no buttons are pressed and HOLD priority is active and xblock not active
			{
				master->print(0, 0, "STOP CMD");
				stopCmd();
			}
		}

		void selectCmd() 
		{
			// R2: mg fwd + bb false on TOGGLE
			
			// R1: mg reverse + bb false on HOLD
			// L1: mg fwd + bb true + descore false on HOLD
			// L2: mg fwd + bb true + descore true on HOLD
			// all HOLD should take holdPriority over TOGGLE
			// stop on: nothing pressed AND holdPriority active - because we dont want it to stop when R2 is toggled

			// do this later - only worry about HOLD/TOGGLE interactions for now
			// X: mg reverse slow only for 1 second - then stop.

			if (master->get_digital(pros::E_CONTROLLER_DIGITAL_R1))
			{
				handleR1();
			}
			else if (master->get_digital(pros::E_CONTROLLER_DIGITAL_L2))
			{
				handleL2();
			}
			else if (master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
			{
				handleR2();
			}
			else if (master->get_digital(pros::E_CONTROLLER_DIGITAL_L1))
			{
				handleL1();
			}
			else if (master->get_digital(pros::E_CONTROLLER_DIGITAL_B))
			{
				handleB();
			}
			else if (master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
			{
				handleX();
			}
			else 
			{
				stopCheck();
			}

			if (xBlock) 
			{
				handleXTimer();
			}
		} 


		void opControl() override
		{
			selectCmd();
		}
	}; // class Disperser

	/// @brief Class to manage opcontrol shortcuts to simplify driving (e.g. single buttons to trigger complex functions)
	class ShortcutManager : public AbstractComponent
	{
		private:
		protected:
		public:
			const pros::controller_analog_e_t ANALOG_CHANNELS[4] = {ANALOG_LEFT_X, ANALOG_LEFT_Y, ANALOG_RIGHT_X, ANALOG_RIGHT_Y};

			int interruptThreshold = 10; // Threshold for joystick movement to detect interrupt	

			// Detects whether the threshold of the joystick has been crossed to cancel a shortcut while its happening.
			// e.g. if you decide you don't want the shortcut to run anymore: simply move the joystick and this should
			// detect and cancel the shortcut.
			bool detectInterrupt() {
				for (const pros::controller_analog_e_t &channel : ANALOG_CHANNELS) {
					const int value = master->get_analog(channel);
					
					if (std::fabs(value) >= interruptThreshold) {
						return true;
					}
				}

				return false;
			}

			/// @brief Args for shortcut manager object
			/// @param abstractComponentArgs Args for AbstractComponent object
			struct ShortcutManagerArgs
			{
				AbstractComponentArgs abstractComponentArgs;
			};

			/// @brief Creates shortcut manager object
			/// @param args Args for shortcut manager object (check args struct for more info)
			ShortcutManager(ShortcutManagerArgs args) : 
				AbstractComponent(args.abstractComponentArgs) {};

			void opControl() override
			{
				// to be implemented: shortcut functions for easy driver control
			}
	}; // class ShortcutManager

	/// @brief Class which manages all components
	class ComponentManager : public AbstractComponent
	{
	private:
	protected:
	public:
		Drivetrain::DriveManager drive;

		DynamicScreen screen;

		Disperser disp;

		ForkMech fork;
		ForkMech descore;
		ForkMech ballBlocker;

		Timer timer;

		ShortcutManager shortcuts;

		// All components are stored in this vector
		vector<AbstractComponent *> components;

		/// @brief Args for component manager object passed to the chassis, such as ports
		/// @param dvtPorts Ports for drivetrain
		/// @param dispPorts Ports for disperser
		struct ComponentManagerUserArgs
		{
			Drivetrain::DriveManager::DriveManagerUserArgs driveArgs;
			Disperser::DisperserPorts dispPorts;
			DynamicScreen::SensorPorts screenPorts;
			AnalogPort forkPort;
			AnalogPort descorePort;
			AnalogPort ballBlockerPort;
		};

		/// @brief Args for component manager object
		/// @param aca Args for AbstractComponent object
		/// @param user Args for component manager object passed to the chassis
		struct ComponentManagerArgs
		{
			AbstractComponentArgs aca;
			ComponentManagerUserArgs user;
		};

		/// @brief Constructor for component manager object
		/// @param args Args for component manager object (see args struct for more info)
		ComponentManager(ComponentManagerArgs args) : AbstractComponent(args.aca),

													  drive({args.aca, args.user.driveArgs}),
													  screen({args.aca, args.user.screenPorts}),
													  fork({{{args.aca, args.user.forkPort}, pros::E_CONTROLLER_DIGITAL_LEFT}}),
													  descore({{{args.aca, args.user.descorePort}, pros::E_CONTROLLER_DIGITAL_DOWN}}),
													  ballBlocker({{{args.aca, args.user.ballBlockerPort}, pros::E_CONTROLLER_DIGITAL_UP}}),
													  disp({args.aca, args.user.dispPorts, {&ballBlocker, &descore}}), 
													  timer({args.aca}),
													  shortcuts({args.aca})
		{
			// Add component pointers to vector
			// MUST BE DONE AFTER INITIALISATION not BEFORE because of pointer issues
			components = {
				&drive,
				&disp,
				&timer,
				&fork,
				&descore,
				&ballBlocker,
				&screen,
				&shortcuts
			};
		};

		// Nice and simple :) definitely better than having to call each component individually
		void opControl() override
		{
			for (AbstractComponent *component : components)
			{
				component->opControl();
			}
		}

		void skillsPrep() override
		{
			for (AbstractComponent *component : components)
			{
				component->skillsPrep();
			}
		}

		void postAuton() override
		{
			for (AbstractComponent *component : components)
			{
				component->postAuton();
			}
		}
	}; // class ComponentManager

	/// @brief Abstract class for auton e.g. match or skills autonomous
	class AbstractAuton
	{
	private:
	protected:
		ComponentManager *cm;

	public:
		/// @brief Args for auton object
		/// @param cm Component manager object
		struct AutonArgs
		{
			ComponentManager *cm;
		};

		/// @brief Creates auton object
		/// @param args Args for auton object (check args struct for more info)
		AbstractAuton(AutonArgs args) : cm(args.cm) {};

		/// @brief Runs the auton
		virtual void run() = 0;

		virtual ~AbstractAuton() = default;
	}; // class AbstractAuton

	class MatchAuton : public AbstractAuton
	{
	private:
		void defaultLeft()
		{
			cm->descore.actuate(true);

			cm->fork.actuate(false);
			cm->disp.handleR2();

			cm->drive.pid.lateral(25, 3);

			pros::delay(1000);

			cm->drive.pid.turn(-110, 2, 2500);
			pros::lcd::print(0, "Stopping");
			//cm->disp.handleStop();
			// cm->fork.actuate(true);

			pros::delay(250);
			pros::lcd::print(0, "TLAT Start");

			// to be implemented once our lateral function accepts dynamic checkpoint triggers
			/*cm->drive.pid.lateral(15, 4, 2500, true, [this]() {
				this->cm->fork.actuate(true);
			});*/

			pros::lcd::print(0, "TLAT End");
			pros::delay(250);
			cm->drive.pid.lateral(-20, 3);
			cm->disp.handleR1();
			pros::delay(500);
			cm->disp.handleL1(80);
			pros::delay(5000);
		}

		void defaultRight()
		{
			cm->disp.handleR2();
			// Mirror of defaultLeft: invert turn angles and use bottom goal instead of mid goal
			cm->fork.actuate(true);
			//cm->disp.handleIntake();

			cm->drive.pid.lateral(20, 2);

			

			// cm->drive.pid.lateral(-2, 4, 1000); // small back up to align with goal

			pros::delay(500);

			cm->drive.pid.turn(90, 2); // inverted angle

			pros::lcd::print(0, "Stopping");
			//cm->disp.handleStop();

			// was told not to deploy - just dont? alr ig :)))
			// cm->fork.actuate(true);

			pros::delay(250);
			pros::lcd::print(0, "TLAT Start");
			cm->drive.pid.lateral(3, 3, 1500);
			pros::lcd::print(0, "TLAT End");

			//cm->disp.handleBottomGoal(); // bottom goal instead of mid goal
			pros::delay(1500);
			cm->drive.pid.lateral(-15, 2);
			cm->drive.pid.turn(15, 2); // inverted angle
			cm->drive.pid.lateral(-25, 2);
			pros::delay(500);
			cm->disp.handleR1();
			pros::delay(50);
			cm->disp.handleL1();
			pros::delay(5000);
		}

		void advancedAuton()
		{
			cm->fork.actuate(true);
			// Mirror of defaultLeft: invert turn angles and use bottom goal instead of mid goal
			

			cm->drive.pid.lateral(25, 2);

			// cm->drive.pid.lateral(-2, 4, 1000); // small back up to align with goal

			pros::delay(500);

			cm->drive.pid.turn(80, 2); // inverted angle

			pros::lcd::print(0, "Stopping");
			//cm->disp.handleIntake();
			pros::delay(700);
			

			// was told not to deploy - just dont? alr ig :)))
			// cm->fork.actuate(true);

			pros::delay(250);
			pros::lcd::print(0, "TLAT Start");
			cm->drive.pid.lateral(15, 1, 1500);
			pros::delay(500);
			cm->drive.pid.lateral(-10, 3, 1000);

			
			pros::lcd::print(0, "TLAT End");
			cm->drive.pid.turn(175, 2,1000); // inverted angle
			cm->fork.actuate(false);
			//cm->disp.handleStop();
											  // bottom goal instead of mid goal
			pros::delay(500);
			cm->drive.pid.lateral(4, 3);
			pros::delay(500);
			//cm->disp.handleTopGoal();
			pros::delay(5000);
			//cm->disp.handleStop();
			cm->drive.pid.lateral(-12, 3);
			pros::delay(500);
			cm->drive.pid.turn(-50, 2, 2500); // inverted angle
			pros::delay(500);
			//cm->disp.handleIntake();
			cm->drive.pid.lateral(40, 2);
			pros::delay(500);
			//cm->disp.handleStop();
			pros::delay(500);
			//cm->disp.handleBottomGoal();
			pros::delay(5000);
		}

		// NG = New Generation
		// Cleansheet design, cleansheeet ideas.
		void ngLeft()
		{
			// Prepare: Fork enabled
			cm->fork.actuate(true);

			// Advance to RIGHT/0.5, turn 90 towards RIGHT/0, advance towards RIGHT/0, spin to intake
			cm->drive.pid.lateral(23);
			cm->drive.pid.turn(95);
			cm->fork.actuate(false);
			//cm->disp.handleIntake();
			cm->drive.pid.lateral(8, 4);

			// Get blocks from RIGHT/0
			pros::delay(1000);

			// Reverse to RIGHT/0.5, uturn towards RIGHT/1, advance towards RIGHT/1
			cm->drive.pid.lateral(-6);
			cm->drive.pid.uTurn();
			//cm->disp.handleStop();
			cm->fork.actuate(true);
			cm->drive.pid.lateral(8);

			// Set blocks to RIGHT/1, stop disp
			//cm->disp.handleTopGoal();
			pros::delay(5000);
			//cm->disp.handleStop();

			// Reverse to RIGHT/0.75, turn towards MID/0.5, capture CROSS/X
			cm->drive.pid.lateral(-5);
		}

		void ngRight()
		{
		}

		void testRight90()
		{
		}

		void testReverse()
		{
			cm->drive.pid.lateral(-15);
			pros::delay(10000);
		}

		void testTinyLat()
		{
			pros::delay(1000);
			cm->drive.pid.lateral(10, 4);
			pros::delay(10000);
		}

		void testFwd2Tiles()
		{
			cm->drive.pid.lateral(48);
		}

		void testMechs()
		{
			cm->fork.actuate(false);
		}

	protected:
	public:
		/// @brief Args for match auton object
		/// @param autonArgs Args for auton object
		struct MatchAutonArgs
		{
			AutonArgs autonArgs;
		};

		/// @brief Creates match auton object
		/// @param args Args for match auton object (check args struct for more info)
		MatchAuton(MatchAutonArgs args) : AbstractAuton(args.autonArgs) {};

		void run() override
		{
			defaultLeft();
			//defaultRight();

			
			//advancedAuton();

			//ngLeft();
			// ngRight();

			// testRight90();
			// testFwd2Tiles();

			// testTinyLat();
			// testMechs();
			// testReverse();
		}
	}; // class MatchAuton

	class SkillsAuton : public AbstractAuton
	{
	private:
		void sector1()
		{ 
			//cm->disp.handleIntake();
			cm->drive.pid.lateral(20);

			cm->fork.actuate(true);
			pros::delay(500);
			cm->drive.pid.turn(-90);
			pros::delay(500);
			cm->drive.pid.lateral(10, 2, 2000);
			pros::delay(5000);
			cm->drive.pid.lateral(10, 2, 2000);
			pros::delay(5000);
		
			cm->drive.pid.lateral(-5);
			pros::delay(5000);
			//cm->disp.handleStop();
			pros::delay(5000);
			cm->fork.actuate(false);
			cm->drive.pid.uTurn();
			pros::delay(5000);
			cm->drive.pid.lateral(5);
			//cm->disp.handleTopGoal();
		}

		void sector2()
		{
		}

	protected:
	public:
		/// @brief Args for skills auton object
		/// @param autonArgs Args for auton object
		struct SkillsAutonArgs
		{
			AutonArgs autonArgs;
		};

		/// @brief Creates skills auton object
		/// @param args Args for skills auton object (check args struct for more info)
		SkillsAuton(SkillsAutonArgs args) : AbstractAuton(args.autonArgs) {};

		void run() override
		{
			cm->tell(0, "Skills auton running");

			sector1();
		}
	}; // class SkillsAuton

	/// @brief Chassis class for controlling auton/driver control
	class Chassis : public AbstractChassis
	{
	private:
	protected:
	public:
		/// @brief Args for chassis object
		/// @param cmUserArgs Args for component manager object
		struct ChassisArgs
		{
			ComponentManager::ComponentManagerUserArgs cmUserArgs;
		};

		ComponentManager cm;

		MatchAuton matchAutonManager;
		SkillsAuton skillsAutonManager;

		/// @brief Creates chassis object
		/// @param args Args for chassis object (check args struct for more info)
		Chassis(ChassisArgs args) : AbstractChassis(),
									cm({this, args.cmUserArgs}),
									matchAutonManager({&cm}),
									skillsAutonManager({&cm}) {};

		/// @brief Runs the opcontrol functions for each component
		void opControl() override
		{
			cm.opControl();
		}

		/// @brief Auton function for the chassis
		// 1000 = 70cm
		void auton() override
		{
			matchAutonManager.run();
		}

		/// @brief Skills auton function for the chassis
		void skillsAuton() override
		{
			skillsAutonManager.run();
		}

		/// @brief Skills preparation for opcontrol on the chassis
		void skillsPrep() override
		{
			// We need to run postAuton() first because these are what would prep for opcontrol normally
			cm.skillsPrep();
		}

		/// @brief Post auton function for the chassis
		void postAuton() override
		{
			cm.postAuton();
		}
	}; // class Chassis
} // namespace hyper

// Global variables

// DONT say just "chassis" because certain class properties have the same name
hyper::AbstractChassis *currentChassis;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void initDefaultChassis()
{
	static hyper::Chassis defaultChassis({{// ComponentManagerUserArgs
										   {
											   // Drivetrain args
											   {{LEFT_DRIVE_PORTS, RIGHT_DRIVE_PORTS, IMU_PORT, LAT_ROT_DRIVE_PORT}},
											   // Disperser ports
											   {DISP_SCORING_PORT},
											   // Dynamic screen ports
											   //{SCREEN_TOP_PORT},
											   //AI Vision Sensor Port
											   {AI_VISION_PORT},
											   {FORK_MECH_PORT},
											   {DESCORE_MECH_PORT},
											   {BALL_BLOCKER_MECH_PORT}
											}}});

	currentChassis = &defaultChassis;
}

void initialize()
{
	pros::lcd::initialize();

	INIT_CHASSIS();
}

void disabled()
{
	// when robot is paused
}

void competition_initialize()
{
	// addiitonal initialization for competition mode
}

void autonomous()
{
#if DO_MATCH_AUTON
	currentChassis->auton();
#elif DO_SKILLS_AUTON
	currentChassis->skillsAuton();
#endif
}

void preControl()
{
	pros::lcd::set_text(0, "> 1408Hyper mainControl ready");

	bool inComp = pros::competition::is_connected();

	// Run autonomous even if we are NOT in the compeition
	if (!inComp)
	{
		autonomous();
	}

#if DO_SKILLS_PREP
	currentChassis->skillsPrep();
#endif

// only do post auton if we are not in skills prep
// 21/06/2025: What the hell does this comment even mean?
#if DO_POST_AUTON
	currentChassis->postAuton();
#endif
}

void mainloopControl()
{
	bool opControlRunning = DO_OP_CONTROL;
	// Chassis control loop
	while (opControlRunning)
	{
		// Chassis opcontrol
		currentChassis->opControl();
	}
}

void mainControl()
{
	preControl();
	mainloopControl();
}

void opcontrol()
{
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
// aaaaaaa