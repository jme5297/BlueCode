#pragma once
#include <Parser.h>

namespace Times {

	using namespace std::chrono;
	using std::vector;
	using std::string;
	using std::tuple;

	/*!
	   * Main class for handling time-based commands.
	   * This class has different implementations whether DEBUG flag is activated
	 * when compiling this code. If DEBUG flag is not active, then c++ std::chrono information
	 * is used to track time. If DEBUG is active, then a standard counter is used (with the counter simDelta
	 * being set in the config file) to track the passage of "fake" time.
	   */
	class TimeModule {
	public:
		TimeModule();

		/// Create a process counter. Used to tracking frequency-dependent commands
		static void InitProccessCounter(string str, double dt);
		/// Add a milestone at a specific time of the simulation.
		static void AddMilestone(string str);
		/// Check to see if enough time has passed to cycle a frequency-based function.
		static bool ProccessUpdate(string str);
		/// Get elapsed time since the previous milestone.
		static double GetElapsedTime(string str);
		/// Get the amount of time passed since the last cycle of a frequency-based function.
		static double GetProccessDelta(string str);
		static double GetLastProccessDelta(string str);
		/// Log a message (requires call from both a person/entity, and then the message itself)
		static void Log(string, string);
#ifdef DEBUG
		/// Setup - the amount of time passed for each iteration when debugging.
		static void SetTimeSimDelta(double d);
		/// Update the current simulation time.
		static void Run();
#endif

		static void Initialize();

	protected:

#ifndef DEBUG
		/// Milestones - each milestone is a name/ID and a time.
		static std::vector< tuple<string, time_point<steady_clock> > > milestones;
		/// Proccess - Each process is a name/ID, process delta, and time.
		static std::vector< tuple<string, double, time_point<steady_clock>, double > > processes;
#else
		static double currentSimTime;
		static double simDelta;
		static std::vector< tuple<string, double > > milestones;
		static std::vector< tuple<string, double, double, double > > processes;
#endif
		static int FindMilestoneIndex(string str);
		static int FindProccessIndex(string str);

	};
}
