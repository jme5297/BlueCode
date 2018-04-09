#include <TimeModule.h>

using namespace Times;
using namespace std::chrono;

#ifndef DEBUG
std::vector< tuple<string, time_point<steady_clock> > > TimeModule::milestones;
std::vector< tuple<string, double, time_point<steady_clock>, double > > TimeModule::processes;
#else
double TimeModule::currentSimTime;
double TimeModule::simDelta;
std::vector< tuple<string, double > > TimeModule::milestones;
std::vector< tuple<string, double, double, double > > TimeModule::processes;
#endif

// Output stream
std::ofstream output;
std::string name;

TimeModule::TimeModule() {

}

void TimeModule::Initialize(){
	#ifdef DEBUG
		currentSimTime = 0.0;
	#endif
		if(Parser::GetWriteToLogFile()){
			// Data
			int ii = 0;
			name = "mxlog_" + std::to_string(ii) + ".csv";
			std::ifstream f(name.c_str());
			while (f.good()) {
				ii = ii + 1;
				name = "mxlog_" + std::to_string(ii) + ".csv";
				f.close();
				f.open(name.c_str());
			}
			f.close();
		}
}

void TimeModule::Log(string agent, string message) {
	std::cout <<
		"[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "][" <<
		agent << "]: " << message << "\n";

	// Open a file to save the output information the vehicle.
	if(Parser::GetWriteToLogFile()){
		output.open((name).c_str(), std::ofstream::app);
		output <<
			std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "," <<
			agent << "," <<
			message << "\n";
		output.close();
	}
}

#ifdef DEBUG
void TimeModule::SetTimeSimDelta(double d) {
	simDelta = d;
	return;
}
void TimeModule::Run() {
	currentSimTime += simDelta;
	return;
}
#endif

void TimeModule::InitProccessCounter(string str, double dt) {
#ifndef DEBUG
	processes.push_back(tuple<string, double, time_point<steady_clock>, double >{str, dt, steady_clock::now(), 0.0});
#else
	processes.push_back(tuple<string, double, double, double >{str, dt, currentSimTime, 0.0});
#endif
	return;
}
void TimeModule::AddMilestone(string str) {
#ifndef DEBUG
	milestones.push_back(tuple<string, time_point<steady_clock> >{str, steady_clock::now()});
#else
	milestones.push_back(tuple<string, double >{str, currentSimTime});
#endif
	return;
}
bool TimeModule::ProccessUpdate(string str) {
	int ind = FindProccessIndex(str);
	if (ind == -1)
		return false;

#ifndef DEBUG
	duration<double, std::ratio<1, 1> > delta = steady_clock::now() - std::get<2>(processes[ind]);
	if (delta.count() >= std::get<1>(processes[ind])) {
		std::get<2>(processes[ind]) = steady_clock::now();
		std::get<3>(processes[ind]) = delta.count();
		return true;
	}
#else
	double delta = currentSimTime - std::get<2>(processes[ind]);
	if (delta >= std::get<1>(processes[ind])) {
		std::get<2>(processes[ind]) = currentSimTime;
		std::get<3>(processes[ind]) = delta;
		return true;
	}
#endif
	return false;
}
double TimeModule::GetElapsedTime(string str) {
	int ind = FindMilestoneIndex(str);
	if (ind == -1)
		return 0.0;

#ifndef DEBUG
	duration<double, std::ratio<1, 1> > delta = steady_clock::now() - std::get<1>(milestones[ind]);
	return delta.count();
#else
	double delta = currentSimTime - std::get<1>(milestones[ind]);
	return delta;
#endif
}
double TimeModule::GetProccessDelta(string str) {
	int ind = FindProccessIndex(str);
	if (ind == -1)
		return 0.0;

#ifndef DEBUG
	duration<double, std::ratio<1, 1> > delta = steady_clock::now() - std::get<2>(processes[ind]);
	return delta.count();
#else
	double delta = currentSimTime - std::get<2>(processes[ind]);
	return delta;
#endif
}
double TimeModule::GetLastProccessDelta(string str) {
	int ind = FindProccessIndex(str);
	if (ind == -1)
		return 0.0;

	return std::get<3>(processes[ind]);
}
int TimeModule::FindMilestoneIndex(string str) {
	unsigned int i = 0;
	bool found = false;
	for (i = 0; i < milestones.size(); i++) {
		if (std::get<0>(milestones[i]) == str) {
			found = true;
			break;
		}
	}
	if (!found)
		return -1;

	return (int)i;
}
int TimeModule::FindProccessIndex(string str) {
	unsigned int i = 0;
	bool found = false;
	for (i = 0; i < processes.size(); i++) {
		if (std::get<0>(processes[i]) == str) {
			found = true;
			break;
		}
	}
	if (!found)
		return -1;

	return (int)i;
}
