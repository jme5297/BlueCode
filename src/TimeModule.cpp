#include <TimeModule.h>

using namespace Times;
using namespace std::chrono;

TimeModule::TimeModule(){
  #ifdef DEBUG
  currentSimTime = 0.0;
  #endif
}

#ifdef DEBUG
void TimeModule::SetTimeSimDelta(double d){
  simDelta = d;
  return;
}
void TimeModule::Run(){
  currentSimTime += simDelta;
  return;
}
#endif

void TimeModule::InitProccessCounter(string str, double dt){
  #ifndef DEBUG
  processes.push_back({str, dt, steady_clock::now(), 0.0});
  #else
  processes.push_back({str, dt, currentSimTime, 0.0});
  #endif
  return;
}
void TimeModule::AddMilestone(string str){
  #ifndef DEBUG
  milestones.push_back({str, steady_clock::now()});
  #else
  milestones.push_back({str, currentSimTime});
  #endif
  return;
}
bool TimeModule::ProccessUpdate(string str){
  int ind = FindProccessIndex(str);
  #ifndef DEBUG
  duration<double, std::ratio<1,1> > delta = steady_clock::now() - std::get<2>(processes[ind]);
  if(delta.count() >= std::get<1>(processes[ind])){
    std::get<2>(processes[ind]) = steady_clock::now();
    std::get<3>(processes[ind]) = delta.count();
    return true;
  }
  #else
  double delta = currentSimTime - std::get<2>(processes[ind]);
  if(delta >= std::get<1>(processes[ind])){
    std::get<2>(processes[ind]) = currentSimTime;
    std::get<3>(processes[ind]) = delta;
    return true;
  }
  #endif
  return false;
}
double TimeModule::GetElapsedTime(string str){
  int ind = FindMilestoneIndex(str);
  #ifndef DEBUG
  duration<double, std::ratio<1,1> > delta = steady_clock::now() - std::get<1>(milestones[ind]);
  return delta.count();
  #else
  double delta = currentSimTime - std::get<1>(milestones[ind]);
  return delta;
  #endif
}
double TimeModule::GetProccessDelta(string str){
  int ind = FindProccessIndex(str);
  #ifndef DEBUG
  duration<double, std::ratio<1,1> > delta = steady_clock::now() - std::get<2>(processes[ind]);
  return delta.count();
  #else
  double delta = currentSimTime - std::get<2>(processes[ind]);
  return delta;
  #endif
}
double TimeModule::GetLastProccessDelta(string str){
  int ind = FindProccessIndex(str);
  return std::get<3>(processes[ind]);
}
int TimeModule::FindMilestoneIndex(string str){
  int i = 0;
  for(i = 0; i < milestones.size(); i++){
     if(std::get<0>(milestones[i]) == str){
       break;
     }
  }
  return i;
}
int TimeModule::FindProccessIndex(string str){
  int i = 0;
  for(i = 0; i < processes.size(); i++){
     if(std::get<0>(processes[i]) == str){
       break;
     }
  }
  return i;
}
