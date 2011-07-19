#ifndef __COAXSIMULATOR__
#define __COAXSIMULATOR__
#include <string>
#include "CoaXModel.h"

class CoaXSimulator
{
 public:
  CoaXSimulator();
  ~CoaXSimulator();

  double GetSimulationTime();
  void SendCommand();
  void Update();
  CoaXModel* GetModelPtr();

  void ResetSimulation();

private:
  double time_step;
  double simulation_time;

  CoaXModel coax;

  int CheckCollision();
};
#endif
