#include "CoaXSimulator.h"

#include <cstdio>
#include <cassert>
#include <cmath>

using namespace std;

CoaXSimulator::CoaXSimulator()
{
  simulation_time = 0;
  // This is where we set the simulation time
  // It should match the processing rate onboard the robot
  time_step = 1e-2;

  return;
}

CoaXSimulator::~CoaXSimulator()
{
  return;
}

double CoaXSimulator::GetSimulationTime()
{
  return simulation_time;
}

CoaXModel* CoaXSimulator::GetModelPtr()
{
  return &coax;
}

int CoaXSimulator::CheckCollision()
{
  double x, y, z;
  coax.GetXYZ(x, y, z);

  // Simple ground plane check
  if (z <= 0)
    {
      // Put the robot just above the ground
      coax.SetXYZ(x, y, 1e-2);
      return -1;
    }

  return 0;
}

void CoaXSimulator::ResetSimulation()
{
  simulation_time = 0;
  coax.ResetSimulation();
}

void CoaXSimulator::Update()
{
  simulation_time += time_step;
  coax.Update(simulation_time);

  if (CheckCollision() != 0)
    {
      double x, y, z;
      double roll, pitch, yaw;
		printf("simulation reset (ground collision) \n");
      coax.GetXYZ(x, y, z);
      coax.GetRotation(roll, pitch, yaw);
      coax.ResetSimulation(simulation_time,
                           x, y, z, roll, pitch, yaw);
    }

  return;
}
