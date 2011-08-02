#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Empty.h>

#include "CoaXSimulator.h"
#include "ROSCoaX.h"

CoaXSimulator simulator;

void reset(const std_msgs::Empty::ConstPtr &msg)
{
  ROS_INFO("%s: resetting simulation", ros::this_node::getName().c_str());
  simulator.ResetSimulation();
}

void load_model_params(ros::NodeHandle &n)
{
  CoaXModel *model = simulator.GetModelPtr();

  double m;
  n.getParam("mass", m);
  model->SetMass(m);

  double Ixx, Iyy, Izz;
  n.getParam("inertia/Ixx", Ixx);
  n.getParam("inertia/Iyy", Iyy);
  n.getParam("inertia/Izz", Izz);
  model->SetInertia(Ixx, Iyy, Izz);

  double d_up, d_lo;
  n.getParam("offset/upper", d_up);
  n.getParam("offset/lower", d_lo);
  model->SetRotorOffset(d_up, d_lo);

  double l_up, l_lo;
  n.getParam("linkage_factor/upper", l_up);
  n.getParam("linkage_factor/lower", l_lo);
  model->SetRotorLinkageFactor(l_up, l_lo);

  double k_springup, k_springlo;
  n.getParam("spring_constant/upper", k_springup);
  n.getParam("spring_constant/lower", k_springlo);
  model->SetRotorSpringConstant(k_springup, k_springlo);

  double k_Tup, k_Tlo;
  n.getParam("thrust_factor/upper", k_Tup);
  n.getParam("thrust_factor/lower", k_Tlo);
  model->SetRotorThrustFactor(k_Tup, k_Tlo);

  double k_Mup, k_Mlo;
  n.getParam("moment_factor/upper", k_Mup);
  n.getParam("moment_factor/lower", k_Mlo);
  model->SetRotorMomentFactor(k_Mup, k_Mlo);

  double Tf_up;
  n.getParam("following_time/bar", Tf_up);
  model->SetUpperRotorFollowingTime(Tf_up);

  double Tf_motup, Tf_motlo;
  n.getParam("following_time/motors/upper", Tf_motup);
  n.getParam("following_time/motors/lower", Tf_motlo);
  model->SetMotorFollowingTime(Tf_motup, Tf_motlo);

  double rs_mup, rs_bup;
  n.getParam("speed_conversion/slope/upper", rs_mup);
  n.getParam("speed_conversion/offset/upper", rs_bup);
  model->SetUpperRotorSpeedConversion(rs_mup, rs_bup);

  double rs_mlo, rs_blo;
  n.getParam("speed_conversion/slope/lower", rs_mlo);
  n.getParam("speed_conversion/offset/lower", rs_blo);
  model->SetLowerRotorSpeedConversion(rs_mlo, rs_blo);
	
  double zeta_mup, zeta_bup;
  n.getParam("phase_lag/slope/upper", zeta_mup);
  n.getParam("phase_lag/offset/upper", zeta_bup);
  model->SetUpperPhaseLag(zeta_mup, zeta_bup);
	
  double zeta_mlo, zeta_blo;
  n.getParam("phase_lag/slope/lower", zeta_mlo);
  n.getParam("phase_lag/offset/lower", zeta_blo);
  model->SetLowerPhaseLag(zeta_mlo, zeta_blo);

  double max_SPangle;
  n.getParam("max_swashplate_angle", max_SPangle);
  model->SetMaximumSwashPlateAngle(max_SPangle);
  
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coax_simulator");
  ros::NodeHandle n("~");

  bool use_sim_time;
  n.param("use_sim_time", use_sim_time, true);

  if (use_sim_time)
    n.setParam("/use_sim_time", true);

  // Need to load model params before instantiating ROSCoaX object
  load_model_params(n);

  std::string name("coax");
  ROSCoaX coax(simulator.GetModelPtr(), n, name);

  std::string frame_id;
  n.param("frame_id", frame_id, std::string("coax"));
  coax.SetFrameId(frame_id);

  double init_x, init_y, init_z;
  n.param("init/x", init_x, 0.0);
  n.param("init/y", init_y, 0.0);
  n.param("init/z", init_z, 0.0);

  simulator.GetModelPtr()->SetInitialXYZ(init_x, init_y, init_z);
  simulator.GetModelPtr()->SetInitialRotation(0, 0, 0);
  
  double init_Omega_up, init_Omega_lo;
  n.param("init/Omega_up", init_Omega_up, 0.0);
  n.param("init/Omega_lo", init_Omega_lo, 0.0);
  simulator.GetModelPtr()->SetInitialRotorSpeeds(init_Omega_up, init_Omega_lo);

  simulator.GetModelPtr()->SetInitialStabilizerBar(0, 0, 1);

  int speedup;
  n.param("speedup",speedup,1);
  if (speedup < 1)
    {
      ROS_ERROR("Simulation speedup of %d is not possible",speedup);
      return -1;
    }

  ros::WallRate r(speedup*100);

  ros::Subscriber sub = n.subscribe("reset", 10, reset);
  ros::Publisher clock_pub = n.advertise<rosgraph_msgs::Clock>("/clock", 100);

  rosgraph_msgs::Clock msgc;

  while (n.ok())
    {
      simulator.Update();

      if (use_sim_time)
        {
          msgc.clock = ros::Time(simulator.GetSimulationTime());
          clock_pub.publish(msgc);
        }

      ros::spinOnce();

      simulator.GetModelPtr()->SendCommand();

      r.sleep();
    }

  return 0;
}
