#ifndef __COAX_MODEL__
#define __COAX_MODEL__

#include <string>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_odeiv.h>

#include "CoaXOnboardControl.h"

#define DIMENSION 17

typedef struct
{
  double mass;
  double Ixx, Iyy, Izz;
  double d_up, d_lo;
  double k_springup, k_springlo;
  double l_up, l_lo;
  double k_Tup, k_Tlo;
  double k_Mup, k_Mlo;
  double Tf_motup, Tf_motlo;
  double Tf_up;
  double rs_mup, rs_bup;
  double rs_mlo, rs_blo;
  double zeta_mup, zeta_bup;
  double zeta_mlo, zeta_blo;
  double max_SPangle;
  double acc[3];

  // Control inputs
  double control[4];
} model_params_t;

class CoaXModel
{
public:
  CoaXModel();
  ~CoaXModel();

  void Update(double time);
  void ResetSimulation();
  void ResetSimulation(double time_,
                       double x, double y, double z,
                       double roll, double pitch, double yaw);

  void PrintModelDescription();

  void SetTime(double time);
  double GetTime();

  void SetXYZ(double x, double y, double z);
  void GetXYZ(double& x, double& y, double& z);

  void SetRotation(double roll, double pitch, double yaw);
  void GetRotation(double& roll, double& pitch, double& yaw);

  void SetWorldLinearVelocity(double x, double y, double z);
  void GetWorldLinearVelocity(double& xdot, double& ydot, double& zdot);

  void SetBodyAngularVelocity(double wb1, double wb2, double wb3);
  void GetBodyAngularVelocity(double& wb1, double& wb2, double& wb3);

  void SetWorldLinearAcceleration(double x, double y, double z);
  void GetWorldLinearAcceleration(double& x, double& y, double& z);

  void SetRotorSpeed(double upper, double lower);
  void GetRotorSpeed(double &upper, double &lower);

  void SetInitialXYZ(double x, double y, double z);
  void SetInitialRotation(double roll, double pitch, double yaw);
  void SetInitialRotorSpeeds(double Omega_up, double Omega_lo);
  void SetInitialStabilizerBar(double x, double y, double z);
	
  void SetMass(double mass);
  void SetInertia(double Ixx, double Iyy, double Izz);
  void SetRotorOffset(double d_up, double d_lo);
  void SetUpperRotorFollowingTime(double Tf_up);
  void SetRotorLinkageFactor(double l_up, double l_lo);
  void SetRotorSpringConstant(double k_springup, double k_springlo);
  void SetRotorThrustFactor(double k_Tup, double k_Tlo);
  void SetRotorMomentFactor(double k_Mup, double k_Mlo);
  void SetMotorFollowingTime(double Tf_motup, double Tf_motlo);
  void SetUpperRotorSpeedConversion(double rs_mup, double rs_bup);
  void SetLowerRotorSpeedConversion(double rs_mlo, double rs_blo);
  void SetUpperPhaseLag(double zeta_mup, double zeta_bup);
  void SetLowerPhaseLag(double zeta_mlo, double zeta_blo);
  void SetMaximumSwashPlateAngle(double max_SPangle);

  void SetCommand(double u_motup, double u_motlo,
                  double u_serv1, double u_serv2);
  void SendCommand();
  double LimitRotorSpeed(double rotor_speed);

private:
  static int ODEStep(double t, const double* x, double* xdot, void* params);

  double pos[3];
  double vel[3];
  double rot[3];
  double angvel[3];
  double acc[3];
  double rotors[2];
  double bar[3];

  double init_pos[3];
  double init_rot[3];
  double init_rotors[2];
  double init_bar[3];

  double time;

  double statespace[DIMENSION];

  model_params_t model_params;

  gsl_odeiv_step* step;
  gsl_odeiv_control* control;
  gsl_odeiv_evolve* evolve;

  double u_motup_w;
  double u_motlo_w;
  double u_serv1_w;
  double u_serv2_w;

  CoaXOnboardControl c;
  bool cmd_updated;
};
#endif
