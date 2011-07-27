#include <cmath>
#include <cstring>

#include "CoaXModel.h"
#include <armadillo>

using namespace std;

CoaXModel::CoaXModel()
{
  time = 0;
  memset((void*)statespace, 0, sizeof(statespace));
  memset((void*)&model_params, 0, sizeof(model_params));

  const gsl_odeiv_step_type* step_type = gsl_odeiv_step_rkf45;
  step = gsl_odeiv_step_alloc(step_type, DIMENSION);
  control = gsl_odeiv_control_y_new(1e-5, 0.0);
  evolve = gsl_odeiv_evolve_alloc(DIMENSION);

  memset(pos, 0, sizeof(pos));
  memset(rot, 0, sizeof(rot));
  memset(vel, 0, sizeof(vel));
  memset(angvel, 0, sizeof(angvel));
  memset(acc, 0, sizeof(acc));
  memset(rotors, 0, sizeof(rotors));
  memset(bar, 0, sizeof(bar));

  memset(init_pos, 0, sizeof(init_pos));
  memset(init_rot, 0, sizeof(init_rot));

  cmd_updated = false;

  return;
}

CoaXModel::~CoaXModel()
{
  gsl_odeiv_evolve_free(evolve);
  gsl_odeiv_control_free(control);
  gsl_odeiv_step_free(step);

  return;
}

void CoaXModel::SetTime(double time_)
{
  time = time_;
}

double CoaXModel::GetTime()
{
  return time;
}

void CoaXModel::SetMass(double mass)
{
  model_params.mass = mass;
}

void CoaXModel::SetInertia(double Ixx, double Iyy, double Izz)
{
  model_params.Ixx = Ixx;
  model_params.Iyy = Iyy;
  model_params.Izz = Izz;
}

void CoaXModel::SetRotorOffset(double d_up, double d_lo)
{
  model_params.d_up = d_up;
  model_params.d_lo = d_lo;
}

void CoaXModel::SetUpperRotorFollowingTime(double Tf_up)
{
  model_params.Tf_up = Tf_up;
}

void CoaXModel::SetRotorLinkageFactor(double l_up, double l_lo)
{
  model_params.l_up = l_up;
  model_params.l_lo = l_lo;
}

void CoaXModel::SetRotorSpringConstant(double k_springup, double k_springlo)
{
  model_params.k_springup = k_springup;
  model_params.k_springlo = k_springlo;
}

void CoaXModel::SetRotorThrustFactor(double k_Tup, double k_Tlo)
{
  model_params.k_Tup = k_Tup;
  model_params.k_Tlo = k_Tlo;
}

void CoaXModel::SetRotorMomentFactor(double k_Mup, double k_Mlo)
{
  model_params.k_Mup = k_Mup;
  model_params.k_Mlo = k_Mlo;
}

void CoaXModel::SetMotorFollowingTime(double Tf_motup, double Tf_motlo)
{
  model_params.Tf_motup = Tf_motup;
  model_params.Tf_motlo = Tf_motlo;
}

void CoaXModel::SetUpperRotorSpeedConversion(double rs_mup, double rs_bup)
{
  model_params.rs_mup = rs_mup;
  model_params.rs_bup = rs_bup;
}

void CoaXModel::SetLowerRotorSpeedConversion(double rs_mlo, double rs_blo)
{
  model_params.rs_mlo = rs_mlo;
  model_params.rs_blo = rs_blo;
}

void CoaXModel::SetMaximumSwashPlateAngle(double max_SPangle)
{
	model_params.max_SPangle = max_SPangle;
}

void CoaXModel::SetCommand(double u_motup, double u_motlo,
                           double u_serv1, double u_serv2)
{
  u_motup_w = u_motup;
  u_motlo_w = u_motlo;
  u_serv1_w = u_serv1;
  u_serv2_w = u_serv2;

  cmd_updated = true;
}

void CoaXModel::SendCommand()
{
  if (!cmd_updated)
    return;

  cmd_updated = false;

  c.SetCommands(u_motup_w, u_motlo_w, u_serv1_w, u_serv2_w);
  
}

int CoaXModel::ODEStep(double t, const double* state, double* xdot, void* params)
{
  model_params_t* param = reinterpret_cast<model_params_t*>(params);

  // rotation
  double roll = state[6];
  double pitch = state[7];
  double yaw = state[8];

  // angular velocity
  double p = state[9];
  double q = state[10];
  double r = state[11];

  // rotor speeds
  double Omega_up = state[12];
  double Omega_lo = state[13];

	//printf("Omega_up: %f  Omega_lo: %f \n", Omega_up, Omega_lo);
  // stabilizer bar direction
  double a_up = state[14];
  double b_up = state[15];

  // Parameters
  double g = 9.81;
  double m = param->mass;
  double Ixx = param->Ixx;
  double Iyy = param->Iyy;
  double Izz = param->Izz;
  double d_up = param->d_up;
  double d_lo = param->d_lo;
  double k_springup = param->k_springup;
  double k_springlo = param->k_springlo;
  double l_up = param->l_up;
  double l_lo = param->l_lo;
  double k_Tup = param->k_Tup;
  double k_Tlo = param->k_Tlo;
  double k_Mup = param->k_Mup;
  double k_Mlo = param->k_Mlo;
  double Tf_motup = param->Tf_motup;
  double Tf_motlo = param->Tf_motlo;
  double Tf_up = param->Tf_up;
  double rs_mup = param->rs_mup;
  double rs_bup = param->rs_bup;
  double rs_mlo = param->rs_mlo;
  double rs_blo = param->rs_blo;
  double max_SPangle = param->max_SPangle;
	
  // Controls
  double u_motup = param->control[0];
  double u_motlo = param->control[1];
  double u_serv1 = param->control[2];
  double u_serv2 = param->control[3];

  // Thrust vector directions
  double a_lo = l_lo*u_serv1*max_SPangle;
  double b_lo = l_lo*u_serv2*max_SPangle;

  // rot around body-x then body-y
  arma::colvec z_Tup(3);
  z_Tup(0) = cos(a_up)*sin(b_up);
  z_Tup(1) = -sin(a_up);
  z_Tup(2) = cos(a_up)*cos(b_up);

  arma::colvec z_Tlo(3);
  z_Tlo(0) = cos(a_lo)*sin(b_lo);
  z_Tlo(1) = -sin(a_lo);
  z_Tlo(2) = cos(a_lo)*cos(b_lo);

  // Coordinate transformation body to world coordinates
  double c_r = cos(roll);
  double s_r = sin(roll);
  double c_p = cos(pitch);
  double s_p = sin(pitch);
  double c_y = cos(yaw);
  double s_y = sin(yaw);

  arma::mat Rb2w(3, 3);
  Rb2w(0, 0) = c_p*c_y;
  Rb2w(0, 1) = s_r*s_p*c_y - c_r*s_y;
  Rb2w(0, 2) = c_r*s_p*c_y + s_r*s_y;

  Rb2w(1, 0) = c_p*s_y;
  Rb2w(1, 1) = s_r*s_p*s_y + c_r*c_y;
  Rb2w(1, 2) = c_r*s_p*s_y - s_r*c_y;

  Rb2w(2, 0) = -s_p;
  Rb2w(2, 1) = s_r*c_p;
  Rb2w(2, 2) = c_r*c_p;

  // Flapping Moments

  // z_b x z_Tup
  arma::colvec cp(3);
  cp(0) = -z_Tup(1);
  cp(1) = z_Tup(0);
  cp(2) = 0;

  double norm_cp = sqrt(cp(0)*cp(0) + cp(1)*cp(1) + cp(2)*cp(2));
  arma::colvec M_flapup = arma::zeros(3);
  if (fabs(norm_cp) > 1e-6)
    M_flapup = 2*k_springup*cp/norm_cp*acos(z_Tup(2));

  // z_b x z_Tlo
  cp(0) = -z_Tlo(1);
  cp(1) = z_Tlo(0);
  cp(2) = 0;
  norm_cp = sqrt(cp(0)*cp(0) + cp(1)*cp(1) + cp(2)*cp(2));

  arma::colvec M_flaplo = arma::zeros(3);
  if (fabs(norm_cp) > 1e-6)
    M_flaplo = 2*k_springlo*cp/norm_cp*acos(z_Tlo(2));

  // Thrust magnitudes
  double T_up = k_Tup*Omega_up*Omega_up;
  double T_lo = k_Tlo*Omega_lo*Omega_lo;

  // Summarized Forces
  arma::colvec F_thrust = T_up*z_Tup + T_lo*z_Tlo;
  double Fx = arma::as_scalar(Rb2w.row(0)*F_thrust);
  double Fy = arma::as_scalar(Rb2w.row(1)*F_thrust);
  double Fz = -m*g + arma::as_scalar(Rb2w.row(2)*F_thrust);
	//printf("x: %f  y: %f  z: %f  Omega_up: %f  Omega_lo: %f \n", state[0], state[1], state[2], Omega_up, Omega_lo);
	//printf("Inputs: %f   %f   %f   %f \n",u_motup,u_motlo,u_serv1,u_serv2);
	//printf("Fx: %f  Fy: %f  Fz: %f \n",Fx,Fy,Fz);
	//printf("z_Tup: %f   %f   %f \n",z_Tup(0),z_Tup(1),z_Tup(2));
	//printf("z_Tlo: %f   %f   %f \n",z_Tlo(0),z_Tlo(1),z_Tlo(2));
	//printf("a_lo: %f  b_lo: %f \n",a_lo,b_lo);
  // Summarized Moments
  double Mx = q*r*(Iyy-Izz) - T_up*z_Tup(1)*d_up - T_lo*z_Tlo(1)*d_lo + M_flapup(0) + M_flaplo(0);
  double My = p*r*(Izz-Ixx) + T_up*z_Tup(0)*d_up + T_lo*z_Tlo(0)*d_lo + M_flapup(1) + M_flaplo(1);
  double Mz = p*q*(Ixx-Iyy) - k_Mup*Omega_up*Omega_up + k_Mlo*Omega_lo*Omega_lo;

  // State derivatives
  double xddot = 1.0/m*Fx;
  double yddot = 1.0/m*Fy;
  double zddot = 1.0/m*Fz;

  double rolldot = p + q*s_r*s_p/c_p + r*c_r*s_p/c_p;
  double pitchdot = q*c_r - r*s_r;
  double yawdot = q*s_r/c_p + r*c_r/c_p;

  double pdot = 1/Ixx*Mx;
  double qdot = 1/Iyy*My;
  double rdot = 1/Izz*Mz;

  double Omega_up_des = rs_mup*u_motup + rs_bup;
  double Omega_lo_des = rs_mlo*u_motlo + rs_blo;
  double Omega_updot = 1/Tf_motup*(Omega_up_des - Omega_up);
  double Omega_lodot = 1/Tf_motlo*(Omega_lo_des - Omega_lo);

  double a_updot = -1/Tf_up*a_up - l_up*p;
  double b_updot = -1/Tf_up*b_up - l_up*q;

  xdot[0] = state[3];
  xdot[1] = state[4];
  xdot[2] = state[5];
  xdot[3] = xddot;
  xdot[4] = yddot;
  xdot[5] = zddot;
  xdot[6] = rolldot;
  xdot[7] = pitchdot;
  xdot[8] = yawdot;
  xdot[9] = pdot;
  xdot[10] = qdot;
  xdot[11] = rdot;
  xdot[12] = Omega_updot;
  xdot[13] = Omega_lodot;
  xdot[14] = a_updot;
  xdot[15] = b_updot;

  return GSL_SUCCESS;
}

void CoaXModel::Update(double time_)
{
  double tstart = time;
  double tstop = time_;
  double h = 1e-5;

  memcpy(statespace, pos, sizeof(pos));
  memcpy((void*)(&statespace[3]), vel, sizeof(vel));
  memcpy((void*)(&statespace[6]), rot, sizeof(rot));
  memcpy((void*)(&statespace[9]), angvel, sizeof(angvel));
  memcpy((void*)(&statespace[12]), rotors, sizeof(rotors));
  memcpy((void*)(&statespace[14]), bar, sizeof(bar));

  gsl_odeiv_system sys = {CoaXModel::ODEStep, NULL,
                          DIMENSION, (void*)&model_params};

  while (tstart < tstop)
    {
      int status = gsl_odeiv_evolve_apply(evolve,
                                          control,
                                          step,
                                          &sys,
                                          &tstart, tstop,
                                          &h, statespace);

      if (status != GSL_SUCCESS)
        break;
    }

  time = tstop;

  memcpy(pos, statespace, sizeof(pos));
  memcpy(vel, (void*)(&statespace[3]), sizeof(vel));
  memcpy(rot, (void*)(&statespace[6]), sizeof(rot));
  memcpy(angvel, (void*)(&statespace[9]), sizeof(angvel));
  memcpy(rotors, (void*)(&statespace[12]), sizeof(rotors));
  memcpy(bar, (void*)(&statespace[14]), sizeof(bar));
  memcpy(acc, model_params.acc, sizeof(model_params.acc));

  double u1, u2, u3, u4;
  c.GetControls(u1, u2, u3, u4);

  model_params.control[0] = u1;
  model_params.control[1] = u2;
  model_params.control[2] = u3;
  model_params.control[3] = u4;

  return;
}

void CoaXModel::SetXYZ(double x, double y, double z)
{
  pos[0] = x;
  pos[1] = y;
  pos[2] = z;
}

void CoaXModel::SetRotation(double roll, double pitch, double yaw)
{
  rot[0] = roll;
  rot[1] = pitch;
  rot[2] = yaw;
}

void CoaXModel::SetInitialXYZ(double x, double y, double z)
{
  pos[0] = x;
  init_pos[0] = x;
  pos[1] = y;
  init_pos[1] = y;
  pos[2] = z;
  init_pos[2] = z;
}

void CoaXModel::SetInitialRotation(double roll, double pitch, double yaw)
{
  rot[0] = roll;
  init_rot[0] = roll;
  rot[1] = pitch;
  init_rot[1] = pitch;
  rot[2] = yaw;
  init_rot[2] = yaw;
}

void CoaXModel::GetXYZ(double& x, double& y, double& z)
{
  x = pos[0];
  y = pos[1];
  z = pos[2];
}

void CoaXModel::GetRotation(double& roll, double& pitch, double& yaw)
{
  roll = rot[0];
  pitch = rot[1];
  yaw = rot[2];
}

void CoaXModel::SetWorldLinearVelocity(double x, double y, double z)
{
  vel[0] = x;
  vel[1] = y;
  vel[2] = z;
}

void CoaXModel::SetBodyAngularVelocity(double wb1, double wb2, double wb3)
{
  angvel[0] = wb1;
  angvel[1] = wb2;
  angvel[2] = wb3;
}

void CoaXModel::GetWorldLinearVelocity(double& xdot,
                                       double& ydot,
                                       double& zdot)
{
  xdot = vel[0];
  ydot = vel[1];
  zdot = vel[2];
}

void CoaXModel::GetBodyAngularVelocity(double& wb1,
                                       double& wb2,
                                       double& wb3)
{
  wb1 = angvel[0];
  wb2 = angvel[1];
  wb3 = angvel[2];
}

void CoaXModel::GetWorldLinearAcceleration(double& x,
                                           double& y,
                                           double& z)
{
  x = acc[0];
  y = acc[1];
  z = acc[2];
}

void CoaXModel::SetWorldLinearAcceleration(double x,
                                           double y,
                                           double z)
{
  acc[0] = x;
  acc[1] = y;
  acc[2] = z;
}

void CoaXModel::SetRotorSpeed(double upper, double lower)
{
  rotors[0] = upper;
  rotors[1] = lower;

  return;
}

void CoaXModel::GetRotorSpeed(double &upper, double &lower)
{
  upper = rotors[0];
  lower = rotors[1];

  return;
}

void CoaXModel::ResetSimulation(double time_,
                                double x, double y, double z,
                                double roll, double pitch, double yaw)
{
  // Reset the time and state
  time = time_;

  pos[0] = x;
  pos[1] = y;
  pos[2] = z;

  rot[0] = roll;
  rot[1] = pitch;
  rot[2] = yaw;

  memset(vel, 0, sizeof(vel));
  //memset(rotors, 0, sizeof(rotors));
  memset(acc, 0, sizeof(acc));

  // Reset the evolution of the ODE
  gsl_odeiv_evolve_reset(evolve);

  return;
}

void CoaXModel::ResetSimulation()
{
  time = 0;

  memcpy(pos, init_pos, sizeof(pos));
  memcpy(rot, init_rot, sizeof(rot));
  memset(vel, 0, sizeof(vel));
  memset(angvel, 0, sizeof(angvel));
  memset(rotors, 238.9734, 226.7098);
  memset(acc, 0, sizeof(acc));

  // Reset the evolution of the ODE
  gsl_odeiv_evolve_reset(evolve);
}

void CoaXModel::PrintModelDescription()
{
  puts("Model Description:");
  printf("\tmass = %f\n", model_params.mass);
  printf("\tinertia = %f, %f, %f\n",
         model_params.Ixx, model_params.Iyy, model_params.Izz);
}
