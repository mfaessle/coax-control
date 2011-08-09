// Mex wrapper around coax model
// N. Michael

#include <mex.h>
#include "CoaXModel.h"

CoaXModel model;

#define CHECKINPUTCOUNT(nrhs, count) \
  if (nrhs != count)\
    {\
      char buf[128]; sprintf(buf, "Incorrect input. Expected %i, got %i", count, nrhs); mexWarnMsgTxt(buf);\
      plhs[0] = mxCreateDoubleScalar(-1);\
      return;\
    }

#define ERRORWRAP(x) if (x != 0) {plhs[0] = mxCreateDoubleScalar(-1); return;}

extern "C"
{
  static int GetDoubleArray(mxArray *in, const char* field_name, const int length, double* out)
  {
    int nrows = mxGetM(in);
    int ncols = mxGetN(in);

    bool ok = false;
    if (nrows == length)
      if (ncols == 1)
        ok = true;

    if (ncols == length)
      if (nrows == 1)
        ok = true;

    if (!ok)
      {
        char buf[128];
        sprintf(buf, "%s: Expected a %i x 1 or 1 x %i array, got %i x %i",
                field_name, length, length, nrows, ncols);
        mexWarnMsgTxt(buf);
        return -1;
      }

    double* p = mxGetPr(in);
    unsigned int i = 0;
    for (i = 0; i < length; i++)
      out[i] = p[i];

    return 0;
  }

  void mexExit()
  {
    return;
  }

  void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
  {
    // Register the exit function
    mexAtExit(mexExit);

    // Verify the input is good, something must be given
    if (nrhs == 0)
      {
        mexWarnMsgTxt("Need input argument");
        plhs[0] = mxCreateDoubleScalar(-1);
        return;
      }

    int buflen = 128;
    char buf[buflen];

    // Check to see that we can read the input string
    if (mxGetString(prhs[0], buf, buflen) != 0)
      {
        mexWarnMsgTxt("Could not read string.");
        plhs[0] = mxCreateDoubleScalar(-1);
        return;
      }

    if (strcmp("update", buf) == 0)
      {
#ifdef DEBUG
        mexPrintf("dt: %f\n", mxGetScalar(prhs[1]));
#endif
        model.Update(mxGetScalar(prhs[1]));
        plhs[0] = mxCreateDoubleScalar(0);
        return;
      }
    else if (strcmp("get_state", buf) == 0)
      {
        CHECKINPUTCOUNT(nrhs, 1);

        const char *fields[] = {"time",
                                "position",
                                "linear_velocity",
                                "rotation",
                                "angular_velocity",
                                "rotor_speed",
                                "bar_direction",
								"acceleration"};
        const int nfields = sizeof(fields)/sizeof(*fields);
        plhs[0] = mxCreateStructMatrix(1, 1, nfields, fields);

        double time = model.GetTime();
        mxSetField(plhs[0], 0, "time", mxCreateDoubleScalar(time));

        mxArray *position = mxCreateDoubleMatrix(1, 3, mxREAL);
        double *p = mxGetPr(position);
        model.GetXYZ(p[0], p[1], p[2]);
        mxSetField(plhs[0], 0, "position", position);

        mxArray *linear_velocity = mxCreateDoubleMatrix(1, 3, mxREAL);
        p = mxGetPr(linear_velocity);
        model.GetWorldLinearVelocity(p[0], p[1], p[2]);
        mxSetField(plhs[0], 0, "linear_velocity", linear_velocity);

		mxArray *acceleration = mxCreateDoubleMatrix(1, 3, mxREAL);
		p = mxGetPr(acceleration);
		model.GetWorldLinearAcceleration(p[0], p[1], p[2]);
		mxSetField(plhs[0], 0, "acceleration", acceleration);
		  
        mxArray *rotation = mxCreateDoubleMatrix(1, 3, mxREAL);
        p = mxGetPr(rotation);
        double roll, pitch, yaw;
        model.GetRotation(roll, pitch, yaw);
        p[0] = roll;
        p[1] = pitch;
        p[2] = yaw;
        mxSetField(plhs[0], 0, "rotation", rotation);

        mxArray *angular_velocity = mxCreateDoubleMatrix(1, 3, mxREAL);
        p = mxGetPr(angular_velocity);
        model.GetBodyAngularVelocity(p[0], p[1], p[2]);
        mxSetField(plhs[0], 0, "angular_velocity", angular_velocity);

        mxArray *rotor_speed = mxCreateDoubleMatrix(1, 2, mxREAL);
        p = mxGetPr(rotor_speed);
        model.GetRotorSpeed(p[0], p[1]);
        mxSetField(plhs[0], 0, "rotor_speed", rotor_speed);

        mxArray *bar_direction = mxCreateDoubleMatrix(1, 3, mxREAL);
        p = mxGetPr(bar_direction);
        model.GetBarDirection(p[0], p[1], p[2]);
        mxSetField(plhs[0], 0, "bar_direction", bar_direction);

        return;
      }
    else if (strcmp("set_state", buf) == 0)
      {
        CHECKINPUTCOUNT(nrhs, 2);

        mxArray *field;

        field = mxGetField(prhs[1], 0, "position");
        double position[3];
        ERRORWRAP(GetDoubleArray(field, "position", 3, position))
        model.SetXYZ(position[0], position[1], position[2]);

        field = mxGetField(prhs[1], 0, "linear_velocity");
        double linear_velocity[3];
        ERRORWRAP(GetDoubleArray(field, "linear_velocity", 3, linear_velocity))
        model.SetWorldLinearVelocity(linear_velocity[0],
                                     linear_velocity[1],
                                     linear_velocity[2]);

        field = mxGetField(prhs[1], 0, "rotation");
        double rotation[3];
        ERRORWRAP(GetDoubleArray(field, "rotation", 3, rotation))
        model.SetRotation(rotation[0], rotation[1], rotation[2]);

        field = mxGetField(prhs[1], 0, "angular_velocity");
        double angular_velocity[3];
        ERRORWRAP(GetDoubleArray(field, "angular_velocity", 3, angular_velocity))
        model.SetBodyAngularVelocity(angular_velocity[0],
                                     angular_velocity[1],
                                     angular_velocity[2]);

        field = mxGetField(prhs[1], 0, "rotor_speed");
        double rotor_speed[2];
        ERRORWRAP(GetDoubleArray(field, "rotor_speed", 2, rotor_speed))
        model.SetRotorSpeed(rotor_speed[0], rotor_speed[1]);

        field = mxGetField(prhs[1], 0, "bar_direction");
        double bar_direction[3];
        ERRORWRAP(GetDoubleArray(field, "bar_direction", 3, bar_direction))
        model.SetBarDirection(bar_direction[0], bar_direction[1], bar_direction[2]);

        return;
      }
    else if (strcmp("set_cmd", buf) == 0)
      {
        int nrows = mxGetM(prhs[1]);
        int ncols = mxGetN(prhs[1]);

        bool ok = false;
        if (((nrows == 1) && (ncols == 4)) ||
            ((nrows == 4) && (ncols == 1)))
          ok = true;

        if (!ok)
          {
            mexWarnMsgTxt("Expected a 1 x 4 or 4 x 1 array");
            return;
          }

        double *p = mxGetPr(prhs[1]);
#ifdef DEBUG
        mexPrintf("cmd: ");
        for (int i = 0; i < 4; i++)
          mexPrintf("%f ", p[i]);
        mexPrintf("\n");
#endif
        model.SetCommand(p[0], p[1], p[2], p[3]);
        model.SendCommand();
        return;
      }
    else if (strcmp("set_params", buf) == 0)
      {
        CHECKINPUTCOUNT(nrhs, 2);
        mxArray *field;

        field = mxGetField(prhs[1], 0, "mass");
        model.SetMass(mxGetScalar(field));

        field = mxGetField(prhs[1], 0, "inertia");
        double inertia[3];
        ERRORWRAP(GetDoubleArray(field, "inertia", 3, inertia))
        model.SetInertia(inertia[0], inertia[1], inertia[2]);

        field = mxGetField(prhs[1], 0, "offset");
        double offset[2];
        ERRORWRAP(GetDoubleArray(field, "offset", 2, offset))
        model.SetRotorOffset(offset[0], offset[1]);

        field = mxGetField(prhs[1], 0, "linkage_factor");
        double linkage_factor[2];
        ERRORWRAP(GetDoubleArray(field, "linkage_factor", 2, linkage_factor))
        model.SetRotorLinkageFactor(linkage_factor[0], linkage_factor[1]);

        field = mxGetField(prhs[1], 0, "spring_constant");
        double spring_constant[2];
        ERRORWRAP(GetDoubleArray(field, "spring_constant", 2, spring_constant))
        model.SetRotorSpringConstant(spring_constant[0], spring_constant[1]);

        field = mxGetField(prhs[1], 0, "thrust_factor");
        double thrust_factor[2];
        ERRORWRAP(GetDoubleArray(field, "thrust_factor", 2, thrust_factor))
        model.SetRotorThrustFactor(thrust_factor[0], thrust_factor[1]);

        field = mxGetField(prhs[1], 0, "moment_factor");
        double moment_factor[2];
        ERRORWRAP(GetDoubleArray(field, "moment_factor", 2, moment_factor))
        model.SetRotorMomentFactor(moment_factor[0], moment_factor[1]);

        field = mxGetField(prhs[1], 0, "following_time");
        double following_time[3];
        ERRORWRAP(GetDoubleArray(field, "following_time", 3, following_time))
        model.SetUpperRotorFollowingTime(following_time[0]);
        model.SetMotorFollowingTime(following_time[1], following_time[2]);

        field = mxGetField(prhs[1], 0, "speed_conversion");
        double speed_conversion[4];
        ERRORWRAP(GetDoubleArray(field, "speed_conversion", 4, speed_conversion))
        model.SetUpperRotorSpeedConversion(speed_conversion[0], speed_conversion[1]);
        model.SetLowerRotorSpeedConversion(speed_conversion[2], speed_conversion[3]);

        field = mxGetField(prhs[1], 0, "phase_lag");
        double phase_lag[4];
        ERRORWRAP(GetDoubleArray(field, "phase_lag", 4, phase_lag))
        model.SetUpperPhaseLag(phase_lag[0], phase_lag[1]);
        model.SetLowerPhaseLag(phase_lag[2], phase_lag[3]);

        field = mxGetField(prhs[1], 0, "max_swashplate_angle");
        model.SetMaximumSwashPlateAngle(mxGetScalar(field));

        return;
      }
    else if (strcmp("reset", buf) == 0)
      {
        CHECKINPUTCOUNT(nrhs, 1);

        model.ResetSimulation();
        plhs[0] = mxCreateDoubleScalar(0);
        return;
      }
    else
      {
        mexWarnMsgTxt("Unhandled Input.");
        return;
      }
  }
}
