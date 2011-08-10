/* Template file for IDNLGREY model specification.

   Use this file to create a MEX function that specifies the model
   structure and equations. The MEX file syntax is
      [dx, y] = mymodel(t, x, u, p1, p2, ..., pn, auxvar)
   where
      * t is the time (scalar).
      * x is the state vector at time t (column vector).
      * u is the vector of inputs at time t (column vector).
      * p1, p2,... pn: values of the estimated parameters specified
        in the IDNLGREY model.
      * auxvar: a cell array containing auxiliary data in any format
        (optional).
      * dx is the vector of state derivatives at time t (column vector).
      * y is the vector of outputs at time t.

   To create the MEX file "mymodel", do the following:
      1) Save this template as "mymodel.c" (replace "mymodel" by the
         name of your choice).
      2) Define the number NY of outputs below.
      3) Specify the state derivative equations in COMPUTE_DX below.
      4) Specify the output equations in COMPUTE_Y below.
      5) Build the MEX file using
            >> mex mymodel.c
*/

/* Include libraries. */
#include "mex.h"
#include "math.h"

/* Specify the number of outputs here. */
#define NY 6

/* State equations. */
void compute_dx(
    double *dx,  /* Vector of state derivatives (length nx). */
    double t,    /* Time t (scalar). */
    double *x,   /* State vector (length nx). */
    double *u,   /* Input vector (length nu). */
    double **p,  /* p[j] points to the j-th estimated model parameters (a double array). */
    const mxArray *auxvar  /* Cell array of additional data. */
   )
{
  int i;

    // States
    double roll     = x[6];      // roll angle
    double pitch    = x[7];      // pitch angle
    double yaw      = x[8];      // yaw angle
    double wp       = x[9];      // body roll rate
    double wq       = x[10];     // body pitch rate
    double wr       = x[11];     // body yaw rate
    double Omega_up = x[12];     // upper rotor speed
    double Omega_lo = x[13];     // lower rotor speed
    double z_barx   = x[14];     // stabilizer bar z-axis x-component
    double z_bary   = x[15];     // stabilizer bar z-axis y-component
    double z_barz   = x[16];     // stabilizer bar z-axis z-component

    // Controls
    double u_motup = u[0];
    double u_motlo = u[1];
    double u_serv1 = u[2];
    double u_serv2 = u[3];

    // Parameters
    double m           = *(p[0]);
    double g           = *(p[1]);
    double Ixx         = *(p[2]);
    double Iyy         = *(p[3]);
    double Izz         = *(p[4]);
    double d_up        = *(p[5]);
    double d_lo        = *(p[6]);
    double k_springup  = *(p[7]);
    double k_springlo  = *(p[8]);
    double l_up        = *(p[9]);
    double l_lo        = *(p[10]);
    double k_Tup       = *(p[11]);
    double k_Tlo       = *(p[12]);
    double k_Mup       = *(p[13]);
    double k_Mlo       = *(p[14]);
    double Tf_motup    = *(p[15]);
    double Tf_motlo    = *(p[16]);
    double Tf_up       = *(p[17]);
    double rs_mup      = *(p[18]);
    double rs_bup      = *(p[19]);
    double rs_mlo      = *(p[20]);
    double rs_blo      = *(p[21]);
    double zeta_mup    = *(p[22]);
    double zeta_bup    = *(p[23]);
    double zeta_mlo    = *(p[24]);
    double zeta_blo    = *(p[25]);
    double max_SPangle = *(p[26]);

    // Normalize z_bar
    double norm_z_bar = sqrt(z_barx*z_barx + z_bary*z_bary + z_barz*z_barz);
    x[14] = z_barx/norm_z_bar;
    x[15] = z_bary/norm_z_bar;
    x[16] = z_barz/norm_z_bar;

    // Upper thrust vector direction
    double z_Tupz = cos(l_up*acos(z_barz));
    double z_Tup_p[3] = {0,0,1};
    if (z_Tupz < 1){
      double temp = sqrt((1-z_Tupz*z_Tupz)/(z_barx*z_barx + z_bary*z_bary));
        z_Tup_p[0] = z_barx*temp;
        z_Tup_p[1] = z_bary*temp;
        z_Tup_p[2] = z_Tupz;
    }
    double zeta = zeta_mup*Omega_up + zeta_bup;
    double z_Tup[3];
    z_Tup[0] = cos(zeta)*z_Tup_p[0] - sin(zeta)*z_Tup_p[1];
    z_Tup[1] = sin(zeta)*z_Tup_p[0] + cos(zeta)*z_Tup_p[1];
    z_Tup[2] = z_Tup_p[2];

    // Lower thrust vector direction
    double a_SP = u_serv1*max_SPangle;
    double b_SP = u_serv2*max_SPangle;
    double z_SP[3];
    z_SP[0] = sin(b_SP);
    z_SP[1] = -sin(a_SP)*cos(b_SP);
    z_SP[2] = cos(a_SP)*cos(b_SP);
    double z_Tloz = cos(l_lo*acos(z_SP[2]));
    double z_Tlo_p[3] = {0,0,1};
    if (z_Tloz < 1){
      double temp = sqrt((1-z_Tloz*z_Tloz)/(z_SP[0]*z_SP[0] + z_SP[1]*z_SP[1]));
        z_Tlo_p[0] = z_SP[0]*temp;
        z_Tlo_p[1] = z_SP[1]*temp;
        z_Tlo_p[2] = z_Tloz;
    }
    zeta = zeta_mlo*Omega_lo + zeta_blo;
    double z_Tlo[3];
    z_Tlo[0] = cos(zeta)*z_Tlo_p[0] + sin(zeta)*z_Tlo_p[1];
    z_Tlo[1] = -sin(zeta)*z_Tlo_p[0] + cos(zeta)*z_Tlo_p[1];
    z_Tlo[2] = z_Tlo_p[2];

    // Coordinate transformation body to world coordinates
    double c_r = cos(roll);
    double s_r = sin(roll);
    double c_p = cos(pitch);
    double s_p = sin(pitch);
    double c_y = cos(yaw);
    double s_y = sin(yaw);

    double Rb2w0[3];
    Rb2w0[0] = c_p*c_y; // first row of Rb2w
    Rb2w0[1] = s_r*s_p*c_y - c_r*s_y;
    Rb2w0[2] = c_r*s_p*c_y + s_r*s_y;

    double Rb2w1[3];
    Rb2w1[0] = c_p*s_y; // second row of Rb2w
    Rb2w1[1] = s_r*s_p*s_y + c_r*c_y;
    Rb2w1[2] = c_r*s_p*s_y - s_r*c_y;

    double Rb2w2[3];
    Rb2w2[0] = -s_p; // third row of Rb2w
    Rb2w2[1] = s_r*c_p;
    Rb2w2[2] = c_r*c_p;

    // Flapping Moments
    double cp[3];
    cp[0] = -z_Tup[1]; // z_b x z_Tup
    cp[1] = z_Tup[0];
    cp[2] = 0;
    double norm_cp = sqrt(cp[0]*cp[0] + cp[1]*cp[1] + cp[2]*cp[2]);
    double M_flapup[3] = {0,0,0};
    if (fabs(norm_cp) > 1e-6){
        M_flapup[0] = 2*k_springup*cp[0]/norm_cp*acos(z_Tup[2]);
        M_flapup[1] = 2*k_springup*cp[1]/norm_cp*acos(z_Tup[2]);
        M_flapup[2] = 2*k_springup*cp[2]/norm_cp*acos(z_Tup[2]);
    }

    cp[0] = -z_Tlo[1]; // z_b x z_Tlo
    cp[1] = z_Tlo[0];
    cp[2] = 0;
    norm_cp = sqrt(cp[0]*cp[0] + cp[1]*cp[1] + cp[2]*cp[2]);
    double M_flaplo[3] = {0,0,0};
    if (fabs(norm_cp) > 1e-6){
        M_flaplo[0] = 2*k_springlo*cp[0]/norm_cp*acos(z_Tlo[2]);
        M_flaplo[1] = 2*k_springlo*cp[1]/norm_cp*acos(z_Tlo[2]);
        M_flaplo[2] = 2*k_springlo*cp[2]/norm_cp*acos(z_Tlo[2]);
    }

    // Thrust magnitudes
    double T_up = k_Tup*Omega_up*Omega_up;
    double T_lo = k_Tlo*Omega_lo*Omega_lo;

    // Summarized Forces
    double Fx = 0;
    double Fy = 0;
    double Fz = 0;

    for(i=0; i<3; i++){
      Fx += Rb2w0[i]*(T_up*z_Tup[i] + T_lo*z_Tlo[i]);
      Fy += Rb2w1[i]*(T_up*z_Tup[i] + T_lo*z_Tlo[i]);
      Fz += Rb2w2[i]*(T_up*z_Tup[i] + T_lo*z_Tlo[i]);
    }
    Fz -= m*g;

    // Summarized Moments
    double Mx =
      wq*wr*(Iyy-Izz) - T_up*z_Tup[1]*d_up - T_lo*z_Tlo[1]*d_lo + M_flapup[0] + M_flaplo[0];
    double My =
      wp*wr*(Izz-Ixx) + T_up*z_Tup[0]*d_up + T_lo*z_Tlo[0]*d_lo + M_flapup[1] + M_flaplo[1];
    double Mz =
      wp*wq*(Ixx-Iyy) - k_Mup*Omega_up*Omega_up + k_Mlo*Omega_lo*Omega_lo;

    // State derivatives
    double xddot = 1.0/m*Fx;
    double yddot = 1.0/m*Fy;
    double zddot = 1.0/m*Fz;

    double rolldot  = wp + wq*s_r*s_p/c_p + wr*c_r*s_p/c_p;
    double pitchdot = wq*c_r - wr*s_r;
    double yawdot   = wq*s_r/c_p + wr*c_r/c_p;

    double pdot = 1/Ixx*Mx;
    double qdot = 1/Iyy*My;
    double rdot = 1/Izz*Mz;

    double Omega_up_des = rs_mup*u_motup + rs_bup;
    double Omega_lo_des = rs_mlo*u_motlo + rs_blo;
    double Omega_updot  = 1/Tf_motup*(Omega_up_des - Omega_up);
    double Omega_lodot  = 1/Tf_motlo*(Omega_lo_des - Omega_lo);

    double b_z_bardotz = 1/Tf_up*acos(z_barz)*sqrt(z_barx*z_barx + z_bary*z_bary);
    double b_z_bardot[3] = {0,0,0};
    if (fabs(b_z_bardotz) > 1e-6){
      double temp          = z_barz*b_z_bardotz/(z_barx*z_barx + z_bary*z_bary);
        b_z_bardot[0] = -z_barx*temp;
        b_z_bardot[1] = -z_bary*temp;
        b_z_bardot[2] = b_z_bardotz;
    }

    double z_barxdot = b_z_bardot[0] - wq*z_barz + wr*z_bary;
    double z_barydot = b_z_bardot[1] - wr*z_barx + wp*z_barz;
    double z_barzdot = b_z_bardot[2] - wp*z_bary + wq*z_barx;

    dx[0]  = x[3];
    dx[1]  = x[4];
    dx[2]  = x[5];
    dx[3]  = xddot;
    dx[4]  = yddot;
    dx[5]  = zddot;
    dx[6]  = rolldot;
    dx[7]  = pitchdot;
    dx[8]  = yawdot;
    dx[9]  = pdot;
    dx[10] = qdot;
    dx[11] = rdot;
    dx[12] = Omega_updot;
    dx[13] = Omega_lodot;
    dx[14] = z_barxdot;
    dx[15] = z_barydot;
    dx[16] = z_barzdot;

}

/* Output equations. */
void compute_y(
    double *y,   /* Vector of outputs (length NY). */
    double *x   /* State vector (length nx). */
   )
{
    y[0] = x[3];
    y[1] = x[4];
    y[2] = x[5];
    y[3] = x[9];
    y[4] = x[10];
    y[5] = x[11];
}



/*----------------------------------------------------------------------- *
   DO NOT MODIFY THE CODE BELOW UNLESS YOU NEED TO PASS ADDITIONAL
   INFORMATION TO COMPUTE_DX AND COMPUTE_Y

   To add extra arguments to compute_dx and compute_y (e.g., size
   information), modify the definitions above and calls below.
 *-----------------------------------------------------------------------*/

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    /* Declaration of input and output arguments. */
    double *x, *u, **p, *dx, *y, *t;
    int     i, np, nu, nx;
    const mxArray *auxvar = NULL; /* Cell array of additional data. */

    if (nrhs < 3) {
        mexErrMsgIdAndTxt("IDNLGREY:ODE_FILE:InvalidSyntax",
        "At least 3 inputs expected (t, u, x).");
    }

    /* Determine if auxiliary variables were passed as last input.  */
    if ((nrhs > 3) && (mxIsCell(prhs[nrhs-1]))) {
        /* Auxiliary variables were passed as input. */
        auxvar = prhs[nrhs-1];
        np = nrhs - 4; /* Number of parameters (could be 0). */
    } else {
        /* Auxiliary variables were not passed. */
        np = nrhs - 3; /* Number of parameters. */
    }

    if (np < 27)
      mexErrMsgIdAndTxt("IDNLGREY:ODE_FILE:InvalidSyntax",
                        "Must provide 27 parameters");

    /* Determine number of inputs and states. */
    nx = mxGetNumberOfElements(prhs[1]); /* Number of states. */
    nu = mxGetNumberOfElements(prhs[2]); /* Number of inputs. */

    /* Obtain double data pointers from mxArrays. */
    t = mxGetPr(prhs[0]);  /* Current time value (scalar). */
    x = mxGetPr(prhs[1]);  /* States at time t. */
    u = mxGetPr(prhs[2]);  /* Inputs at time t. */

    p = mxCalloc(np, sizeof(double*));
    for (i = 0; i < np; i++) {
        p[i] = mxGetPr(prhs[3+i]); /* Parameter arrays. */
    }

    /* Create matrix for the return arguments. */
    plhs[0] = mxCreateDoubleMatrix(nx, 1, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(NY, 1, mxREAL);
    dx      = mxGetPr(plhs[0]); /* State derivative values. */
    y       = mxGetPr(plhs[1]); /* Output values. */

    /*
      Call the state and output update functions.

      Note: You may also pass other inputs that you might need,
      such as number of states (nx) and number of parameters (np).
      You may also omit unused inputs (such as auxvar).

      For example, you may want to use orders nx and nu, but not time (t)
      or auxiliary data (auxvar). You may write these functions as:
          compute_dx(dx, nx, nu, x, u, p);
          compute_y(y, nx, nu, x, u, p);
    */

    /* Call function for state derivative update. */
    compute_dx(dx, *t, x, u, p, auxvar);

    /* Call function for output update. */
    //compute_y(y, t[0], x, u, p, auxvar);
    compute_y(y, x);

    /* Clean up. */
    mxFree(p);
}
