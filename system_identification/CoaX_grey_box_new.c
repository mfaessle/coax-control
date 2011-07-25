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
    double roll, pitch, yaw, wp, wq, wr, Omega_up, Omega_lo, z_barx, z_bary, z_barz; // Used states
    double u_motup, u_motlo, u_serv1, u_serv2; // Control inputs
    double *m, *g, *Ixx, *Iyy, *Izz, *d_up, *d_lo, *k_springup, *k_springlo, 
            *l_up, *l_lo, *k_Tup, *k_Tlo, *k_Mup, *k_Mlo, *Tf_motup,
            *Tf_motlo, *Tf_up, *rs_mup, *rs_bup, *rs_mlo, *rs_blo, *zeta_mup,
            *zeta_bup, *zeta_mlo, *zeta_blo, *max_SPangle; // Estimated model parameters.
    
    double a_lo, b_lo, c_r, s_r, c_p, s_p, c_y, s_y, z_Tupz, z_Tloz, zeta;
    double norm_cp, T_up, T_lo, Fx, Fy, Fz, Mx, My, Mz, b_z_bardotz, temp, norm_z_bar;
    double z_Tup[3], z_Tlo[3], Rb2w0[3], Rb2w1[3], Rb2w2[3], cp[3], z_SP[3];
    double M_flapup[3] = {0,0,0}, M_flaplo[3] = {0,0,0};
    double z_Tup_p[3] = {0,0,1}, z_Tlo_p[3] = {0,0,1}, b_z_bardot[3] = {0,0,0};
    int i;
    double xddot, yddot, zddot, rolldot, pitchdot, yawdot, pdot, qdot, rdot;
    double Omega_up_des, Omega_lo_des, Omega_updot, Omega_lodot, z_barxdot, z_barydot, z_barzdot;
    
    // States
    roll     = x[6];      // roll angle
    pitch    = x[7];      // pitch angle
    yaw      = x[8];      // yaw angle
    wp       = x[9];      // body roll rate
    wq       = x[10];     // body pitch rate
    wr       = x[11];     // body yaw rate 
    Omega_up = x[12];     // upper rotor speed
    Omega_lo = x[13];     // lower rotor speed
    z_barx   = x[14];     // stabilizer bar z-axis x-component
    z_bary   = x[15];     // stabilizer bar z-axis y-component
    z_barz   = x[16];     // stabilizer bar z-axis z-component
    
    // Controls
    u_motup = u[0];
    u_motlo = u[1];
    u_serv1 = u[2];
    u_serv2 = u[3];
    
    // Parameters
    m           = p[0];
    g           = p[1];
    Ixx         = p[2];
    Iyy         = p[3];
    Izz         = p[4];
    d_up        = p[5];
    d_lo        = p[6];
    k_springup  = p[7];
    k_springlo  = p[8];
    l_up        = p[9];
    l_lo        = p[10];
    k_Tup       = p[11];
    k_Tlo       = p[12];
    k_Mup       = p[13];
    k_Mlo       = p[14];
    Tf_motup    = p[15];
    Tf_motlo    = p[16];
    Tf_up       = p[17];
    rs_mup      = p[18];
    rs_bup      = p[19];
    rs_mlo      = p[20];
    rs_blo      = p[21];
    zeta_mup    = p[22];
    zeta_bup    = p[23];
    zeta_mlo    = p[24];
    zeta_blo    = p[25];
    max_SPangle = p[26];
    
    // Normalize z_bar
    norm_z_bar = sqrt(z_barx*z_barx + z_bary*z_bary + z_barz*z_barz);
    x[14] = z_barx/norm_z_bar;
    x[15] = z_bary/norm_z_bar;
    x[16] = z_barz/norm_z_bar;
    
    // Upper thrust vector direction
    z_Tupz         = cos(l_up[0]*acos(z_barz));
    
    if (z_Tupz < 1){
        temp       = sqrt((1-z_Tupz*z_Tupz)/(z_barx*z_barx + z_bary*z_bary));
        z_Tup_p[0] = z_barx*temp;
        z_Tup_p[1] = z_bary*temp;
        z_Tup_p[2] = z_Tupz;
    }
    zeta           = zeta_mup[0]*Omega_up + zeta_bup[0];
    z_Tup[0]       = cos(zeta)*z_Tup_p[0] - sin(zeta)*z_Tup_p[1];
    z_Tup[1]       = sin(zeta)*z_Tup_p[0] + cos(zeta)*z_Tup_p[1];
    z_Tup[2]       = z_Tup_p[2];
    
    // Lower thrust vector direction
    a_lo           = l_lo[0]*u_serv1*max_SPangle[0];
    b_lo           = l_lo[0]*u_serv2*max_SPangle[0];
    z_SP[0]        = sin(b_lo);
    z_SP[1]        = -sin(a_lo)*cos(b_lo);
    z_SP[2]        = cos(a_lo)*cos(b_lo);
    
    z_Tloz         = cos(l_lo[0]*acos(z_SP[2]));
    if (z_Tloz < 1){
        temp       = sqrt((1-z_Tloz*z_Tloz)/(z_SP[1]*z_SP[1] + z_SP[2]*z_SP[2]));
        z_Tlo_p[0] = z_SP[1]*temp;
        z_Tlo_p[1] = z_SP[2]*temp;
        z_Tlo_p[2] = z_Tloz;
    }
    zeta           = zeta_mlo[0]*Omega_lo + zeta_blo[0];
    z_Tlo[0]       = cos(zeta)*z_Tlo_p[0] + sin(zeta)*z_Tlo_p[1];
    z_Tlo[1]       = -sin(zeta)*z_Tlo_p[0] + cos(zeta)*z_Tlo_p[1];
    z_Tlo[2]       = z_Tlo_p[2];
    
    // Coordinate transformation body to world coordinates
    c_r      = cos(roll);
    s_r      = sin(roll);
    c_p      = cos(pitch);
    s_p      = sin(pitch);
    c_y      = cos(yaw);
    s_y      = sin(yaw);

    Rb2w0[0] = c_p*c_y; // first row of Rb2w
    Rb2w0[1] = s_r*s_p*c_y - c_r*s_y;
    Rb2w0[2] = c_r*s_p*c_y + s_r*s_y;

    Rb2w1[0] = c_p*s_y; // second row of Rb2w
    Rb2w1[1] = s_r*s_p*s_y + c_r*c_y;
    Rb2w1[2] = c_r*s_p*s_y - s_r*c_y;

    Rb2w2[0] = -s_p; // third row of Rb2w
    Rb2w2[1] = s_r*c_p;
    Rb2w2[2] = c_r*c_p;
    
    // Flapping Moments
    cp[0]   = -z_Tup[1]; // z_b x z_Tup
    cp[1]   = z_Tup[0];
    cp[2]   = 0;
    norm_cp = sqrt(cp[0]*cp[0] + cp[1]*cp[1] + cp[2]*cp[2]);
    
    if (fabs(norm_cp) > 1e-6){
        M_flapup[0] = 2*k_springup[0]*cp[0]/norm_cp*acos(z_Tup[2]);
        M_flapup[1] = 2*k_springup[0]*cp[1]/norm_cp*acos(z_Tup[2]);
        M_flapup[2] = 2*k_springup[0]*cp[2]/norm_cp*acos(z_Tup[2]);
    }

    cp[0]   = -z_Tlo[1]; // z_b x z_Tlo
    cp[1]   = z_Tlo[0];
    cp[2]   = 0;
    norm_cp = sqrt(cp[0]*cp[0] + cp[1]*cp[1] + cp[2]*cp[2]);
    
    if (fabs(norm_cp) > 1e-6){
        M_flaplo[0] = 2*k_springlo[0]*cp[0]/norm_cp*acos(z_Tlo[2]);
        M_flaplo[1] = 2*k_springlo[0]*cp[1]/norm_cp*acos(z_Tlo[2]);
        M_flaplo[2] = 2*k_springlo[0]*cp[2]/norm_cp*acos(z_Tlo[2]);
    }
    
    // Thrust magnitudes
    T_up = k_Tup[0]*Omega_up*Omega_up;
    T_lo = k_Tlo[0]*Omega_lo*Omega_lo;
    
    // Summarized Forces
    for(i=0; i<3; i++){
        Fx += Rb2w0[i]*(T_up*z_Tup[i] + T_lo*z_Tlo[i]);
        Fy += Rb2w1[i]*(T_up*z_Tup[i] + T_lo*z_Tlo[i]);
        Fz += Rb2w2[i]*(T_up*z_Tup[i] + T_lo*z_Tlo[i]);
    }
    Fz -= m[0]*g[0];

    // Summarized Moments
    Mx = wq*wr*(Iyy[0]-Izz[0]) - T_up*z_Tup[1]*d_up[0] - T_lo*z_Tlo[1]*d_lo[0] + M_flapup[0] + M_flaplo[0];
    My = wp*wr*(Izz[0]-Ixx[0]) + T_up*z_Tup[0]*d_up[0] + T_lo*z_Tlo[0]*d_lo[0] + M_flapup[1] + M_flaplo[1];
    Mz = wp*wq*(Ixx[0]-Iyy[0]) - k_Mup[0]*Omega_up*Omega_up + k_Mlo[0]*Omega_lo*Omega_lo;
    
    // State derivatives
    xddot = 1.0/m[0]*Fx;
    yddot = 1.0/m[0]*Fy;
    zddot = 1.0/m[0]*Fz;

    rolldot  = wp + wq*s_r*s_p/c_p + wr*c_r*s_p/c_p;
    pitchdot = wq*c_r - wr*s_r;
    yawdot   = wq*s_r/c_p + wr*c_r/c_p;

    pdot = 1/Ixx[0]*Mx;
    qdot = 1/Iyy[0]*My;
    rdot = 1/Izz[0]*Mz;

    Omega_up_des = rs_mup[0]*u_motup + rs_bup[0];
    Omega_lo_des = rs_mlo[0]*u_motlo + rs_blo[0];
    Omega_updot  = 1/Tf_motup[0]*(Omega_up_des - Omega_up);
    Omega_lodot  = 1/Tf_motlo[0]*(Omega_lo_des - Omega_lo);
    
    b_z_bardotz       = 1/Tf_up[0]*acos(z_barz)*sqrt(z_barx*z_barx + z_bary*z_bary);
    if (fabs(b_z_bardotz) > 1e-6){
        temp          = z_barz*b_z_bardotz/(z_barx*z_barx + z_bary*z_bary);
        b_z_bardot[0] = -z_barx*temp;
        b_z_bardot[1] = -z_bary*temp;
        b_z_bardot[2] = b_z_bardotz;
    }

    z_barxdot = b_z_bardot[1] - wq*z_barz + wr*z_bary;
    z_barydot = b_z_bardot[2] - wr*z_barx + wp*z_barz;
    z_barzdot = b_z_bardot[3] - wp*z_bary + wq*z_barx;
    
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
    
}

/* Output equations. */
void compute_y(
    double *y,   /* Vector of outputs (length NY). */
    double *x   /* State vector (length nx). */
   )
{    
    y[0] = x[0];
    y[1] = x[1];
    y[2] = x[2];
    y[3] = x[6];
    y[4] = x[7];
    y[5] = x[8];
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
    compute_dx(dx, t[0], x, u, p, auxvar);
    
    /* Call function for output update. */
    //compute_y(y, t[0], x, u, p, auxvar);
    compute_y(y, x);
    
    /* Clean up. */
    mxFree(p);
}
