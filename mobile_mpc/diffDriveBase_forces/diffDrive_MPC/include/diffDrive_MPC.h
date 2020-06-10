/*
diffDrive_MPC : A fast customized optimization solver.

Copyright (C) 2013-2020 EMBOTECH AG [info@embotech.com]. All rights reserved.


This software is intended for simulation and testing purposes only. 
Use of this software for any commercial purpose is prohibited.

This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/

/* Generated by FORCES PRO v3.0.1 on Friday, June 5, 2020 at 1:22:43 PM */

#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#endif

#ifndef diffDrive_MPC_H
#define diffDrive_MPC_H

/* DATA TYPE ------------------------------------------------------------*/
typedef double diffDrive_MPC_float;

typedef double diffDrive_MPCinterface_float;

#ifndef SOLVER_STANDARD_TYPES
#define SOLVER_STANDARD_TYPES

typedef signed char solver_int8_signed;
typedef unsigned char solver_int8_unsigned;
typedef char solver_int8_default;
typedef signed short int solver_int16_signed;
typedef unsigned short int solver_int16_unsigned;
typedef short int solver_int16_default;
typedef signed int solver_int32_signed;
typedef unsigned int solver_int32_unsigned;
typedef int solver_int32_default;
typedef signed long long int solver_int64_signed;
typedef unsigned long long int solver_int64_unsigned;
typedef long long int solver_int64_default;

#endif


/* SOLVER SETTINGS ------------------------------------------------------*/

/* MISRA-C compliance */
#ifndef MISRA_C_diffDrive_MPC
#define MISRA_C_diffDrive_MPC (0)
#endif

/* restrict code */
#ifndef RESTRICT_CODE_diffDrive_MPC
#define RESTRICT_CODE_diffDrive_MPC (0)
#endif

/* print level */
#ifndef SET_PRINTLEVEL_diffDrive_MPC
#define SET_PRINTLEVEL_diffDrive_MPC    (2)
#endif

/* timing */
#ifndef SET_TIMING_diffDrive_MPC
#define SET_TIMING_diffDrive_MPC    (0)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define SET_MAXIT_diffDrive_MPC			(250)	

/* scaling factor of line search (FTB rule) */
#define SET_FLS_SCALE_diffDrive_MPC		(diffDrive_MPC_float)(0.99)      

/* maximum number of supported elements in the filter */
#define MAX_FILTER_SIZE_diffDrive_MPC	(250) 

/* maximum number of supported elements in the filter */
#define MAX_SOC_IT_diffDrive_MPC			(4) 

/* desired relative duality gap */
#define SET_ACC_RDGAP_diffDrive_MPC		(diffDrive_MPC_float)(0.0001)

/* desired maximum residual on equality constraints */
#define SET_ACC_RESEQ_diffDrive_MPC		(diffDrive_MPC_float)(1E-06)

/* desired maximum residual on inequality constraints */
#define SET_ACC_RESINEQ_diffDrive_MPC	(diffDrive_MPC_float)(1E-06)

/* desired maximum violation of complementarity */
#define SET_ACC_KKTCOMPL_diffDrive_MPC	(diffDrive_MPC_float)(1E-06)


/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define OPTIMAL_diffDrive_MPC      (1)

/* maximum number of iterations has been reached */
#define MAXITREACHED_diffDrive_MPC (0)

/* wrong number of inequalities error */
#define INVALID_NUM_INEQ_ERROR_diffDrive_MPC  (-4)

/* factorization error */
#define FACTORIZATION_ERROR_diffDrive_MPC   (-5)

/* NaN encountered in function evaluations */
#define BADFUNCEVAL_diffDrive_MPC  (-6)

/* no progress in method possible */
#define NOPROGRESS_diffDrive_MPC   (-7)

/* invalid values in parameters */
#define PARAM_VALUE_ERROR_diffDrive_MPC   (-11)

/* licensing error - solver not valid on this machine */
#define LICENSE_ERROR_diffDrive_MPC  (-100)

/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct
{
    /* vector of size 5 */
    diffDrive_MPC_float xinit[5];

    /* vector of size 75 */
    diffDrive_MPC_float x0[75];

    /* vector of size 150 */
    diffDrive_MPC_float all_parameters[150];


} diffDrive_MPC_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct
{
    /* vector of size 5 */
    diffDrive_MPC_float x01[5];

    /* vector of size 5 */
    diffDrive_MPC_float x02[5];

    /* vector of size 5 */
    diffDrive_MPC_float x03[5];

    /* vector of size 5 */
    diffDrive_MPC_float x04[5];

    /* vector of size 5 */
    diffDrive_MPC_float x05[5];

    /* vector of size 5 */
    diffDrive_MPC_float x06[5];

    /* vector of size 5 */
    diffDrive_MPC_float x07[5];

    /* vector of size 5 */
    diffDrive_MPC_float x08[5];

    /* vector of size 5 */
    diffDrive_MPC_float x09[5];

    /* vector of size 5 */
    diffDrive_MPC_float x10[5];

    /* vector of size 5 */
    diffDrive_MPC_float x11[5];

    /* vector of size 5 */
    diffDrive_MPC_float x12[5];

    /* vector of size 5 */
    diffDrive_MPC_float x13[5];

    /* vector of size 5 */
    diffDrive_MPC_float x14[5];

    /* vector of size 5 */
    diffDrive_MPC_float x15[5];


} diffDrive_MPC_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct
{
    /* iteration number */
    solver_int32_default it;

	/* number of iterations needed to optimality (branch-and-bound) */
	solver_int32_default it2opt;
	
    /* inf-norm of equality constraint residuals */
    diffDrive_MPC_float res_eq;
	
    /* inf-norm of inequality constraint residuals */
    diffDrive_MPC_float res_ineq;

	/* norm of stationarity condition */
    diffDrive_MPC_float rsnorm;

	/* max of all complementarity violations */
    diffDrive_MPC_float rcompnorm;

    /* primal objective */
    diffDrive_MPC_float pobj;	
	
    /* dual objective */
    diffDrive_MPC_float dobj;	

    /* duality gap := pobj - dobj */
    diffDrive_MPC_float dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    diffDrive_MPC_float rdgap;		

    /* duality measure */
    diffDrive_MPC_float mu;

	/* duality measure (after affine step) */
    diffDrive_MPC_float mu_aff;
	
    /* centering parameter */
    diffDrive_MPC_float sigma;
	
    /* number of backtracking line search steps (affine direction) */
    solver_int32_default lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    solver_int32_default lsit_cc;
    
    /* step size (affine direction) */
    diffDrive_MPC_float step_aff;
    
    /* step size (combined direction) */
    diffDrive_MPC_float step_cc;    

	/* solvertime */
	diffDrive_MPC_float solvetime;   

	/* time spent in function evaluations */
	diffDrive_MPC_float fevalstime;  

} diffDrive_MPC_info;







/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* Time of Solver Generation: (UTC) Friday, June 5, 2020 1:22:44 PM */
/* User License expires on: (UTC) Wednesday, September 23, 2020 10:00:00 PM (approx.) (at the time of code generation) */
/* Solver Static License expires on: (UTC) Wednesday, September 23, 2020 10:00:00 PM (approx.) */
/* Solver Generation Request Id: 7ac51207-67a7-4617-ae70-108d7248cb0c */
/* examine exitflag before using the result! */
#ifdef __cplusplus
extern "C" {
#endif		

typedef void (*diffDrive_MPC_extfunc)(diffDrive_MPC_float* x, diffDrive_MPC_float* y, diffDrive_MPC_float* lambda, diffDrive_MPC_float* params, diffDrive_MPC_float* pobj, diffDrive_MPC_float* g, diffDrive_MPC_float* c, diffDrive_MPC_float* Jeq, diffDrive_MPC_float* h, diffDrive_MPC_float* Jineq, diffDrive_MPC_float* H, solver_int32_default stage, solver_int32_default iterations);

extern solver_int32_default diffDrive_MPC_solve(diffDrive_MPC_params *params, diffDrive_MPC_output *output, diffDrive_MPC_info *info, FILE *fs, diffDrive_MPC_extfunc evalextfunctions_diffDrive_MPC);	





#ifdef __cplusplus
}
#endif

#endif
