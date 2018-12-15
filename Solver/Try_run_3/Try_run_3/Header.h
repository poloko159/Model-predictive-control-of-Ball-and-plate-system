#ifndef HEADER_H
#define HEADER_H

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
using namespace std;

typedef double real_t;
typedef struct pub_
{
	/*
	*	for pWHITEict
	*/

	/** To store pWHITEicted value of X
	 *	Matrix of size: 21 x 8 (row major format)
	 *
	 *  Matrix containing 20 differential variable vectors + x0.
	 */
	real_t x[168];

	/** Matrix of size: 20 x 2 (row major format)
	 *
	 *  Matrix containing 20 control variable vectors.
	 */
	real_t u[40];

	/** Matrix of size: 20x20 (row major format) 
	*	format:
	*	*/
	real_t W[400];

	/** Matrix of size: 8 x 8 (row major format) */
	real_t WN[64];


	/*
	*	for estimated 
	*/
	/* Column vector of size: 8
	 *
	 * desiWHITE value of X.
	 */
	real_t x_des[8];

	/* Column vector of size: 8
	 *
	 *  Current state feedback vector.
	 */
	real_t x0[8];
	/* Column vector of size: 2
	 *
	 *  Current control vector.
	 */
	real_t u0[2];


	/* Column vector of size: 2
	 *
	 *  condensing WN.
	 */
	real_t Q_con[160];

	/* Column vector of size: 2
	 *
	 *  condensing R.
	 */
	real_t R_con[1600];
	double costs;
}pub;

typedef struct priv_
{
	/*
	*	for Runge kutta 4th order
	*	runge_kutta_k(x)_inter	: intermediate variables
	*	runge_kutta_k(x)		: K(x) variables
	*	first					: calculate intermediate variables
	*	then					: calculate k(x)=h*f(runge_kutta_k(x)_inter)
	*/
	/* runge kutta init */
	real_t runge_kutta_k1[8];
	real_t runge_kutta_k1_inter[8];

	/* runge kutta init */
	real_t runge_kutta_k2[8];
	real_t runge_kutta_k2_inter[8];

	/* runge kutta init */
	real_t runge_kutta_k3[8];
	real_t runge_kutta_k3_inter[8];

	/* runge kutta init */
	real_t runge_kutta_k4[8];
	real_t runge_kutta_k4_inter[8];

	/* runge kutta init */
	real_t runge_kutta_final[8];
	////////////////////////////////
	/*
	*	next 20 A 20*8*8
	*/
	real_t A_20[1280];

	/*
	*	next 20 B 20*8*2
	*/
	real_t B_20[320];

	/*
	*	next 20 A 20*8*1
	*/
	real_t c_20[160];

	////////////////////////////////

	/*
	*	to get cost function 
	*/
	real_t lsq_20[20];

	real_t dx0[8];

	real_t dx[160];

	////////////////////////////////
	/*
	*	For condensing block
	*/
	// for g:	(20*8)*1 = 160*1
	real_t g_con[160];
	// for G:	(20*8)*(20*2) = 160*40 = row*col
	real_t G_con[6400];
	// for transposed G:		40*160 = 6400
	real_t GT_con[6400];
	// for h:	20*2
	real_t h_con[40];
	// for H:	40*40
	real_t H_con[1600];

}priv;

/*		functions  of runge kutta 4th*/

// function of model
void model_function(const real_t* in_x, const real_t* in_u, real_t* out);

// x(k+1), using maple to get the final model
int maple_exp( real_t* in_x,  real_t* in_u, real_t* out);

// get x(k+1) = runge kutta (Recursion using model_function)
void rugenkutta_classic();
/*	end	functions  of runge kutta 4th*/






// get next 20 * 8 x val - pWHITEictive
int next_20_xk( real_t* x0,  real_t* u, real_t* out_x);
// END get next








// cost function
int cost_function_k_0_N();
/*end cost function*/





/*for matrix calculate*/

int B_m(real_t* in_x, real_t* in_u, real_t* out);

int A_m(real_t* in_x, real_t* in_u, real_t* out);

/*	Multi matrix A and B	*/
int multi_AB(real_t* A, real_t* B, real_t* out);

/*	Multi matrix A and g	*/
int multi_A_g(real_t* A, real_t* g, real_t* out);

/*	Multi matrix A and x	*/
int multi_A_x(real_t* A, real_t* x, real_t* out);

/*	Multi matrix A and G	*/
int multi_A_G(real_t* A, real_t* G, real_t* out);

/*	Multi matrix GT * Q * g	*/
int multi_GT_Q_g(real_t* GT_con, real_t* Q_con, real_t* g_con, real_t* h_con);

/*	Multi matrix GT * Q * G + R	*/
int multi_GT_Q_G_plus_R(real_t* R, real_t* GT_con, real_t* G_con, real_t* out);

/* END for matrix calculate*/




void A20();

void B20();

/*inverse kinematics function*/
int inverse_kinematics(real_t x, real_t y);
/*END inverse kinematics function*/





/*for condensing N2 */

/*	Condensing g */
int condensing_g_cal();

/*	Condensing h */
int condensing_h_cal();

/*	Condensing G */
int condensing_G_cal();

/*	Condensing H */
int condensing_H_cal();

// extern 
extern pub pb_vars;
extern priv pv_vars;


#endif /* HEADER_H */
