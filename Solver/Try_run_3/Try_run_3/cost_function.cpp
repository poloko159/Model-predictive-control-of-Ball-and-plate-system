#include "Header.h"


int cost_function(real_t* in_x, real_t* in_u)
{
	int i, j;
	// X: 8x1
	// U: 2x1
	real_t in_x_e[8];
	real_t in_u_e[2];
	pb_vars.costs = 0;
	for (i = 0; i < 8; i++) in_x_e[i] = in_x[i]-pb_vars.x_des[i];
	for (j = 0; j < 2; j++) in_u_e[i] = in_u[i];
	for (i = 0; i < 8; i++) { pb_vars.costs += in_x_e[i] * in_x_e[i]; }
	for (i = 0; i < 2; i++)  pb_vars.costs += in_u_e[i] * in_u_e[i];
	
	cout << pb_vars.costs;
	return 0;
}

int cost_function_k_0_N()
{
	int j, i;


	for (j = 0; j < 8; j++)
	{
		pv_vars.dx[j] = pb_vars.x_des[j] - pb_vars.x0[j];
	}
	for (i = 1; i < 20; i++)
	{
		for (j = 0; j < 8; j++)
		{
			pv_vars.dx[i * 8 + j] = pb_vars.x_des[j] - pb_vars.x[i * 8 + j];
		}

	}

	real_t Jz[20];
	for (i = 0; i < 20; i++) Jz[i] = 0;
	cout << "Cost function"<<"\n";
	cout << "[" << endl;
	for (i = 0; i < 20; i++)
	{
		for (j = 0; j < 8; j++)
			Jz[i] += +pow(pv_vars.dx[i * 8 + j], 2);
		for (j = 0; j < 2; j++)
			Jz[i] += +pow(pb_vars.u[i * 2 + j], 2);
		cout << "J[" << i + 1 <<"] = " << Jz[i] << "\n";
	}
	cout << "]" << endl;

	// cout
	// store value
	for (i = 0; i < 20; i++)
		pv_vars.lsq_20[i] = Jz[i];
	return 0;
}
