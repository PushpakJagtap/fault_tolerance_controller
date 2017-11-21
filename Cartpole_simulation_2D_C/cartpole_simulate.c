/*
 * cartpole_simulate.cc
 *
 *  created on: 19.11.2017
 *      author: pushpak
 */

/*
 * information about this example is given in the readme file
 *
 */

#include <stdio.h>
#include <math.h>

/* header file which computes control action */
#include "cartpole_controller_functions.h"


/* state space dim */
#define sDIM 2
/* input space dim */
#define iDIM 1

/* data types for the ode solver */
double x[sDIM];
double u[iDIM];

/* select sampling time to test
 * we consider "restart_time(tau_r)=0.250" and "controller_sampling_time(tau_c)=0.125") */
const double tau = 0.3;

/* number of intermediate steps in the ode solver */
const int nint=5;

/* system parameters */
const double omega=1;
const double ga=0.0125;

/* we integrate the cartpole ode by tau_r or tau_c sec (the result is stored in x)  */
void  cartpole_ode(double* x,double* u)
{
    double dxdt[sDIM], dxdt1[sDIM], dxdt2[sDIM], dxdt3[sDIM],tmp[sDIM],h;
    int t, i;
    h=tau/nint;
   	// dxdt[0] = x[1];
	//dxdt[1] = -omega*omega*(sin(x[0])+u[0]*cos(x[0]))-2*ga*x[1];
    
    for(t=0; t<nint; t++) {
        //rhs(k[0],x);
        dxdt[0]=x[1];
        dxdt[1] = -omega*omega*(sin(x[0])+u[0]*cos(x[0]))-2*ga*x[1];
        for(i=0;i<sDIM;i++)
            tmp[i]=x[i]+h/2*dxdt[i];
        
        dxdt1[0]=tmp[1];
        dxdt1[1] = -omega*omega*(sin(tmp[0])+u[0]*cos(tmp[0]))-2*ga*tmp[1];
        for(i=0;i<sDIM;i++)
            tmp[i]=x[i]+h/2*dxdt1[i];
        
        dxdt2[0]=tmp[1];
        dxdt2[1] = -omega*omega*(sin(tmp[0])+u[0]*cos(tmp[0]))-2*ga*tmp[1];
        for(i=0;i<sDIM;i++)
            tmp[i]=x[i]+h*dxdt2[i];
        
        dxdt3[0]=tmp[1];
        dxdt3[1] = -omega*omega*(sin(tmp[0])+u[0]*cos(tmp[0]))-2*ga*tmp[1];
        for(i=0; i<sDIM; i++)
            x[i] = x[i] + (h/6)*(dxdt[i] + 2*dxdt1[i] + 2*dxdt2[i] + dxdt3[i]);
    }
    
}

int main() {
	int i=0, j=0, k;
    unsigned long int uInt;
    double u[iDIM];
    double x[sDIM];
    long int id[2];
    long int xx, temp, sum;
    /* quantization parameter */
    double eta[sDIM]={.08,.1};
    /*number of bits used to store BDD variable*/
    int nBit[2]={5,6};
    
    /* initial state (starting point for simulation)*/
    x[0]=3.0; x[1]=0.0;
    
    /* starting point of grid */
    double first_grid_point[sDIM]={2.24, -2};
    
    /* simulating for 100 iterations */
    for(i=0; i<100; i++) {
        //std::cout << x[0] <<  " "  << x[1] << "\n";
        printf("%f %f\n", x[0],x[1]);
        /* finding quantized state id*/
        for(j=0; j<sDIM; j++){
            id[j]=round((x[j]-first_grid_point[j])/eta[j]);
            //std::cout<<j<<" "<<id[j]<<"\n";
        }
        
        xx=id[0]; 
        sum=nBit[0];
        for(k=1; k<sDIM; k++){
            temp=id[k]<<sum;
            xx=xx|temp;
            sum=sum+nBit[k];
        }
        /* getting control input from controller */
        uInt = getControlAction(xx);
        u[0]= -4+uInt*0.1;
        cartpole_ode(x,u);
    }
    return 1;
}
