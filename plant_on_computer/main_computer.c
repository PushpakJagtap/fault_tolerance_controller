
#include <stdio.h>
#include <math.h>


/* state space dim */
#define sDIM 2
/* input space dim */
#define iDIM 1

/* data types for the ode solver */
double x[sDIM];
double u[iDIM];

/* select sampling time to test
 * we consider "restart_time(tau_r)=0.250" and "controller_sampling_time(tau_c)=0.125") */
const double tau = 0.05;

/* system parameters */
const double omega=1;
const double ga=0.0125;


/* plant simulation  */
void  cartpole_ode(double* x,double* u)
{
    double dxdt[sDIM], dxdt1[sDIM], dxdt2[sDIM], dxdt3[sDIM],tmp[sDIM],h;
    int t, i;
    h=tau/5;
    
    for(t=0; t<5; t++) {
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

int main(void){ 
	int i, nSteps=100;
	/* initial state (starting point for simulation)*/
    x[0]=3.0; x[1]=0.0;
   /* send it to the board via serial to get control action  and store it in u*/
   for(i=0;i<=nSteps;i++){
		cartpole_ode(x,u);   //plant
	/* get updated states in x and again send it to the board via serial to get control action and store it in u (every 50 ms) */	
	/* it continues till nSteps times */
   }
}

