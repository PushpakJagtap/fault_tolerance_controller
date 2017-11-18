/*
 * cartpole_simulate.cc
 *
 *  created on: 09.11.2017
 *      author: pushpak
 */

/*
 * information about this example is given in the readme file
 *
 */

#include <array>
#include <iostream>
#include <math.h>

/* header file which computes control action */
#include "cartpole_controller_functions.h"
#include "RungeKutta4.hh"

/* state space dim */
#define sDIM 4
/* input space dim */
#define iDIM 1

/* data types for the ode solver */
typedef std::array<double,4> state_type;
typedef std::array<double,1> input_type;

/* select sampling time to test
 * we consider "restart_time(tau_r)=0.250" and "controller_sampling_time(tau_c)=0.125") */
const double tau = 0.25;

/* number of intermediate steps in the ode solver */
const int nint=5;
OdeSolver ode_solver(sDIM,nint,tau);

/* system parameters */
const double omega=1;
const double ga=0.0125;

/* we integrate the cartpole ode by tau_r or tau_c sec (the result is stored in x)  */
auto  cartpole_ode = [](state_type &x, input_type &u) -> void {
    
    /* the ode describing the cartpole (DYNAMICS)*/
    auto rhs =[](state_type& dxdt,  const state_type &x, input_type &u) {
        dxdt[0] = x[1];
        dxdt[1] = -omega*omega*(sin(x[0])+u[0]*cos(x[0]))-2*ga*x[1];
        dxdt[2] = x[3];
        dxdt[3] = u[0]-2*x[3];
    };
    ode_solver(rhs,x,u);
};

int main() {
    /* initial state (starting point for simulation)*/
    state_type x={{3.0, 0.0, 0.1, 0.0}};
    
    /* quantization parameter */
    double eta[sDIM]={.05,.1,.1,.1};
    
    /*number of bits used to store BDD variable*/
    int nBit[4]={5,5,6,6};
    
    /* starting point of grid */
    double first_grid_point[sDIM]={2.4, -1.5, -2, -2};
    long int id[4];
    
    /* simulating for 100 iterations */
    for(int i=0; i<100; i++) {
        std::cout << x[0] <<  " "  << x[1] <<  " " << x[2] <<  " "  << x[3] << "\n";
        /* finding quantized state id*/
        for(int j=0; j<sDIM; j++){
            id[j]=round((x[j]-first_grid_point[j])/eta[j]);
            //std::cout<<j<<" "<<id[j]<<"\n";
        }
        
        long int xx=id[0], temp, sum=nBit[0];
        for(int i=1; i<sDIM; i++){
            temp=id[i]<<sum;
            xx=xx|temp;
            sum=sum+nBit[i];
        }
        /* getting control input from controller */
        auto uInt = getControlAction(xx);
        input_type u= {{-4+uInt*0.1}};
        //std::cout << u[0] << "\n";
        cartpole_ode(x,u);
    }
    return 1;
}
