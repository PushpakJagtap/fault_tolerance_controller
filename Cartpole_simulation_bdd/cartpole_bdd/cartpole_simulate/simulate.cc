/*
 * cartpole.cc
 *
 * created: 9 nov 2017
 *  author: pushpak
 */

/*
 * information about this example is given in the readme file
 */
#include <iostream>
#include <array>
#include <cmath>

/* SCOTS header */
#include "scots.hh"
/* ode solver */
#include "RungeKutta4.hh"

/* state space dim */
const int state_dim=4;
/* input space dim */
const int input_dim=1;
/* sampling time */
const double tau = 0.25;

/*
 * data types for the elements of the state space 
 * and input space used by the ODE solvers
 */
using state_type = std::array<double,state_dim>;
using input_type = std::array<double,input_dim>;

/* system parameters */
const double omega=1;
const double ga=0.0125;
  /* the ode describing the system */
  auto  rhs = [](state_type &dxdt, const state_type &x, const input_type &u) {
    dxdt[0] = x[1];
    dxdt[1] = -omega*omega*(std::sin(x[0])+u[0]*std::cos(x[0]))-2*ga*x[1];
    dxdt[2] = x[3];
    dxdt[3] = u[0]-2*x[3];
    //dxdt[0] = x[1];
    //dxdt[1] = u[0];
};
  
/* we integrate the cart pole system  (the result is stored in x)  */
void cartpole(state_type &x, const input_type &u){
  scots::runge_kutta_fixed4(rhs,x,u,state_dim,tau,10);
};


int main() {
  
  /* Cudd manager */
  Cudd manager;

  /* read controller from file */
  BDD C;
  scots::SymbolicSet con;
  if(!read_from_file(manager,con,C,"controller")) {
    std::cout << "Could not read controller from controller.scs\n";
    return 0;
  }
  
  std::cout << "\nSimulation:\n " << std::endl;

	/*initial condition*/
  state_type x={{3.0, 0.0, 0.1, 0.0}};

 /* getting control action */
  for(int i=0; i<100; i++) {
    /* returns a std vector with the valid control inputs */
    auto u1 = con.restriction(manager,C,x);
    input_type u;
    u[0]=u1[0];
    std::cout << x[0] <<  " "  << x[1] <<  " "  << x[2]<<  " "  << x[3] << "\n";
    
    /* sending it to cartpole dynamics */
    cartpole(x,u);
  }

  return 1;
}
