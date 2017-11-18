Cartpole system dynamics (inverted pendulum on cart):
	
Parameters:
	omega=1;
	ga=0.0125;
ODE:
	dxdt[0] = x[1];
    	dxdt[1] = -omega*omega*(std::sin(x[0])+u[0]*std::cos(x[0]))-2*ga*x[1];
    	dxdt[2] = x[3];
    	dxdt[3] = u[0]-2*x[3];

Safety Bounds on state space:
	x[0]=> [0.75*pi, 1.25*pi]
	x[1]=> [-1, 1]
	x[2]=> [-1.8, 1.8]
	x[3]=> [-1.8, 1.8]

Control input bounds:
	u=> [-5; 5]

Restart time considered:
	t_r=0.250 sec

Controller sampling time considered:
	t_c=0.125 sec