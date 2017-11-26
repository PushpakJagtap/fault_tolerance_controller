
#include <stdio.h>
#include <math.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

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


#define SERIAL_DATA_SIZE 9
#define MAX_VOLTAGE 2 //TODO change this later to 3

int set_interface_attribs (int fd, int speed, int parity)
{

    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        //                printf("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        //printf("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void set_blocking (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        //printf("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 8 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0){
        //printf("error %d setting term attributes", errno);
    }
}



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

    char *portname = "/dev/ttyACM0";
    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf("error %d opening %s: %s\n", errno, portname, strerror (errno));
        return;
    }

    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set no blocking


    /* .. Writing the control values to the left and write motor......*/

    int step = 0;
    double vol_step = 0.1;  

    int sensor_reading[3];



    unsigned char buf8[SERIAL_DATA_SIZE];
    unsigned char buf;
    printf("reading\n");

        /* initial state (starting point for simulation)*/
    x[0]=3.0; x[1]=0.0;
   /* send it to the board via serial to get control action  and store it in u*/



   for(i=0;i<=nSteps;i++){
        cartpole_ode(x,u);   //plant
    /* get updated states in x and again send it to the board via serial to get control action and store it in u (every 50 ms) */   
    /* it continues till nSteps times */
   }

   sensor_reading[0] = x[0]*10000.0;
   sensor_reading[1] = x[1]*10000.0;


    while(1){

        int n = read (fd, &buf, 1);
        
        if (n != 1) {
            printf("READ: n is not 1, n is: %d\n", n);
        }
        else{
            printf("READ: char read: %x\n", buf); 
            if (buf == 0xDD){
                //board is sending data to PC
                
                int i;
                n = read (fd, &buf8, SERIAL_DATA_SIZE);
                printf("READ: n is: %d\n", n);
                printf("READ: recieved bytes:\n");
                
                int vol_left = *(int*)buf8;
                int vol_right = *(int*) &buf8[4];
                
                printf("vol left : %d \n", vol_left);   

                vol_left = ((double) vol_left)/10000.0;
                vol_right = ((double) vol_right)/10000.0;               
                
            }
            else if (buf == 0xCC){
                // When the board requests the sensor data from PC
    
                unsigned char Txbuffer[13];
                int i;
                unsigned char buf_w;

                for (i = 0; i < 2; i ++ ) {
                    Txbuffer[4*i]              = (unsigned char) (sensor_reading[i] & 0xff); /* first byte */
                    Txbuffer[4*i + 1]          = (unsigned char) (sensor_reading[i] >> 8  & 0xff); /* second byte */
                    Txbuffer[4*i + 2]          = (unsigned char) (sensor_reading[i] >> 16 & 0xff); /* third byte */
                    Txbuffer[4*i + 3]          = (unsigned char) (sensor_reading[i] >> 24 & 0xff); /* fourth byte */
                }

                unsigned char start = 0xAA;
                Txbuffer[8] = 0xFF;

                int max_loop = 10; 
                int loop_count = 0;
                while (loop_count < max_loop){
                    loop_count++;
                    printf("SEND: in the while loop. loop_count = %d\n", loop_count);
                    write(fd, &start, 1);
                    
                    int n = read (fd, &buf_w, 1);
                    if (n == 1) {
                        printf("SEND: char is: %x\n", buf_w);
                        if (buf == 0xBB){
                            printf("SEND: I recieved BB\n");
                            write(fd, Txbuffer, 9);
                            usleep (5000);             // sleep enough to transmit the 7 plus
                            printf("SEND: data is written.\n");
                            break;
                        }
                    }
                }
            }
        }
    }
}