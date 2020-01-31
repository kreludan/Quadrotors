#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <curses.h>

//gcc -o week1 week_1.cpp -lwiringPi -lncurses -lm

#define frequency 25000000.0
#define CONFIG           0x1A
#define SMPLRT_DIV       0x19
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
// #define M_PI             acos(-1.0) // PI for radian to degree conversion 


// Safety Limits
#define GYRO_LIM	400.0 	// Gyro should not exceed 300 degrees/sec
#define PITCH_ANG	45.0	// Pitch should not exceed (+/-) 45 degrees
#define ROLL_ANG	45.0	// Roll should not exceed (+/-) 45 degrees


enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};
 
enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};
 
int setup_imu();
void calibrate_imu();      
void read_imu();    
void update_filter();

//global variables

struct Keyboard {
  char key_press;
  int heartbeat;
  int version;
};
Keyboard* shared_memory; 
int run_program=1;
int imu;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; //gyro xyz, accel xyz
long time_curr;
long time_prev;
struct timespec te;
float yaw=0;
float pitch_angle=0;
float roll_angle=0;
float Roll=0;
float Pitch=0;
float previous_Pitch=0;
float integrated_gyro_roll = 0;
float integrated_gyro_pitch = 0;
int last_heartbeat=-1;
long safety_time_prev;
long safety_time_curr;
float time_since_heartbeat=0;
int last_version=-1;
int pwm;
float Pitch_I_term=0;

// DEFINES

#define MAX_GYRO_ALLOWED 300
#define MAX_ROLL_ANGLE_ALLOWED 45
#define MAX_PITCH_ANGLE_ALLOWED 45
#define PWM_MAX 1600
#define frequency 25000000.0
#define LED0 0x6			
#define LED0_ON_L 0x6		
#define LED0_ON_H 0x7		
#define LED0_OFF_L 0x8		
#define LED0_OFF_H 0x9		
#define LED_MULTIPLYER 4	

void init_pwm()
{

    pwm=wiringPiI2CSetup (0x40);
    if(pwm==-1)
    {
      printf("-----cant connect to I2C device %d --------\n",pwm);
     
    }
    else
    {
  
      float freq =400.0*.95;
      float prescaleval = 25000000;
      prescaleval /= 4096;
      prescaleval /= freq;
      prescaleval -= 1;
      uint8_t prescale = floor(prescaleval+0.5);
      int settings = wiringPiI2CReadReg8(pwm, 0x00) & 0x7F;
      int sleep	= settings | 0x10;
      int wake 	= settings & 0xef;
      int restart = wake | 0x80;
      wiringPiI2CWriteReg8(pwm, 0x00, sleep);
      wiringPiI2CWriteReg8(pwm, 0xfe, prescale);
      wiringPiI2CWriteReg8(pwm, 0x00, wake);
      delay(10);
      wiringPiI2CWriteReg8(pwm, 0x00, restart|0x20);
    }
}


void init_motor(uint8_t channel)
{
	int on_value=0;

	int time_on_us=900;
	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1200;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1000;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

}

void set_PWM( uint8_t channel, float time_on_us)
{
  if(run_program==1)
  {
    if(time_on_us>PWM_MAX)
    {
      time_on_us=PWM_MAX;
    }
    else if(time_on_us<1000)
    {
      time_on_us=1000;
    }
  	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
  	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
  else
  {  
    time_on_us=1000;   
  	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
  	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
}

void update_filter()
{
  
  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr=te.tv_nsec;
  //compute time since last execution
  float imu_diff=time_curr-time_prev;           
  
  //check for rollover
  if(imu_diff<=0)
  {
    imu_diff+=1000000000;
  }
  //convert to seconds
  imu_diff=imu_diff/1000000000;
  time_prev=time_curr;
    
  float az = imu_data[5];  // pulling out the values to display
  float ay = imu_data[4];
  float ax = imu_data[3];
  
            
  float pitch_gyro = imu_data[0];

  float pitch_gyro_delta;
  pitch_gyro_delta = pitch_gyro * imu_diff;
  integrated_gyro_pitch += pitch_gyro_delta;
  
  float roll_gyro = imu_data[1];     
  
  float roll_gyro_delta;
  roll_gyro_delta = roll_gyro * imu_diff;
  integrated_gyro_roll += roll_gyro_delta;

  float a;  // tunes roll
  a = 0.01;
  Roll = roll_angle * a + ((1-a) * (roll_gyro_delta + Roll));

  float b;  // tunes pitch
  b = 0.01;
  Pitch = pitch_angle * b + (1-b) * (pitch_gyro_delta + Pitch);


  // Print graph values for roll - checkpoint 2
  // printf("%10.5f, %10.5f, %10.5f \n\r", Roll, roll_angle, integrated_gyro_roll);
}

void pid_update() {
  
  float neutral_power = 1300;
  float error = -Pitch;
  float P = 14;

  float D = 100;
  float I_gain = 0.03;

  Pitch_I_term += error * I_gain;
  if(Pitch_I_term < -50) {
    Pitch_I_term = -50;
  }
  if(Pitch_I_term > 50) {
    Pitch_I_term = 50;
  }

  float pitch_velocity = Pitch - previous_Pitch;

  float right_D = -pitch_velocity*D; 
  float left_D = pitch_velocity*D;
  
  float right_PD = neutral_power + error*P + right_D;
  float left_PD = neutral_power - error*P + left_D;
  
  /*  CODE FOR PD-CONTROLLER ONLY : USE TO GET GRAPH / DEMONSTRATION FOFR CHECKPOINT 3
  if(right_PD > PWM_MAX) {
    right_PD = PWM_MAX;
  }
  if(left_PD > PWM_MAX) {
    left_PD = PWM_MAX;
  }
  if(right_PD < 1000) {
    right_PD = 1000;
  }
  if(left_PD < 1000) {
    left_PD = 1000;
  }  


  set_PWM(0, right_PD);
  set_PWM(1, right_PD);
  set_PWM(2, left_PD);
  set_PWM(3, left_PD);

  printf("%10.5f, %10.5f, %f, %f, %f, %f \n\r", pitch_angle, Pitch, right_PD, right_PD, left_PD, left_PD);
  */

  // PID CONTROLLER CODE : FOR CHECKPOINT 4 --- NEED TO CHECK SIGN FOR PITCH_I_TERM

  float right_PID = neutral_power + error*P + right_D + Pitch_I_term;
  float left_PID = neutral_power - error*P + left_D - Pitch_I_term;
  if(right_PID > PWM_MAX) {
    right_PID = PWM_MAX;
  }
  if(left_PID > PWM_MAX) {
    left_PID = PWM_MAX;
  }
  if(right_PID < 1000) {
    right_PID = 1000;
  }
  if(left_PD < 1000) {
    left_PID = 1000;
  }  


  set_PWM(0, right_PID);
  set_PWM(1, right_PID);
  set_PWM(2, left_PID);
  set_PWM(3, left_PID);

  printf("%10.5f, %10.5f, %f, %f, %f, %f \n\r", pitch_angle, Pitch, right_PID, right_PID, left_PID, left_PID);
}

void safety_check()
{
// Use the limits and catch keyboard events to check if we need to stop the machine
// GYRO_LIM, PITCH_ANG, ROLL_ANG
// Also turn off if space is pressed, keyboard times out, or we catch CTRL+C (as configured above in main())

// Refresh memory
    Keyboard keyboard=*shared_memory;

//get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  safety_time_curr=te.tv_nsec;
  //compute time since last execution
  float imu_diff=safety_time_curr-safety_time_prev;  
  int curr_heartbeat = shared_memory->heartbeat;         
  
  //check for rollover
  if(imu_diff<=0 )
  {
    imu_diff+=1000000000;
  }
  //convert to seconds
  imu_diff=imu_diff/1000000000.0;
  
  safety_time_prev=safety_time_curr;
  

  
  if(curr_heartbeat != last_heartbeat){
    time_since_heartbeat = 0;
  }
  
  // printf("curr: %d, last: %d, time: %d", curr_heartbeat, last_heartbeat, time_since_heartbeat);
  
  if(curr_heartbeat == last_heartbeat){
  
  time_since_heartbeat += imu_diff;
  }

  if(time_since_heartbeat >= 0.25 && last_heartbeat == curr_heartbeat){
  printf("FAILING DUE TO NO HEARTBEAT SIGNAL, %10.5f", imu_diff);
  
  run_program=0;
  }
  
  last_heartbeat = curr_heartbeat;

  char curr_key = shared_memory->key_press;
	if(curr_key == ' '){
   printf("FAILING DUE TO SPACE BEING PRESSED");
		run_program=0;
	}
	
	if(Roll > ROLL_ANG || Roll < -ROLL_ANG){
   printf("FAILING DUE TO ROLL");
		run_program=0;
	}   
	
	if(Pitch > PITCH_ANG || Pitch < -PITCH_ANG){
		run_program=0;
  printf("FAILING DUE TO PITCH");
	}
	
	
// xyz gyro values read from imu_data[0..2]
	if(imu_data[0] > GYRO_LIM || imu_data[1] > GYRO_LIM || imu_data[2] > GYRO_LIM){
 printf("FAILING DUE TO GYRO LIMITS");
		run_program=0;
	}
	
	
}


//function to add
void setup_keyboard()
{

  int segment_id;   
  struct shmid_ds shmbuffer; 
  int segment_size; 
  const int shared_segment_size = 0x6400; 
  int smhkey=33222;
  
  /* Allocate a shared memory segment.  */ 
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666); 
  /* Attach the shared memory segment.  */ 
  shared_memory = (Keyboard*) shmat (segment_id, 0, 0); 
  printf ("shared memory attached at address %p\n", shared_memory); 
  /* Determine the segment's size. */ 
  shmctl (segment_id, IPC_STAT, &shmbuffer); 
  segment_size  =               shmbuffer.shm_segsz; 
  printf ("segment size: %d\n", segment_size); 
  /* Write a string to the shared memory segment.  */ 
  //sprintf (shared_memory, "test!!!!."); 

}


//when cntrl+c pressed, kill motors

void trap(int signal)

{

  set_PWM(0, 1000);
  set_PWM(1, 1000);
  set_PWM(2, 1000);
  set_PWM(3, 1000); 
 
  printf("ending program\n\r");

   run_program=0;
}
 
int main (int argc, char *argv[])
{

    setup_imu();
    
    init_pwm();
    init_motor(0);
    init_motor(1);
    init_motor(2);
    init_motor(3);
    delay(1000);

    calibrate_imu();

    //in main before while(1) loop add...
    setup_keyboard();
    signal(SIGINT, &trap);

    //to refresh values from shared memory first 
    Keyboard keyboard=*shared_memory;
    
    while(run_program==1)
    {
    
      read_imu();
      previous_Pitch = Pitch;
      update_filter();  
      pid_update(); 
      safety_check();
      
     
    }
    
    set_PWM(0, 1000);
    set_PWM(1, 1000);
    set_PWM(2, 1000);
    set_PWM(3, 1000); 
       
    return 0;
      
  
}

void calibrate_imu()
{
  x_gyro_calibration = 0; // set all values to 0 initially
  y_gyro_calibration = 0;
  z_gyro_calibration = 0;
  roll_calibration = 0;
  pitch_calibration = 0;
  accel_z_calibration = 0;
  float ax = 0;
  float az = 0;
  float ay = 0;
  
  int i;
  for (i = 1; i <= 1000; i++) { // averaging over 1000 calculations for initial calculation
    
    int address = 67; // x gyro value address
    
    // high and low
    int vh = wiringPiI2CReadReg8(imu, address);
    int vl = wiringPiI2CReadReg8(imu, address+1);
    
    
    // two's complement
    int vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
      vw=vw ^ 0xffff;
      vw=-vw-1;
    }
    
    x_gyro_calibration += (vw * 500.0 / 32768.0) / 1000.0; // normalized to dps + averaging
    
    address = 69; // y gyro value address
       
    // high and low
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address+1);
    
    
    // two's complement
    vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
      vw=vw ^ 0xffff;
      vw=-vw-1;
    }
    
    y_gyro_calibration += (vw * 500.0 / 32768.0) / 1000.0; // normalized to dps + averaging
    
    address = 71; // z gyro value address
    
        
    // high and low
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address+1);
    
    
    // two's complement
    vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
      vw=vw ^ 0xffff;
      vw=-vw-1;
    }
    
    z_gyro_calibration += (vw * 500.0 / 32768.0) / 1000.0; // normalized to dps + averaging
    
    
    // getting x accel and y accel values to calculate roll and pitch

    address = 59; // x accel value location
    
    // high and low
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address+1);
    
    
    // two's complement
    vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
      vw=vw ^ 0xffff;
      vw=-vw-1;
    }
    
    ax = vw;  // setting ax value
    
    address = 61; // y accel value location
    
    // high and low
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address+1);
    
    
    // two's complement
    vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
      vw=vw ^ 0xffff;
      vw=-vw-1;
    }
    
    ay = vw;  // setting ay value
        
    address = 63; // z accel value location
       
    // high and low
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address+1);
    
    
    // two's complement
    vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
      vw=vw ^ 0xffff;
      vw=-vw-1;
    }
    
    
    accel_z_calibration += (vw * 2.0 / 32768.0) / 1000.0; // normalized to dps + averaging

    az = vw;  // setting az value
     

    // roll and pitch calculations

    

    roll_calibration += (atan2(ax, -az) * (180.0/M_PI)) / 1000.0;
  
    pitch_calibration += (atan2(ay, -az) * (180.0/M_PI)) / 1000.0;

  }
  
  // printf("calibration complete, %f %f %f %f %f %f\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);


}

void read_imu()
{
  int address=59; // address for accel x value 
  float ax=0;
  float az=0;
  float ay=0; 
  int vh,vl;
  
  //read in data
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  //convert 2 complement
  int vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[3] = (vw * 2.0 / 32768.0); //convert vw from raw values to g's (normalize by 2/32768.0)
  
  
  address=61; // address value for accel y value
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[4] = (vw * 2.0 / 32768.0); //convert vw from raw values to g's
  
  
  address=63; //address value for accel z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[5] = (vw * 2.0 / 32768.0); //convert vw from raw values to g's
  
  
  address=67; // gyro x value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[0]= -x_gyro_calibration + (vw * 500.0 / 32768.0); //// convert vw from raw values to degrees/second (normalize by 500/32768.0)
  
  address=69;//todo: set addres value for gyro y value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[1]= -y_gyro_calibration + (vw * 500.0 / 32768.0); //// convert vw from raw values to degrees/second
  
  address=71;//// gyro z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[2]= -z_gyro_calibration + (vw * 500.0 / 32768.0);////todo: convert vw from raw values to degrees/second
  
  
  az = imu_data[5];  // pulling out the values to display
  ay = imu_data[4];
  ax = imu_data[3];
  
  // roll and pitch calculations

  roll_angle = roll_calibration + (atan2(ax, -az) * (180.0/M_PI));
  
  pitch_angle = -pitch_calibration - (atan2(ay, -az) * (180.0/M_PI));

  //printf("Gyros: (%10.5f %10.5f %10.5f), Roll:  %10.5f, Pitch: %10.5f\n\r",imu_data[0], imu_data[1], imu_data[2], roll, pitch);

 


}


int setup_imu()
{
  wiringPiSetup ();
  
  
  //setup imu on I2C
  imu=wiringPiI2CSetup (0x68) ; //accel/gyro address
  
  if(imu==-1)
  {
    printf("-----cant connect to I2C device %d --------\n",imu);
    return -1;
  }
  else
  {
  
    printf("connected to i2c device %d\n",imu);
    printf("imu who am i is %d \n",wiringPiI2CReadReg8(imu,0x75));
    
    uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
    uint8_t Gscale = GFS_500DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
    
    
    //init imu
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x00);
    printf("                    \n\r");
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x01);
    wiringPiI2CWriteReg8(imu, CONFIG, 0x00);  
    wiringPiI2CWriteReg8(imu, SMPLRT_DIV, 0x00); //0x04        
    int c=wiringPiI2CReadReg8(imu,  GYRO_CONFIG);
    wiringPiI2CWriteReg8(imu,  GYRO_CONFIG, c & ~0xE0);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0x18);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c | Gscale << 3);       
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG);
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c | Ascale << 3);      
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);         
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2, c & ~0x0F); //
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2,  c | 0x00);
  }
  return 0;
}


