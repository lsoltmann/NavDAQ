/*
NavDAQ.cpp

Code from:
	Lars Soltmann
      		https://github.com/lsoltmann/NavDAQ
	EMLID
		https://github.com/emlid/Navio
		http://www.emlid.com

Revision History
17 Jan 2015 - Created and debugged
30 Jan 2015 - Program split into different versions to cut out what is not needed for the current test
                     - This version records the raw data only and will require post processing after download
11 Feb 2015 - Added MS5805 code
		     - Added heartbeat
		     - General improvements and clean up
13 Feb 2015 - Added check on magnetomer readings to make sure 'device opened error' is caught
17 Feb 2015 - Added 20Hz LPF to gyro and accel
24 Feb 2015 - Added AHRS code and thread
10 Mar 2015 - Renamed NavDAQ_raw_AHRS to NavDAQ2, added installed hard iron calibration
26 Mar 2015 - Removed LPF on accel and gyro, increased sigfigs on baro and static transducer, chnaged loop time to run at true 50Hz
04 Apr 2015 - Upgraded to NavDAQ3, GPS thread added, display ouput modified, change LED to reflect GPS status
21 Apr 2015 - Hardcoded C6 for MS5805 from coeff data provided by MeasSpec
09 May 2015 - Major overhaul of code due to excessive CPU usage causing lag in data, specifically GPS (turned out not to be the root cause)
27 May 2015 - GPS switched to new library, NMEA messages disabled, GPS lag appears to be gone
19 Jun 2015 - Modified log file to reflect changes to GPS library, added configuration file reader code
23 Jun 2015 - Added second PPM encoder, updated installed hard iron calibrations for new system (+)
10 Jul 2015 - Added telemetry option
*/

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <iostream>
#include <pigpio.h>
#include "/home/pi/Navio/C++/Navio/MS5611.h"
#include "/home/pi/Navio/C++/Navio/ADS1115.h"
#include "/home/pi/Navio/C++/Navio/MPU9250.h"
#include "/home/pi/Navio/C++/Navio/PCA9685.h"
#include "/home/pi/Navio/C++/Examples/AHRS/AHRS.hpp"
#include "/home/pi/Navio/C++/Navio/gpio.h"
#include <wiringSerial.h>
#include <wiringPi.h>
#include <time.h>
#include "/home/pi/Libraries/PID.h"
#include "/home/pi/Libraries/MS4515.h"
#include "/home/pi/Libraries/MS5805.h"
#include "/home/pi/Libraries/UbloxGPS.h"
#include "/home/pi/Libraries/SSC005D.h"
#include "/home/pi/Libraries/readConfig.h"
#include "/home/pi/Libraries/wifiCom.h"

using namespace Navio;
using namespace std;

/*
//================================ PPM Decoder and Servo Setup  =====================================
unsigned int samplingRate      = 5;      // 1 microsecond (can be 1,2,4,5,10)
unsigned int ppmInputGpio      = 4;      // PPM input on Navio's 2.54 header
unsigned int ppmSyncLength     = 7000;   // Length of PPM sync pause
unsigned int ppmChannelsNumber = 5;      // Number of channels packed in PPM
unsigned int servoFrequency    = 50;     // Servo control frequency

// **** This data is set specifically for the RMILEC high-precision PWM/PPM/SBus Signal Converter V2
// **** R8 has been removed on the NAVIO, 3.3V PPM ONLY!
// This encoder only appears to work with original NAVIO
*/
//================================ PPM Decoder and Servo Setup  =====================================
unsigned int samplingRate      = 2;      // 1 microsecond (can be 1,2,4,5,10)
unsigned int ppmInputGpio      = 4;      // PPM input on Navio's 2.54 header
unsigned int ppmSyncLength     = 10500;   // Length of PPM sync pause
unsigned int ppmChannelsNumber = 8;      // Number of channels packed in PPM
unsigned int servoFrequency    = 50;     // Servo control frequency

// **** This data is set specifically for the Geeetech PPM encoder (tested with NAVIO+)


MS5805 ms5805;
MS4515 ms4515;
MPU9250 imu;
ADS1115 adc;
AHRS ahrs;
Ublox gps;
PCA9685 pwm;
HWSSC arspd;
readConfig configs;
COMMS link;

//============================ Variable Setup  ==================================
// PPM variables
float channels[8]; // <- Geeetech = 8, RMILEC = 5
float throttle, elevator, aileron, rudder, gear;
unsigned int currentChannel = 0;
unsigned int previousTick;
unsigned int deltaTime;

// Timer and loop variables
unsigned long t1, t2, t4, t5, tstart;
unsigned long t3=0;
unsigned long prev_t3=0;
float dt1, dt2, dt4;
struct timeval tv;
struct timeval tv_test;
int error_flag=0;
int arg=0;
int exitflag=0;
int gearflag=0;
time_t current_time;
double nextloopt, dataSampleRate_t;

// ADC variables
float mV0=0;
float mV1=0;
float mV2=0;
float mV3=0;

// AHRS and IMU variables
float roll=0;
float pitch=0;
float yaw=0;
float ax=0;
float ay=0;
float az=0;
float gx=0;
float gy=0;
float gz=0;
float mx=0;
float my=0;
float mz=0;
//float orientation_array[12];

// MS5611 variables
float baroT=0;
float baroP=0;

// MS5805 variables
float static_pressure=0;

// MS4515 variables
int count=0;
int V_press=0;
int alfa_press=0;
int beta_press=0;

// RPM variables
int rpmflag, fd;
int rpm=0;

// Hearbeat variables
int hbcount=0;
int hboffcount;
int hboncount;

//AHRS Timing Data
float ahrs_offset[3];
struct timeval ahrs_tv;
float ahrs_dt, maxdt;
float mindt = 0.01;
unsigned long previoustime, currenttime;
float dtsumm = 0;
int isFirst = 1;

// Telemetry variables
unsigned char TMessage[3];
int gcs_ip_addr[4];

#define RC1 3
#define RC2 4
#define RC3 5
#define RC4 6
#define RC5 7
#define SERVO_MIN 1.100 /*mS*/
#define SERVO_MAX 1.900 /*mS*/
#define RED 2
#define GREEN 1
#define BLUE 0
#define LEDMAX 0
#define LEDMIN 4095
#define LEDLOW 3000
#define S0 0
#define S1 5
#define ALFA 2
#define BETA 3
#define VEL 1


//======================== PPM Decoder Function and Thread =====================
void ppmOnEdge(int gpio, int level, uint32_t tick)
{
	if (level == 0) {
		deltaTime = tick - previousTick;
		previousTick = tick;

		if (deltaTime >= ppmSyncLength) { // Sync
			currentChannel = 0;
		}
		else
			if (currentChannel < ppmChannelsNumber)
				channels[currentChannel++] = deltaTime;
	}
}


void * PPMDecode(void * args){

    // GPIO setup
    gpioCfgClock(samplingRate, PI_DEFAULT_CLK_PERIPHERAL, 0);
    gpioInitialise();
    previousTick = gpioTick();
    gpioSetAlertFunc(ppmInputGpio, ppmOnEdge);

}

//==================== Barometer Thread ===================== 
void * acquireBarometerData(void * barom)
{
    MS5611* barometer = (MS5611*)barom;

    while (true) {
	barometer->refreshPressure();
        usleep(10000); // Waiting for pressure data ready
	barometer->readPressure();
        barometer->refreshTemperature();
        usleep(10000); // Waiting for temperature data ready
	barometer->readTemperature();
        barometer->calculatePressureAndTemperature();
    }
}

//==================== MS5805 Static Pressure Thread ============
void * MS5805_data(void *arg){
    ms5805.initialize();
    while(true){
        ms5805.read_pressure_temperature();
	static_pressure=ms5805.getPressure();
    }
}

//====================== RPM Function ================
int get_rpm(int fd){
	while(rpmflag==1){
		char result = serialGetchar(fd);
		if (result=='x'){
			rpmflag=0;
		}
	}
	rpmflag=1;
	int b2=serialGetchar(fd) ;
	int b1=serialGetchar(fd) ;
	int b0=serialGetchar(fd) ;
	serialFlush(fd) ;
	int pw = b0+(b1<<8)+(b2<<16);
    return pw;
}

//==================== MS4515 Function =================
int get_diff_press(int sensor_number){
    if (sensor_number == 1){
	digitalWrite(S0, LOW);
  	digitalWrite(S1, LOW);
        count=arspd.readPressure_raw();
    }
    if (sensor_number == 2){
       	digitalWrite(S0, HIGH);
       	digitalWrite(S1, LOW);
	count=ms4515.readPressure_raw();
    }
    if (sensor_number == 3){
       	digitalWrite(S0, LOW);
       	digitalWrite(S1, HIGH);
       	count=ms4515.readPressure_raw();
    }
    return count;
}


//============================= IMU setup =================================

void imuSetup()
{
    	imu.initialize();
	for(int i = 0; i<100; i++)
	{
		imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		ahrs_offset[0] += (-gx*0.0175);
		ahrs_offset[1] += (-gy*0.0175);
		ahrs_offset[2] += (-gz*0.0175);
		usleep(10000);
	}
	ahrs_offset[0]/=100.0;
	ahrs_offset[1]/=100.0;
	ahrs_offset[2]/=100.0;
	ahrs.setGyroOffset(ahrs_offset[0], ahrs_offset[1], ahrs_offset[2]);
}

//============================== AHRS Loop ====================================

void imuLoop()
{
	gettimeofday(&ahrs_tv,NULL);
	previoustime = currenttime;
	currenttime = 1000000 * ahrs_tv.tv_sec + ahrs_tv.tv_usec;
	ahrs_dt = (currenttime - previoustime) / 1000000.0;
	if(ahrs_dt < 1/1300.0) usleep((1/1300.0-ahrs_dt)*1000000);
        gettimeofday(&ahrs_tv,NULL);
        currenttime = 1000000 * ahrs_tv.tv_sec + ahrs_tv.tv_usec;
	ahrs_dt = (currenttime - previoustime) / 1000000.0;

	imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
// Bench hard iron offsets - RPiB & NAVIO
//     mx-=17.0317;
//     my-=25.7831;
//     mz+=12.5051;

// Installed hard iron offsets - RPiB & NAVIO
//     	mx-=17.9105;
//     	my-=27.4368;
//     	mz+=19.0404;

// Installed hard iron offsets - RPiB+ & NAVIO+
     	mx+=37.4836;
     	my-=212.7862;
     	mz+=8.0719;
     	ahrs.update(ax, ay, az, gx*0.0175, gy*0.0175, gz*0.0175, my, mx, -mz, ahrs_dt);
	ahrs.getEuler(&roll, &pitch, &yaw);

	if (!isFirst)
    	{
    		if (ahrs_dt > maxdt) maxdt = ahrs_dt;
    		if (ahrs_dt < mindt) mindt = ahrs_dt;
	}
    	isFirst = 0;
    	dtsumm += ahrs_dt;
    	if(dtsumm > 0.05)
    	{
        	dtsumm = 0;
    	}
}

// =============================== AHRS Thread ===============================
void * AHRS_Thread(void *arg){
    imuSetup();
    while(1){
        imuLoop();
        usleep(5);
    }
}


// ============================== GPS Thread ==============================
void * GPS_Thread(void *arg){
    if (gps.initialize() != 0){
	error_flag++;
	printf("GPS initialization error!\n");
    }
    gps.getMessages();
}

// ============================== Telemetry Thread ==============================
void * telem_Thread(void *arg){
    gcs_ip_addr[0]=configs.ip1;
    gcs_ip_addr[1]=configs.ip2;
    gcs_ip_addr[2]=configs.ip3;
    gcs_ip_addr[3]=configs.ip4;
    if (link.openConnection(configs.udpport,int gcs_ip_addr[4]) != 0){
	error_flag++;
	printf("Telemetry link error!\n");
    }
    while(1){
    	TMessage[0]=V_press;
    	TMessage[1]=(int)gps.gps_hmsl;
    	TMessage[2]=(int)gps.gps_D;
    	link.sendData(unsigned char TMessage[3]);
    	usleep(500000);
    }
}

// ################################### MAIN SCRIPT ##################################
int main(int argc, char **argv) {
    // RealTime Setup
    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    sched_setscheduler(0, SCHED_FIFO, &sp);

    // Read configuration file data
    if(configs.readfile() != 0){
    	error_flag++;
    	printf("Config file read error!\n\n");
	exit(0);
    };
    // Echo config file to screen
    printf("Sampling Frequency: %d\n",configs.dataSampleRate);
    printf("System Orientation: %d\n",configs.sys_orientation);
    printf("Devices Active: %d %d %d %d %d %d %d %d %d %d\n",configs.IMU_active,configs.AHRS_active,configs.MS5611_active,configs.MS5805_active,configs.MS4515_active,configs.SSC005D_active,configs.RPM_active,configs.PPMdecode_active,configs.GPS_active,configs.ADC_active);
    printf("Telemetry Active: %d\n",configs.telem_active);
    printf("Ground station IP: %d.%d.%d.%d\n",configs.ip1,configs.ip2,configs.ip3,configs.ip4);
    printf("Thread Priorities: %d %d %d %d %d %d\n",configs.gps_priority,configs.ppm_priority,configs.MS5611_priority,configs.MS5805_priority,configs.ahrs_priority,configs.telem_priority);
    printf("Output to Screen: %d\n",configs.OUTPUT_TO_SCREEN);

    if (configs.dataSampleRate>100){
    	printf("WARNING! Data sample rate is high!\n\n");
    }
    if (configs.sys_orientation != 1 && configs.sys_orientation != 2 && configs.sys_orientation != 3 && configs.sys_orientation != 4){
    	printf("System orientation value not valid! Must be either 1,2,3,4\n\n");
    	error_flag++;
    }

    dataSampleRate_t=1.0/(double)configs.dataSampleRate;

    // Setup wiringPi and set ouput pins for MS4515 bank
    wiringPiSetup();
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);

    // Heart beat count
    hboffcount=1.5*configs.dataSampleRate; //secs LED off
    hboncount=0.25*configs.dataSampleRate; //secs LED on

    // ***************** Device Setups ***********************
    // RPM ******
    if (configs.RPM_active == 1) {
    	fd = serialOpen ("/dev/ttyAMA0",115200) ;
    	if (fd==-1){
            error_flag++;
      	    printf("Serial setup error!\n");
    	}
    }

    // ADC ******
    if (configs.ADC_active == 1) {
        adc.setMode(ADS1115_MODE_SINGLESHOT);
        adc.setRate(ADS1115_RATE_860);
    }

    // BARO ******
    MS5611 baro;
    if (configs.MS5611_active == 1) {
    	baro.initialize();
    }

    // IMU ******
    if (configs.AHRS_active == 0) {
	imu.initialize();
    }

    //GPIO setup ******
    static const uint8_t outputEnablePin = RPI_GPIO_27;
    Pin pin(outputEnablePin);
    if (pin.init()) {
        pin.setMode(Pin::GpioModeOutput);
        pin.write(0); /* drive Output Enable low */
    } else {
        error_flag++;
      printf("GPIO setup error!\n");
    }

    // PWM ********************************************************
    pwm.initialize();
    pwm.setFrequency(servoFrequency);

    pwm.setPWM(RED, LEDMIN);
    pwm.setPWM(GREEN, LEDMIN);
    pwm.setPWM(BLUE, LEDMIN);

    // Create Threads ********************************************

    if (configs.GPS_active==1) {
    	pthread_t GPSThread;
        // GPS thread scheduling
        struct sched_param param_gps;
        param_gps.sched_priority = configs.gps_priority;
        pthread_attr_t attr_gps;
        pthread_attr_init(&attr_gps);
        pthread_attr_setinheritsched(&attr_gps, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr_gps, SCHED_FIFO);
        pthread_attr_setschedparam(&attr_gps, &param_gps);

        if(pthread_create(&GPSThread, &attr_gps, GPS_Thread, (void *)arg) != 0)
        {
            error_flag++;
            printf("GPS thread error!\n");
        }
        pthread_attr_destroy(&attr_gps);
        usleep(250000);
    }

    if (configs.PPMdecode_active==1) {
    	pthread_t PPMThread;
        // PPM thread scheduling
        struct sched_param param_ppm;
        param_ppm.sched_priority = configs.ppm_priority;
        pthread_attr_t attr_ppm;
        pthread_attr_init(&attr_ppm);
        pthread_attr_setinheritsched(&attr_ppm, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr_ppm, SCHED_FIFO);
        pthread_attr_setschedparam(&attr_ppm, &param_ppm);

        if(pthread_create(&PPMThread, &attr_ppm, PPMDecode, (void *)arg) != 0)
        {
            error_flag++;
            printf("PPM thread error!\n");
        }
        pthread_attr_destroy(&attr_ppm);
        usleep(250000);
    }

    if (configs.MS5611_active==1) {
    	pthread_t BaroThread;
        // MS5611 thread scheduling
        struct sched_param param_baro1;
        param_baro1.sched_priority = configs.MS5611_priority;
        pthread_attr_t attr_baro1;
        pthread_attr_init(&attr_baro1);
        pthread_attr_setinheritsched(&attr_baro1, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr_baro1, SCHED_FIFO);
        pthread_attr_setschedparam(&attr_baro1, &param_baro1);

    if(pthread_create(&BaroThread, &attr_baro1, acquireBarometerData, (void *)&baro) !=0)
        {
            error_flag++;
            printf("MS5611 thread error!\n");
        }
        pthread_attr_destroy(&attr_baro1);
        usleep(250000);
    }

    if (configs.MS5805_active==1) {
    	pthread_t MS5805Thread;
        // MS5805 thread scheduling
        struct sched_param param_baro2;
        param_baro2.sched_priority = configs.MS5805_priority;
        pthread_attr_t attr_baro2;
        pthread_attr_init(&attr_baro2);
        pthread_attr_setinheritsched(&attr_baro2, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr_baro2, SCHED_FIFO);
        pthread_attr_setschedparam(&attr_baro2, &param_baro2);

    if(pthread_create(&MS5805Thread, &attr_baro2, MS5805_data, (void *)arg) != 0)
        {
            error_flag++;
            printf("MS5805 thread error!\n");
        }
        pthread_attr_destroy(&attr_baro2);
        usleep(250000);
    }

    if (configs.AHRS_active==1) {
    	pthread_t AHRSThread;
        // AHRS thread scheduling
        struct sched_param param_ahrs;
        param_ahrs.sched_priority = configs.ahrs_priority;
        pthread_attr_t attr_ahrs;
        pthread_attr_init(&attr_ahrs);
        pthread_attr_setinheritsched(&attr_ahrs, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr_ahrs, SCHED_FIFO);
        pthread_attr_setschedparam(&attr_ahrs, &param_ahrs);

    if(pthread_create(&AHRSThread, &attr_ahrs, AHRS_Thread, (void *)arg) != 0)
        {
            error_flag++;
            printf("AHRS thread error!\n");
        }
        pthread_attr_destroy(&attr_ahrs);
        usleep(250000);
    }
    
    if (configs.telem_active==1) {
    	pthread_t telemThread;
        // Telemetry thread scheduling
        struct sched_param param_telem;
        param_telem.sched_priority = configs.telem_priority;
        pthread_attr_t attr_telem;
        pthread_attr_init(&attr_telem);
        pthread_attr_setinheritsched(&attr_telem, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr_telem, SCHED_FIFO);
        pthread_attr_setschedparam(&attr_telem, &param_telem);

    if(pthread_create(&telemThread, &attr_telem, telem_Thread, (void *)arg) != 0)
        {
            error_flag++;
            printf("Telemetry thread error!\n");
        }
        pthread_attr_destroy(&attr_telem);
        usleep(250000);
    }    

 /*   if (GPS_active==1) {
        if(pthread_create(&GPSThread, NULL, GPS_Thread, (void *)arg) != 0)
        {
            error_flag++;
            printf("GPS thread error!\n");
        }
    }

    if (PPMdecode_active==1) {
        if(pthread_create(&PPMThread, NULL, PPMDecode, (void *)arg) !=0)
        {
            error_flag++;
            printf("PPM thread error!\n");
        }
    }

    if (MS5611_active==1) {
        if(pthread_create(&BaroThread, NULL, acquireBarometerData, (void *)&baro) !=0)
        {
            error_flag++;
            printf("Baro thread error!\n");
        }
    }

    if (MS5805_active==1) {
        if(pthread_create(&MS5805Thread, NULL, MS5805_data, (void *)arg) != 0)
        {
            error_flag++;
            printf("MS5805 thread error!\n");
        }
    }

    if (AHRS_active==1) {
        if(pthread_create(&AHRSThread, NULL, AHRS_Thread, (void *)arg) != 0)
        {
            error_flag++;
            printf("AHRS thread error!\n");
        }
    }
*/
    //Generate logfile **************************************************
    FILE *logf;
    current_time=time(NULL);
    struct tm *t=localtime(&current_time);
    char logfile[30];
    char filetime[30];
    char filedate[30];
    strftime(logfile,sizeof(logfile),"logfile_%m%d%y_%H%M.txt",t);
    strftime(filedate,sizeof(filedate),"%c",t);
    logf=fopen(logfile,"w");
    if (logf==0) {
        error_flag++;
    }
    fprintf(logf,"*** This log file contains raw data! ***\n\n");
    fprintf(logf,"%s\n\n",filedate);
    fprintf(logf,"Sampling Frequency:\n");
    fprintf(logf,"%d\n\n",configs.dataSampleRate);
    fprintf(logf,"System Orientation:\n");
    fprintf(logf,"%d\n\n",configs.sys_orientation);
    fprintf(logf,"Devices Active:\n");
    fprintf(logf,"IMU AHRS MS5611 MS5805 MS4515 SSC005D RPM PPM GPS ADC\n");
    fprintf(logf,"%d %d %d %d %d %d %d %d %d %d\n\n",configs.IMU_active,configs.AHRS_active,configs.MS5611_active,configs.MS5805_active,configs.MS4515_active,configs.SSC005D_active,configs.RPM_active,configs.PPMdecode_active,configs.GPS_active,configs.ADC_active);
    fprintf(logf,"Time Ax Ay Az Gx Gy Gz Mx My Mz Phi Theta Psi BaroTemp BaroPress StaticPress Alpha Beta V RPM Throttle Aileron Elevator Rudder GPSLat GPSLon GPShMSL GPSNorthV GPSEastV GPSDownV GPS2Dspeed GPS3Dspeed GPSCourse GPSStat GPSnSat GPSPDOP GPSAltAcc GPSVelAcc\n");
    fprintf(logf,"sec,g,g,g,deg/s,deg/s,deg/s,microT,microT,microT,deg,deg,deg,degC,mbar,mbar,count,count,count,pulseWidth,usec,usec,usec,usec,deg,deg,ft,ft,ft/s,ft/s,ft/s,ft/s,ft/s,deg,none,none,ft,ft/s\n");

    // Sometimes a "Device not opened" error occurs and the magnetometer is reading zero
    // Check to see if magnetometer is reading zero and set errorflag
    usleep(500000);
    if (configs.IMU_active == 1 || configs.AHRS_active == 1) {
        imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        if(mx==0 & my==0 & mz==0){
            error_flag++;
            printf("Mag values equal to zero!\n");
        }
    }



    // Flash green LED if no errors occured in startup and red if errors
    // occurred in startup
    if(error_flag == 0){
        pwm.setPWM(GREEN, LEDMAX);
        usleep(500000);
        pwm.setPWM(GREEN, LEDMIN);
        usleep(500000);
        pwm.setPWM(GREEN, LEDMAX);
        usleep(500000);
        pwm.setPWM(GREEN, LEDMIN);
        usleep(500000);
    }
    else{
        pwm.setPWM(RED, LEDMAX);
        usleep(500000);
        pwm.setPWM(RED, LEDMIN);
        usleep(500000);
        pwm.setPWM(RED, LEDMAX);
        usleep(500000);
        pwm.setPWM(RED, LEDMIN);
        usleep(500000);
        pwm.setPWM(RED, LEDMAX);
        exit(0);
    }

    // Get time when the data loop starts running, that way we can get a complete
    // time history of everything.
    gettimeofday(&tv,NULL);
    tstart = 1000000 * tv.tv_sec + tv.tv_usec;

    // Just keep looping
    while(exitflag==0){
        // Get time at start of loop so that we have a reference point
        gettimeofday(&tv,NULL);
        t1 = 1000000 * tv.tv_sec + tv.tv_usec;
        // Set green LED to ON (low intensity) if GPS has 3-D fix
        if (gps.gps_stat == 3){
            pwm.setPWM(GREEN, LEDLOW);
        }
        else {
            pwm.setPWM(GREEN,LEDMIN);
        }
        // READ IMU
	if (configs.IMU_active == 1 && configs.AHRS_active == 0) {
        	imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
	}

        // READ ADC
        if (configs.ADC_active == 1) {
        adc.setMultiplexer(ADS1115_MUX_P0_NG);
        mV0 = adc.getMilliVolts();
        adc.setMultiplexer(ADS1115_MUX_P1_NG);
        mV1 = adc.getMilliVolts();
        adc.setMultiplexer(ADS1115_MUX_P2_NG);
        mV2 = adc.getMilliVolts();
        adc.setMultiplexer(ADS1115_MUX_P3_NG);
        mV3 = adc.getMilliVolts();
        }

        // READ BAROMETER
        if (configs.MS5611_active==1) {
            baroT=baro.getTemperature();
            baroP=baro.getPressure();
        }


        // READ PPM, OUTPUT PWM
	if (configs.PPMdecode_active == 1) {
    		throttle=channels[0];
        	elevator=channels[2];
        	aileron=channels[1];
        	rudder=channels[3];
        	gear=channels[4];

        	pwm.setPWMmS(RC1,throttle/1000.0);
        	pwm.setPWMmS(RC3,elevator/1000.0);
        	pwm.setPWMmS(RC2,aileron/1000.0);
        	pwm.setPWMmS(RC4,rudder/1000.0);
	}

        // READ ALPHA, BETA, VELOCITY
	if (configs.SSC005D_active == 1) {
        	V_press=get_diff_press(VEL);
	}
        if (configs.MS4515_active == 1) {
        	alfa_press=get_diff_press(ALFA);
        	beta_press=get_diff_press(BETA);
	}

        // READ RPM
	if (configs.RPM_active == 1) {
        	rpm=get_rpm(fd);
	}
/*
	// Correct IMU/AHRS data for orientation
	if(sys_orientation==1){
        	orientation_array={ay, ax, -az, gy, gx, -gz, mx, my, mz, pitch, roll, -yaw};
        }
        else if(sys_orientation==2){
        	orientation_array={-ax, ay, -az, -gx, gy, -gz, -my, mx, mz, -roll, pitch, -yaw};
        }
        else if(sys_orientation==3){
        	orientation_array={-ay, -ax, -az, -gy, -gx, -gz, -mx, -my, mz, -pitch, -pitch, -yaw};
        }
        else if(sys_orientation==4){
        	orientation_array={ax, -ay, -az, gx, -gy, -gz, my, -mx, mz, roll, -pitch, -yaw};
        }
*/
        gettimeofday(&tv,NULL);
        t2 = 1000000 * tv.tv_sec + tv.tv_usec;
        dt2 = (t2 - tstart) / 1000000.0;
        
        // Write data to log file if the gear switch is high
        prev_t3=t3;
        if(gear>1500){
            if(gearflag==0){
                gettimeofday(&tv,NULL);
                t3 = 1000000 * tv.tv_sec + tv.tv_usec;
                gearflag=1;
            }
            pwm.setPWM(BLUE, LEDLOW);
            if(configs.sys_orientation==1){
            	fprintf(logf,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.2f %.2f %.2f %.2f %.2f %.2f %d %d %d %d %4.f %4.f %4.f %4.f %.6f %.6f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %d %d %.2f %.2f %.2f\n",dt2, ay, ax, -az, gy, gx, -gz, mx, my, mz, pitch, roll, -yaw, baroT, baroP, static_pressure, alfa_press, beta_press, V_press, rpm, throttle, aileron, elevator, rudder, gps.gps_lat, gps.gps_lon, gps.gps_hmsl, gps.gps_N, gps.gps_E, gps.gps_D, gps.gps_2D, gps.gps_3D, gps.gps_crs, gps.gps_stat, gps.gps_nsat, gps.gps_pdop, gps.gps_altacc, gps.gps_velacc);
            }
            else if(configs.sys_orientation==2){
            	fprintf(logf,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.2f %.2f %.2f %.2f %.2f %.2f %d %d %d %d %4.f %4.f %4.f %4.f %.6f %.6f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %d %d %.2f %.2f %.2f\n",dt2, -ax, ay, -az, -gx, gy, -gz, -my, mx, mz, -roll, pitch, -yaw, baroT, baroP, static_pressure, alfa_press, beta_press, V_press, rpm, throttle, aileron, elevator, rudder, gps.gps_lat, gps.gps_lon, gps.gps_hmsl, gps.gps_N, gps.gps_E, gps.gps_D, gps.gps_2D, gps.gps_3D, gps.gps_crs, gps.gps_stat, gps.gps_nsat, gps.gps_pdop, gps.gps_altacc, gps.gps_velacc);
            }
            else if(configs.sys_orientation==3){
            	fprintf(logf,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.2f %.2f %.2f %.2f %.2f %.2f %d %d %d %d %4.f %4.f %4.f %4.f %.6f %.6f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %d %d %.2f %.2f %.2f\n",dt2, -ay, -ax, -az, -gy, -gx, -gz, -mx, -my, mz, -pitch, -pitch, -yaw, baroT, baroP, static_pressure, alfa_press, beta_press, V_press, rpm, throttle, aileron, elevator, rudder, gps.gps_lat, gps.gps_lon, gps.gps_hmsl, gps.gps_N, gps.gps_E, gps.gps_D, gps.gps_2D, gps.gps_3D, gps.gps_crs, gps.gps_stat, gps.gps_nsat, gps.gps_pdop, gps.gps_altacc, gps.gps_velacc);
            }
            else if(configs.sys_orientation==4){
            	fprintf(logf,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.2f %.2f %.2f %.2f %.2f %.2f %d %d %d %d %4.f %4.f %4.f %4.f %.6f %.6f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %d %d %.2f %.2f %.2f\n",dt2, ax, -ay, -az, gx, -gy, -gz, my, -mx, mz, roll, -pitch, -yaw, baroT, baroP, static_pressure, alfa_press, beta_press, V_press, rpm, throttle, aileron, elevator, rudder, gps.gps_lat, gps.gps_lon, gps.gps_hmsl, gps.gps_N, gps.gps_E, gps.gps_D, gps.gps_2D, gps.gps_3D, gps.gps_crs, gps.gps_stat, gps.gps_nsat, gps.gps_pdop, gps.gps_altacc, gps.gps_velacc);
            }
            fflush(logf);
        }
        else{
            pwm.setPWM(BLUE, LEDMIN);
            gearflag=0;
        }

        // Check to see if gear switched was toggled rapidly (less than 0.5sec between ON and ON state)
        // to see if program should be exited.
        if ((t3-prev_t3)<500000 && (t3-prev_t3)>0 && prev_t3 !=0){
            exitflag=1;
        }

        if (configs.OUTPUT_TO_SCREEN==1) {
            printf("Barometer Temp (C): %.1f\n",baroT);
            printf("Barometer Press (mbar): %.1f\n", baroP);
            printf("Static Pressure (PSF): %.1f\n",static_pressure);
            printf("Acc: %.3f %.3f %.3f\n", ax, ay, az);
            printf("Gyr: %.3f %.3f %.3f\n", gx, gy, gz);
            printf("Mag: %.3f %.3f %.3f\n", mx, my, mz);
            printf("Roll: %.2f Pitch: %.2f Yaw: %.2f\n", roll, pitch, yaw);
            printf("mV: %.1f %.1f %.1f %.1f\n", mV0, mV1, mV2, mV3);
            printf("GPS lat lon: %.6f %.6f\n",gps.gps_lat,gps.gps_lon);
            printf("GPS NV, EV, DV (ft/s): %.2f %.2f %.2f\n",gps.gps_N,gps.gps_E,gps.gps_D);
            printf("GPS speed 2D 3D (ft/s): %.2f %.2f\n",gps.gps_2D,gps.gps_3D);
            printf("GPS course (deg): %.2f\n",gps.gps_crs);
            printf("GPS height MSL (ft): %.2f\n",gps.gps_hmsl);
            printf("GPS status, nSat: %d %d\n",gps.gps_stat,gps.gps_nsat);
            printf("GPS PDOP, altAcc, velAcc: %.2f %.2f %.2f\n",gps.gps_pdop,gps.gps_altacc,gps.gps_velacc);
            printf("THR: %4.f AIL: %4.f ELEV: %4.f RUD: %4.f\n",throttle, aileron, elevator, rudder);
            printf("Alpha count: %d\n",alfa_press);
            printf("Beta count: %d\n",beta_press);
            printf("Vel count: %d\n",V_press);
            printf("RPM: %d\n\n",rpm);
            printf("Loop freq (Hz): %.1f\n", 1/dt1);
            printf("Adj Loop freq (Hz): %.1f\n",1/dt4);
            printf("Elapsed Time: %.2f\n", dt2);
            printf("delta gear: %lu\n\n",(t3-prev_t3));
        }

        //======== Heartbeat =========
        if(hbcount<hboffcount){
            pwm.setPWM(RED, LEDMIN);
            hbcount=hbcount+1;
        }
        if(hbcount>=hboffcount){
            pwm.setPWM(RED,LEDMAX);
            hbcount=hbcount+1;
        }
        if(hbcount>(hboffcount+hboncount)){
            hbcount=0;
        }

        // Now that everything has been complete, get the current time so that we can subtract the time
        // we took at the begining of the loop to see how long it took.
        gettimeofday(&tv,NULL);
        t5 = 1000000 * tv.tv_sec + tv.tv_usec;
        dt1 = (t5 - t1) / 1000000.0;

        // Sleep for the remainder of the time
        if (dt1 < dataSampleRate_t) {
            nextloopt=(dataSampleRate_t-dt1)*1000000.0;
	    usleep(nextloopt);
        }

	if (configs.OUTPUT_TO_SCREEN==1) {
            gettimeofday(&tv,NULL);
            t4 = 1000000 * tv.tv_sec + tv.tv_usec;
            dt4 = (t4 - t1) / 1000000.0;
	}
    }

    // Close devices and log file
    printf("Exiting ...\n");
    fclose(logf);
    serialClose(fd);
    pwm.setPWM(GREEN, LEDMIN);
    pwm.setPWM(BLUE, LEDMIN);
    pwm.setPWM(RED, LEDLOW);
    usleep(100000);
    pwm.setPWM(RED, LEDMIN);
    pwm.setPWM(BLUE, LEDMAX);
    usleep(100000);
    pwm.setPWM(BLUE, LEDMIN);
    pwm.setPWM(RED, LEDLOW);
    usleep(100000);
    pwm.setPWM(RED, LEDMIN);
    pwm.setPWM(BLUE, LEDMAX);
    usleep(100000);
    pwm.setPWM(BLUE, LEDMIN);
    pwm.setPWM(RED, LEDLOW);
    usleep(100000);
    pwm.setPWM(RED, LEDMIN);
    pwm.setPWM(BLUE, LEDMAX);
    usleep(100000);
    pwm.setPWM(BLUE, LEDMIN);
    pwm.setPWM(RED, LEDLOW);
    usleep(100000);
    pwm.setPWM(RED, LEDMIN);
    pwm.setPWM(BLUE, LEDMAX);
    usleep(100000);
    pwm.setPWM(BLUE, LEDMIN);
    pwm.setPWM(RED, LEDLOW);
    usleep(100000);
    pwm.setPWM(RED, LEDMIN);
    return 0;
}
