/*
NavDAQ.cpp

Code from:
	Lars Soltmann
      		https://github.com/lsoltmann/NavDAQ
	EMLID
		https://github.com/emlid/Navio
		http://www.emlid.com

Revision History
Rev A - 17 Jan 2015 - Created and debugged
Rev B - 30 Jan 2015 - Program split into different versions to cut out what is not needed for the current test
                     - This version records the raw data only and will require post processing after download
Rev C - 11 Feb 2015 - Added MS5805 code
		       - Added heartbeat
		       - General improvements and clean up
Rev D - 13 Feb 2015 - Added check on magnetomer readings to make sure 'device opened error' is caught
Rev E - 17 Feb 2015 - Added 20Hz LPF to gyro and accel
Rev F - 24 Feb 2015 - Added AHRS code and thread
Rev G - 10 Mar 2015 - Renamed NavDAQ_raw_AHRS to NavDAQ2, added installed hard iron calibration
Rev H - 26 Mar 2015 - Removed LPF on accel and gyro, increased sigfigs on baro and static transducer, chnaged loop time to run at true 50Hz
Rev I - 04 Apr 2015 - Upgraded to NavDAQ3, GPS thread added, display ouput modified, change LED to reflect GPS status
Rev J - 21 Apr 2015 - Hardcoded C6 for MS5805 from coeff data provided by MeasSpec
Rev K - 09 May 2015 - Major overhaul of code due to excessive CPU usage causing lag in data, specifically GPS
Rev L - 27 May 2015 - GPS switched to new library, NMEA messages disabled, GPS lag appears to be gone
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
//#include "/home/pi/Navio/C++/Navio/Ublox.h"
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

using namespace Navio;
using namespace std;


// *!*!*!*!*!*!*!*!*!*! System and Data Options *!*!*!*!*!*!*!*!*!*!*!
int dataSampleRate=25; // Hertz

int MS5805_active=1; // 1 = yes, 0 = no
int MS5611_active=0;
int PPMdecode_active=1;
int AHRS_active=1;
int GPS_active=1;
int MS4515_active=1;
int ADC_active=0;
int RPM_active=0;
int IMU_active=1;

int gps_priority=45;
int ppm_priority=30;
int MS5611_priority=20;
int MS5805_priority=20;
int ahrs_priority=33;

int OUTPUT_TO_SCREEN=0;
//pthread_mutex_t i2c_mutex=PTHREAD_MUTEX_INITIALIZER;
//pthread_mutex_t spi_mutex=PTHREAD_MUTEX_INITIALIZER;
// *!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!

//================================ PPM Decoder and Servo Setup  =====================================
unsigned int samplingRate      = 5;      // 1 microsecond (can be 1,2,4,5,10)
unsigned int ppmInputGpio      = 4;      // PPM input on Navio's 2.54 header
unsigned int ppmSyncLength     = 7000;   // Length of PPM sync pause
unsigned int ppmChannelsNumber = 5;      // Number of channels packed in PPM
unsigned int servoFrequency    = 50;     // Servo control frequency

// **** This data is set specifically for the RMILEC high-precision PWM/PPM/SBus Signal Converter V2
// **** R8 has been removed on the NAVIO, 3.3V PPM ONLY!

MS5805 ms5805;
MS4515 ms4515;
MPU9250 imu;
ADS1115 adc;
AHRS ahrs;
Ublox gps;
PCA9685 pwm;

//============================ Variable Setup  ==================================
// PPM variables
float channels[5], throttle, elevator, aileron, rudder, gear;
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
//        pthread_mutex_lock(&i2c_mutex);
	barometer->refreshPressure();
//  	pthread_mutex_unlock(&i2c_mutex);

        usleep(10000); // Waiting for pressure data ready

//	pthread_mutex_lock(&i2c_mutex);
	barometer->readPressure();
//	pthread_mutex_unlock(&i2c_mutex);

//	pthread_mutex_lock(&i2c_mutex);
        barometer->refreshTemperature();
//	pthread_mutex_unlock(&i2c_mutex);

        usleep(10000); // Waiting for temperature data ready

//        pthread_mutex_lock(&i2c_mutex);
	barometer->readTemperature();
//	pthread_mutex_unlock(&i2c_mutex);

        barometer->calculatePressureAndTemperature();
    }
}

//==================== MS5805 Static Pressure Thread ============
void * MS5805_data(void *arg){
    ms5805.initialize();
    while(true){
//	pthread_mutex_lock(&i2c_mutex);
        ms5805.read_pressure_temperature();
//        pthread_mutex_unlock(&i2c_mutex);
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
	int b1=serialGetchar(fd) ;
	int b0=serialGetchar(fd) ;
	serialFlush(fd) ;
	int pw = b0+(b1<<8);
    return pw;
}

//==================== MS4515 Function =================
int get_diff_press(int sensor_number){
    if (sensor_number == 1){
	digitalWrite(S0, LOW);
  	digitalWrite(S1, LOW);
//	pthread_mutex_lock(&i2c_mutex);
        count=ms4515.readPressure_raw();
//	pthread_mutex_unlock(&i2c_mutex);
    }
    if (sensor_number == 2){
       	digitalWrite(S0, HIGH);
       	digitalWrite(S1, LOW);
//	pthread_mutex_lock(&i2c_mutex);
	count=ms4515.readPressure_raw();
//	pthread_mutex_unlock(&i2c_mutex);
    }
    if (sensor_number == 3){
       	digitalWrite(S0, LOW);
       	digitalWrite(S1, HIGH);
//	pthread_mutex_lock(&i2c_mutex);
       	count=ms4515.readPressure_raw();
//	pthread_mutex_unlock(&i2c_mutex);
    }
    return count;
}


//============================= IMU setup =================================

void imuSetup()
{
    //----------------------- MPU initialization ------------------------------

    imu.initialize();

    //-------------------------------------------------------------------------

//	printf("Beginning Gyro calibration...\n");
	for(int i = 0; i<100; i++)
	{
//		pthread_mutex_lock(&spi_mutex);
		imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//		pthread_mutex_unlock(&spi_mutex);
		ahrs_offset[0] += (-gx*0.0175);
		ahrs_offset[1] += (-gy*0.0175);
		ahrs_offset[2] += (-gz*0.0175);
		usleep(10000);
	}
	ahrs_offset[0]/=100.0;
	ahrs_offset[1]/=100.0;
	ahrs_offset[2]/=100.0;

//	printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);
	ahrs.setGyroOffset(ahrs_offset[0], ahrs_offset[1], ahrs_offset[2]);
}

//============================== AHRS Loop ====================================

void imuLoop()
{
    //----------------------- Calculate delta time ----------------------------

	gettimeofday(&ahrs_tv,NULL);
	previoustime = currenttime;
	currenttime = 1000000 * ahrs_tv.tv_sec + ahrs_tv.tv_usec;
	ahrs_dt = (currenttime - previoustime) / 1000000.0;
	if(ahrs_dt < 1/1300.0) usleep((1/1300.0-ahrs_dt)*1000000);
        gettimeofday(&ahrs_tv,NULL);
        currenttime = 1000000 * ahrs_tv.tv_sec + ahrs_tv.tv_usec;
	ahrs_dt = (currenttime - previoustime) / 1000000.0;

    //-------- Read raw measurements from the MPU and update AHRS --------------

//    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//    ahrs.updateIMU(ax, ay, az, gx*0.0175, gy*0.0175, gz*0.0175, dt);

    // FIXME In order to use magnetometer it's orientation has to be fixed
    // according to MPU9250 datasheet. Also, soft and hard iron calibration
    // would be required.
//     pthread_mutex_lock(&spi_mutex);
     imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
//     pthread_mutex_unlock(&spi_mutex);
// Bench hard iron offsets
//     mx-=17.0317;
//     my-=25.7831;
//     mz+=12.5051;

// Installed hard iron offsets
     mx-=17.9105;
     my-=27.4368;
     mz+=19.0404;
     ahrs.update(ax, ay, az, gx*0.0175, gy*0.0175, gz*0.0175, my, mx, -mz, ahrs_dt);

    //------------------------ Read Euler angles ------------------------------

    ahrs.getEuler(&roll, &pitch, &yaw);

    //------------------- Discard the time of the first cycle -----------------

    if (!isFirst)
    {
    	if (ahrs_dt > maxdt) maxdt = ahrs_dt;
    	if (ahrs_dt < mindt) mindt = ahrs_dt;
    }
    isFirst = 0;

    //------------- Console and network output with a lowered rate ------------

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


// ################################### MAIN SCRIPT ##################################
int main(int argc, char **argv) {
    // RealTime Setup
    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    sched_setscheduler(0, SCHED_FIFO, &sp);

    dataSampleRate_t=1.0/(double)dataSampleRate;

    // Setup wiringPi and set ouput pins for MS4515 bank
    wiringPiSetup();
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);

    // Heart beat count
    hboffcount=1.5*dataSampleRate; //secs LED off
    hboncount=0.25*dataSampleRate; //secs LED on

    // ***************** Device Setups ***********************
    // RPM ******
    if (RPM_active == 1) {
    	fd = serialOpen ("/dev/ttyAMA0",115200) ;
    	if (fd==-1){
            error_flag++;
      	    printf("Serial setup error!\n");
    	}
    }

    // ADC ******
    if (ADC_active == 1) {
        adc.setMode(ADS1115_MODE_SINGLESHOT);
        adc.setRate(ADS1115_RATE_860);
    }

    // BARO ******
    MS5611 baro;
    if (MS5611_active == 1) {
    	baro.initialize();
    }

    // IMU ******
    if (AHRS_active == 0) {
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
    //pthread_mutex_init(&_mutex);
    pthread_t PPMThread;
    pthread_t BaroThread;
    pthread_t MS5805Thread;
    pthread_t AHRSThread;
    pthread_t GPSThread;

    if (GPS_active==1) {
        // GPS thread scheduling
        struct sched_param param_gps;
        param_gps.sched_priority = gps_priority;
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
    }

    if (PPMdecode_active==1) {
        // PPM thread scheduling
        struct sched_param param_ppm;
        param_ppm.sched_priority = ppm_priority;
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
    }

    if (MS5611_active==1) {
        // MS5611 thread scheduling
        struct sched_param param_baro1;
        param_baro1.sched_priority = MS5611_priority;
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
    }

    if (MS5805_active==1) {
        // MS5805 thread scheduling
        struct sched_param param_baro2;
        param_baro2.sched_priority = MS5805_priority;
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
    }

    if (AHRS_active==1) {
        // AHRS thread scheduling
        struct sched_param param_ahrs;
        param_ahrs.sched_priority = ahrs_priority;
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
    //Set logfile **************************************************
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
    fprintf(logf,"Devices Active:\n");
    fprintf(logf,"IMU AHRS MS5611 MS5805 MS4515 RPM PPM GPS ADC\n");
    fprintf(logf,"%d %d %d %d %d %d %d %d %d\n\n",IMU_active,AHRS_active,MS5611_active,MS5805_active,MS4515_active,RPM_active,PPMdecode_active,GPS_active,ADC_active);
    fprintf(logf,"Time Ax Ay Az Gx Gy Gz Mx My Mz Roll Pitch Yaw BaroTemp BaroPress StaticPress Alpha Beta V RPM Throttle Aileron Elevator Rudder GPSLat GPSLon GPShAGL GPShMSL GPSNorthV GPSEastV GPSDownV GPS2Dspeed GPS3Dspeed GPSCourse GPSStat\n");
    fprintf(logf,"sec,g,g,g,deg/s,deg/s,deg/s,microT,microT,microT,deg,deg,deg,degC,mbar,mbar,count,count,count,pulseWidth,usec,usec,usec,usec,deg,deg,ft,ft,ft/s,ft/s,ft/s,ft/s,ft/s,deg,none\n");

    // Sometimes a "Device not opened" error occurs and the magnetometer is reading zero
    // Check to see if magnetometer is reading zero and set errorflag
    usleep(500000);
    if (IMU_active == 1 || AHRS_active == 1) {
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
	if (IMU_active == 1 && AHRS_active == 0) {
//		pthread_mutex_lock(&spi_mutex);
        	imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
//		pthread_mutex_unlock(&spi_mutex);
	}

        // READ ADC
//        adc.setMultiplexer(ADS1115_MUX_P0_NG);
//        mV0 = adc.getMilliVolts();
//        adc.setMultiplexer(ADS1115_MUX_P1_NG);
//        mV1 = adc.getMilliVolts();
//        adc.setMultiplexer(ADS1115_MUX_P2_NG);
//        mV2 = adc.getMilliVolts();
//        adc.setMultiplexer(ADS1115_MUX_P3_NG);
//        mV3 = adc.getMilliVolts();

        // READ BAROMETER
        if (MS5611_active==1) {
            baroT=baro.getTemperature();
            baroP=baro.getPressure();
        }


        // READ PPM, OUTPUT PWM
	if (PPMdecode_active == 1) {
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
	if (MS4515_active == 1) {
        	V_press=get_diff_press(VEL);
        	alfa_press=get_diff_press(ALFA);
        	beta_press=get_diff_press(BETA);
	}

        // READ RPM
	if (RPM_active == 1) {
        	rpm=get_rpm(fd);
	}

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
            fprintf(logf,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.2f %.2f %.2f %.2f %.2f %.2f %d %d %d %d %4.f %4.f %4.f %4.f %.6f %.6f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %d\n",dt2, ax, ay, az, gx, gy, gz, mx, my, mz, roll, pitch, yaw, baroT, baroP, static_pressure, alfa_press, beta_press, V_press, rpm, throttle, aileron, elevator, rudder, gps.gps_lat, gps.gps_lon, gps.gps_h, gps.gps_hmsl, gps.gps_N, gps.gps_E, gps.gps_D, gps.gps_2D, gps.gps_3D, gps.gps_crs, gps.gps_stat);
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

        if (OUTPUT_TO_SCREEN==1) {
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
            printf("GPS height AGL MSL (ft): %.2f %.2f\n",gps.gps_h,gps.gps_hmsl);
            printf("GPS status: %d\n",gps.gps_stat);
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

	if (OUTPUT_TO_SCREEN==1) {
            gettimeofday(&tv,NULL);
            t4 = 1000000 * tv.tv_sec + tv.tv_usec;
            dt4 = (t4 - t1) / 1000000.0;
	}
    }

    // Close devices and log file
    fclose(logf);
    serialClose(fd);
    pwm.setPWM(GREEN, LEDMIN);
    pwm.setPWM(BLUE, LEDMIN);
    printf("Exiting ...\n");
    pwm.setPWM(RED, LEDMAX);
    usleep(100000);
    pwm.setPWM(RED, LEDMIN);
    pwm.setPWM(GREEN, LEDMAX);
    usleep(100000);
    pwm.setPWM(GREEN, LEDMIN);
    pwm.setPWM(RED, LEDMAX);
    usleep(100000);
    pwm.setPWM(RED, LEDMIN);
    pwm.setPWM(GREEN, LEDMAX);
    usleep(100000);
    pwm.setPWM(GREEN, LEDMIN);
    pwm.setPWM(RED, LEDMAX);
    usleep(100000);
    pwm.setPWM(RED, LEDMIN);
    pwm.setPWM(GREEN, LEDMAX);
    usleep(100000);
    pwm.setPWM(GREEN, LEDMIN);
    pwm.setPWM(RED, LEDMAX);
    usleep(100000);
    pwm.setPWM(RED, LEDMIN);
    pwm.setPWM(GREEN, LEDMAX);
    usleep(100000);
    pwm.setPWM(GREEN, LEDMIN);
    pwm.setPWM(RED, LEDMAX);
    usleep(100000);
    pwm.setPWM(RED, LEDMIN);
    return 0;
}
