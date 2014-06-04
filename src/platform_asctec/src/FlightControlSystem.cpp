/* 
    This class provides a basic interface to an Ascending Technologies flight control system.
    The FCS provides the ability to request variables, send commands and set parameters. The
    stock C ACI library will be used from Ascec, which enforces some limitations on the code.
    In particular, one cannot pass a non-static class method as a callback to a C library,
    which means that all callbacks (denoted cb_XXX) are in the ::platform_complacs namespace. 
    This would be a problem for multiple class instantiations, but since there should be a 1
    to 1 relationship between the FCS and ROS node, this should not cause any problems.

    DATA INFORMATION

    1. RO_ALLDATA is filled at 1000Hz, but with variables at the following frequency...
        a. Max 1000Hz (angles and angular velocities) or 
        b. Max  333Hz (accelerations, magnetic measurements, height and climb/sink rate).
        c. Max    5Hz (gps)
    2. The maximum number of variable packets is 3 (we have chosen a, b, c above). Thi can be
       increased, but requires a recompilation of the on-board firmware.
    2. The maximum number of command packets is 3. We will use one for settings, another for
       arming the motors and the last for sending control commands
    3. We are using the platform in 'attitude and thrust control mode' : commands are 
       interpreted as remote control stick inputs and therefore used as inputs for standard 
       attitude controller: desired pitch, desired roll, desired yaw rate and thrust.

    USING AN ASCTEC QUAD

    1. Turn on the UAV with the remote control, and move the throttle to its lowest point.
    2. Wait for sufficient satellites.
    2. Turn the serial switch to enabled.
    3. Start the control .

*/
// System libraries
#include <boost/utility.hpp>

// ROS libraries
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// Integration with the CRATES HAL
#include <hal_quadrotor/Quadrotor.h>
#include <hal_sensor_altimeter/Altimeter.h>
#include <hal_sensor_compass/Compass.h>
#include <hal_sensor_gnss/GNSS.h>
#include <hal_sensor_imu/IMU.h>
#include <hal_sensor_orientation/Orientation.h>

// Required serial engine
#include "serial/AsyncSerial.h"

// Required ACI enfine
extern "C"
{
    #include "aci/asctecCommIntf.h"
}

// Possible UAV states
#define STATE_ATTITUDE_CONTROL      0x0001 // ATTITUDE CONTROL 
#define STATE_STATUS_HEIGHTCTL      0x0002 // HEIGHT CONTROL
#define STATE_POSITION_CONTROL      0x0004 // POSITION CONTROL
#define STATE_COMPASS_FAILURE       0x0010 // COMPASS FAILURE
#define STATE_SERIAL_ENABLED        0x0020 // SERIAL INTERFACE ENABLED
#define STATE_SERIAL_ACTIVE         0x0040 // SERIAL INTERFACE ACTIVE - is active when attitude commands are sent to the LL
#define STATE_EMERGENCY_MODE        0x0080 // EMERGENCY MODE - when RC link is lost -> serial interface disabled
#define STATE_CALIBRATION_ERROR     0x0100 // CALIBRATION ERROR
#define STATE_GYR_CAL_ERROR         0x0200 // GYRO CALIBRATION ERRO
#define STATE_ACC_CAL_ERROR         0x0400 // ACC CALIBRATION ERROR
#define STATE_MAG_STRENGTH_ERROR    0x4000 // MAGNETIC FIELD STRENGTH ERROR
#define STATE_MAG_INCLINATION_ERROR 0x8000 // MAGNETIC INCLINATION ERROR 


// Variables
#define VAR_STATUS                  0x0001  // [ INT16] UAV status information (see above)
#define VAR_FLIGHT_TIME             0x0002  // [ INT16] Total flight time  (s)
#define VAR_BATTERY_VOLTAGE         0x0003  // [ INT16] Battery voltage (mV)
#define VAR_HL_CPU_LOAD             0x0004  // [ INT16] High-level CPU load (Hz)
#define VAR_HL_UPTIME               0x0005  // [ INT16] High-level up-time  (ms)
#define VAR_RRM_MOTOR0              0x0100  // [ UINT8] Quadcopter: front, Hexcopter front-left -- RPM measurements (0..200)
#define VAR_RRM_MOTOR1              0x0101  // [ UINT8] Quadcopter: rear, Hexcopter left -- RPM measurements (0..200)
#define VAR_RRM_MOTOR2              0x0102  // [ UINT8] Quadcopter: left, Hexcopter rear-left -- RPM measurements (0..200)
#define VAR_RRM_MOTOR3              0x0103  // [ UINT8] Quadcopter: right, Hexcopter rear-right -- RPM measurements (0..200)
#define VAR_RRM_MOTOR4              0x0104  // [ UINT8] Quadcopter: N/A, Hexcopter right -- RPM measurements (0..200)
#define VAR_RRM_MOTOR5              0x0105  // [ UINT8] Quadcopter: N/A, Hexcopter front-right -- RPM measurements (0..200)
#define VAR_GPS_LAT                 0x0106  // [ INT32] Latitude from the GPS sensor (degrees * 10^7)
#define VAR_GPS_LON                 0x0107  // [ INT32] Longitude from the GPS sensor (degrees * 10^7)
#define VAR_GPS_HEIGHT              0x0108  // [ INT32] Height from the GPS sensor (mm)
#define VAR_GPS_VEL_EW              0x0109  // [ INT32] Speed in East/West from the GPS sensor (mm/s)
#define VAR_GPS_VEL_NS              0x010A  // [ INT32] Speed in North/South from the GPS sensor (mm/s)
#define VAR_GPS_HEADING             0x010B  // [ INT32] GPS Heading  (deg * 1000)
#define VAR_GPS_POS_ACC             0x010C  // [UINT32] GPS position accuracy estimate  (mm)
#define VAR_GPS_HEIGHT_ACC          0x010D  // [UINT32] GPS height accuracy estimate (mm)
#define VAR_GPS_VEL_ACC             0x010E  // [UINT32] GPS speed accuracy estimate (mm/s)
#define VAR_GPS_NUM_SATS            0x010F  // [UINT32] Number of satellites used in NAV solution (count)
#define VAR_GPS_STATUS              0x0110  // [ INT32] GPS status information  B[7:3]: 0, B[2]: long dir; B[1]: lat dir, B[0]: GPS lock
#define VAR_GPS_TIME_OF_WEEK        0x0111  // [UINT32] Time of the week, 1 week = 604,800 s (ms)
#define VAR_GPS_WEEK                0x0112  // [ INT32] Week counter since 1980 (count)
#define VAR_GYR_CAL_Y               0x0200  // [ INT32] Pitch angle velocity  (0.0154 degree/s, bias free)
#define VAR_GYR_CAL_X               0x0201  // [ INT32] Roll angle velocity (0.0154 degree/s, bias free)
#define VAR_GYR_CAL_Z               0x0202  // [ INT32] Yaw angle velocity  (0.0154 degree/s, bias free)
#define VAR_ACC_CAL_X               0x0203  // [ INT16] Acc-sensor output in x, body frame coordinate system (calibrated: -10000..+10000 = -1g..+1g)
#define VAR_ACC_CAL_Y               0x0204  // [ INT16] Acc-sensor output in y, body frame coordinate system (calibrated: -10000..+10000 = -1g..+1g)
#define VAR_ACC_CAL_Z               0x0205  // [ INT16] Acc-sensor output in z, body frame coordinate system (calibrated: -10000..+10000 = -1g..+1g)
#define VAR_MAG_CAL_X               0x0206  // [ INT32] Magnetic field sensors output in x (Offset free and scaled to +-2500 = +- earth field strength)
#define VAR_MAG_CAL_Y               0x0207  // [ INT32] Magnetic field sensors output in y (Offset free and scaled to +-2500 = +- earth field strength)
#define VAR_MAG_CAL_Z               0x0208  // [ INT32] Magnetic field sensors output in z (Offset free and scaled to +-2500 = +- earth field strength)
#define VAR_EST_ANG_Y               0x0300  // [ INT32] Pitch angle derived by integration of gyro outputs and drift compensated by data fusion (degree*1000, Range: -90000..+90000)
#define VAR_EST_ANG_X               0x0301  // [ INT32] Roll angle derived by integration of gyro outputs and drift compensated by data fusion (degree*1000, Range: -90000..+90000)
#define VAR_EST_ANG_Z               0x0302  // [ INT32] Yaw angle derived by integration of gyro outputs and drift compensated by data fusion (degree*1000, Range: -90000..+90000)
#define VAR_EST_POS_X               0x0303  // [ INT32] Fused latitude with all other sensors (best estimations) (degrees * 10^7)
#define VAR_EST_POS_Y               0x0304  // [ INT32] Fused longitude with all other sensors (best estimations) (degrees * 10^7)
#define VAR_EST_POS_Z               0x0305  // [ INT32] Height after data fusion (mm)
#define VAR_EST_VEL_Z               0x0306  // [ INT32] Differential height after data fusion (mm/s)
#define VAR_EST_VEL_Y               0x0307  // [ INT16] Fused speed in East/West with all other sensors (best estimations)  INT16   mm/s    since V1.0
#define VAR_EST_VEL_X               0x0308  // [ INT16] Fused speed in North/South with all other sensors (best estimations)    INT16   mm/s    since V1.0
#define VAR_CHANNEL0                0x0600  // [UINT16] channel[0] Pitch command received from the remote control (0..4095)
#define VAR_CHANNEL1                0x0601  // [UINT16] channel[1] Roll command received from the remote control (0..4095)
#define VAR_CHANNEL2                0x0602  // [UINT16] channel[2] Thrust command received from the remote control (0..4095)
#define VAR_CHANNEL3                0x0603  // [UINT16] channel[3] Yaw command received from the remote control (0..4095)
#define VAR_CHANNEL4                0x0604  // [UINT16] channel[4] Serial interface enable/disable (>2048 enabled, else disabled)
#define VAR_CHANNEL5                0x0605  // [UINT16] channel[5] Manual / height control / GPS + height control  (0 -> manual mode; 2048 -> height mode; 4095 -> GPS mode)
#define VAR_CHANNEL6                0x0606  // [UINT16] channel[6] Custom remote control data (n/a)
#define VAR_CHANNEL7                0x0607  // [UINT16] channel[7] Custom remote control data (n/a)

// Commands
#define CMD_DIMC_MOTOR_0            0x0500  // [ UINT8] Direct individual motor control, Quadcopter: front, Hexcopter: front-left (0..200 = 0..100 %; 0 = motor off!)
#define CMD_DIMC_MOTOR_1            0x0501  // [ UINT8] Direct individual motor control, Quadcopter: rear, Hexcopter: left (0..200 = 0..100 %; 0 = motor off!)
#define CMD_DIMC_MOTOR_2            0x0502  // [ UINT8] Direct individual motor control, Quadcopter: left, Hexcopter: rear-left (0..200 = 0..100 %; 0 = motor off!)
#define CMD_DIMC_MOTOR_3            0x0503  // [ UINT8] Direct individual motor control, Quadcopter: right, Hexcopter: rear-right (0..200 = 0..100 %; 0 = motor off!)
#define CMD_DIMC_MOTOR_4            0x0504  // [ UINT8] Direct individual motor control, Quadcopter: N/A, Hexcopter: right (0..200 = 0..100 %; 0 = motor off! only used by the AscTec Firefly)
#define CMD_DIMC_MOTOR_5            0x0505  // [ UINT8] Direct individual motor control, Quadcopter: N/A, Hexcopter: front-right (0..200 = 0..100 %; 0 = motor off!  only used by the AscTec Firefly)
#define CMD_DMC_PITCH               0x0506  // [ UINT8] Direct motor control, Pitch rate (0..200 = - 100..+100%)
#define CMD_DMC_ROLL                0x0507  // [ UINT8] Direct motor control, Roll rate (0..200 = - 100..+100%)
#define CMD_DMC_YAW                 0x0508  // [ UINT8] Direct motor control, Yaw rate (0..200 = - 100..+100%)
#define CMD_DMC_THRUST              0x0509  // [ UINT8] Direct motor control, Thrust (0..200 = 0..100%)
#define CMD_ATT_PITCH               0x050A  // [ INT16] Attitude control, Pitch angle (-2047..+2047 (0=neutral))
#define CMD_ATT_ROLL                0x050B  // [ INT16] Attitude control, Roll angle (-2047..+2047 (0=neutral))
#define CMD_ATT_YAW                 0x050C  // [ INT16] Attitude control, Yaw angle (-2047..+2047 (0=neutral))
#define CMD_ATT_THROTTLE            0x050D  // [ INT16] Attitude control, Throttle (0..4095 = 0..100%)
#define CMD_ATT_MASK                0x050E  // [ INT16] Control byte for attitude control (bit 0: pitch, bit 1: roll, bit 2: yaw, bit 3: thrust, bit 4: height, bit 5: GPS position)
#define CMD_CTRL_MODE               0x0600  // [ UINT8] Parameter to set control mode (0x00: direct individual motor control (DIMC), 0x01: direct motor control using standard output mapping (DMC), 0x02: attitude and throttle control (CTRL), 0x03: GPS waypoint control)
#define CMD_CTRL_ENABLED            0x0601  // [ UINT8] Control commands are accepted/ignored by LL processor   UINT8   0x00: ignored, 0x01: accepted)
#define CMD_CTRL_STICKCTL           0x0602  // [ UINT8] Setting if motors can be turned on by using the stick input UINT8   0x00: disable (motors can not be turned on/off by "minimum thrust + full yaw" command), 0x01 enable (motors can be turned on/off by "minimum thrust + full yaw" command)

// Parameters
#define PAR_BATTERY_V_WARN_HIGH     0x0001  // [UINT16] Upper battery warning level -- battery almost empty (mV)
#define PAR_BATTERY_V_WARN_LOW      0x0002  // [UINT16] Lower battery warning level -- battery empty  (mV)
#define PAR_ACOUSTIC_WARNINGS       0x0003  // [ UINT8] Enable/Disable acoustic warnings  (system init 0x01, GPS quality warning 0x02)
#define PAR_PTU_CAM_OPTION4         0x0004  // [ UINT8] Version of Pelican/Firefly PanTilt camera mount option 4 (1 or 2)
#define PAR_CAM_ANGLE_ROLL_OFF      0x0400  // [ INT32] Camera roll angle offset  (0.001deg)
#define PAR_CAM_ANGLE_PITCH_OFF     0x0401  // [ INT32] Camera pitch angle offset (0.001deg)

// LOOP RATES AND FLAGS ///////////////////////////////////////////////////////////

// Receive flags
bool rcv_aci = false;
bool rcv_var = false;
bool rcv_cmd = false;
bool rcv_par = false;
bool ready   = false;
bool error   = false;

// Data polling rates
int rate_imu = 0;       // Rate of state estimate
int rate_sen = 0;       // Rate of state estimate
int rate_pos = 0;       // Position estimate rate

// Origin position (LTP <-> WGS84 conversion)
double latitude  = 51.714403;
double longitude = -0.213223;
double altitude  = 93.070000;

// SERIAL CALLBACKS ///////////////////////////////////////////////////////////////

// Handle to the serial port
CallbackAsyncSerial serial;

// Asycronous callback to transmit data to the serial port
void cb_tx(void* byte, unsigned short cnt)
{
    if (serial.isOpen())
        serial.write((const char*)byte,(size_t)cnt);
}

// Asynchronous callback to receive data from the serial port
void cb_rx(const char* byte, size_t cnt)
{
    for (size_t i = 0; i < cnt; i++)
        aciReceiveHandler(byte[i]);
}

// Heartbeat callback for the ACI engine
void cb_hb(const ros::TimerEvent& event)
{
    // This needs to be called to keep the ACI interface alive
    aciEngine();

    // Heartbeat received
    rcv_aci = true;
}

// PARAMETERS //////////////////////////////////////////////////////////////////////

// Camera pitch and roll
struct _raw_cam
{
    int32_t     pitch;
    int32_t     roll;
} raw_cam;

// This is called when the ACI engine's internal list of parameters is updated
// At this point we are able to setup our own custom packets
void cb_parlist(void)
{
    ROS_INFO("Parameter list received from ACI device");
    aciAddContentToParamPacket(0, PAR_CAM_ANGLE_ROLL_OFF,  &raw_cam.roll);
    aciAddContentToParamPacket(0, PAR_CAM_ANGLE_PITCH_OFF, &raw_cam.pitch);

    ROS_INFO("Updating parameter configuration");
    aciSendParameterPacketConfiguration(0);

    // Parameters received
    rcv_par = true;
}

// COMMANDS //////////////////////////////////////////////////////////////////////

// Raw control messages
struct _raw_ctl
{
    int16_t     roll;
    int16_t     pitch;
    int16_t     yaw;
    int16_t     throttle;
    int16_t     mask;
} raw_ctl;

// Raw settings
struct _raw_set
{
    uint8_t    mode;
    uint8_t    enabled;
    uint8_t    stick;
    int8_t     m0;
    int8_t     m1;
    int8_t     m2;
    int8_t     m3;
} raw_set;

// This is called when the ACI engine's internal list of commands is updated
// At this point we are able to setup our own custom packets
void cb_cmdlist(void)
{
    ROS_INFO("Command list received from ACI device");

    // This is the packet that will be used to send motor control commands
    ROS_INFO("Creating control packet");
    aciAddContentToCmdPacket(0, CMD_ATT_ROLL,       &raw_ctl.roll);
    aciAddContentToCmdPacket(0, CMD_ATT_PITCH,      &raw_ctl.pitch);
    aciAddContentToCmdPacket(0, CMD_ATT_YAW,        &raw_ctl.yaw);
    aciAddContentToCmdPacket(0, CMD_ATT_THROTTLE,   &raw_ctl.throttle);
    aciAddContentToCmdPacket(0, CMD_ATT_MASK,       &raw_ctl.mask);

    // This is the packet that will be used to enable and disable control
    ROS_INFO("Creating settings packet");
    aciAddContentToCmdPacket(1, CMD_CTRL_MODE,      &raw_set.mode);
    aciAddContentToCmdPacket(1, CMD_CTRL_ENABLED,   &raw_set.enabled);
    aciAddContentToCmdPacket(1, CMD_CTRL_STICKCTL,  &raw_set.stick);
    aciAddContentToCmdPacket(1, CMD_DIMC_MOTOR_0,   &raw_set.m0);
    aciAddContentToCmdPacket(1, CMD_DIMC_MOTOR_1,   &raw_set.m1);
    aciAddContentToCmdPacket(1, CMD_DIMC_MOTOR_2,   &raw_set.m2);
    aciAddContentToCmdPacket(1, CMD_DIMC_MOTOR_3,   &raw_set.m3);

    ROS_INFO("Updating command configuration");
    aciSendCommandPacketConfiguration(0,0);
    aciSendCommandPacketConfiguration(1,1);

    // Commands received
    rcv_cmd = true;
}

// VARIABLES ////////////////////////////////////////////////////////////////////

// Raw IMU messages
struct _raw_imu
{
    int32_t     gyr_x;
    int32_t     gyr_y;
    int32_t     gyr_z;
    int16_t     acc_x;
    int16_t     acc_y;
    int16_t     acc_z;
} raw_imu;

// Raw state messages
struct _raw_sen
{
    int32_t     pos_x;
    int32_t     pos_y;
    int32_t     pos_z;
    int32_t     ang_x;
    int32_t     ang_y;
    int32_t     ang_z;
    int32_t     vel_x;
    int32_t     vel_y;
    int32_t     vel_z;
    int32_t     mag_x;
    int32_t     mag_y;
    int32_t     mag_z;
} raw_sen;

// Raw GPS messages
struct _raw_pos
{    
    int32_t     latitude;
    int32_t     longitude;
    int32_t     altitude;
    int32_t     vel_ew;
    int32_t     vel_ns;
    int32_t     heading;
    uint32_t    pdop;
    uint32_t    hdop;
    uint32_t    vdop;
    uint32_t    numsvs;
} raw_pos;

// This is called when the ACI engine's internal list of variables is updated
// At this point we are able to setup our own custom packets
void cb_varlist(void)
{
    ROS_INFO("Variable list received from ACI device");

    int pkt = 0;
    if (rate_imu > 0)
    {
        ROS_INFO("Setting up IMU data in packet %d",pkt);
        aciAddContentToVarPacket(pkt,VAR_GYR_CAL_X,           &raw_imu.gyr_x);
        aciAddContentToVarPacket(pkt,VAR_GYR_CAL_Y,           &raw_imu.gyr_y);
        aciAddContentToVarPacket(pkt,VAR_GYR_CAL_Z,           &raw_imu.gyr_z);
        aciAddContentToVarPacket(pkt,VAR_ACC_CAL_X,           &raw_imu.acc_x);
        aciAddContentToVarPacket(pkt,VAR_ACC_CAL_Y,           &raw_imu.acc_y);
        aciAddContentToVarPacket(pkt,VAR_ACC_CAL_Z,           &raw_imu.acc_z);
        aciSetVarPacketTransmissionRate(pkt,1000/rate_imu);
        pkt++;
    }

    if (rate_sen > 0)
    {
        ROS_INFO("Setting up SEN data in packet %d",pkt);
        aciAddContentToVarPacket(pkt,VAR_EST_POS_X,           &raw_sen.pos_x);
        aciAddContentToVarPacket(pkt,VAR_EST_POS_Y,           &raw_sen.pos_y);
        aciAddContentToVarPacket(pkt,VAR_EST_POS_Z,           &raw_sen.pos_z);
        aciAddContentToVarPacket(pkt,VAR_EST_ANG_X,           &raw_sen.ang_x);
        aciAddContentToVarPacket(pkt,VAR_EST_ANG_Y,           &raw_sen.ang_y);
        aciAddContentToVarPacket(pkt,VAR_EST_ANG_Z,           &raw_sen.ang_z);
        aciAddContentToVarPacket(pkt,VAR_EST_VEL_X,           &raw_sen.vel_x);
        aciAddContentToVarPacket(pkt,VAR_EST_VEL_Y,           &raw_sen.vel_y);
        aciAddContentToVarPacket(pkt,VAR_EST_VEL_Z,           &raw_sen.vel_z);
        aciAddContentToVarPacket(pkt,VAR_MAG_CAL_X,           &raw_sen.mag_x);
        aciAddContentToVarPacket(pkt,VAR_MAG_CAL_Y,           &raw_sen.mag_y);
        aciAddContentToVarPacket(pkt,VAR_MAG_CAL_Z,           &raw_sen.mag_z);
        aciSetVarPacketTransmissionRate(pkt,1000/rate_sen);
        pkt++;
    }

    if (rate_pos > 0)
    {
        ROS_INFO("Setting up POS data in packet %d",pkt);        
        aciAddContentToVarPacket(pkt,VAR_GPS_LAT,             &raw_pos.latitude);
        aciAddContentToVarPacket(pkt,VAR_GPS_LON,             &raw_pos.longitude);
        aciAddContentToVarPacket(pkt,VAR_GPS_HEIGHT,          &raw_pos.altitude);
        aciAddContentToVarPacket(pkt,VAR_GPS_VEL_NS,          &raw_pos.vel_ns);
        aciAddContentToVarPacket(pkt,VAR_GPS_VEL_EW,          &raw_pos.vel_ew);
        aciAddContentToVarPacket(pkt,VAR_GPS_HEADING,         &raw_pos.heading);
        aciAddContentToVarPacket(pkt,VAR_GPS_POS_ACC,         &raw_pos.pdop);
        aciAddContentToVarPacket(pkt,VAR_GPS_HEIGHT_ACC,      &raw_pos.hdop);
        aciAddContentToVarPacket(pkt,VAR_GPS_VEL_ACC,         &raw_pos.vdop);
        aciAddContentToVarPacket(pkt,VAR_GPS_NUM_SATS,        &raw_pos.numsvs);
        aciSetVarPacketTransmissionRate(pkt,1000/rate_pos);
        pkt++;
    }

    ROS_INFO("Updating packet rates");
    aciVarPacketUpdateTransmissionRates();

    ROS_INFO("Updating packet configuration");
    for (int i = 0; i < pkt; i++)
        aciSendVariablePacketConfiguration(i);

    // We have received the variables
    rcv_var = true;
}

// FLIGHT CONTROL SYSTEM ///////////////////////////////////////////////////////////

namespace platform_asctec
{
    // Class derives from Nodelet and all HALs
    class FlightControlSystem : 
        public nodelet::Nodelet, 
        public hal::quadrotor::Quadrotor,
        public hal::sensor::Altimeter,
        public hal::sensor::IMU,
        public hal::sensor::Compass,
        public hal::sensor::GNSS,
        public hal::sensor::Orientation
    {

    private:
        
        // Callback timers for probing data
        ros::Timer timerHeartbeat;
        ros::Timer timerTimeout;

        // Used for timeout checking
        bool timeout;

        // Called when the HAL wants an altimeter reading
        bool GetMeasurement(hal_sensor_altimeter::Data& msg)
        {
            // If the FCS hasn't initialised yet
            if (!ready) return false;

            // Copy over the variables
            aciSynchronizeVars();

            // Return the normalised value
            msg.t   = ros::Time::now().toSec();
            msg.z   = (double) raw_sen.pos_z / 1e3; // [ INT32] Height after data fusion (mm)
            msg.w   = (double) raw_sen.vel_z / 1e3; // [ INT32] Differential height after data fusion (mm/s)

            // Feed the measurement to the navigation engine
            GetNavPtr()->Process(msg);

            // Success!
            return true;
        }

        // Called when the HAL wants a compass reading
        bool GetMeasurement(hal_sensor_compass::Data& msg)
        {
            // If the FCS hasn't initialised yet
            if (!ready) return false;

            // Copy over the variables
            aciSynchronizeVars();

            // Return the normalised value
            msg.t   = ros::Time::now().toSec();
            msg.x   = 0.000400000 * (double) raw_sen.mag_x; // Magnetic field strength: +-2500 = +- earth field strength
            msg.y   = 0.000400000 * (double) raw_sen.mag_y; // Magnetic field strength: +-2500 = +- earth field strength
            msg.z   = 0.000400000 * (double) raw_sen.mag_z; // Magnetic field strength: +-2500 = +- earth field strength

            // Feed the measurement to the navigation engine
            GetNavPtr()->Process(msg);

            // Success!
            return true;
        }
        
        // Called when the HAL wants an imu reading
        bool GetMeasurement(hal_sensor_imu::Data& msg)
        {
            // If the FCS hasn't initialised yet
            if (!ready) return false;

            // Copy over the variables
            aciSynchronizeVars();

            // Return the scaled value (in rads/sec and m/s/s)
            msg.t   = ros::Time::now().toSec();
            msg.p   = 0.000268780 * (double) raw_imu.gyr_x;
            msg.q   = 0.000268780 * (double) raw_imu.gyr_y;
            msg.r   = 0.000268780 * (double) raw_imu.gyr_z;
            msg.du  = 0.000980665 * (double) raw_imu.acc_x;
            msg.dv  = 0.000980665 * (double) raw_imu.acc_y;
            msg.dw  = 0.000980665 * (double) raw_imu.acc_z;

            // Feed the measurement to the navigation engine
            GetNavPtr()->Process(msg);

            // Get the imu
            return true;
        }
        
        // Called when the HAL wants a gnss reading
        bool GetMeasurement(hal_sensor_gnss::Data& msg)
        {
            // If the FCS hasn't initialised yet
            if (!ready) return false;

            // Copy over the variables
            aciSynchronizeVars();

            // Return the scaled value
            msg.t         = ros::Time::now().toSec();
            msg.latitude  = (double) raw_pos.latitude  / 1e7;   // [ INT32] Latitude from the GPS sensor (degrees * 10^7)
            msg.longitude = (double) raw_pos.longitude / 1e7;   // [ INT32] Longitude from the GPS sensor (degrees * 10^7)
            msg.altitude  = (double) raw_pos.altitude  / 1e3;   // [ INT32] Height from the GPS sensor (mm)
            msg.vel_ew    = (double) raw_pos.vel_ew    / 1e3;   // [ INT32] Speed in East/West from the GPS sensor (mm/s)
            msg.vel_ns    = (double) raw_pos.vel_ns    / 1e3;   // [ INT32] Speed in North/South from the GPS sensor (mm/s)
            msg.heading   = (double) raw_pos.heading   / 1e3;   // [ INT32] GPS Heading  (deg * 1000)
            msg.pdop      = (double) raw_pos.pdop      / 1e3;   // [UINT32] GPS position accuracy estimate  (mm)
            msg.hdop      = (double) raw_pos.hdop      / 1e3;   // [UINT32] GPS height accuracy estimate (mm)
            msg.vdop      = (double) raw_pos.vdop      / 1e3;   // [UINT32] GPS speed accuracy estimate (mm/s)
            msg.numsvs    = raw_pos.numsvs;                     // [UINT32] Number of satellites used in NAV solution (count)

            // Feed the measurement to the navigation engine
            GetNavPtr()->Process(msg);

            // Get the GNSS
            return false;
        }
        
        // Called when the HAL wants an orientation reading
        bool GetMeasurement(hal_sensor_orientation::Data& msg)
        {
            // If the FCS hasn't initialised yet
            if (!ready) return false;

            // Copy over the variables
            aciSynchronizeVars();

            // Return the normalised values (in radians)
            msg.t     = ros::Time::now().toSec();
            msg.pitch = 0.01745329251 * (double) raw_sen.ang_x / 1e3;   // Drift-free Pitch (degree*1000, Range: -90000..+90000)
            msg.roll  = 0.01745329251 * (double) raw_sen.ang_y / 1e3;   // Drift-free Roll (degree*1000, Range: -90000..+90000)
            msg.yaw   = 0.01745329251 * (double) raw_sen.ang_z / 1e3;   // Drift-free Yaw (degree*1000, Range: -90000..+90000)
            msg.p     = 0.00026878000 * (double) raw_imu.gyr_x;         // Drift-free gyro x
            msg.q     = 0.00026878000 * (double) raw_imu.gyr_y;         // Drift-free gyro y
            msg.r     = 0.00026878000 * (double) raw_imu.gyr_z;         // Drift-free gyro z

            // Feed the measurement to the navigation engine
            GetNavPtr()->Process(msg);

            // Success!
            return true;
        }

        // Called when the HAL wants the truthful state of the platform
        void GetTruth(hal_quadrotor::State& state)
        {
            // Not valid for hardware platforms
        }

        // Called when the HAL wants to set the truthful state of the platform
        void SetTruth(const hal_quadrotor::State& state)
        {
            // Not valid for hardware platforms
        }
        
        // Called when the HAL wants to pass down some control to the platform
        double SetControl(const hal_quadrotor::Control &control)
        {
            /*
            // Only issue control when system is ready
            if (ready)
            {
                raw_ctl.roll        = -2291.83118052329 * control.roll;
                raw_ctl.pitch       = -2291.83118052329 * control.pitch;
                raw_ctl.yaw         = -460.597254433196 * control.yaw;
                raw_ctl.throttle    = 4097.00000000000 * control.throttle;
                raw_ctl.mask        = 0b00001111;   // Direct control (no height/gps)
                
                // Send the control command
                aciUpdateCmdPacket(0);
            }
            */
            // Time st which control was applied
            return ros::Time::now().toSec();
        }

        // Arm the motors
        bool ArmMotors(bool arm)
        {
            /*
            // Only issue control when system is ready
            if (ready)
            {
                if (arm)
                {
                    ROS_INFO("Arming motors");
                    raw_set.mode        = 0x02;
                    raw_set.enabled     = 0x01;
                    raw_set.stick       = 0x01;   
                    raw_set.m0          = 0x00;
                    raw_set.m1          = 0x00;
                    raw_set.m2          = 0x00;
                    raw_set.m3          = 0x00; 
                }
                else
                {
                    ROS_INFO("Disarming motors");
                    raw_set.mode        = 0x00;
                    raw_set.enabled     = 0x01;
                    raw_set.stick       = 0x01;
                    raw_set.m0          = 0x00;
                    raw_set.m1          = 0x00;
                    raw_set.m2          = 0x00;
                    raw_set.m3          = 0x00;    
                }

                // Send the control command
                aciUpdateCmdPacket(1);
            }
            */

            // Success
            return arm;
        }

        // Error callback
        void Timeout(const ros::TimerEvent& event)
        {
            timeout = true;
        }

    public:

        // Constructor
        FlightControlSystem() :
            nodelet::Nodelet(), 
            hal::quadrotor::Quadrotor(),
            hal::sensor::Altimeter(),
            hal::sensor::IMU(),
            hal::sensor::Compass(),
            hal::sensor::GNSS(),
            hal::sensor::Orientation()
        {
            // Do nothing
        }

        // When nodelet is initialised
        void onInit()
        {
            // Get the node handle
            ros::NodeHandle nh = getPrivateNodeHandle();

            // Initialise the HALs
            ROS_INFO("Initialising HALS");
            hal::quadrotor::Quadrotor::Init(nh);
            hal::sensor::Altimeter::Init(nh);
            hal::sensor::Compass::Init(nh);
            hal::sensor::GNSS::Init(nh);
            hal::sensor::IMU::Init(nh);
            hal::sensor::Orientation::Init(nh);

            // Extract launch file parameters
            ROS_INFO("Grabbing parameters from launch file");
            std::string port; int baud;
            if (!nh.getParam("serial_port",port))     port      = "/dev/ttyUSB0";
            if (!nh.getParam("serial_baud",baud))     baud      = 57600;
            if (!nh.getParam("rate_imu",rate_imu))    rate_imu  = 0;
            if (!nh.getParam("rate_sen",rate_sen))    rate_sen  = 0;
            if (!nh.getParam("rate_pos",rate_pos))    rate_pos  = 0;
            if (!nh.getParam("origin_lat",latitude))  latitude  = 0;
            if (!nh.getParam("origin_lon",longitude)) longitude = 0;
            if (!nh.getParam("origin_alt",altitude))  altitude  = 0;

            // Calibrate navigation
            ROS_INFO("Calibrating navigation %f %f %f", latitude, longitude, altitude);
            GetNavPtr()->SetOrigin(latitude, longitude, altitude);

            // Initialise the ACI engine
            ROS_INFO("Initialising ACI engine");
            aciInit();
            
            // Ope serial port
            ROS_INFO("Opening serial port");
            serial.open(port, baud);
            if (serial.isOpen())
            {
                ROS_INFO("Configuring ACI receive callback");
                serial.setCallback(&cb_rx);
                
                ROS_INFO("Configuring ACI transmit callback");
                aciSetSendDataCallback(&cb_tx);
                
                ROS_INFO("Configuring ACI messages");
                aciSetVarListUpdateFinishedCallback(&cb_varlist);
                aciSetCmdListUpdateFinishedCallback(&cb_cmdlist);
                aciSetParamListUpdateFinishedCallback(&cb_parlist);

                ROS_INFO("Configuring ACI engine rate");
                aciSetEngineRate(1000,1);

                ROS_INFO("Starting ACI heartbeat");
                timerHeartbeat = nh.createTimer(
                    ros::Duration(0.001),                  // duration
                    &cb_hb,                              // callback
                    false                                // object
                );
                
                ROS_INFO("Waiting for first heartbeat");
                while (!rcv_aci)
                    usleep(1000);

                // Timeout-based parameter list querying
                do
                {
                    ROS_INFO("Querying parameter list");
                    aciGetDeviceParametersList();
                    timeout = false;
                    timerTimeout = nh.createTimer(
                        ros::Duration(5.0),                   // duration
                        &FlightControlSystem::Timeout,         // callback
                        this,                                  // object
                        true                                   // oneshot
                    );
                    while (!rcv_par && !timeout)
                        usleep(1000);
                    timerTimeout.stop();
                    if (!timeout)
                        break;
                    ROS_WARN("Timeout");
                }
                while (timeout);

                // Timeout-based command list querying
                do
                {
                    ROS_INFO("Querying command list");
                    aciGetDeviceCommandsList();
                    timeout = false;
                    timerTimeout = nh.createTimer(
                        ros::Duration(5.0),                   // duration
                        &FlightControlSystem::Timeout,         // callback
                        this,                                  // object
                        true                                   // oneshot
                    );
                    while (!rcv_cmd && !timeout)
                        usleep(1000);
                    timerTimeout.stop();
                    if (!timeout)
                        break;
                    ROS_WARN("Timeout");
                }
                while (timeout);

                // Disarm the motors
                ROS_INFO("Disarming motors");
                ArmMotors(false);

                // Timeout-based variable list querying
                do
                {
                    ROS_INFO("Querying variable list");
                    aciGetDeviceVariablesList();
                    timeout = false;
                    timerTimeout = nh.createTimer(
                        ros::Duration(5.0),                   // duration
                        &FlightControlSystem::Timeout,         // callback
                        this,                                  // object
                        true                                   // oneshot
                    );
                    while (!rcv_var && !timeout)
                        usleep(1000);
                    timerTimeout.stop();
                    if (!timeout)
                        break;
                    ROS_WARN("Timeout");
                }
                while (timeout);

                // FCS ready
                ready = true;
            }
        }
    };

    // Required in order to be used as a nodelet
    PLUGINLIB_EXPORT_CLASS(platform_asctec::FlightControlSystem, nodelet::Nodelet);
}