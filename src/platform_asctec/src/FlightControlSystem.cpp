/* 
    This class provides a basic interface to an Ascending Technologies flight control system.
    The FCS provides the ability to request variables, send commands and set parameters. The
    stock C ACI library will be used from Ascec, which enforces some limitations on the code.
    In particular, one cannot pass a non-static class method as a callback to a C library,
    which means that all callbacks (denoted cb_XXX) are in the ::platform_complacs namespace. 
    This would be a problem for multiple class instantiations, but since there should be a 1
    to 1 relationship between the FCS and ROS node, this should not cause any problems.
*/
// System libraries
#include <boost/utility.hpp>

// ROS libraries
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// Integration with the CRATES HAL
#include <hal/quadrotor/Quadrotor.h>
#include <hal/sensor/Altimeter.h>
#include <hal/sensor/Compass.h>
#include <hal/sensor/GNSS.h>
#include <hal/sensor/IMU.h>
#include <hal/sensor/Orientation.h>

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
#define CMD_ATT_THRUST              0x050D  // [ INT16] Attitude control, Thrust (0..4095 = 0..100%)
#define CMD_CTRL_OPTS               0x050E  // [ INT16] Control byte for attitude control (bit 0: pitch, bit 1: roll, bit 2: yaw, bit 3: thrust, bit 4: height, bit 5: GPS position)
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

// Handle to the serial port
CallbackAsyncSerial serial;

// Called when FCS is ready for probing
bool fcsready = false;

// Data polling rates
int rate_max = 0;       // Maximum data rate of any packet
int rate_aci = 0;       // Rate of aci engine
int rate_est = 0;       // Rate of state estimate
int rate_pos = 0;       // Position rate
int rate_mag = 0;       // Magnetic rate
int rate_imu = 0;       // Inertial rate
int rate_alt = 0;       // Altitude rate
int rate_inf = 0;       // Information rate

// Raw state messages
struct _raw_est
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
} raw_est;

// Raw GPS messages
struct _raw_pos
{
    int32_t     week;
    uint32_t    timeofweek;
    int32_t     pos_lat;
    int32_t     pos_lon;
    int32_t     alt;
    int32_t     vel_lat;
    int32_t     vel_lon;
    uint32_t    pos_acc;
    uint32_t    alt_acc;
    uint32_t    vel_acc;
    int32_t     status;
    int32_t     numsat;
    int32_t     heading;
} raw_pos;

// Raw GPS messages
struct _raw_mag
{
    int32_t     mag_x;
    int32_t     mag_y;
    int32_t     mag_z;
} raw_mag;

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

// Raw IMU messages
struct _raw_alt
{
    int32_t     pos_z;
    int32_t     vel_z;
} raw_alt;

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
void cb_heartbeat(const ros::TimerEvent& event)
{
    // This needs to be called to keep the ACI interface alive
    aciEngine();
}

// This is called when the ACI engine's internal list of variables is updated
// At this point we are able to setup our own custom packets
void cb_config(void)
{
    ROS_INFO("Variable list received from ACI device");

    if (rate_est > 0)
    {
        ROS_INFO("Setting up EST data packets");
        aciAddContentToVarPacket(0,VAR_EST_POS_X,&raw_est.pos_x);
        aciAddContentToVarPacket(0,VAR_EST_POS_Y,&raw_est.pos_y);
        aciAddContentToVarPacket(0,VAR_EST_POS_Z,&raw_est.pos_z);
        aciAddContentToVarPacket(0,VAR_EST_ANG_X,&raw_est.ang_x);
        aciAddContentToVarPacket(0,VAR_EST_ANG_Y,&raw_est.ang_y);
        aciAddContentToVarPacket(0,VAR_EST_ANG_Z,&raw_est.ang_z);
        aciAddContentToVarPacket(0,VAR_EST_VEL_X,&raw_est.vel_x);
        aciAddContentToVarPacket(0,VAR_EST_VEL_Y,&raw_est.vel_y);
        aciAddContentToVarPacket(0,VAR_EST_VEL_Z,&raw_est.vel_z);
        aciSetVarPacketTransmissionRate(0,1000/rate_est);
    }

    if (rate_pos > 0)
    {
        ROS_INFO("Setting up POS data packets");
        aciAddContentToVarPacket(1,VAR_GPS_WEEK,&raw_pos.week);
        aciAddContentToVarPacket(1,VAR_GPS_TIME_OF_WEEK,&raw_pos.timeofweek);
        aciAddContentToVarPacket(1,VAR_GPS_LAT,&raw_pos.pos_lat);
        aciAddContentToVarPacket(1,VAR_GPS_LON,&raw_pos.pos_lon);
        aciAddContentToVarPacket(1,VAR_GPS_HEIGHT,&raw_pos.alt);
        aciAddContentToVarPacket(1,VAR_GPS_VEL_NS,&raw_pos.vel_lat);
        aciAddContentToVarPacket(1,VAR_GPS_VEL_EW,&raw_pos.vel_lon);
        aciAddContentToVarPacket(1,VAR_GPS_POS_ACC,&raw_pos.pos_acc);
        aciAddContentToVarPacket(1,VAR_GPS_HEIGHT_ACC,&raw_pos.alt_acc);
        aciAddContentToVarPacket(1,VAR_GPS_VEL_ACC,&raw_pos.vel_acc);
        aciAddContentToVarPacket(1,VAR_GPS_STATUS,&raw_pos.status);
        aciAddContentToVarPacket(1,VAR_GPS_NUM_SATS,&raw_pos.numsat);
        aciAddContentToVarPacket(1,VAR_GPS_HEADING,&raw_pos.heading);
        aciSetVarPacketTransmissionRate(1,1000/rate_pos);
    }

    if (rate_imu > 0)
    {
        ROS_INFO("Setting up IMU data packets");
        aciAddContentToVarPacket(2,VAR_GYR_CAL_X,&raw_imu.gyr_x);
        aciAddContentToVarPacket(2,VAR_GYR_CAL_Y,&raw_imu.gyr_y);
        aciAddContentToVarPacket(2,VAR_GYR_CAL_Z,&raw_imu.gyr_z);
        aciAddContentToVarPacket(2,VAR_ACC_CAL_X,&raw_imu.acc_x);
        aciAddContentToVarPacket(2,VAR_ACC_CAL_Y,&raw_imu.acc_y);
        aciAddContentToVarPacket(2,VAR_ACC_CAL_Z,&raw_imu.acc_z);
        aciSetVarPacketTransmissionRate(2,1000/rate_imu);
    }

    if (rate_mag > 0)
    {
        ROS_INFO("Setting up MAG data packets");
        aciAddContentToVarPacket(3,VAR_MAG_CAL_X,&raw_mag.mag_x);
        aciAddContentToVarPacket(3,VAR_MAG_CAL_Y,&raw_mag.mag_y);
        aciAddContentToVarPacket(3,VAR_MAG_CAL_Z,&raw_mag.mag_z);
        aciSetVarPacketTransmissionRate(3,1000/rate_mag);
    }

    if (rate_alt > 0)
    {
        ROS_INFO("Setting up ALT data packets");
        aciAddContentToVarPacket(4,VAR_MAG_CAL_X,&raw_alt.pos_z);
        aciAddContentToVarPacket(4,VAR_MAG_CAL_Y,&raw_alt.vel_z);
        aciSetVarPacketTransmissionRate(4,1000/rate_alt);
    }

    if (rate_inf > 0)
    {
        ROS_INFO("Setting up INF data packets");
        aciSetVarPacketTransmissionRate(5,1000/rate_inf);
    }

    ROS_INFO("Updating packet configuration");
    aciVarPacketUpdateTransmissionRates();
    aciSendVariablePacketConfiguration(0);
    aciSendVariablePacketConfiguration(1);
    aciSendVariablePacketConfiguration(2);
    aciSendVariablePacketConfiguration(3);
    aciSendVariablePacketConfiguration(4);
    aciSendVariablePacketConfiguration(5);

    // We are now ready to capture
    fcsready = true;

}

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
        ros::Timer timerEstimate;
        ros::Timer timerInertial;
        ros::Timer timerPosition;
        ros::Timer timerMagnetic;
        ros::Timer timerAltitude;
        ros::Timer timerInfo;

        // Configure variables
        void cb_est(const ros::TimerEvent& event)
        {
            // Syncrhonise variables
            if (fcsready)
            {
                // Copy the data from the ACI to raw packet 
                aciSynchronizeVars();
            }
        }

        // Configure variables
        void cb_imu(const ros::TimerEvent& event)
        {
            // Syncrhonise variables
            if (fcsready)
            {
                // Copy the data from the ACI to raw packet 
                aciSynchronizeVars();
            }
        }

        // Configure variables
        void cb_pos(const ros::TimerEvent& event)
        {
            // Syncrhonise variables
            if (fcsready)
            {
                // Copy the data from the ACI to raw packet 
                aciSynchronizeVars();
            }
        }

        // Configure variables
        void cb_mag(const ros::TimerEvent& event)
        {
            // Syncrhonise variables
            if (fcsready)
            {
                // Copy the data from the ACI to raw packet 
                aciSynchronizeVars();
            }
        }

        // Configure variables
        void cb_alt(const ros::TimerEvent& event)
        {
            // Syncrhonise variables
            if (fcsready)
            {
                // Copy the data from the ACI to raw packet 
                aciSynchronizeVars();
            }
        }

        // Configure variables
        void cb_inf(const ros::TimerEvent& event)
        {
            // Syncrhonise variables
            if (fcsready)
            {
                // Copy the data from the ACI to raw packet 
                aciSynchronizeVars();
            }
        }

        // Called when the HAL wants an altimeter reading
        bool GetMeasurement(hal_sensor_altimeter::Data& msg)
        {
            // Get the altimeter
            return false;
        }

        // Called when the HAL wants a compass reading
        bool GetMeasurement(hal_sensor_compass::Data& msg)
        {
            // Get the compass
            return false;
        }
        
        // Called when the HAL wants an imu reading
        bool GetMeasurement(hal_sensor_imu::Data& msg)
        {
            // Get the imu
            return false;
        }
        
        // Called when the HAL wants a gnss reading
        bool GetMeasurement(hal_sensor_gnss::Data& msg)
        {
            // Get the GNSS
            return false;
        }
        
        // Called when the HAL wants an orientation reading
        bool GetMeasurement(hal_sensor_orientation::Data& msg)
        {
            // Get the orientation
            return false;
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
        void SetControl(const hal_quadrotor::Control &control)
        {
            // Pass control to the propulsion module
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
            if (!nh.getParam("serial_port",port))   port     = "/dev/ttyUSB0";
            if (!nh.getParam("serial_baud",baud))   baud     = 57600;
            if (!nh.getParam("rate_max",rate_max))  rate_max = 100;
            if (!nh.getParam("rate_aci",rate_aci))  rate_aci = 10;
            if (!nh.getParam("rate_est",rate_est))  rate_est = 0;
            if (!nh.getParam("rate_imu",rate_imu))  rate_imu = 0;
            if (!nh.getParam("rate_pos",rate_pos))  rate_pos = 0;
            if (!nh.getParam("rate_alt",rate_alt))  rate_alt = 0;
            if (!nh.getParam("rate_inf",rate_inf))  rate_inf = 0;
            if (!nh.getParam("rate_mag",rate_inf))  rate_inf = 0;

            // So we don't try and poll before some data is there
            fcsready = false;

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
                aciSetVarListUpdateFinishedCallback(&cb_config);

                ROS_INFO("Configuring ACI engine rate");
                aciSetEngineRate(rate_max,rate_aci);

                ROS_INFO("Querying variable list");
                aciGetDeviceVariablesList();

                ROS_INFO("Starting timers");
                if (rate_aci > 0)
                {
                    ROS_INFO("- Starting ACI polling timer");
                    timerHeartbeat = nh.createTimer(
                        ros::Duration(1.0/(double)rate_aci),        // duration
                        &cb_heartbeat,                              // callback
                        false                                       // oneshot?
                    );
                }
                if (rate_est > 0)
                {
                    ROS_INFO("- Starting EST polling timer");
                    timerEstimate = nh.createTimer(
                        ros::Duration(1.0/(double)rate_est),                    // duration
                        boost::bind(&FlightControlSystem::cb_est, this, _1),    // callback
                        false                                                   // oneshot?
                    );
                }
                if (rate_pos > 0)
                {
                    ROS_INFO("- Starting POS polling timer");
                    timerPosition = nh.createTimer(
                        ros::Duration(1.0/(double)rate_pos),                    // duration
                        boost::bind(&FlightControlSystem::cb_pos, this, _1),    // callback
                        false                                                   // oneshot?
                    );
                }
                if (rate_imu > 0)
                {
                    ROS_INFO("- Starting IMU polling timer");
                    timerInertial = nh.createTimer(
                        ros::Duration(1.0/(double)rate_imu),                    // duration
                        boost::bind(&FlightControlSystem::cb_imu, this, _1),    // callback
                        false                                                   // oneshot?
                    );
                }
                if (rate_mag > 0)
                {
                    ROS_INFO("- Starting MAG polling timer");
                    timerMagnetic = nh.createTimer(
                        ros::Duration(1.0/(double)rate_mag),                    // duration
                        boost::bind(&FlightControlSystem::cb_mag, this, _1),    // callback
                        false                                                   // oneshot?
                    );
                }
                if (rate_alt > 0)
                {
                    ROS_INFO("- Starting ALT polling timer");
                    timerAltitude = nh.createTimer(
                        ros::Duration(1.0/(double)rate_alt),                    // duration
                        boost::bind(&FlightControlSystem::cb_alt, this, _1),    // callback
                        false                                                   // oneshot?
                    );
                }
                if (rate_inf > 0)
                {
                    ROS_INFO("- Starting INF polling timer");
                    timerInfo = nh.createTimer(
                        ros::Duration(1.0/(double)rate_inf),                    // duration
                        boost::bind(&FlightControlSystem::cb_inf, this, _1),    // callback
                        false                                                   // oneshot?
                    );
                }
            }
        }
    };

    // Required in order to be used as a nodelet
    PLUGINLIB_EXPORT_CLASS(FlightControlSystem, nodelet::Nodelet);
}