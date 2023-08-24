#include "gps.h"
#include "math.h"
#include "string.h"
#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "../quadrotor/scheduler.h"
#include "gpsconfig.h"
#include "maths.h"

#define FALSE 0
#define TRUE  1
#define LON 1
#define LAT 0
enum {
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_POSLLH = 0x2,
    MSG_STATUS = 0x3,
    MSG_SOL = 0x6,
    MSG_VELNED = 0x12,
    MSG_SVINFO = 0x30,
    MSG_CFG_PRT = 0x00,
    MSG_CFG_RATE = 0x08,
    MSG_CFG_SET_RATE = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24
} ubs_protocol_bytes;

enum {
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
	FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
} ubs_nav_fix_type;

enum {
    NAV_STATUS_FIX_VALID = 1
} ubx_nav_status_bits;
// UBX support
typedef struct {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
} ubx_header;

typedef struct {
    uint32_t time;              // GPS msToW
    int32_t longitude;
    int32_t latitude;
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
} ubx_nav_posllh;

typedef struct {
    uint32_t time;              // GPS msToW
    uint8_t fix_type;
    uint8_t fix_status;
    uint8_t differential_status;
    uint8_t res;
    uint32_t time_to_first_fix;
    uint32_t uptime;            // milliseconds
} ubx_nav_status;

typedef struct {
    uint32_t time;
    int32_t time_nsec;
    int16_t week;
    uint8_t fix_type;
    uint8_t fix_status;
    int32_t ecef_x;
    int32_t ecef_y;
    int32_t ecef_z;
    uint32_t position_accuracy_3d;
    int32_t ecef_x_velocity;
    int32_t ecef_y_velocity;
    int32_t ecef_z_velocity;
    uint32_t speed_accuracy;
    uint16_t position_DOP;
    uint8_t res;
    uint8_t satellites;
    uint32_t res2;
} ubx_nav_solution;

typedef struct {
    uint32_t time;              // GPS msToW
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_3d;
    uint32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
} ubx_nav_velned;

typedef struct {
    uint8_t chn;                // Channel number, 255 for SVx not assigned to channel
    uint8_t svid;               // Satellite ID
    uint8_t flags;              // Bitmask
    uint8_t quality;            // Bitfield
    uint8_t cno;                // Carrier to Noise Ratio (Signal Strength)
    uint8_t elev;               // Elevation in integer degrees
    int16_t azim;               // Azimuth in integer degrees
    int32_t prRes;              // Pseudo range residual in centimetres
} ubx_nav_svinfo_channel;

typedef struct {
    uint32_t time;              // GPS Millisecond time of week
    uint8_t numCh;              // Number of channels
    uint8_t globalFlags;        // Bitmask, Chip hardware generation 0:Antaris, 1:u-blox 5, 2:u-blox 6
    uint16_t reserved2;         // Reserved
    ubx_nav_svinfo_channel channel[16];         // 16 satellites * 12 byte
} ubx_nav_svinfo;

// State machine state
static uint8_t _step;
static uint8_t _msg_id;
static uint16_t _payload_length;
static uint16_t _payload_counter;

#define UBLOX_BUFFER_SIZE 200

static union {
    ubx_nav_posllh posllh;
    ubx_nav_status status;
    ubx_nav_solution solution;
    ubx_nav_velned velned;
    //ubx_nav_svinfo svinfo;
    uint8_t bytes[UBLOX_BUFFER_SIZE];
} _buffer;

static void _update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    while (len--) {
        *ck_a += *data;
        *ck_b += *ck_a;
        data++;
    }
}
static uint8_t _new_position;
static gpsData_t gps_t;
static uint8_t UBLOX_parse_gps(){
    static uint8_t _new_speed;
    static uint8_t next_fix;
    static uint32_t lastTime;
    switch (_msg_id) {
        case MSG_POSLLH:
            gps_t.coord[LON] = _buffer.posllh.longitude;
            gps_t.coord[LAT] = _buffer.posllh.latitude;
            gps_t.altitude = _buffer.posllh.altitude_msl / 10 / 100;  //alt in m
            //gps_t.HorizontalAcc = _buffer.posllh.horizontal_accuracy;
            //gps_t.VerticalAcc = _buffer.posllh.vertical_accuracy;
            //gps_t.fix = next_fix;
 
            gps_t.coord_update_time = millis() - lastTime;
            lastTime = millis();

            // if (!sensors(SENSOR_BARO) && f.FIXED_WING)
            //    EstAlt = (altitude - home[ALT]) * 100;    // Use values Based on GPS
            _new_position = TRUE;
            break;
        case MSG_STATUS:
            //next_fix = (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.status.fix_type == FIX_3D);
            //if (!next_fix)
            //    gps_t.fix = FALSE;
            gps_t.fix = _buffer.status.fix_type;
            break;
        case MSG_SOL:
            //next_fix = (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
            //if (!next_fix)
            //    gps_t.fix = FALSE;
            //gps_t.fix = _buffer.solution.fix_type;
            gps_t.numSat = _buffer.solution.satellites;
            break;
        case MSG_VELNED:
            gps_t.speed = _buffer.velned.speed_2d;    // cm/s
            gps_t.ground_course = (uint16_t) (_buffer.velned.heading_2d / 10000);     // Heading 2D deg * 100000 rescaled to deg * 10
            _new_speed = TRUE;
            //if (!sensors(SENSOR_MAG) && speed > 100) {
            //    ground_course = wrap_18000(ground_course * 10) / 10;
            //    heading = ground_course / 10;    // Use values Based on GPS if we are moving.
            //}
            break;
        case MSG_SVINFO:
           /*
            numCh = _buffer.svinfo.numCh;
            if (numCh > 32)
                numCh = 32;
            for (i = 0; i < numCh; i++) {
                svinfo_chn[i] = _buffer.svinfo.channel[i].chn;
                svinfo_svid[i] = _buffer.svinfo.channel[i].svid;
                svinfo_quality[i] = _buffer.svinfo.channel[i].quality;
                svinfo_cno[i] = _buffer.svinfo.channel[i].cno;
            }
            // Update GPS SVIFO update rate table.
            svinfo_rate[0] = svinfo_rate[1];
            svinfo_rate[1] = millis();
            */
            break;
        default:
            return FALSE;
    }
    if (_new_position && _new_speed) {
        _new_speed = _new_position = FALSE;
        return TRUE;
    }
    return FALSE;
}


static uint8_t gpsNewFrameUBLOX(uint8_t data)
{
    uint8_t parsed = FALSE;
    static uint8_t _ck_a;
    static uint8_t _ck_b;

    switch (_step) {
        case 0: // Sync char 1 (0xB5)
            if (PREAMBLE1 == data)
                _step++;
            break;
        case 1: // Sync char 2 (0x62)
            if (PREAMBLE2 == data) {
                _step++;
                break;
            }
        case 2: // Class
            _step++;
            _ck_b = _ck_a = data;  
            break;
        case 3: // ID
            _step++;
            _ck_b += (_ck_a += data);       
            _msg_id = data;
            break;
        case 4: 
            _step++;
            _ck_b += (_ck_a += data);       
            _payload_length = data; 
            break;
        case 5: 
            _step++;
            _ck_b += (_ck_a += data);      
            _payload_length += (uint16_t)(data << 8);
            if (_payload_length > UBLOX_BUFFER_SIZE) {
                _payload_length = 0;
                _step = 0;
            }
            _payload_counter = 0;   
            break;
        case 6:
            _ck_b += (_ck_a += data);     
            if (_payload_counter < UBLOX_BUFFER_SIZE) {
                _buffer.bytes[_payload_counter] = data;
            }
            if (++_payload_counter == _payload_length)
                _step++;
            break;
        case 7:
            _step++;
            if (_ck_a != data)
                _step = 0;         
            break;
        case 8:
            _step = 0;
            if (_ck_b != data)
                break;            
            if (UBLOX_parse_gps())
                parsed = TRUE;
    } 
    return parsed;
}
//---------callback function---------
uint8_t char_;
void gpsCallback()
{
   gpsNewFrameUBLOX(char_);
   //HAL_UART_Receive_IT(uarttt, &char_,1);
}


#define  EARTH_RADIUS 63781370 // m


//----------------------------------------
static int32_t home_latitude;
static int32_t home_longitude;
//
static UART_HandleTypeDef *uarttt = NULL;
static uint16_t Dt_gps;
/* Cal velocity */
static float v_lat;  //   cm/s
static float v_lon;  //   cm/s
static void GPS_update_home_position(void);
static void GPS_calc_velocity(void)
{
    static int32_t last_Longitude,last_Latitude;
    int32_t temp_lat,temp_lon;
    temp_lat = gps_t.coord[LAT] - last_Latitude;
    temp_lon = gps_t.coord[LON] - last_Longitude;
    last_Longitude = gps_t.coord[LON];
    last_Latitude  = gps_t.coord[LAT];
    if(temp_lat == gps_t.coord[LAT] && temp_lon == gps_t.coord[LON])
        return;
    float v_scale = RAD*(float)EARTH_RADIUS/1000000;
    v_lat = v_lat*v_scale;
    v_lon = v_lon*v_scale;
    v_lat /= (Dt_gps/1000);
    v_lon /= (Dt_gps/1000);
}

/* Distance and bearing to next waypoint*/
static void GPS_distance_cm_bearing(int32_t *lat1, int32_t *lon1, int32_t *lat2, int32_t *lon2, int32_t *dist, int32_t *bearing)
{

}
void GPS_thread()
{
    static uint32_t p_time; 
    Dt_gps = millis() -  p_time;
    p_time = millis();
    if(Dt_gps > 2000)
        return;   
    // calculate velocity
    GPS_calc_velocity();
 
}



void gpsInit(UART_HandleTypeDef *uartt,uint32_t baudrate)
{
	uarttt = uartt;
    _payload_length = 0;
    _payload_counter = 0;
    _msg_id =0;
    _step = 0;

    HAL_UART_Transmit(uarttt,ubloxInit,sizeof(ubloxInit),10000);
    HAL_Delay(10);
    HAL_UART_Transmit(uarttt,uart57600,sizeof(uart57600),1000);
    HAL_Delay(10);
    uarttt->Init.BaudRate = baudrate;
	HAL_UART_Init(uarttt); //
	HAL_UART_Receive_IT(uarttt, &char_,1);
}

