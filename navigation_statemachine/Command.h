#ifndef Command_h
#define Command_h

#include "Arduino.h"
#include "Streaming.h"

#define ARsrl Serial2
#define PCsrl Serial
#define WIFIsrl ARsrl

#define BAUD 115200
// adjust this base on how often you read your ring buffer
#define SERIAL_BUFFER_SIZE 1
// adjust this base on how often you receive message
#define SERIAL_INTERVAL_USEC 30000
#define COMWDG_INTERVAL_USEC 600000

typedef enum {
	TAKEOFF,
	LANDING,
	EMERGENCY_TOGGLE
} flying_status;

typedef enum {
	ARDRONE_ANIM_PHI_M30_DEG= 0,
	ARDRONE_ANIM_PHI_30_DEG,
	ARDRONE_ANIM_THETA_M30_DEG,
	ARDRONE_ANIM_THETA_30_DEG,
	ARDRONE_ANIM_THETA_20DEG_YAW_200DEG,
	ARDRONE_ANIM_THETA_20DEG_YAW_M200DEG,
	ARDRONE_ANIM_TURNAROUND,
	ARDRONE_ANIM_TURNAROUND_GODOWN,
	ARDRONE_ANIM_YAW_SHAKE,
	ARDRONE_ANIM_YAW_DANCE,
	ARDRONE_ANIM_PHI_DANCE,
	ARDRONE_ANIM_THETA_DANCE,
	ARDRONE_ANIM_VZ_DANCE,
	ARDRONE_ANIM_WAVE,
	ARDRONE_ANIM_PHI_THETA_MIXED,
	ARDRONE_ANIM_DOUBLE_PHI_THETA_MIXED,
	ARDRONE_NB_ANIM_MAYDAY
} anim_mayday_t;

class Command {
  public:
    Command();
    
    /* connect to the drone using wifi, set up connection protocols
     * return: 0
     */
    int start_wifi_connection();
    
    /* send reset communication watchdog command repeatedly for a few milliseconds
     * precondition: msec > 0
     */
    void sendComwdg(int msec);
    
    // send flat trim and tell the drone it's lying flat
    void sendFtrim();
    
    /* send configuration options
     * preconditions: option and value between double quotes
     */
    void sendConfig(String option, String value);
    
    /* send basic behavior commands (takeoff, landing etc.)
     * precondtions: fs is TAKEOFF, LANDING, or EMERGENCY_TOGGLE in caps
     */
    void sendRef(flying_status fs);
    
    // send drone control mode commands
    void send_control_commands();

    // clear emergency flag 
    void drone_emergency_toggle();
    
    /* send LED animation command
     * preconditions: animseq is an integer picked from a predefined list of animations, durtaion is in seconds
     */
    void LEDAnim(int animseq, int duration);
    
    // send basic preset drone animation commands, not used
    String makeAnim(anim_mayday_t anim, int time);
    
    // only used under serial connection, abandoned
    int start_s2ip();
    void quit_s2ip();
    
    /* initialize the drone after the wifi connection is established, setup flight configuration
     * preconditions: drone place on a flat surface for ftrim
     * return: 1
     */
    int init_drone();
    
    /* make the drone hover for a few milliseconds
     * precondition: msec > 0
     * return: 1
     */
    void drone_hover(int msec);
    
    // send takeoff command
    void drone_takeoff();
    
    // send landing command
    void drone_landing();
    
    // Read serial coming from Drone
    void readARsrl();
    
    int s2ip_running;
    int drone_is_init;
    
    /* Movement functions
     * preconditions: distanceInMeters > 0, yawInDegrees can be either positive or negative
     * comments: positive angle is clockwise rotation (top view)
     * return: 1
     */
    int moveForward(float distanceInMeters);
	//speed range from 1 to 100
	int moveForward_time(unsigned int msec, int speed);
    int moveBackward(float distanceInMeters);
    int moveUp(float distanceInMeters);
    int moveDown(float distanceInMeters);
    int moveLeft(float distanceInMeters);
    int moveRight(float distanceInMeters);
    int moveRotate(int degree);
	int staticRotate(int yawInDegrees);
    
    // can only call after wifi's connection established and CID is given as 0
    void sendwifi(String s);
    
  private:
    String at;
    String command;
    
    /* send progressive commands that make the drone move (translate/rotate)
     * preconditions: roll, pitch, gaz, and yaw values in range [-1..1]
     * return: string with the AT*PCMD command
     */
    String makePcmd(int enable, float roll, float pitch, float gaz, float yaw);
    
    // convert float values into 32-bit integer values
    long fl2int(float value);
};

struct ring_buffer
{
  unsigned char buffer[SERIAL_BUFFER_SIZE];
  volatile int head;
  volatile int tail;
};

// utilized by fl2int()
union resultint_
{
  long i;
  float f;
};

inline void store_char(unsigned char c, ring_buffer *buffer);
void SrlRead();
void read_rx_buf();

#endif
