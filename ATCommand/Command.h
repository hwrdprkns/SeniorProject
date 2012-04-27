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
    
    int start_wifi_connection();
    
    void sendComwdg(int msec);
    void sendFtrim();
    void sendConfig(String option, String value);
    void sendRef(flying_status fs);
    void send_control_commands();
	  
    // clear emergency flag 
    void drone_emergency_toggle();
	
    String makeAnim(anim_mayday_t anim, int time);
    void LEDAnim(int animseq, int duration);
    
    // only used under serial connection, abandoned
    int start_s2ip();
    void quit_s2ip();
    
    int init_drone();
    
    int drone_hover(int msec);
	int drone_takeoff();
	int drone_landing();
    
    void readARsrl();
    
    int s2ip_running;
    int drone_is_init;
    
    // Movement functions
    int moveForward(float distanceInMeters);
	int moveBackward(float distanceInMeters);
    int moveRotate(float yawInDegrees);
    
    // can only call after wifi's connection established and CID is given as 0
    void sendwifi(String s);
    
    String makePcmd(int enable, float roll, float pitch, float gaz, float yaw);
    
  private:
    String at;
    String command;
    
    long fl2int(float value);
};

struct ring_buffer
{
  unsigned char buffer[SERIAL_BUFFER_SIZE];
  volatile int head;
  volatile int tail;
};

union resultint_
{
  long i;
  float f;
};

inline void store_char(unsigned char c, ring_buffer *buffer);
void SrlRead();
void read_rx_buf();

#endif
