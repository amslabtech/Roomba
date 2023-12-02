uint8 DRIVE=0
uint8 SAFE=1
uint8 FULL=2
uint8 POWER=3
uint8 SPOT=4
uint8 CLEAN=5
uint8 MAX=6
uint8 MOTORS=7
uint8 FORCE_SEEK_DOCK=8
uint8 WAKEUP=9
uint8 MOTORS_OFF=10
uint8 DRIVE_DIRECT=11
uint8 DOCK=12
uint8 DRIVE_PWM=13
uint8 DRIVE_FB=14
uint8 SONG=15


int32 DEFAULT_VELOCITY=200
int32 STRAIGHT_RADIUS=32768
int16 TURN_CLOCK=-1
int16 TURN_CNT_CLOCK=1
int32 mode
int32 velocity
int32 radius
geometry_msgs/Twist cntl
int16 r_pwm
int16 l_pwm

uint8[] song_melody
