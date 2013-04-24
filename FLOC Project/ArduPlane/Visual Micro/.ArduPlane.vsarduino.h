//Board = Arduino Mega 2560 or Mega ADK
#define ARDUINO 101
#define __AVR_ATmega2560__
#define F_CPU 16000000L
#define __AVR__
#define __cplusplus
#define __attribute__(x)
#define __inline__
#define __asm__(x)
#define __extension__
#define __ATTR_PURE__
#define __ATTR_CONST__
#define __inline__
#define __asm__ 
#define __volatile__
#define __builtin_va_list
#define __builtin_va_start
#define __builtin_va_end
#define __DOXYGEN__
#define prog_void
#define PGM_VOID_P int
#define NOINLINE __attribute__((noinline))

typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {}

//already defined in arduno.h
//already defined in arduno.h
static void fast_loop();
static void medium_loop();
static void slow_loop();
static void one_second_loop();
static void update_GPS(void);
static void update_current_flight_mode(void);
static void update_navigation();
static void update_alt();
static float get_speed_scaler(void);
static bool stick_mixing_enabled(void);
static void stabilize();
static void crash_checker();
static void calc_throttle();
static void calc_nav_yaw(float speed_scaler, float ch4_inf);
static void calc_nav_pitch();
static void calc_nav_roll();
static void throttle_slew_limit();
static bool suppress_throttle(void);
static void set_servos(void);
static void demo_servos(byte i);
static bool alt_control_airspeed(void);
static NOINLINE void fcom_status_LEDs(enum XBee_Addresses address_id);
static NOINLINE void fcom_send_heartbeat(enum XBee_Addresses address_id);
static NOINLINE void fcom_send_location(enum XBee_Addresses address_id);
static NOINLINE void fcom_send_flock_status(enum XBee_Addresses address_id);
static NOINLINE void fcom_send_pf_field(enum XBee_Addresses address_id);
static NOINLINE void fcom_send_vwp(enum XBee_Addresses address_id);
static NOINLINE void fcom_send_rel_state(enum XBee_Addresses address_id);
static void handle_fcom_sender(mavlink_message_t* msg);
static void handle_fcom_message(flock_member* p_flockmember, bool* p_rollcall, mavlink_message_t* msg);
static void process_flockmember_heartbeat(uint8_t sysid, flock_member* p_flockmember, bool* p_rollcall, mavlink_heartbeat_t* packet);
static void process_flockmember_location(flock_member* p_flockmember, mavlink_global_position_int_t* packet);
static void process_flockmember_status(flock_member* p_flockmember, mavlink_ff_flock_status_t* packet);
static void fcom_send_message(enum XBee_Addresses address_id, enum ff_message id);
static bool fcom_try_send_message(enum XBee_Addresses address_id, enum ff_message id);
static NOINLINE void send_heartbeat(mavlink_channel_t chan);
static NOINLINE void send_attitude(mavlink_channel_t chan);
static NOINLINE void send_fence_status(mavlink_channel_t chan);
static NOINLINE void send_extended_status1(mavlink_channel_t chan, uint16_t packet_drops);
static void NOINLINE send_meminfo(mavlink_channel_t chan);
static void NOINLINE send_location(mavlink_channel_t chan);
static void NOINLINE send_nav_controller_output(mavlink_channel_t chan);
static void NOINLINE send_gps_raw(mavlink_channel_t chan);
static void NOINLINE send_servo_out(mavlink_channel_t chan);
static void NOINLINE send_radio_in(mavlink_channel_t chan);
static void NOINLINE send_radio_out(mavlink_channel_t chan);
static void NOINLINE send_vfr_hud(mavlink_channel_t chan);
static void NOINLINE send_raw_imu1(mavlink_channel_t chan);
static void NOINLINE send_raw_imu2(mavlink_channel_t chan);
static void NOINLINE send_raw_imu3(mavlink_channel_t chan);
static void NOINLINE send_ahrs(mavlink_channel_t chan);
static void NOINLINE send_simstate(mavlink_channel_t chan);
static void NOINLINE send_hwstatus(mavlink_channel_t chan);
static void NOINLINE send_wind(mavlink_channel_t chan);
static void NOINLINE send_current_waypoint(mavlink_channel_t chan);
static void NOINLINE send_statustext(mavlink_channel_t chan);
static bool telemetry_delayed(mavlink_channel_t chan);
static bool mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops);
static void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops);
void mavlink_send_text(mavlink_channel_t chan, gcs_severity severity, const char *str);
static void mavlink_delay(unsigned long t);
static void gcs_send_message(enum ap_message id);
static void gcs_data_stream_send(void);
static void gcs_update(void);
static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str);
static bool print_log_menu(void);
static void do_erase_logs(void);
static void Log_Write_Attitude(int16_t log_roll, int16_t log_pitch, uint16_t log_yaw);
static void Log_Write_Performance();
static void Log_Write_Cmd(byte num, struct Location *wp);
static void Log_Write_Startup(byte type);
static void Log_Write_Control_Tuning();
static void Log_Write_Nav_Tuning();
static void Log_Write_Mode(byte mode);
static void Log_Write_GPS(      int32_t log_Time, int32_t log_Lattitude, int32_t log_Longitude, int32_t log_gps_alt, int32_t log_mix_alt,                                 int32_t log_Ground_Speed, int32_t log_Ground_Course, byte log_Fix, byte log_NumSats);
static void Log_Write_Raw();
static void Log_Write_Current();
static void  Log_Write_Flock_Status(int32_t log_time, byte leader, byte member_iv, int32_t member_ids, int32_t dist2goal);
static void Log_Write_PF_Field(int32_t log_time, byte coordinate_frame, int16_t phix_att, int16_t phiy_att, int16_t phiz_att, int16_t phix_rep, int16_t phiy_rep, int16_t phiz_rep, 							int16_t phix_norm, int16_t phiy_norm, int16_t phiz_norm, byte regime_mask);
static void Log_Write_VWP(int32_t log_time,byte coordinate_frame, int32_t latitude, int32_t longitude, int32_t altitude, int16_t airspeed);
static void Log_Write_Relative(int32_t log_time, byte coordinate_frame, int16_t relx, int16_t rely, int16_t relz, int16_t rel2L, int16_t relvx, int16_t relvy, int16_t relvz);
static void Log_Write_Error_Assist(int32_t log_time, int16_t gps_hdop);
static void Log_Read_Flock_Status();
static void Log_Read_PF_Field();
static void Log_Read_VWP();
static void Log_Read_Relative();
static void Log_Read_Error_Assist();
static void Log_Read_Current();
static void Log_Read_Control_Tuning();
static void Log_Read_Nav_Tuning();
static void Log_Read_Performance();
static void Log_Read_Cmd();
static void Log_Read_Startup();
static void Log_Read_Attitude();
static void Log_Read_Mode();
static void Log_Read_GPS();
static void Log_Read_Raw();
static void Log_Read(int16_t start_page, int16_t end_page);
static int16_t Log_Read_Process(int16_t start_page, int16_t end_page);
static void Log_Write_Mode(byte mode);
static void Log_Write_Startup(byte type);
static void Log_Write_Cmd(byte num, struct Location *wp);
static void Log_Write_Current();
static void Log_Write_Nav_Tuning();
static void Log_Write_GPS(      int32_t log_Time, int32_t log_Lattitude, int32_t log_Longitude, int32_t log_gps_alt, int32_t log_mix_alt,                                 int32_t log_Ground_Speed, int32_t log_Ground_Course, byte log_Fix, byte log_NumSats);
static void Log_Write_Performance();
static void Log_Write_Attitude(int16_t log_roll, int16_t log_pitch, uint16_t log_yaw);
static void Log_Write_Control_Tuning();
static void Log_Write_Raw();
static void load_parameters(void);
void add_altitude_data(unsigned long xl, long y);
static void init_commands();
static void update_auto();
static void reload_commands_airstart();
static struct Location get_cmd_with_index_raw(int16_t i);
static struct Location get_cmd_with_index(int16_t i);
static void set_cmd_with_index(struct Location temp, int16_t i);
static void decrement_cmd_index();
static int32_t read_alt_to_hold();
static void set_next_WP(struct Location *wp);
static void set_guided_WP(void);
void init_home();
static void handle_process_nav_cmd();
static void handle_process_condition_command();
static void handle_process_do_command();
static void handle_no_commands();
static bool verify_nav_command();
static bool verify_condition_command();
static void do_RTL(void);
static void do_takeoff();
static void do_nav_wp();
static void do_land();
static void do_loiter_unlimited();
static void do_loiter_turns();
static void do_loiter_time();
static bool verify_takeoff();
static bool verify_land();
static bool verify_nav_wp();
static bool verify_loiter_unlim();
static bool verify_loiter_time();
static bool verify_loiter_turns();
static bool verify_RTL();
static void do_wait_delay();
static void do_change_alt();
static void do_within_distance();
static bool verify_wait_delay();
static bool verify_change_alt();
static bool verify_within_distance();
static void do_loiter_at_location();
static void do_jump();
static void do_change_speed();
static void do_set_home();
static void do_set_servo();
static void do_set_relay();
static void do_repeat_servo(uint8_t channel, uint16_t servo_value,                             int16_t repeat, uint8_t delay_time);
static void do_repeat_relay();
void change_command(uint8_t cmd_index);
static void update_commands(void);
static void verify_commands(void);
static void process_next_command();
static void process_non_nav_command();
static void read_control_switch();
static byte readSwitch(void);
static void reset_control_switch();
static void failsafe_short_on_event(int16_t fstype);
static void failsafe_long_on_event(int16_t fstype);
static void failsafe_short_off_event();
void low_battery_event(void);
static void update_events(void);
void failsafe_check(uint32_t tnow);
int freeRam ();
static void update_ac_flockmember();
static void update_formation_flight_commands();
static void update_flock_leadership();
static void update_flock_side();
static void set_goal_WP();
static void update_goal_wp_distance();
static void update_goal_loiter();
static void update_distance_to_goal();
static bool member_in_view(int32_t* p_lat, int32_t* p_lon, int32_t* p_alt);
static void check_formation_health();
static Vector2l get_fence_point_with_index(unsigned i);
static void set_fence_point_with_index(Vector2l &point, unsigned i);
static void geofence_load(void);
static bool geofence_enabled(void);
static bool geofence_check_minalt(void);
static bool geofence_check_maxalt(void);
static void geofence_check(bool altitude_check_only);
static bool geofence_stickmixing(void);
static void geofence_send_status(mavlink_channel_t chan);
bool geofence_breached(void);
static void geofence_check(bool altitude_check_only);
static bool geofence_stickmixing(void);
static bool geofence_enabled(void);
static void navigate();
void calc_distance_error();
static void calc_airspeed_errors();
static void calc_gndspeed_undershoot();
static void calc_bearing_error();
static void calc_altitude_error();
static int32_t wrap_360_cd(int32_t error);
static int32_t wrap_180_cd(int32_t error);
static void update_loiter();
static void update_crosstrack(void);
static void reset_crosstrack();
static void init_rc_in();
static void init_rc_out();
static void read_radio();
static void control_failsafe(uint16_t pwm);
static void trim_control_surfaces();
static void trim_radio();
static void init_barometer(void);
static int32_t read_barometer(void);
static void read_airspeed(void);
static void zero_airspeed(void);
static void read_battery(void);
void read_receiver_rssi(void);
static int32_t adjusted_altitude_cm(void);
static void report_batt_monitor();
static void report_radio();
static void report_gains();
static void report_xtrack();
static void report_throttle();
static void report_imu();
static void report_compass();
static void report_flight_modes();
static void print_PID(PID * pid);
static void print_radio_values();
static void print_switch(byte p, byte m);
static void print_done();
static void print_blanks(int16_t num);
static void print_divider(void);
static int8_t radio_input_switch(void);
static void zero_eeprom(void);
static void print_enabled(bool b);
static void print_accel_offsets(void);
static void print_gyro_offsets(void);
static void run_cli(void);
static void init_ardupilot();
static void startup_ground(void);
static void set_mode(byte mode);
static void check_long_failsafe();
static void check_short_failsafe();
static void startup_IMU_ground(bool force_accel_level);
static void update_GPS_light(void);
static void resetPerfData(void);
static uint32_t map_baudrate(int8_t rate, uint32_t default_baud);
static void check_usb_mux(void);
void flash_leds(bool on);
uint16_t board_voltage(void);
static void reboot_apm(void);
static void print_flight_mode(uint8_t mode);
static void print_comma(void);
static void print_hit_enter();
static void test_wp_print(struct Location *cmd, byte wp_index);

#include "C:\Users\God\ARDUINO\arduino-1.0.1\hardware\arduino\variants\mega\pins_arduino.h" 
#include "C:\Users\God\ARDUINO\arduino-1.0.1\hardware\arduino\cores\arduino\arduino.h"
#include "C:\Users\God\Documents\Arduino\ArduPlane\ArduPlane.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\APM_Config.h"
#include "C:\Users\God\Documents\Arduino\ArduPlane\APM_Config_Formation.h"
#include "C:\Users\God\Documents\Arduino\ArduPlane\AP_XBee.cpp"
#include "C:\Users\God\Documents\Arduino\ArduPlane\AP_XBee.h"
#include "C:\Users\God\Documents\Arduino\ArduPlane\Attitude.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\FCOM_Mavlink.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\GCS.h"
#include "C:\Users\God\Documents\Arduino\ArduPlane\GCS_Mavlink.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\Log.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\Parameters.h"
#include "C:\Users\God\Documents\Arduino\ArduPlane\Parameters.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\checksum.h"
#include "C:\Users\God\Documents\Arduino\ArduPlane\climb_rate.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\commands.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\commands_logic.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\commands_process.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\config.h"
#include "C:\Users\God\Documents\Arduino\ArduPlane\control_modes.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\defines.h"
#include "C:\Users\God\Documents\Arduino\ArduPlane\events.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\failsafe.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\fcom_types.h"
#include "C:\Users\God\Documents\Arduino\ArduPlane\flock_member.cpp"
#include "C:\Users\God\Documents\Arduino\ArduPlane\flock_member.h"
#include "C:\Users\God\Documents\Arduino\ArduPlane\formation_common.h"
#include "C:\Users\God\Documents\Arduino\ArduPlane\formation_flight.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\geofence.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\navigation.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\pf_field.cpp"
#include "C:\Users\God\Documents\Arduino\ArduPlane\pf_field.h"
#include "C:\Users\God\Documents\Arduino\ArduPlane\planner.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\radio.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\sensors.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\setup.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\system.ino"
#include "C:\Users\God\Documents\Arduino\ArduPlane\test.ino"
