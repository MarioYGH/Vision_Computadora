#include "stdbool.h"
#include "esp_err.h"

#define R                               1980 /* R = 11 X 4 X 45 = 1980 */
#define PPR                             1943 /* pulses per revolution (experimentally calculated) */
#define SAMPLE_TIME                     100
#define PI                              3.14159
#define D_WHEEL                         0.065 /* diameter of the wheel 65cm -> 0.065m */
#define DIST_WHEELS                     0.187 /* distance between wheels */
#define W_MAX                           12.1458

#define PWM_MOTOR_DEAD_ZONE             70     /* Motor dead zone filtering */
#define PWM_MOTOR_FREQ_HZ               25000   /* 25KHz PWM */

#define PWM_GPIO_M1A                    5
#define PWM_GPIO_M1B                    18
#define ENCODER_GPIO_H1B                4 // 15
#define ENCODER_GPIO_H1A                16 // 2

#define PWM_GPIO_M2A                    19
#define PWM_GPIO_M2B                    21
#define ENCODER_GPIO_H2B                23
#define ENCODER_GPIO_H2A                22

#define PWM_MOTOR_TIMER_RESOLUTION_HZ   10000000 /* PWM motor clock frequency, 10MHz, 1 tick = 0.1us */
#define PWM_MOTOR_DUTY_TICK_MAX         (PWM_MOTOR_TIMER_RESOLUTION_HZ / PWM_MOTOR_FREQ_HZ) /* PWM Theoretical maximum (400) */
#define PWM_MOTOR_MAX_VALUE             (PWM_MOTOR_DUTY_TICK_MAX - PWM_MOTOR_DEAD_ZONE) /* Maximum motor PWM input value */

#define ENCODER_PCNT_HIGH_LIMIT   1000
#define ENCODER_PCNT_LOW_LIMIT    -1000

typedef enum _motor_id {
    MOTOR_ID_ALL = 0,
    MOTOR_ID_M1 = 1,
    MOTOR_ID_M2 = 2,
} motor_id_t;

typedef enum _stop_mode {
    STOP_COAST = 0,
    STOP_BRAKE = 1
} stop_mode_t;

typedef enum _encoder_id {
    ENCODER_ID_M1 = 1,
    ENCODER_ID_M2 = 2,
} encoder_id;

typedef struct {
    float Kp;            /* PID Kp parameter */
    float ti;            /* PID Ki parameter */
    float td;            /* PID Kd parameter */
    
    float previous_err; /* e(k-1) */
    float integral;  /* Sum of error */
    float sample_time;

    float kp;
    float ki;
    float kd;
} PIDController;

/* Initialize motor 1, bind GPIO and configure timer */
esp_err_t init_motor_m1(void);

/* Initialize motor 2, bind GPIO and configure timer */
esp_err_t init_motor_m2(void);

/* Motor dead zone filtering */
int motor_ignore_dead_zone(int);

/* Limits the maximum and minimum input values. */
int motor_limit_speed(int);

/* Control motor 1 speed, speed input range: ±PWM_MOTOR_MAX_VALUE */
esp_err_t set_speed_m1(int);

/* Control motor 2 speed, speed input range: ±PWM_MOTOR_MAX_VALUE */
esp_err_t set_speed_m2(int);

/* Motor rotation is controlled by motor ID number. speed input range: ±PWM_MOTOR_MAX_VALUE */
esp_err_t set_speed(motor_id_t, int);

/* Control motor rotation. speed input range: ±PWM_MOTOR_MAX_VALUE */
esp_err_t set_speed_all(int, int);

/* Motor stop */
esp_err_t motor_stop(motor_id_t, bool);

/* Initialize motor 1 encoder, bind GPIO and configure PCNT counter */
esp_err_t init_encoder_m1(void);

/* Initialize motor 2 encoder, bind GPIO and configure PCNT counter */
esp_err_t init_encoder_m2(void);

/* Get the cumulative number of encoder pulses for motor 1 */
int get_count_m1(void);

/* Get the cumulative number of encoder pulses for motor 2 */
int get_count_m2(void);

/* Get the cumulative number of encoder pulses for a motor */
int get_count(uint8_t);

/* Get the revolutions per minute for motor 1 */
float get_rpm_m1(void);

/* Get the revolutions per minute for motor 2 */
float get_rpm_m2(void);

/* Get the revolutions per minute for a motor */
float get_rpm(uint8_t);

/* Get the angular velocity in rad/s for motor 1 */
float get_w_m1(void);

/* Get the angular velocity in rad/s for motor 2 */
float get_w_m2(void);

/* Get the angular velocity in rad/s for a motor */
float get_w(uint8_t);

/* Get the linear velocity in m/s for motor 1 */
float get_u_m1(void);

/* Get the linear velocity in m/s for motor 2 */
float get_u_m2(void);

/* Get the linear velocity in m/s for a motor */
float get_u(uint8_t);

/* Convert the percentage to angular velocity */
float percent2w(int);

/* Convert the angular velocity to percentage */
float w2percent(float);

/* Compute the angular velocity for motor 1 from linear and angular velocities */
float get_w_ref_m1(float u, float w);

/* Compute the angular velocity for motor 2 from linear and angular velocities */
float get_w_ref_m2(float u, float w);

/* Compute the angular velocity for a motor from linear and angular velocities */
float get_w_ref(uint8_t, float, float);

/* Compute the linear velocity of the robot */
float get_u_robot(float, float);

/* Compute the angular velocity of the robot */
float get_w_robot(float, float);

/* Lambda sintonization technique to find the gains of the PID control */
esp_err_t lambda_tunning(float, float, float, float, PIDController *);

esp_err_t pid_init(float, float, float, float, PIDController *);

/* 
 Calculate the pid control value by location formula
 u(k) = e(k)*Kp + (e(k)-e(k-1))*Kd + integral*Ki
*/
float pid_compute(PIDController *, float, float);