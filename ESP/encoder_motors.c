#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "bdc_motor.h"
#include "driver/pulse_cnt.h"
#include "encoder_motors.h"

bdc_motor_handle_t motor_m1 = NULL;
bdc_motor_handle_t motor_m2 = NULL;
pcnt_unit_handle_t encoder_unit_m1;
pcnt_unit_handle_t encoder_unit_m2;
extern struct pid global_pid;

static bool stop_brake = false;

/* Initialize motor 1, bind GPIO and configure timer */
esp_err_t init_motor_m1(void) {
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = PWM_MOTOR_FREQ_HZ,
        .pwma_gpio_num = PWM_GPIO_M1A,
        .pwmb_gpio_num = PWM_GPIO_M1B,
    };

    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = PWM_MOTOR_TIMER_RESOLUTION_HZ,
    };

    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    motor_m1 = motor;

    return ESP_OK;
}

/* Initialize motor 2, bind GPIO and configure timer */
esp_err_t init_motor_m2(void) {
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = PWM_MOTOR_FREQ_HZ,
        .pwma_gpio_num = PWM_GPIO_M2A,
        .pwmb_gpio_num = PWM_GPIO_M2B,
    };

    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = PWM_MOTOR_TIMER_RESOLUTION_HZ,
    };

    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    motor_m2 = motor;

    return ESP_OK;
}

/* Motor dead zone filtering */
int motor_ignore_dead_zone(int speed) {
    if (speed > 0)
        return speed + PWM_MOTOR_DEAD_ZONE;

    if (speed < 0) 
        return speed - PWM_MOTOR_DEAD_ZONE;

    return 0;
}

/* Limits the maximum and minimum input values. */
int motor_limit_speed(int speed) {
    if (speed > PWM_MOTOR_DUTY_TICK_MAX)
        return PWM_MOTOR_DUTY_TICK_MAX;

    if (speed < -PWM_MOTOR_DUTY_TICK_MAX)
        return -PWM_MOTOR_DUTY_TICK_MAX;

    return speed;
}

/* Control motor 1 speed, speed input range: ±PWM_MOTOR_MAX_VALUE */
esp_err_t set_speed_m1(int speed) {
    //speed = motor_ignore_dead_zone(speed);
    speed = motor_limit_speed(speed);

    // forward
    if (speed > 0) {
        ESP_ERROR_CHECK(bdc_motor_forward(motor_m1));
        ESP_ERROR_CHECK(bdc_motor_set_speed(motor_m1, speed));
    }
    // back
    else if (speed < 0) {
        ESP_ERROR_CHECK(bdc_motor_reverse(motor_m1));
        ESP_ERROR_CHECK(bdc_motor_set_speed(motor_m1, -speed));
    }
    // stop
    else {
        if (stop_brake)
            ESP_ERROR_CHECK(bdc_motor_brake(motor_m1));
        else 
            ESP_ERROR_CHECK(bdc_motor_coast(motor_m1));
    }

    return ESP_OK;
}

/* Control motor 2 speed, speed input range: ±PWM_MOTOR_MAX_VALUE */
esp_err_t set_speed_m2(int speed) {
    //speed = motor_ignore_dead_zone(speed);
    speed = motor_limit_speed(speed);

    // forward
    if (speed > 0) {
        ESP_ERROR_CHECK(bdc_motor_forward(motor_m2));
        ESP_ERROR_CHECK(bdc_motor_set_speed(motor_m2, speed));
    }
    // back
    else if (speed < 0) {
        ESP_ERROR_CHECK(bdc_motor_reverse(motor_m2));
        ESP_ERROR_CHECK(bdc_motor_set_speed(motor_m2, -speed));
    }
    // stop
    else {
        if (stop_brake)
            ESP_ERROR_CHECK(bdc_motor_brake(motor_m2));
        else
            ESP_ERROR_CHECK(bdc_motor_coast(motor_m2));
    }

    return ESP_OK;
}

/* Motor rotation is controlled by motor ID number. speed input range: ±PWM_MOTOR_MAX_VALUE */
esp_err_t set_speed(motor_id_t motor_id, int speed) {
    if (motor_id == MOTOR_ID_M1)
        set_speed_m1(speed);
    else if (motor_id == MOTOR_ID_M2)
        set_speed_m2(speed);
    else if (motor_id == MOTOR_ID_ALL) {
        set_speed_m1(speed);
        set_speed_m2(speed);
    }

    return ESP_OK;
}

/* Control motor rotation. speed input range: ±PWM_MOTOR_MAX_VALUE */
esp_err_t set_speed_all(int speed_1, int speed_2) {
    set_speed_m1(speed_1);
    set_speed_m2(speed_2);

    return ESP_OK;
}

/* Motor stop */
esp_err_t motor_stop(motor_id_t motor_id, bool brake) {
    if (brake){
        if (motor_id == MOTOR_ID_M1)
            ESP_ERROR_CHECK(bdc_motor_brake(motor_m1));
        else if (motor_id == MOTOR_ID_M2)
            ESP_ERROR_CHECK(bdc_motor_brake(motor_m2));
        else if (motor_id == MOTOR_ID_ALL) {
            ESP_ERROR_CHECK(bdc_motor_brake(motor_m1));
            ESP_ERROR_CHECK(bdc_motor_brake(motor_m2));
        }

        stop_brake = true;
    }
    else {
        if (motor_id == MOTOR_ID_M1)
            ESP_ERROR_CHECK(bdc_motor_coast(motor_m1));
        else if (motor_id == MOTOR_ID_M2)
            ESP_ERROR_CHECK(bdc_motor_coast(motor_m2));
        else if (motor_id == MOTOR_ID_ALL){
            ESP_ERROR_CHECK(bdc_motor_coast(motor_m1));
            ESP_ERROR_CHECK(bdc_motor_coast(motor_m2));
        }

        stop_brake = false;
    }

    return ESP_OK;
}

/* Initialize motor 1 encoder, bind GPIO and configure PCNT counter */
esp_err_t init_encoder_m1(void) {
    pcnt_unit_config_t unit_config = {
        .high_limit = ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true, // enable counter accumulation
    };

    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };

    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_GPIO_H1A,
        .level_gpio_num = ENCODER_GPIO_H1B,
    };

    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENCODER_GPIO_H1B,
        .level_gpio_num = ENCODER_GPIO_H1A,
    };

    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    encoder_unit_m1 = pcnt_unit;

    return ESP_OK;
}

/* Initialize motor 2 encoder, bind GPIO and configure PCNT counter */
esp_err_t init_encoder_m2(void) {
    pcnt_unit_config_t unit_config = {
        .high_limit = ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true, // enable counter accumulation
    };

    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };

    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_GPIO_H2A,
        .level_gpio_num = ENCODER_GPIO_H2B,
    };

    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENCODER_GPIO_H2B,
        .level_gpio_num = ENCODER_GPIO_H2A,
    };

    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    encoder_unit_m2 = pcnt_unit;

    return ESP_OK;
}

/* Get the cumulative number of encoder pulses for motor 1 */
int get_count_m1(void) {
    static int count_m1 = 0;

    pcnt_unit_get_count(encoder_unit_m1, &count_m1);
    pcnt_unit_clear_count(encoder_unit_m1);

    return count_m1;
}

/* Get the cumulative number of encoder pulses for motor 2 */
int get_count_m2(void) {
    static int count_m2 = 0;

    pcnt_unit_get_count(encoder_unit_m2, &count_m2);
    pcnt_unit_clear_count(encoder_unit_m2);

    return count_m2;
}

/* Get the cumulative number of encoder pulses for a motor */
int get_count(uint8_t encoder_id) {
    if (encoder_id == ENCODER_ID_M1)
        return get_count_m1();
    if (encoder_id == ENCODER_ID_M2)
        return get_count_m2();
    
    return 0;
}

/* Get the revolutions per minute for motor 1 */
float get_rpm_m1(void) {
    return ((get_count_m1() * 60.0 * 1000.0)/(SAMPLE_TIME * R));
}

/* Get the revolutions per minute for motor 2 */
float get_rpm_m2(void) {
    return ((get_count_m2() * 60.0 * 1000.0)/(SAMPLE_TIME * R));
}

/* Get the revolutions per minute for a motor */
float get_rpm(uint8_t encoder_id) {
    if (encoder_id == ENCODER_ID_M1)
        return get_rpm_m1(); 
    if (encoder_id == ENCODER_ID_M2)
        return get_rpm_m2();
    
    return 0.0;
}

/* Get the angular velocity in rad/s for motor 1 */
float get_w_m1(void){
    return ((2 * PI * 1000.0 * get_count_m1()) / (SAMPLE_TIME * R));
}

/* Get the angular velocity in rad/s for motor 2 */
float get_w_m2(void){
    return ((2 * PI * 1000.0 * get_count_m2()) / (SAMPLE_TIME * R));
}

/* Get the angular velocity in rad/s for a motor */
float get_w(uint8_t encoder_id){
    if (encoder_id == ENCODER_ID_M1)
        return ((2 * PI * 1000.0 * get_count_m1()) / (SAMPLE_TIME * R));
    if (encoder_id == ENCODER_ID_M2)
        return ((2 * PI * 1000.0 * get_count_m2()) / (SAMPLE_TIME * R));
    
    return 0.0;
}

/* Get the linear velocity in m/s for motor 1 */
float get_u_m1(void){
    return (D_WHEEL * PI * 1000.0 * get_count_m1()) / (SAMPLE_TIME * R);
}

/* Get the linear velocity in m/s for motor 2 */
float get_u_m2(void){    
    return (D_WHEEL * PI * 1000.0 * get_count_m2()) / (SAMPLE_TIME * R);
}

/* Get the linear velocity in m/s for a motor */
float get_u(uint8_t encoder_id){
    if (encoder_id == ENCODER_ID_M1)
        return (D_WHEEL * PI * 1000.0 * get_count_m1()) / (SAMPLE_TIME * R);
    if (encoder_id == ENCODER_ID_M2)
        return (D_WHEEL * PI * 10000.0 * get_count_m1()) / (SAMPLE_TIME * R);
    
    return 0.0;
}

/* Convert the percentage to angular velocity */
float percent2w(int input){
    if (input > 0)
        return input * (PWM_MOTOR_DUTY_TICK_MAX - PWM_MOTOR_DEAD_ZONE) / (100.0) + PWM_MOTOR_DEAD_ZONE;
    else if (input < 0)
        return input * (-PWM_MOTOR_DUTY_TICK_MAX + PWM_MOTOR_DEAD_ZONE) / (-100.0) - PWM_MOTOR_DEAD_ZONE;
    return 0.0;
}

/* Convert the angular velocity to percentage */
float w2percent(float w){
    return w*(100.0/W_MAX);
}

/* Compute the angular velocity for motor 1 from linear and angular velocities */
float get_w_ref_m1(float u, float w){
    return ((u + (DIST_WHEELS * w / 2.0)) / (D_WHEEL / 2));
}

/* Compute the angular velocity for motor 2 from linear and angular velocities */
float get_w_ref_m2(float u, float w){
    return ((u - (DIST_WHEELS * w / 2.0)) / (D_WHEEL / 2)); 
}

/* Compute the angular velocity for a motor from linear and angular velocities */
float get_w_ref(uint8_t encoder_id, float u, float w){
    if (encoder_id == ENCODER_ID_M1)
        return get_w_ref_m1(u, w);
    if (encoder_id == ENCODER_ID_M2)
        return get_w_ref_m2(u, w);
    return 0.0;
}

/* Compute the linear velocity of the robot */
float get_u_robot(float w1, float w2){
    return (((D_WHEEL / 2) * (w1 + w2)) / 2.0);
}

/* Compute the angular velocity of the robot */
float get_w_robot(float w1, float w2){
    return (((D_WHEEL / 2) * (w1 - w2)) / DIST_WHEELS);
}

/* Lambda sintonization technique to find the gains of the PID control */
esp_err_t lambda_tunning(float K, float tau, float delay, float sampleTime, PIDController *pid){
    float lambda = 3 * tau;
    float Kp = tau / (K * (0.5 * delay + lambda));
    float Ti = tau;
    float Td = 0.5 * delay;

    pid->kp = Kp * ((Ti + Td) / Ti);
    pid->ti = Ti + Td;
    pid->td = Ti * Td / (Ti + Td);

    pid->ki = pid->kp / pid->ti;
    pid->kd = pid->kp * pid->td;

    pid->previous_err = 0.0f;
    pid->integral = 0.0f;
    pid->sample_time = (float)sampleTime/1000.0;

    return ESP_OK;
}

esp_err_t pid_init(float Kp, float Ki, float Kd, float sampleTime, PIDController *pid){
    pid->kp = Kp;
    pid->ki = Ki;
    pid->kd = Kd;

    pid->previous_err = 0.0f;
    pid->integral = 0.0f;
    pid->sample_time = (float)sampleTime/1000.0;

    return ESP_OK;
}

/* 
 Calculate the pid control value by location formula
 u(k) = e(k)*Kp + (e(k)-e(k-1))*Kd + integral*Ki
*/
float pid_compute(PIDController *pid, float setpoint, float current_value) {
    float error = setpoint - current_value;

    pid->integral += error * pid->sample_time;

    float derivative = (error - pid->previous_err) / pid->sample_time;
    
    // PID output
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    //float output = pid->kp*(error + (1/pid->ti) * pid->integral + pid->td * derivative);
    
    // Save current error for the next iteration
    pid->previous_err = error;
    
    return output;
}