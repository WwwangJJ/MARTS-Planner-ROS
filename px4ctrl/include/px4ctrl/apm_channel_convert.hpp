#ifndef __APM_CHANNEL_CONVERT_HPP
#define __APM_CHANNEL_CONVERT_HPP

#include <ros/ros.h>
#include <algorithm>

namespace APM_Tools
{
    
#define APM_PWM_MIN     1100
#define APM_PWM_MAX     1900
#define APM_SPIN_MIN    0.15
#define APM_SPIN_MAX    0.95

static int16_t limit_pwm(int16_t pwm)
{
    if(pwm > APM_PWM_MAX)
        return APM_PWM_MAX;
    else if(pwm < APM_PWM_MIN)
        return APM_PWM_MIN;
    else
        return pwm;
} 

static double pwm_to_actuator(int16_t pwm)
{
    return (limit_pwm(pwm) - APM_PWM_MIN) / (double)(APM_PWM_MAX - APM_PWM_MIN);
}

static int16_t pwm_to_actuator(double actuator)
{
    return actuator * (APM_PWM_MAX - APM_PWM_MIN) + APM_PWM_MIN;
}


}


#endif 


