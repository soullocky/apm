#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
float aim_roll_deg;
float aim_pitch_deg;
float delta_pitch_deg_s;
float ahrs_roll_deg;
float ahrs_pitch_deg;

void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    uint16_t roll = hal.rcin->read(CH_9);//PWM在1600-1850
    if (roll < 1600 && roll >1400)
    {
        if (aim_pitch_deg < 70.0f)
        {
            aim_pitch_deg += 0.1f;
            delta_pitch_deg_s = 0.1f;
        }
        else if (aim_pitch_deg < 80.0f)
        {
            aim_pitch_deg += 0.05f;
            delta_pitch_deg_s = 0.05f;
        }
        else if (aim_pitch_deg < 90.0f)
        {
            aim_pitch_deg += 0.02f;
            delta_pitch_deg_s = 0.02f;
        }
        else
        {
            delta_pitch_deg_s = 0.0f;
        }
    }
    else if (roll < 2050 && roll >1850)
    {
        if (aim_pitch_deg > 20.0f)
        {
            aim_pitch_deg -= 0.1f;
            delta_pitch_deg_s = -0.1f;
        }
        else if (aim_pitch_deg > 10.0f)
        {
            aim_pitch_deg -= 0.05f;
            delta_pitch_deg_s = -0.05f;
        }
        else if (aim_pitch_deg > 0.0f)
        {
            aim_pitch_deg -= 0.02f;
            delta_pitch_deg_s = -0.02f;
        }
        else
        {
            delta_pitch_deg_s = 0.0f;
        }
    }
    else
    {
        delta_pitch_deg_s = 0.0f;
    }

    ahrs_roll_deg = degrees(ahrs.get_roll());

    uint16_t pitch = hal.rcin->read(CH_6);
    if (pitch < 1600 && pitch >1400)
    {
        if (aim_pitch_deg < 70.0f)
        {
            aim_pitch_deg += 0.1f;
            delta_pitch_deg_s = 0.1f;
        }
        else if (aim_pitch_deg < 80.0f)
        {
            aim_pitch_deg += 0.05f;
            delta_pitch_deg_s = 0.05f;
        }
        else if (aim_pitch_deg < 90.0f)
        {
            aim_pitch_deg += 0.02f;
            delta_pitch_deg_s = 0.02f;
        }
        else
        {
            delta_pitch_deg_s = 0.0f;
        }
    }
    else if (pitch < 2050 && pitch >1850)
    {
        if (aim_pitch_deg > 20.0f)
        {
            aim_pitch_deg -= 0.1f;
            delta_pitch_deg_s = -0.1f;
        }
        else if (aim_pitch_deg > 10.0f)
        {
            aim_pitch_deg -= 0.05f;
            delta_pitch_deg_s = -0.05f;
        }
        else if (aim_pitch_deg > 0.0f)
        {
            aim_pitch_deg -= 0.02f;
            delta_pitch_deg_s = -0.02f;
        }
        else
        {
            delta_pitch_deg_s = 0.0f;
        }
    }
    else
    {
        delta_pitch_deg_s = 0.0f;
    }

    ahrs_pitch_deg = degrees(ahrs.get_pitch());

    
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
