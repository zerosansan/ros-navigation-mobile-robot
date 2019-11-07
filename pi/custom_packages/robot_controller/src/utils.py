import time
import math

# Twist to velocity modifiers
MOD1 = (33.3 / 100)
MOD2 = (15.5 / 100)
MOD3 = (20.1 / 100)
MOD4 = (5.5 / 100)
MOD5 = (27.1 / 100)
MOD6 = (21.3 / 100)

def twist_to_velocity(self, vx, vy, vth, axle_len):
    """
        Function: convert Twist message to velocity commands

        For every Twist message received i.e velocity x, y and theta,
        convert to velocity of left and right wheel
    """
    vel = {\
        'left': 0,\
        'right': 0\
    }
    _vleft = 0
    _vright = 0
    
    if (vx == 0):
        # turning
        if (vth < 0):
            _vleft = vth * (axle_len / 2) \
                + (vth * (axle_len / 2) * MOD1)
            _vright = (-1) * ((vth * (axle_len / 2)) \
                + (vth * (axle_len / 2) * MOD2))
        if (vth > 0):
            _vleft = vth * (axle_len / 2)
            _vright = (-1) * ((vth * (axle_len / 2)) \
            + (vth * (axle_len / 2) * MOD3))
        if (vth == 0):
            _vleft = vright = vth

    elif (vth == 0):
        # forward or reverse
        if (vx > 0):
            _vleft = vx
            _vright = vx + (vx * MOD4)
        if (vx < 0):
            _vleft = vx + (vx * MOD5)
            _vright = vx + (vx * MOD6)
        if (vx == 0):
            _vleft = _vright = vx
    else:       
        # moving doing arcs
        _vleft = vx + vth * (axle_len / 2)
        _vright = vx - vth * (axle_len / 2)
    
    vel['left'] = _vleft
    vel['right'] = _vright
    
    return vel
    
def velocity_to_pulse(vel, wheel_rad, MOD):
    """
        Function: convert velocity comands to pulse width
    """ 
    pulse = vel / (MOD * ((2 * math.pi)/ 60) * wheel_rad)
    
    return pulse
