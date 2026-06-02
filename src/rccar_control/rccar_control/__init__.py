"""
Mecanum wheel controller utilities
"""

def mecanum_kinematics(vx, vy, wz, wb, tw, wheel_r):
    """
    Convert linear/angular velocity to individual wheel velocities
    
    Args:
        vx: linear velocity forward (m/s)
        vy: linear velocity strafe (m/s)
        wz: angular velocity (rad/s)
        wb: wheel base (front-rear distance) (m)
        tw: track width (left-right distance) (m)
        wheel_r: wheel radius (m)
    
    Returns:
        (v_fl, v_fr, v_rl, v_rr): individual wheel velocities (rad/s)
    """
    L = (wb + tw) / 2.0
    
    v_fl = (vx - vy - L * wz) / wheel_r
    v_fr = (vx + vy + L * wz) / wheel_r
    v_rl = (vx + vy - L * wz) / wheel_r
    v_rr = (vx - vy + L * wz) / wheel_r
    
    return v_fl, v_fr, v_rl, v_rr


def normalize_wheel_velocities(velocities, max_vel=100.0):
    """
    Normalize wheel velocities if any exceeds max
    
    Args:
        velocities: tuple of (v_fl, v_fr, v_rl, v_rr)
        max_vel: maximum wheel velocity
    
    Returns:
        normalized velocities tuple
    """
    v_fl, v_fr, v_rl, v_rr = velocities
    max_v = max(abs(v_fl), abs(v_fr), abs(v_rl), abs(v_rr))
    
    if max_v > max_vel:
        scale = max_vel / max_v
        return (v_fl * scale, v_fr * scale, v_rl * scale, v_rr * scale)
    
    return velocities
