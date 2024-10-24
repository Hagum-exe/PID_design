def compute_yaw_pitch_roll(w1, w2, w3, w4, b, d):   #angular velocities of rotors, thrust coefficient and drag coefficient
    u1 = b(w1**2+w2**2+w3**2+w4**2)
    u2 = b(-w2**2+w4**2)
    u3 = b(w1**2-w3**2)
    u4 = d(-w1**2+w2**2-w3**2+w4**2)
    