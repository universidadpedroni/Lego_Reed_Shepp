import umath
def infoControl(elapsed_time, position_reference, position_vehicle, position_error, position_command,
                              steering_reference, steering_vehicle, steering_error, steering_command):
    mensaje = (
        "E_Tim: {:.1f}s | P_Ref: {:.2f}, P_Veh: {:.2f}, P_Err: {:.2f}, P_Com: {:.2f}| "
        "St_Ref: {:.2f}, St_Veh: {:.2f}, St_Err: {:.2f}, St_Com: {:.2f} "
    ).format(elapsed_time, position_reference, position_vehicle, position_error, position_command,
                           umath.degrees(steering_reference), steering_vehicle, steering_error, steering_command)

    print (mensaje)