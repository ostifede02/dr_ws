'''
This class manage all add-ons of the delta robot.
These are the main characteristics:
    +++ init
        + import robot model    ok
        + init viewer           ok
        + init ig solvers       ok
        + init trajectory       ok
        + init micro com
        + init gui
        + init camera
    
    +++ trajectory
        + configure path: based on routine_type, start and goal position
            --> reset x and t traveled
        + get_pos_next: --> keep track of x traveled
        + get_t_next:   --> keep track of t traveled

    +++ inverse geometry
        + compute inverse geometry: pos_next -> q_next  --> keep track of q current
        + check collisions
    
    +++ stepper motors interface
        + convert delta_q to steps
        + manage discretization error
        

    + manage errors
    + manage exceptions


    +++ finite state machine flow
    states:             events:
        routine             pick
                            place
                            quick

        cmd_camera          camera
        
        cmd_user            new_pos
                            break

                            
+ set_path -> input: pos_end, path_routine_type
+ get_pos_next     -> update: x_current, s
+ get_q_continuos
+ get_delta_q_discrete   -> q_remainder
+ get_delta_t      -> t_current

while True:
                                        
    # check errors
    if nak_error_counter > 3:
        state = error

    if read_camera:
        read camera
        
        if position_detected
            pos_end = pos_predicted
            pos_place = object_type()
            path_routine = pick
            state = set_path

    if external_cmd:
        if stop_cmd
            state = idle

    if set_path:
        + set_path -> input: pos_end, path_routine_type
        state = run
    
    if run:
        + get_pos_next     -> update: x_current, s
            if s_next > 1:              # arrived to goal position
                state = routine_handler
                continue    jump back to top
        + get_q_continuos
            if collision:
                error = collision
                state = error
                continue    jump back to top
        + get_delta_q_discrete   -> q_remainder
        + get_delta_t      -> t_current
        state = send_cmd

    if send_cmd_micro:
        send to micro:
            delta_t
            delta_q_discrete
        state = recive_cmd_micro
        micro_cmd = (ack/nak/other_error)

    if recive_cmd_micro:
        if ack:
            state = run
        if nak:
            state = send_cmd_micro
            if error_nak_counter > 3:
                state = error
                error = nak_com
                continue
            else:
                error_nak_counter += 1

    if quick_neutral return
        pos_end = pos_neutral
        routine_type = quick
        state = set_path

    if routine_handler:
        if routine == pick
            routine = place
            pos_end = pos_place
            state = set_path
        
        if routine == place
            routine = quick
            pos_end = pos_neutral
        
        if routine == quick
            routine = pick
            state = get_cmd
    
    if cmd_handler:
        if cmd == camera_cmd
            pass
        if cmd == user_input_cmd
            pass


'''
