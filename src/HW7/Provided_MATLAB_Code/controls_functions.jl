Base.@kwdef mutable struct Integrators
    integrator = 0
    error_d1 = 0
end

Base.@kwdef mutable struct StateMachines
    alt_mode = 0
    reset_flag = true
end

function SLCWithFeedForwardAutopilot(time, aircraft_state, wind_angles, control_objectives, control_gain_struct,
                                slp_pst, crs_pst, rll_pst, p_pst, t_pst, a_pst, alt_pst)

    # process inputs
    euler_angles    = aircraft_state[4:6];
    velocity_body   = aircraft_state[7:9]; #[u v w]
    omega_body      = aircraft_state[10:12]; #[p q r]

    flight_angles   = FlightPathAnglesFromState(aircraft_state); #[Vg, chi, gamma]
    chi = flight_angles[2];


    pn       = aircraft_state[1];   # inertial North position
    pe       = aircraft_state[2];   # inertial East position
    h        = -aircraft_state[3];   # altitude

    phi      = aircraft_state[4];  # roll angle
    theta    = aircraft_state[5];  # pitch angle
    psi      = aircraft_state[6];  # yaw angle

    Va      = wind_angles[1];
    beta    = wind_angles[2];
    alpha   = wind_angles[3];

    h_c         = control_objectives[1];   # commanded altitude (m)
    h_dot_c     = control_objectives[2];   # commanded altitude rate (m)
    chi_c       = control_objectives[3];   # commanded course (rad)
    chi_dot_ff  = control_objectives[4];   # commanded course rate (rad)
    Va_c        = control_objectives[5];   # commanded airspeed (m/s)

    t = time;   # time

    if (t==0)
        flag  = 1;
    else
        flag = 0;
    end;




    #----------------------------------------------------------
    # lateral autopilot
    chi_dot_c = control_gain_struct.Kff_course_rate*chi_dot_ff + control_gain_struct.Kp_course_rate*(unwrap_angle(chi_c - chi));

    phi_des, q_des, r_des = coordinated_turn_rates(chi_dot_c, Va, control_gain_struct);

    phi_c = phi_des;
    delta_a, p_c = roll_hold(phi_c, euler_angles[1], omega_body[1], flag, control_gain_struct, rll_pst);

    delta_r = sideslip_hold(beta, r_des, omega_body[3], flag, control_gain_struct, slp_pst);


    ## ----------------------------------------------------------
    ##  longitudinal autopilot
    delta_t, theta_c, alt_mode = altitude_state_machine(h_c, h, Va_c, Va, flag, control_gain_struct, alt_pst, p_pst, t_pst, a_pst);

    delta_e = pitch_hold(theta_c, theta, q_des, omega_body[2], flag, control_gain_struct);


    ## ----------------------------------------------------------
    ##  create outputs

    # control outputs
    delta = [control_gain_struct.u_trim[1] + delta_e; control_gain_struct.u_trim[2] + delta_a; control_gain_struct.u_trim[3] + delta_r; delta_t];

    # commanded (desired) states
    x_command = [
        0;                    # pn
        0;                    # pe
        h_c;                  ##  h
        Va_c;                 ##  Va
        0;                    ##  alpha
        0;                    ##  beta
        phi_c;                ##  phi
        # theta_c*control_gain_struct.Kpitch_DC;... #  theta
        theta_c;              ##  theta
        chi_c;                ##  chi
        p_c;                   # #  p
        q_des;                 #   #  q
        r_des;                 #   #  r
        ];


    control_input = delta;

    #y = [delta; x_command];
    #y = delta;

    return control_input, x_command

end



### ## # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#  Autopilot functions
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#  coordinated_turn_rates
#    - calculates desired roll angle, pitch rate, and yaw rate for
#    coordinated turn
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
function coordinated_turn_rates(chi_dot_c, V, control_gains)

    phi_des = atan2(chi_dot_c*V, control_gains.g);
    phi_des = sat(phi_des, control_gains.max_roll, -control_gains.max_roll);

    q_des = chi_dot_c*sin(phi_des);
    r_des = chi_dot_c*cos(phi_des);

    return phi_des, q_des, r_des
end

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#  course_hold
#    - regulate heading using the roll command
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
function course_hold(chi_c, chi, phi_ff, flag, control_gains, pst)
  # persistent integrator;
  # persistent error_d1;
  #  initialize persistent variables at beginning of simulation
  if flag
      pst.integrator = 0;
      pst.error_d1   = 0;
  end

  #  compute the current error
  error = unwrap_angle(chi_c - chi);

  #  update the integrator
  pst.integrator = pst.integrator + (control_gains.Ts/2)*(error + pst.error_d1); #  trapazoidal rule

  #  proportional term
  up = control_gains.Kp_course * error;

  #  integral term
  ui = control_gains.Ki_course * pst.integrator;

  #  implement PID control
  phi_c = sat(phi_ff + up + ui, control_gains.max_roll, -control_gains.max_roll);

  #  implement integrator anti-wind-up
  if control_gains.Ki_course!=0
      phi_c_unsat = phi_ff + up + ui;
      pst.integrator = pst.integrator + (control_gains.Ts/control_gains.Ki_course) * (phi_c - phi_c_unsat);
  end

  #  update persistent variables
  pst.error_d1 = error;

  return phi_c
end


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#  roll_hold
#    - regulate roll using aileron
#    - first calculate desired roll rate from PI of roll angle command
#    - then use feedforward to aileron plus P control on roll rate
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
function roll_hold(phi_c, phi, p, flag, control_gains, pst)

  # persistent integrator;
  # persistent error_d1;
  #  initialize persistent variables at beginning of simulation
  if flag
      pst.integrator = 0;
      pst.error_d1   = 0;
  end

  #  compute the current error
  error = phi_c - phi;

  #  update the integrator
  pst.integrator = pst.integrator + (control_gains.Ts/2)*(pst.error + pst.error_d1); #  trapazoidal rule

  #  proportional term
  up = control_gains.Kp_roll * error;

  #  integral term
  ui = control_gains.Ki_roll * pst.integrator;

  #  implement PID control
  p_c = sat(up + ui, control_gains.max_roll_rate, -control_gains.max_roll_rate);

  #  implement integrator anti-wind-up
  if control_gains.Ki_roll!=0
      p_c_unsat = up + ui;
      pst.integrator = pst.integrator + (control_gains.Ts/control_gains.Ki_roll) * (p_c - p_c_unsat);
  end

  delta_a_ff = control_gains.Kff_da*p_c;
  ud = control_gains.Kd_roll*(p_c - p);

  delta_a = sat(delta_a_ff + ud, control_gains.max_da, -control_gains.max_da);


  #  update persistent variables
  pst.error_d1 = error;

  return delta_a, p_c
end

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#  altitude_state_machine
#    - determines throttle and pitch hold commands based on altitude
#    relative to commanded height
#
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
function altitude_state_machine(h_c, h, Va_c, Va, flag, control_gains, pst, p_pst, t_pst, a_pst);

    # persistent alt_mode
    # persistent reset_flag

    if(flag)
        pst.alt_mode = 0;
        pst.reset_flag = 1;
    end

    error_height = h_c - h;


    if (h<control_gains.takeoff_height) #  Take-off (assumes ground is at z = 0;
        if (pst.alt_mode!=1)
            println("Altitude mode: Take Off")
            pst.alt_mode=1;
        end

        delta_t = control_gains.climb_throttle;
        theta_c = control_gains.takeoff_pitch;

    elseif (-error_height < -control_gains.height_hold_limit) #  Climb
        if (pst.alt_mode!=2)
            println("Altitude mode: Climb");
            pst.alt_mode=2;
            pst.reset_flag=true;
        else
            pst.reset_flag=false;
        end

        delta_t = control_gains.climb_throttle;
        theta_c = airspeed_with_pitch_hold(Va_c, Va, pst.reset_flag, control_gains, p_pst);

    elseif (abs(error_height) <= control_gains.height_hold_limit) #  Altitude hold
        if (alt_mode!=3)
            println("Altitude mode: Altitude Hold");
            pst.alt_mode=3;
            pst.reset_flag=true;
        else
            pst.reset_flag=false;
        end

        delta_t = airspeed_with_throttle_hold(Va_c, Va, pst.reset_flag, control_gains, t_pst);
        theta_c = altitude_hold(h_c, h, pst.reset_flag, control_gains, a_pst);

    else #  Descend
        if (alt_mode!=4)
            println("Altitude mode: Descend");
            pst.alt_mode=4;
            pst.reset_flag = true;
        else
            pst.reset_flag= false;
        end

        delta_t = 0;
        theta_c = airspeed_with_pitch_hold(Va_c, Va, pst.reset_flag, control_gains, p_pst);
    end

    mode_out = pst.alt_mode;
    return delta_t, theta_c, mode_out
end


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#  pitch_hold
#    - regulate pitch using elevator
#
#  No integrator so no need to worry about reset and anti-wind-up
#  No differentiator since assume q is known
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
function pitch_hold(theta_c, theta, q_c, q, flag, control_gains)
    uff = control_gains.Kff_de*q_c;
    up = control_gains.Kp_pitch*(theta_c - theta);
    ud = control_gains.Kd_pitch*(q_c - q);
    delta_e = sat(uff + up + ud, control_gains.max_de, -control_gains.max_de);
    return delta_e
end

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#  airspeed_with_pitch_hold
#    - regulate airspeed using pitch angle
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
function airspeed_with_pitch_hold(Va_c, Va, flag, control_gains, pst)

  # persistent integrator;
  # persistent error_d1;
  #  initialize persistent variables at beginning of simulation
  if flag
      pst.integrator = 0;
      pst.error_d1   = 0;
  end

  #  compute the current error
  error = Va_c - Va;

  #  update the integrator
  pst.integrator = pst.integrator + (control_gains.Ts/2)*(error + pst.error_d1); #  trapazoidal rule

  #  proportional term
  up = control_gains.Kp_speed_pitch * error;

  #  integral term
  ui = control_gains.Ki_speed_pitch * pst.integrator;

  #  implement PID control
  theta_c = sat(up + ui, control_gains.max_pitch, -control_gains.max_pitch);

  #  implement integrator anti-wind-up
  if control_gains.Ki_speed_pitch!=0
      theta_c_unsat = up + ui;
      pst.integrator = pst.integrator + control_gains.Ts/control_gains.Ki_speed_pitch * (theta_c - theta_c_unsat);
  end

  #  update persistent variables
  pst.error_d1 = error;
  return theta_c
end

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#  airspeed_with_throttle_hold
#    - regulate airspeed using throttle
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
function airspeed_with_throttle_hold(Va_c, Va, flag, control_gains, pst)

  # persistent integrator;
  # persistent error_d1;
  #  initialize persistent variables at beginning of simulation
    if flag
        pst.integrator = 0;
        pst.error_d1   = 0;
    end

  #  compute the current error
  error = Va_c - Va;

  #  update the integrator
  pst.integrator = pst.integrator + (control_gains.Ts/2)*(error + pst.error_d1); #  trapazoidal rule

  #  proportional term
  up = control_gains.Kp_speed_throttle * error;

  #  integral term
  ui = control_gains.Ki_speed_throttle * pst.integrator;

  #  implement PID control
  delta_t = sat(control_gains.u_trim[4] + up + ui, 1, 0);

  #  implement integrator anti-wind-up
  if control_gains.Ki_speed_throttle!=0
      delta_t_unsat = control_gains.u_trim[4] + up + ui;
      pst.integrator = pst.integrator + control_gains.Ts/control_gains.Ki_speed_throttle * (delta_t - delta_t_unsat);
  end

  #  update persistent variables
  pst.error_d1 = error;
  return delta_t
end

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#  altitude_hold
#    - regulate altitude using pitch angle
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
function altitude_hold(h_c, h, flag, control_gains, pst)

  # persistent integrator;
  # persistent error_d1;
  #  initialize persistent variables at beginning of simulation
  if flag
      pst.integrator = 0;
      pst.error_d1   = 0;
  end

  #  compute the current error
  error = h_c - h;

  #  update the integrator
  pst.integrator = pst.integrator + (control_gains.Ts/2)*(error + pst.error_d1); #  trapazoidal rule

  #  proportional term
  up = control_gains.Kp_height * error;

  #  integral term
  ui = control_gains.Ki_height * pst.integrator;

  #  implement PID control
  theta_c = sat(up + ui, control_gains.max_pitch, -control_gains.max_pitch);

  #  implement integrator anti-wind-up
  if control_gains.Ki_height!=0
      theta_c_unsat = up + ui;
      pst.integrator = pst.integrator + (control_gains.Ts/control_gains.Ki_height) * (theta_c - theta_c_unsat);
  end

  #  update persistent variables
  pst.error_d1 = error;
  return theta_c
end


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#  coordinated_turn_hold
#    - sideslip with rudder
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
function sideslip_hold(beta, r_des, r, flag, control_gains, pst);
  # persistent integrator;
  # persistent error_d1;
  #  initialize persistent variables at beginning of simulation
  if flag
      pst.integrator = 0;
      pst.error_d1   = 0;
  end

  #  compute the current error
  error = - beta;
  error_r = r_des - r;

  #  update the integrator
  pst.integrator = pst.integrator + (control_gains.Ts/2)*(error + pst.error_d1); #  trapazoidal rule

  #  proportional term
  up = control_gains.Kp_beta * error;

  #  integral term
  ui = control_gains.Ki_beta * pst.integrator;

  #  derivative term
  ud = control_gains.Kd_beta * error_r;

  #  feed forward term
  delta_r_ff = control_gains.Kff_dr * r_des;

  #  implement PID control
  delta_r = sat(delta_r_ff + up + ud + ui, control_gains.max_dr, -control_gains.max_dr);

  #  implement integrator anti-wind-up
  if control_gains.Ki_beta!=0
      delta_r_unsat = delta_r_ff + up + ud + ui;
      pst.integrator = pst.integrator + (control_gains.Ts/control_gains.Ki_beta) * (delta_r - delta_r_unsat);
  end

  #  update persistent variables
  pst.error_d1 = error;

  return delta_r

end

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#  coordinated_turn_hold
#    - sideslip with rudder
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
function coordinated_turn_hold(beta, flag, P)
  delta_r = 0;
  return delta_r
end


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#  sat
#    - saturation function
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
function sat(in, up_limit, low_limit)
  if in > up_limit
      out = up_limit;
  elseif in < low_limit
      out = low_limit;
  else
      out = in;
  end
  return out
end

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#  sat
#    - unwrap angle so -pi<out<pi
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
function unwrap_angle(in)
    out = in;
    while (out>pi)
        out = out - 2*pi;
    end
    while (out<-pi)
        out = out + 2*pi;
    end
    return out
end
