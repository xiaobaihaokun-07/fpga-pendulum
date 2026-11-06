// Upright controller adapted from E:/fpga_class/pid_2/src/pid_2.v.
// The swing-up state machine owns capture, sensor qualification and motor safety.
module balance_pid_core #(
    parameter METEOR_PROFILE_ENABLE=0,
    parameter signed [31:0] TARGET_ANGLE = 32'sd19993,
    parameter [11:0] PWM_PERIOD = 12'd3600,
    parameter signed [31:0] KP_ANG = 32'sd240,
    parameter signed [31:0] KI_ANG = 32'sd1,
    parameter signed [31:0] KD_ANG = 32'sd1800,
    parameter signed [31:0] DERIV_MAX = 32'sd120000,
    parameter signed [31:0] KP_POS = 32'sd80,
    parameter signed [31:0] KP_POS_NEAR = 32'sd50,
    parameter signed [31:0] POS_SOFT_RANGE = 32'sd60,
    parameter signed [31:0] KD_POS = 32'sd2500, 
    parameter signed [31:0] MOVE_KD_POS = 32'sd3000,
    parameter signed [31:0] POS_DEADBAND = 32'sd3,
    // Bounded DC correction for a persistent low-speed position error.
    // Defaults off so legacy/reference fixtures retain their exact behavior.
    parameter POS_HOLD_I_ENABLE = 0,
    parameter signed [31:0] POS_HOLD_I_DEADBAND = 32'sd12,
    parameter signed [31:0] POS_HOLD_I_RANGE = 32'sd180,
    parameter signed [31:0] POS_HOLD_I_LIMIT_ADC = 32'sd160,
    parameter signed [31:0] POS_HOLD_I_STEP_MAX = 32'sd64,
    parameter POS_HOLD_I_QUALIFY_SAMPLES = 20,
    // Suppress outer-loop hunting only after genuine stationary qualification.
    // The angle loop remains active; any real motion restores normal PD.
    parameter QUIET_HOLD_ENABLE = 0,
    parameter QUIET_HOLD_QUALIFY_SAMPLES = 20,
    parameter signed [31:0] QUIET_HOLD_ANGLE_SPAN = 32'sd20,
    parameter signed [31:0] QUIET_HOLD_EXIT_ANGLE = 32'sd30,
    parameter signed [31:0] QUIET_HOLD_EXIT_COUNTS = 32'sd4,
    // Restore the user's best auto_pid outer-loop law: raw 50ms speed,
    // uncapped P before the combined limit, no fixed equilibrium offset.
    parameter REFERENCE_SETTLING_ENABLE = 0,
    // Measured equilibrium in the 2026-10-21 log: about 20130--20140 ADC.
    // Keep this calibration separate from the position attraction: otherwise
    // a permanent position error is needed merely to stand upright.
    parameter signed [31:0] BALANCE_TRIM_ADC = 32'sd140,
    // Bound attraction BEFORE adding velocity feedback. Far-away position
    // error must not overpower braking on the return journey.
    parameter signed [31:0] POS_P_OFFSET_MAX = 32'sd120,
    parameter signed [31:0] MOVE_POS_P_OFFSET_MAX = 32'sd240,
    parameter signed [31:0] FAST_SPEED_MIN_20MS = 32'sd6,
    // User-validated static profile is unchanged near home at low speed.
    // Faster return outside a smaller static zone; extra braking is timed
    // from direction and stopping distance, not a large fixed near-home zone.
    // Experimental adaptive branches are retained for historical regression
    // only. Production uses the frozen static controller and a K1-like path.
    parameter ADAPTIVE_RETURN_ENABLE = 0,
    parameter signed [31:0] RETURN_BOOST_START_COUNTS = 32'sd18,
    parameter RETURN_BOOST_SHIFT = 7, // +128/100 ADC per extra encoder count
    parameter signed [31:0] RETURN_P_OFFSET_MAX = 32'sd360,
    parameter signed [31:0] RETURN_BRAKE_RANGE_COUNTS = 32'sd180,
    parameter signed [31:0] RETURN_BRAKE_SPEED_Q8 = 32'sd512,
    parameter PREDICTIVE_BRAKE_ENABLE = 1,
    parameter signed [31:0] RETURN_STOP_MARGIN_COUNTS = 32'sd12,
    parameter RETURN_BRAKE_DISTANCE_SHIFT = 1,
    parameter [3:0] RETURN_BRAKE_RAMP_STEP = 4'd2,
    parameter QUICK_RESPONSE_ENABLE = 1,
    parameter signed [31:0] DISTURBANCE_MIN_10MS = 32'sd2,
    parameter signed [31:0] RESPONSE_TARGET_SLEW = 32'sd75,
    parameter POS_VEL_FILTER_SHIFT = 0,
    parameter MOVE_VEL_FILTER_SHIFT = 1,
    parameter signed [31:0] INT_SEP_ERR = 32'sd200,
    parameter signed [31:0] ANG_INT_MAX = 32'sd4000,
    parameter signed [31:0] INT_DEADZONE = 32'sd5,
    parameter signed [31:0] TARGET_OFFSET_MAX = 32'sd500,
    // Separate moving correction authority from the quiet static profile.
    // With signed 17-bit location_out scaled by 100, max supported is 655.
    parameter signed [31:0] MOVE_TARGET_OFFSET_MAX = TARGET_OFFSET_MAX,
    parameter signed [31:0] TARGET_SLEW = 32'sd25,
    parameter signed [31:0] MOVE_TARGET_SLEW = TARGET_SLEW,
    parameter DERIV_FILTER_SHIFT = 1
) (
    input wire clk,
    input wire tick_1ms,
    input wire enable,
    input wire meteor_mode,
    input wire signed [31:0] angle_avg,
    input wire signed [31:0] enc_pos,
    input wire signed [31:0] arm_home,
    input wire turn_active,
    // Desired encoder displacement over 50 ms in Q8 counts. Zero at rest.
    input wire signed [31:0] arm_velocity_target_q8,
    input wire signed [31:0] capture_delta_5ms,
    output reg signed [16:0] target_offset = 0,
    output reg signed [23:0] angle_out = 0,
    output reg signed [23:0] balance_cmd = 0
);
    wire meteor_selected=METEOR_PROFILE_ENABLE && meteor_mode;
    wire quiet_profile_enabled=QUIET_HOLD_ENABLE && !meteor_selected;
    wire integral_profile_enabled=POS_HOLD_I_ENABLE && !meteor_selected;
    reg active = 0;
    reg [2:0] count_ang = 0;
    reg [3:0] count_pos = 0;
    reg [2:0] ang_stage = 0, pos_stage = 0;

    // Refresh every 10 ms, retaining the old D gain's counts/50 ms units.
    // Raw encoder history also prevents a home change from appearing as speed.
    reg signed [31:0] pos_history_0 = 0, pos_history_1 = 0;
    reg signed [31:0] pos_history_2 = 0, pos_history_3 = 0, pos_history_4 = 0;
    reg signed [31:0] pos_err_sample = 0, pos_delta_sample = 0;
    reg signed [31:0] pos_fast_delta_sample = 0;
    reg signed [31:0] pos_delta_10ms_sample = 0;
    reg signed [31:0] pos_raw_error_sample = 0;
    reg signed [63:0] pos_return_p_term = 0;
    reg signed [63:0] pos_bounded_p = 0, pos_braked_d = 0;
    reg [3:0] return_brake_weight = 0; // 0..8; extra D = weight/16
    reg signed [63:0] pos_near_p_term = 0, pos_far_p_term = 0;
    reg signed [63:0] pos_p_term = 0, pos_d_term = 0, pos_sum = 0;
    reg signed [40:0] pos_velocity_q8 = 0;
    reg signed [16:0] location_out = 0;
    reg signed [16:0] target_goal = 0;
    reg signed [31:0] pos_hold_integral = 0, previous_arm_home = 0;
    reg [7:0] hold_i_qualification = 0;
    reg quiet_hold = 0;
    reg [7:0] quiet_qualification = 0;
    reg signed [31:0] quiet_position = 0, quiet_home = 0;
    reg signed [31:0] quiet_angle_min = 0, quiet_angle_max = 0, quiet_angle = 0;
    reg signed [16:0] quiet_offset = 0;
    always @(posedge clk) begin
        target_goal <= location_out / 17'sd100;
        balance_cmd <= enable ? angle_out / 24'sd100 : 24'sd0;
    end

    wire signed [31:0] next_pos_err = arm_home - enc_pos;
    // The position P term has a continuous deadband; braking uses raw speed.
    wire signed [31:0] pos_err_shaped = next_pos_err > POS_DEADBAND ?
        next_pos_err - POS_DEADBAND : next_pos_err < -POS_DEADBAND ?
        next_pos_err + POS_DEADBAND : 32'sd0;
    // Continuous odd-symmetric P curve, with a gentler slope near home.
    // The deadband affects position attraction, never velocity braking.
    wire signed [31:0] pos_near_sample = pos_err_sample > POS_SOFT_RANGE ?
        POS_SOFT_RANGE : pos_err_sample < -POS_SOFT_RANGE ?
        -POS_SOFT_RANGE : pos_err_sample;
    wire signed [31:0] pos_far_sample = pos_err_sample - pos_near_sample;
    // 140 = 128+8+4: the original far-zone slope without an extra DSP.
    wire signed [63:0] meteor_far_ext={{32{pos_far_sample[31]}},pos_far_sample};
    wire signed [63:0] meteor_far_p=(meteor_far_ext <<< 7)+
        (meteor_far_ext <<< 3)+(meteor_far_ext <<< 2);
    wire signed [40:0] pos_delta_q8 =
        $signed({{9{pos_delta_sample[31]}}, pos_delta_sample}) <<< 8;
    // A 20 ms displacement times 640 has the same Q8 counts/50ms units.
    // Use it only for fast acceleration/reversal. Single encoder edges retain
    // the quiet 50 ms estimator, rather than getting fivefold amplification.
    wire signed [40:0] fast_delta_ext =
        $signed({{9{pos_fast_delta_sample[31]}}, pos_fast_delta_sample});
    wire signed [40:0] fast_velocity_q8 =
        (fast_delta_ext <<< 9) + (fast_delta_ext <<< 7);
    wire [40:0] fast_speed_abs = fast_velocity_q8 < 0 ?
        -fast_velocity_q8 : fast_velocity_q8;
    wire [40:0] slow_speed_abs = pos_delta_q8 < 0 ? -pos_delta_q8 : pos_delta_q8;
    wire use_fast_speed =
        (pos_fast_delta_sample >= FAST_SPEED_MIN_20MS ||
         pos_fast_delta_sample <= -FAST_SPEED_MIN_20MS) &&
        ((fast_velocity_q8[40] != pos_delta_q8[40]) || fast_speed_abs > slow_speed_abs);
    wire signed [40:0] base_measured_velocity_q8 =
        !REFERENCE_SETTLING_ENABLE && use_fast_speed ? fast_velocity_q8 : pos_delta_q8;
    wire signed [40:0] delta_10ms_ext =
        $signed({{9{pos_delta_10ms_sample[31]}},pos_delta_10ms_sample});
    wire signed [40:0] disturbance_velocity_q8 =
        (delta_10ms_ext <<< 10) + (delta_10ms_ext <<< 8);
    wire [40:0] disturbance_speed_abs = disturbance_velocity_q8 < 0 ?
        -disturbance_velocity_q8 : disturbance_velocity_q8;
    wire [40:0] base_speed_abs = base_measured_velocity_q8 < 0 ?
        -base_measured_velocity_q8 : base_measured_velocity_q8;
    // Negative measured velocity and positive position error mean returning
    // toward home. Use the early 10ms path only when moving AWAY from home;
    // steady return motion retains the quieter 50/20ms estimator.
    wire disturbance_outward =
        (pos_raw_error_sample > 0 && pos_delta_10ms_sample > 0) ||
        (pos_raw_error_sample < 0 && pos_delta_10ms_sample < 0);
    wire use_disturbance_speed = ADAPTIVE_RETURN_ENABLE && QUICK_RESPONSE_ENABLE &&
        !turn_active && disturbance_outward &&
        (pos_delta_10ms_sample >= DISTURBANCE_MIN_10MS ||
         pos_delta_10ms_sample <= -DISTURBANCE_MIN_10MS) &&
        (disturbance_velocity_q8[40] != base_measured_velocity_q8[40] ||
         disturbance_speed_abs > base_speed_abs);
    wire signed [40:0] measured_velocity_q8 = use_disturbance_speed ?
        disturbance_velocity_q8 : base_measured_velocity_q8;
    wire signed [40:0] pos_velocity_next = pos_velocity_q8 +
        ((measured_velocity_q8 - pos_velocity_q8) >>>
         (turn_active ? MOVE_VEL_FILTER_SHIFT : POS_VEL_FILTER_SHIFT));
    // pos_velocity_q8 is negative measured displacement. At rest, retain the
    // quiet speed feedback. During a commanded turn, damp only the difference
    // between planned and measured speed; otherwise the controller repeatedly
    // brakes the very motion requested by the moving position target.
    wire signed [40:0] pos_speed_error_q8 = pos_velocity_q8 +
        (turn_active ? $signed({{9{arm_velocity_target_q8[31]}},
                                arm_velocity_target_q8}) : 41'sd0);
    wire signed [63:0] pos_d_unscaled = pos_d_term < 0 ?
        -((-pos_d_term) >>> 8) : (pos_d_term >>> 8);
    wire signed [63:0] p_limit = turn_active ?
        MOVE_POS_P_OFFSET_MAX * 64'sd100 :
        (ADAPTIVE_RETURN_ENABLE ? RETURN_P_OFFSET_MAX : POS_P_OFFSET_MAX) * 64'sd100;
    wire signed [31:0] return_extra_error =
        pos_raw_error_sample > RETURN_BOOST_START_COUNTS ?
        pos_raw_error_sample - RETURN_BOOST_START_COUNTS :
        pos_raw_error_sample < -RETURN_BOOST_START_COUNTS ?
        pos_raw_error_sample + RETURN_BOOST_START_COUNTS : 32'sd0;
    wire signed [63:0] return_extra_extended =
        $signed({{32{return_extra_error[31]}}, return_extra_error});
    wire signed [63:0] pos_p_augmented = pos_p_term + pos_return_p_term;
    wire signed [63:0] pos_p_limited = REFERENCE_SETTLING_ENABLE ? pos_p_term :
        pos_p_augmented > p_limit ? p_limit :
        pos_p_augmented < -p_limit ? -p_limit : pos_p_augmented;
    wire return_has_speed = pos_velocity_q8 > RETURN_BRAKE_SPEED_Q8 ||
                           pos_velocity_q8 < -RETURN_BRAKE_SPEED_Q8;
    wire moving_away =
        (pos_raw_error_sample > 0 && pos_velocity_q8 > 0) ||
        (pos_raw_error_sample < 0 && pos_velocity_q8 < 0);
    wire [40:0] observed_speed_abs = pos_velocity_q8 < 0 ?
        -pos_velocity_q8 : pos_velocity_q8;
    // ceil(abs(speed50ms)), capped at 63. At higher speeds request braking
    // unconditionally. For a design deceleration of 400 counts/s^2:
    // v=20*speed50ms, stopping_distance=v*v/(2*400)=speed50ms^2/2.
    wire [5:0] stop_speed_bin = observed_speed_abs >= 41'd16128 ? 6'd63 :
        (observed_speed_abs + 41'd255) >> 8;
    reg [11:0] stop_speed_square = 0;
    wire signed [31:0] stopping_distance =
        $signed({20'd0,(stop_speed_square >> RETURN_BRAKE_DISTANCE_SHIFT)}) +
        RETURN_STOP_MARGIN_COUNTS;
    wire predictive_near =
        (pos_raw_error_sample <= stopping_distance &&
         pos_raw_error_sample >= -stopping_distance) ||
        observed_speed_abs >= 41'd16128;
    wire return_brake_requested = ADAPTIVE_RETURN_ENABLE && !turn_active &&
        return_has_speed && (PREDICTIVE_BRAKE_ENABLE ?
        (moving_away || predictive_near) :
        (pos_raw_error_sample <= RETURN_BRAKE_RANGE_COUNTS &&
         pos_raw_error_sample >= -RETURN_BRAKE_RANGE_COUNTS));
    wire signed [63:0] d_magnitude = pos_d_unscaled < 0 ?
        -pos_d_unscaled : pos_d_unscaled;
    // Explicit shift/add scaling avoids allocating another DSP multiplier.
    // Two steps per 10ms reach 1.5x D in 40ms when approaching the target.
    // Outward disturbances engage the full extra damping without this ramp.
    wire signed [63:0] extra_d_magnitude =
        (return_brake_weight[0] ? (d_magnitude >>> 4) : 64'sd0) +
        (return_brake_weight[1] ? (d_magnitude >>> 3) : 64'sd0) +
        (return_brake_weight[2] ? (d_magnitude >>> 2) : 64'sd0) +
        (return_brake_weight[3] ? (d_magnitude >>> 1) : 64'sd0);
    wire signed [63:0] extra_d = !ADAPTIVE_RETURN_ENABLE || turn_active ||
        !return_has_speed ? 64'sd0 :
        pos_d_unscaled < 0 ? -extra_d_magnitude : extra_d_magnitude;

    wire signed [31:0] target_delta = target_goal - target_offset;
    wire quick_response = ADAPTIVE_RETURN_ENABLE && QUICK_RESPONSE_ENABLE &&
        !turn_active && (return_has_speed ||
        pos_raw_error_sample > RETURN_BOOST_START_COUNTS ||
        pos_raw_error_sample < -RETURN_BOOST_START_COUNTS);
    wire signed [31:0] slew_now = turn_active ? MOVE_TARGET_SLEW :
        quick_response ? RESPONSE_TARGET_SLEW : TARGET_SLEW;
    wire signed [31:0] target_step = target_delta > slew_now ? slew_now :
        target_delta < -slew_now ? -slew_now : target_delta;
    wire signed [31:0] next_target_offset = target_offset + target_step;
    wire signed [31:0] next_ang_err = TARGET_ANGLE + next_target_offset - angle_avg;

    reg signed [31:0] ang_err_0 = 0;
    reg signed [31:0] last_angle = 0, angle_delta = 0;
    reg signed [31:0] ang_err_int = 0;
    reg signed [31:0] p_term = 0, d_raw = 0, d_filtered = 0;
    wire signed [31:0] d_limited = d_raw > DERIV_MAX ? DERIV_MAX :
        d_raw < -DERIV_MAX ? -DERIV_MAX : d_raw;
    wire signed [31:0] capture_d_raw = -KD_ANG * capture_delta_5ms;
    wire signed [31:0] capture_d_limited = capture_d_raw > DERIV_MAX ? DERIV_MAX :
        capture_d_raw < -DERIV_MAX ? -DERIV_MAX : capture_d_raw;
    wire signed [31:0] integral_sum = ang_err_int + ang_err_0;
    wire signed [31:0] integral_clamped = integral_sum > ANG_INT_MAX ? ANG_INT_MAX :
        integral_sum < -ANG_INT_MAX ? -ANG_INT_MAX : integral_sum;
    wire integrate = ang_err_0 <= INT_SEP_ERR && ang_err_0 >= -INT_SEP_ERR &&
        (ang_err_0 > INT_DEADZONE || ang_err_0 < -INT_DEADZONE);
    wire signed [31:0] integral_candidate = integrate ? integral_clamped : ang_err_int;
    wire signed [31:0] output_candidate =
        p_term + d_filtered + KI_ANG * integral_candidate;
    localparam signed [31:0] OUTPUT_LIMIT = PWM_PERIOD * 32'sd100;
    wire integral_blocked =
        (output_candidate > OUTPUT_LIMIT && ang_err_0 > 0) ||
        (output_candidate < -OUTPUT_LIMIT && ang_err_0 < 0);
    wire signed [31:0] integral_next =
        integral_blocked ? ang_err_int : integral_candidate;
    wire signed [31:0] output_next =
        p_term + d_filtered + KI_ANG * integral_next;
    // Multiply parameters at elaboration time, then select constants.
    // Selecting first inferred a needless runtime DSP for x100.
    wire signed [63:0] LOCATION_LIMIT = turn_active ?
        MOVE_TARGET_OFFSET_MAX*64'sd100 : TARGET_OFFSET_MAX*64'sd100;
    wire signed [63:0] quiet_offset_extended={{47{quiet_offset[16]}},quiet_offset};
    wire signed [63:0] quiet_offset_scaled=(quiet_offset_extended <<< 6)+
        (quiet_offset_extended <<< 5)+(quiet_offset_extended <<< 2);

    wire signed [31:0] hold_i_error =
        pos_raw_error_sample > POS_HOLD_I_DEADBAND ? pos_raw_error_sample-POS_HOLD_I_DEADBAND :
        pos_raw_error_sample < -POS_HOLD_I_DEADBAND ? pos_raw_error_sample+POS_HOLD_I_DEADBAND : 32'sd0;
    // Internal units are ADC*100. At most64/100 ADC per10ms =64 ADC/s.
    wire signed [31:0] hold_i_step = hold_i_error > POS_HOLD_I_STEP_MAX ? POS_HOLD_I_STEP_MAX :
        hold_i_error < -POS_HOLD_I_STEP_MAX ? -POS_HOLD_I_STEP_MAX : hold_i_error;
    wire signed [31:0] hold_i_limit = POS_HOLD_I_LIMIT_ADC * 32'sd100;
    wire signed [31:0] hold_i_sum = pos_hold_integral + hold_i_step;
    wire signed [31:0] hold_i_candidate = hold_i_sum > hold_i_limit ? hold_i_limit :
        hold_i_sum < -hold_i_limit ? -hold_i_limit : hold_i_sum;
    wire signed [63:0] hold_i_extended = integral_profile_enabled ?
        $signed({{32{pos_hold_integral[31]}},pos_hold_integral}) : 64'sd0;
    wire signed [63:0] hold_i_candidate_total = pos_p_limited + pos_d_unscaled +
        $signed({{32{hold_i_candidate[31]}},hold_i_candidate}) +
        (REFERENCE_SETTLING_ENABLE ? 64'sd0 : BALANCE_TRIM_ADC * 64'sd100);
    wire hold_i_eligible = integral_profile_enabled && !turn_active && arm_home==previous_arm_home &&
        !quiet_hold &&
        next_pos_err==pos_raw_error_sample &&
        pos_raw_error_sample<=POS_HOLD_I_RANGE && pos_raw_error_sample>=-POS_HOLD_I_RANGE &&
        hold_i_error!=0 && pos_velocity_next<=41'sd256 && pos_velocity_next>=-41'sd256 &&
        ang_err_0<=INT_SEP_ERR && ang_err_0>=-INT_SEP_ERR &&
        balance_cmd<$signed({1'b0,PWM_PERIOD}) && balance_cmd>-$signed({1'b0,PWM_PERIOD});
    wire hold_i_windup =
        (hold_i_candidate_total>LOCATION_LIMIT && hold_i_step>0) ||
        (hold_i_candidate_total< -LOCATION_LIMIT && hold_i_step<0);
    wire signed [31:0] quiet_min_next = angle_avg<quiet_angle_min ? angle_avg:quiet_angle_min;
    wire signed [31:0] quiet_max_next = angle_avg>quiet_angle_max ? angle_avg:quiet_angle_max;
    wire signed [31:0] quiet_displacement = enc_pos-quiet_position;
    wire signed [31:0] quiet_angle_change = angle_avg-quiet_angle;
    wire quiet_eligible = quiet_profile_enabled && !turn_active && arm_home==previous_arm_home &&
        pos_delta_sample==0 && pos_fast_delta_sample==0 && pos_delta_10ms_sample==0 &&
        ang_err_0<=32'sd60 && ang_err_0>=-32'sd60 &&
        target_delta<=32'sd2 && target_delta>=-32'sd2 &&
        balance_cmd<$signed({1'b0,PWM_PERIOD}) && balance_cmd>-$signed({1'b0,PWM_PERIOD});
    wire quiet_exit = turn_active || arm_home!=quiet_home ||
        quiet_displacement>QUIET_HOLD_EXIT_COUNTS || quiet_displacement< -QUIET_HOLD_EXIT_COUNTS ||
        quiet_angle_change>QUIET_HOLD_EXIT_ANGLE || quiet_angle_change< -QUIET_HOLD_EXIT_ANGLE ||
        pos_velocity_next>41'sd512 || pos_velocity_next< -41'sd512 ||
        pos_fast_delta_sample>=32'sd2 || pos_fast_delta_sample<=-32'sd2;

    // The stages finish in system-clock cycles between 5/10 ms samples.
    always @(posedge clk) begin
        if (!enable) begin
            active <= 0;
            count_ang <= 0; count_pos <= 0;
            ang_stage <= 0; pos_stage <= 0;
            ang_err_0 <= 0; ang_err_int <= 0;
            last_angle <= angle_avg; angle_delta <= 0;
            p_term <= 0; d_raw <= 0; d_filtered <= 0; angle_out <= 0;
            pos_history_0 <= enc_pos; pos_history_1 <= enc_pos;
            pos_history_2 <= enc_pos; pos_history_3 <= enc_pos;
            pos_history_4 <= enc_pos;
            pos_err_sample <= 0; pos_delta_sample <= 0; pos_fast_delta_sample <= 0;
            pos_delta_10ms_sample <= 0; stop_speed_square <= 0;
            pos_raw_error_sample <= 0; pos_return_p_term <= 0;
            pos_bounded_p <= 0; pos_braked_d <= 0; return_brake_weight <= 0;
            pos_near_p_term <= 0; pos_far_p_term <= 0;
            pos_p_term <= 0; pos_d_term <= 0; pos_sum <= 0;
            pos_velocity_q8 <= 0;
            location_out <= 0; target_offset <= 0;
            pos_hold_integral <= 0;hold_i_qualification <= 0;previous_arm_home <= arm_home;
            quiet_hold <= 0;quiet_qualification <= 0;quiet_position <= enc_pos;quiet_home <= arm_home;
            quiet_angle_min <= angle_avg;quiet_angle_max <= angle_avg;quiet_angle <= angle_avg;quiet_offset <= 0;
        end else if (!active) begin
            // Keep the existing swing-up catch's immediate speed damping.
            active <= 1;
            count_ang <= 0; count_pos <= 0;
            pos_history_0 <= enc_pos; pos_history_1 <= enc_pos;
            pos_history_2 <= enc_pos; pos_history_3 <= enc_pos;
            pos_history_4 <= enc_pos;
            last_angle <= angle_avg; angle_delta <= 0;
            ang_err_0 <= TARGET_ANGLE - angle_avg;
            d_filtered <= capture_d_limited;
            angle_out <= KP_ANG * (TARGET_ANGLE - angle_avg) + capture_d_limited;
            target_offset <= 0;
        end else begin
            if (tick_1ms) begin
                if (count_pos == 4'd9) begin
                    count_pos <= 0;
                    pos_err_sample <= pos_err_shaped;
                    pos_raw_error_sample <= next_pos_err;
                    pos_delta_sample <= pos_history_4 - enc_pos;
                    pos_fast_delta_sample <= pos_history_1 - enc_pos;
                    pos_delta_10ms_sample <= pos_history_0 - enc_pos;
                    pos_history_4 <= pos_history_3;
                    pos_history_3 <= pos_history_2;
                    pos_history_2 <= pos_history_1;
                    pos_history_1 <= pos_history_0;
                    pos_history_0 <= enc_pos;
                    pos_stage <= 1;
                end else count_pos <= count_pos + 1'b1;
                if (count_ang == 3'd4) begin
                    count_ang <= 0;
                    target_offset <= next_target_offset;
                    ang_err_0 <= next_ang_err;
                    // Differentiate measured angle, not moving target.
                    angle_delta <= last_angle - angle_avg;
                    last_angle <= angle_avg;
                    ang_stage <= 1;
                end else count_ang <= count_ang + 1'b1;
            end
            case (pos_stage)
                1: begin
                    pos_near_p_term <= KP_POS_NEAR *
                        $signed({{32{pos_near_sample[31]}}, pos_near_sample});
                    pos_far_p_term <= meteor_selected ? meteor_far_p : KP_POS *
                        $signed({{32{pos_far_sample[31]}}, pos_far_sample});
                    pos_return_p_term <= ADAPTIVE_RETURN_ENABLE && !turn_active ?
                        return_extra_extended <<< RETURN_BOOST_SHIFT : 64'sd0;
                    pos_velocity_q8 <= pos_velocity_next;
                    pos_stage <= 2;
                end
                2: begin
                    stop_speed_square <= stop_speed_bin * stop_speed_bin;
                    pos_p_term <= pos_near_p_term + pos_far_p_term;
                    pos_d_term <= (turn_active ? MOVE_KD_POS : KD_POS) *
                        $signed({{23{pos_speed_error_q8[40]}}, pos_speed_error_q8});
                    pos_stage <= 3;
                end
                3: begin
                    previous_arm_home <= arm_home;
                    if(!quiet_profile_enabled) begin quiet_hold<=0;quiet_qualification<=0;end
                    else if(quiet_hold) begin
                        if(quiet_exit) begin quiet_hold<=0;quiet_qualification<=0;end
                    end else if(!quiet_eligible) quiet_qualification<=0;
                    else if(quiet_qualification==0) begin
                        quiet_angle_min<=angle_avg;quiet_angle_max<=angle_avg;
                        quiet_qualification<=1;
                    end else if(quiet_max_next-quiet_min_next>QUIET_HOLD_ANGLE_SPAN)
                        quiet_qualification<=0;
                    else begin
                        quiet_angle_min<=quiet_min_next;quiet_angle_max<=quiet_max_next;
                        if(quiet_qualification>=QUIET_HOLD_QUALIFY_SAMPLES-1) begin
                            quiet_hold<=1;quiet_offset<=target_offset;
                            quiet_position<=enc_pos;quiet_home<=arm_home;quiet_angle<=angle_avg;
                        end else quiet_qualification<=quiet_qualification+1'b1;
                    end
                    if(!hold_i_eligible) hold_i_qualification <= 0;
                    else begin
                        if(hold_i_qualification<POS_HOLD_I_QUALIFY_SAMPLES)
                            hold_i_qualification <= hold_i_qualification+1'b1;
                        else if(!hold_i_windup) pos_hold_integral <= hold_i_candidate;
                    end
                    if (return_brake_requested) begin
                        if (PREDICTIVE_BRAKE_ENABLE && moving_away)
                            return_brake_weight <= 8;
                        else if (return_brake_weight >= 8 - RETURN_BRAKE_RAMP_STEP)
                            return_brake_weight <= 8;
                        else return_brake_weight <= return_brake_weight + RETURN_BRAKE_RAMP_STEP;
                    end else if (return_brake_weight > RETURN_BRAKE_RAMP_STEP)
                        return_brake_weight <= return_brake_weight - RETURN_BRAKE_RAMP_STEP;
                    else return_brake_weight <= 0;
                    pos_bounded_p <= pos_p_limited;
                    pos_stage <= 4;
                end
                4: begin
                    pos_braked_d <= pos_d_unscaled + extra_d;
                    pos_stage <= 5;
                end
                5: begin
                    pos_sum <= quiet_hold ? quiet_offset_scaled : pos_bounded_p + pos_braked_d +
                        hold_i_extended +
                        (REFERENCE_SETTLING_ENABLE ? 64'sd0 : BALANCE_TRIM_ADC * 64'sd100);
                    pos_stage <= 6;
                end
                6: begin
                    if (pos_sum > LOCATION_LIMIT) location_out <= LOCATION_LIMIT;
                    else if (pos_sum < -LOCATION_LIMIT) location_out <= -LOCATION_LIMIT;
                    else location_out <= pos_sum;
                    pos_stage <= 0;
                end
                default: begin end
            endcase
            case (ang_stage)
                1: begin
                    p_term <= KP_ANG * ang_err_0;
                    d_raw <= KD_ANG * angle_delta;
                    ang_stage <= 2;
                end
                2: begin
                    d_filtered <= d_filtered +
                        ((d_limited - d_filtered) >>> DERIV_FILTER_SHIFT);
                    ang_stage <= 3;
                end
                3: begin
                    ang_err_int <= integral_next;
                    angle_out <= output_next;
                    ang_stage <= 0;
                end
                default: begin end
            endcase
        end
    end
endmodule
