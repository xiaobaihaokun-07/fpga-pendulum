// Automatic swing-up and cascade balance controller for the existing GW2AR board.
// ADC calibration: hanging ~= 6360, upright ~= 19993, high-side limit ~= 26172.
// The encoder tracks the horizontal motor arm's azimuth, not a linear cart.
// The arm can rotate continuously; no encoder-count limit is assumed by default.
module balance_top (
    input  wire clk,
    input  wire key1,          // Tang Nano 20K user key 1, pin 88, active high
    input  wire key2,          // Tang Nano 20K user key 2, pin 87, active high
    input wire k2_track_n,
    input wire k3_meteor_n,
    input wire k4_angle_n,     // External K4/P72, active low
    input wire camera_update,camera_x_valid,camera_configured,camera_locked,
    input wire [15:0] camera_x_q8,camera_red_pixels,
    input wire [15:0] camera_y_q8,camera_failed_register,
    input wire [7:0] camera_error,
    input wire [31:0] camera_sequence,
    input  wire k1_turn_n,     // External K1 on pin 52, active low
    output wire scl,
    inout  wire sda,
    input  wire enc_a,
    input  wire enc_b,
    output reg  pwma = 1'b0,
    output reg  ain1 = 1'b1,
    output reg  ain2 = 1'b1,
    output wire uart_tx_pin,
    output wire ws2812_data
);
    parameter UART_INTERVAL_CYCLES=540000;
    parameter REQUIRE_START_KEY=1;
    parameter START_KEY_HOLD_MS=20;
    parameter METEOR_ENABLE=0; // wrapper enables K3 reference-controller selection
    parameter METEOR_BOTTOM_STABLE_MS=80;
    parameter signed [31:0] METEOR_STATIC_RANGE=32'sd2000;
    parameter signed [31:0] METEOR_STATIC_SPAN=32'sd64;
    parameter [1:0] TRANSITION_PULSE_RETRIES=2'd2;
    parameter signed [31:0] METEOR_RELEASE_PWM=32'sd1000;
    parameter METEOR_RELEASE_MAX_MS=80;
    parameter signed [31:0] METEOR_RELEASE_TRAVEL_COUNTS=32'sd30;
    parameter VISION_ENABLE=0; // wrapper enables integration; legacy tests default off
    parameter ANGLE_SET_ENABLE=0; // production wrapper enables K4 angle entry
    parameter TRACK_SIGN=1;
    parameter TRACK_SPAN_DEG=270;
    parameter TRACK_X_DEADBAND_PX=5;
    parameter TRACK_X_MEDIAN_ENABLE=1;
    parameter [15:0] TRACK_MAX_SPEED_Q16=16'd12000;
    parameter [15:0] TRACK_ACCEL_Q16=16'd16;
    parameter signed [31:0] TARGET_ANGLE = 32'sd20175;
    parameter signed [31:0] HANG_ANGLE = 32'sd6360;
    parameter signed [31:0] CENTER_RANGE = 32'sd1800;
    parameter [11:0] PWM_PERIOD = 12'd3600;

    // Swing-up tuning. SWING_POLARITY reverses both kick directions if necessary.
    parameter signed [31:0] SWING_PWM = 32'sd2250; // 62.5% at PWM_PERIOD=3600
    parameter signed [31:0] SWING_BOOST_PER_KICK = 32'sd150;
    parameter signed [31:0] SWING_PWM_MAX = 32'sd2600;
    parameter SWING_POLARITY = 1;                    // +1 or -1
    parameter signed [31:0] SWING_ARM_STEP_COUNTS = 32'sd340;
    parameter signed [31:0] SWING_RETURN_BAND_COUNTS = 32'sd50;
    // Encoder speed is reported as counts per 100 ms. Wait for the arm to
    // settle before re-arming at its current azimuth after balance falls.
    parameter signed [15:0] RETRY_MAX_ARM_SPEED = 16'sd10;
    // Rest qualification has its own noise tolerance, independent of the
    // minimum directional motion (8 ADC/5ms) used to time recovery pulses.
    parameter signed [31:0] RETRY_MAX_DELTA_5MS = 32'sd32;
    parameter signed [31:0] RETURN_MAX_DISPLACEMENT_COUNTS = 32'sd500;
    parameter [15:0] KICK_MAX_MS = 16'd500;
    parameter [15:0] RETURN_MAX_MS = 16'd800;
    parameter [31:0] RETURN_PROGRESS_MIN_COUNTS = 32'd80;
    // Repeated, bounded phase-matched pulses while the pendulum is moving.
    parameter [7:0] MAX_CORRECTION_KICKS = 8'd24;
    parameter [15:0] RECOVERY_COAST_MIN_MS = 16'd40;
    parameter signed [15:0] RECOVERY_MAX_ARM_SPEED = 16'sd30;
    // A push accelerates the arm. Its running limit must not be the entry
    // limit, otherwise the commanded motion trips its own start condition.
    parameter signed [15:0] RECOVERY_RUN_MAX_ARM_SPEED = 16'sd100;
    parameter signed [31:0] RECOVERY_MIN_DELTA_5MS = 32'sd8;
    parameter signed [31:0] RECOVERY_MAX_DELTA_5MS = 32'sd700;
    // Dynamic recovery must not inherit a boost on every failed passage.
    // These settings leave the proven stationary +2250/-2250 kick unchanged.
    parameter signed [31:0] RECOVERY_PWM_MAX = 32'sd1550;
    parameter signed [31:0] RECOVERY_PWM_MID = 32'sd1250;
    parameter signed [31:0] RECOVERY_PWM_NEAR_TOP = 32'sd500;
    // Select torque from the attained height of the preceding passage.
    // Fast bottom motion is expected even before enough height is reached.
    parameter signed [31:0] RECOVERY_MID_HEIGHT_DEFICIT = 32'sd3500;
    parameter signed [31:0] RECOVERY_NEAR_HEIGHT_DEFICIT = 32'sd1800;
    parameter [7:0] RECOVERY_TOP_OBSERVE_MS = 8'd40;
    // +/-4000 ADC ~= +/-53 degrees about down. The ADC floor still applies.
    parameter signed [31:0] RECOVERY_RANGE = 32'sd4000;
    parameter [15:0] RECOVERY_PULSE_MAX_MS = 16'd150;
    parameter [31:0] RECOVERY_ARM_STEP_COUNTS = 32'd140;
    parameter [31:0] RECOVERY_PROGRESS_MIN_COUNTS = 32'd8;
    parameter signed [31:0] RECOVERY_PWM_RAMP_PER_MS = 32'sd64;
    parameter [15:0] HANG_STABLE_MS = 16'd500;
    parameter signed [31:0] HANG_RANGE = 32'sd500;
    // The pendulum reaches upright about 120-180 ms after leaving the ADC gap.
    // Keep a longer clean interval before an active bottom-crossing kick.
    parameter [7:0] CATCH_RECOVERY_MS = 8'd120;
    parameter [7:0] RECOVERY_SENSOR_GOOD_MS = 8'd80;
    // Enter balance within +/-1500 ADC counts (~20 degrees around upright).
    // Keep this below CENTER_RANGE so entry does not immediately exit.
    parameter signed [31:0] CATCH_RANGE = 32'sd1500;
    // In the outer part of the catch window, enter only while approaching
    // upright. Departing at the edge has repeatedly led to immediate fall.
    parameter signed [31:0] CATCH_OUTWARD_RANGE = 32'sd1100;
    parameter signed [31:0] CATCH_MAX_DELTA_5MS = 32'sd300;
    parameter [15:0] CATCH_STABLE_MS = 16'd10;
    parameter [15:0] COAST_TIMEOUT_MS = 16'd10000;
    // A failed swing must settle at the hanging point before a new attempt.
    parameter [15:0] RETRY_SETTLE_MS = 16'd1000;
    // Optional maximum arm rotation from the swing-up starting azimuth.
    // Zero disables it: the actual rotary arm can rotate continuously.
    parameter signed [31:0] ARM_EXCURSION_LIMIT_COUNTS = 32'sd0;
    parameter signed [31:0] ADC_MIN_VALID = 32'sd1000;
    parameter signed [31:0] ADC_MAX_VALID = 32'sd25500;
    parameter signed [31:0] ADC_MAX_STEP = 32'sd1500;
    parameter [7:0] ADC_TIMEOUT_MS = 8'd30;

    parameter signed [31:0] KP_ANG = 32'sd240;
    parameter signed [31:0] KI_ANG = 32'sd1;
    parameter signed [31:0] KD_ANG = 32'sd1800;
    parameter signed [31:0] DERIV_MAX = 32'sd120000;
    // Quiet 50 ms speed at rest, fast 20 ms path for large disturbances.
    // Attraction is limited independently so velocity braking keeps authority.
    parameter signed [31:0] KP_POS = 32'sd240;
    parameter signed [31:0] KP_POS_NEAR = 32'sd80;
    // Raw-error boundary = 3-count deadband + 60-count soft range = 63.
    parameter signed [31:0] POS_SOFT_RANGE = 32'sd60;
    parameter signed [31:0] KD_POS = 32'sd2500;
    parameter signed [31:0] MOVE_KD_POS = 32'sd3000;
    parameter signed [31:0] POS_DEADBAND = 32'sd3;
    parameter POS_HOLD_I_ENABLE = 0; // Stop-first profile: do not chase static position error.
    parameter QUIET_HOLD_ENABLE = 1;
    parameter QUIET_HOLD_QUALIFY_SAMPLES = 20;
    parameter signed [31:0] QUIET_HOLD_ANGLE_SPAN = 32'sd20;
    parameter signed [31:0] QUIET_HOLD_EXIT_ANGLE = 32'sd30;
    parameter signed [31:0] QUIET_HOLD_EXIT_COUNTS = 32'sd4;
    parameter signed [31:0] POS_HOLD_I_DEADBAND = 32'sd12;
    parameter signed [31:0] POS_HOLD_I_RANGE = 32'sd180;
    parameter signed [31:0] POS_HOLD_I_LIMIT_ADC = 32'sd160;
    parameter signed [31:0] POS_HOLD_I_STEP_MAX = 32'sd64;
    parameter POS_HOLD_I_QUALIFY_SAMPLES = 20;
    parameter signed [31:0] BALANCE_TRIM_ADC = 32'sd0;
    parameter REFERENCE_SETTLING_ENABLE = 1;
    parameter signed [31:0] POS_P_OFFSET_MAX = 32'sd200;
    parameter signed [31:0] MOVE_POS_P_OFFSET_MAX = 32'sd360;
    parameter signed [31:0] MOVE_POS_OFFSET_LIMIT = 32'sd650;
    parameter signed [31:0] FAST_SPEED_MIN_20MS = 32'sd6;
    parameter ADAPTIVE_RETURN_ENABLE = 0; // retired short-window/gain-switch experiment
    parameter signed [31:0] RETURN_BOOST_START_COUNTS = 32'sd18;
    parameter RETURN_BOOST_SHIFT = 7;
    parameter signed [31:0] RETURN_P_OFFSET_MAX = 32'sd360;
    parameter signed [31:0] RETURN_BRAKE_RANGE_COUNTS = 32'sd180;
    parameter signed [31:0] RETURN_BRAKE_SPEED_Q8 = 32'sd512;
    parameter PREDICTIVE_BRAKE_ENABLE = 1;
    parameter signed [31:0] RETURN_STOP_MARGIN_COUNTS = 32'sd12;
    parameter RETURN_BRAKE_DISTANCE_SHIFT = 1;
    parameter [3:0] RETURN_BRAKE_RAMP_STEP = 4'd2;
    parameter QUICK_RESPONSE_ENABLE = 1;
    parameter signed [31:0] DISTURBANCE_MIN_10MS = 32'sd2;
    parameter signed [31:0] RESPONSE_TARGET_SLEW = 32'sd75;
    parameter POS_VEL_FILTER_SHIFT = 0; // Rolling 50 ms speed, no extra IIR lag.
    parameter MOVE_VEL_FILTER_SHIFT = 1; // Mild smoothing only during K1 turns.
    parameter signed [31:0] POS_OFFSET_LIMIT = 32'sd500;
    parameter signed [31:0] TARGET_SLEW = 32'sd25;
    parameter signed [31:0] MOVE_TARGET_SLEW = 32'sd50;
    parameter DERIV_FILTER_SHIFT = 1;
    parameter signed [31:0] INT_SEP_ERR = 32'sd200;
    parameter signed [31:0] ANG_INT_MAX = 32'sd4000;
    parameter signed [31:0] INT_DEADZONE = 32'sd5;

    // Arm transport: 1040 quadrature counts/output revolution (calibrate on
    // the real arm). Positive encoder direction is provisionally clockwise
    // when viewed from above; change the sign if the wiring is reversed.
    parameter signed [31:0] ARM_COUNTS_PER_REV = 32'sd1040;
    parameter signed [31:0] CW_ENCODER_SIGN = 32'sd1;
    // Q16 counts/ms: 17300 ~= 264 counts/s, about 91.4 degrees/s.
    // With 1040 counts/rev, 1.08 s ramps plus cruise take about 5 s total.
    // The working super_good balance controller is retained unchanged.
    parameter [15:0] TURN_CRUISE_Q16 = 16'd17300;
    parameter [15:0] TURN_ACCEL_Q16 = 16'd16;
    parameter integer K1_COLLECT_MS=1000;
    parameter integer K1_LONG_MS=600;
    parameter [15:0] K1_STOP_DECEL_Q16=16'd64;
    parameter [15:0] K1_MAX_STEPS=16'hffff;
    // LED-only stability qualification, independent of the original azimuth.
    parameter signed [31:0] LED_UPRIGHT_RANGE_ADC = 32'sd300;
    parameter signed [31:0] LED_ANGLE_SPAN_ADC = 32'sd64;
    parameter signed [31:0] LED_SETTLE_BAND_COUNTS = 32'sd5;
    parameter [15:0] LED_SETTLE_MS = 16'd500;
    parameter [15:0] LED_MODE_BLINK_MS = 16'd150;
    parameter [15:0] LED_MODE_PAUSE_MS = 16'd550; // on/off/on/pause = 1 second

    localparam [2:0] ST_WAIT = 3'd0, ST_KICK_1 = 3'd1,
                     ST_KICK_2 = 3'd2, ST_COAST = 3'd3,
                     ST_BALANCE = 3'd4, ST_FAULT = 3'd5, ST_RECOVER = 3'd6,
                     ST_IDLE = 3'd7;
    reg [2:0] state = REQUIRE_START_KEY ? ST_IDLE : ST_WAIT;
    reg [7:0] correction_kicks = 0;
    reg recovery_armed = 1'b1;
    reg recovery_energy_hold = 1'b0;
    reg recovery_hold_negative = 1'b0;
    reg signed [31:0] recovery_peak = 0;
    reg signed [31:0] recovery_power_limit = RECOVERY_PWM_MAX;
    // A partial return can be manually caught but cannot start another kick.
    reg retry_home_ok = 1'b1;

    reg [2:0] key1_sync = 0, key2_sync = 0;
    always @(posedge clk) begin
        key1_sync <= {key1_sync[1:0], key1};
        key2_sync <= {key2_sync[1:0], key2};
    end
    wire board_restart_pressed = key1_sync[2] || key2_sync[2];
    reg tick_1ms = 0;
    // Power-on/configuration is disarmed. Only a qualified BOARD reset-key
    // press may arm it. While the key is held, restart still inhibits PWM;
    // normal swing-up begins only after release and sensor qualification.
    reg run_armed = !REQUIRE_START_KEY;
    reg [7:0] start_key_ms = 0;
    always @(posedge clk) begin
        if(tick_1ms && !run_armed) begin
            if(!board_restart_pressed) start_key_ms<=0;
            else if(start_key_ms>=START_KEY_HOLD_MS-1) begin
                run_armed<=1'b1;start_key_ms<=0;
            end else start_key_ms<=start_key_ms+1'b1;
        end
    end

    // K3 repeats the liuxingchui swing-up/phase-pumping actions. It does not
    // request a yaw speed or capture upright balance until explicit exit.
    // A 5ms neutral interval clears loop history and motion planners.
    // Only physical board keys can arm power-on startup or reset the encoder.
    wire angle_keys_owned,balance_valid,vision_braking,vision_engaged,vision_hold_commit;
    reg turn_active=0;
    reg [2:0] k3_sync=3'b111;
    reg k3_released=1'b1,meteor_mode=1'b0;
    reg windmill_mode=1'b0;
    wire signed [31:0] windmill_command,windmill_center;
    wire windmill_fault,windmill_preparing,windmill_reversing,windmill_acquiring;
    wire windmill_owned=windmill_mode && !windmill_acquiring;
    reg windmill_was_owned=0;
    reg [4:0] k3_debounce_ms=0;
    reg [2:0] mode_restart_ms=0;
    reg meteor_wait_bottom=1'b0;
    reg [15:0] meteor_bottom_ms=0;
    reg meteor_exit_pending=0;
    reg signed [31:0] meteor_rest_min=0,meteor_rest_max=0;
    wire meteor_static_sample_ok,meteor_motion_start_ready,meteor_rest_start_ready;
    wire meteor_transition_active;
    wire signed [31:0] meteor_rest_angle,meteor_min_next,meteor_max_next;
    wire k3_button_press=METEOR_ENABLE && run_armed && !board_restart_pressed &&
        tick_1ms && k3_released && !k3_sync[2] && k3_debounce_ms==19;
    wire k3_press=k3_button_press && !angle_keys_owned;
    // Meteor -> windmill keeps the running swing actuator and its phase
    // history until qualified rotation takeover. Resetting here interrupted
    // an existing recovery pulse and made the new mode start searching again.
    wire k3_windmill_entry=k3_press && meteor_mode && !windmill_mode &&
        !meteor_wait_bottom && state!=ST_FAULT;
    wire restart_pressed=board_restart_pressed || (k3_press && !k3_windmill_entry) || mode_restart_ms!=0;
    always @(posedge clk) begin
        if(restart_pressed)windmill_was_owned<=0;
        else if(tick_1ms)windmill_was_owned<=windmill_owned;
    end
    always @(posedge clk) begin
        k3_sync<={k3_sync[1:0],k3_meteor_n};
        if(board_restart_pressed) begin
            meteor_mode<=0;windmill_mode<=0;mode_restart_ms<=0;k3_released<=1;k3_debounce_ms<=0;
            meteor_wait_bottom<=0;meteor_bottom_ms<=0;
            meteor_exit_pending<=0;
        end else if(tick_1ms) begin
            if(k3_sync[2]==k3_released) k3_debounce_ms<=0;
            else if(k3_debounce_ms==19) begin
                k3_released<=k3_sync[2];k3_debounce_ms<=0;
            end else k3_debounce_ms<=k3_debounce_ms+1'b1;
            if(k3_press) begin
                // K3: normal -> original meteor -> bounded-arm windmill -> normal.
                meteor_mode<=!windmill_mode;
                windmill_mode<=meteor_mode && !windmill_mode;
                mode_restart_ms<=k3_windmill_entry ? 0:5;
                // Entry must start from down even if already balanced at top.
                // Exit immediately restores ordinary swing-up/catch behavior.
                meteor_wait_bottom<=!meteor_mode;meteor_bottom_ms<=0;
                meteor_exit_pending<=windmill_mode;
            end else begin
                if(mode_restart_ms!=0) mode_restart_ms<=mode_restart_ms-1'b1;
                // Fast fallback on exit belongs only to this handoff. Once
                // upright has been recaptured, ordinary fall recovery resumes.
                if(!meteor_mode && state==ST_BALANCE) meteor_exit_pending<=0;
                // Qualify quiet lower poses with a range check, rather than
                // requiring every5ms difference to stay inside +/-8 forever.
                // The same fallback prevents a post-exit coast from waiting
                // indefinitely when a short softened pulse did not move it.
                if(mode_restart_ms==0 && meteor_transition_active && meteor_static_sample_ok &&
                   (meteor_wait_bottom || state==ST_WAIT || state==ST_COAST)) begin
                    if(meteor_bottom_ms==0) begin
                        meteor_bottom_ms<=1;meteor_rest_min<=meteor_rest_angle;meteor_rest_max<=meteor_rest_angle;
                    end else if(meteor_max_next-meteor_min_next>METEOR_STATIC_SPAN) begin
                        meteor_bottom_ms<=1;meteor_rest_min<=meteor_rest_angle;meteor_rest_max<=meteor_rest_angle;
                    end else begin
                        meteor_rest_min<=meteor_min_next;meteor_rest_max<=meteor_max_next;
                        if(meteor_bottom_ms<METEOR_BOTTOM_STABLE_MS) meteor_bottom_ms<=meteor_bottom_ms+1'b1;
                    end
                end else meteor_bottom_ms<=0;
                if(meteor_wait_bottom && mode_restart_ms==0 &&
                   (meteor_motion_start_ready || meteor_rest_start_ready)) meteor_wait_bottom<=0;
            end
        end
    end
    wire signed [31:0] selected_recovery_range=meteor_mode ? 32'sd5000:RECOVERY_RANGE;
    wire [15:0] selected_recovery_ms=meteor_mode ? 16'd180:RECOVERY_PULSE_MAX_MS;
    wire [31:0] selected_recovery_step=meteor_mode ? 32'd160:RECOVERY_ARM_STEP_COUNTS;
    wire signed [31:0] selected_retry_delta=meteor_mode ? RECOVERY_MIN_DELTA_5MS:RETRY_MAX_DELTA_5MS;

    // K1 is a separate active-low switch, not either board reset button.
    // Synchronize it and require 20 consecutive 1 ms samples per edge.
    reg [2:0] k1_sync = 3'b111;
    reg k1_released = 1'b1;
    reg [4:0] k1_debounce_ms = 0;
    always @(posedge clk) begin
        k1_sync <= {k1_sync[1:0], k1_turn_n};
        if (tick_1ms) begin
            if (k1_sync[2] == k1_released) k1_debounce_ms <= 0;
            else if (k1_debounce_ms == 5'd19) begin
                k1_released <= k1_sync[2];
                k1_debounce_ms <= 0;
            end else k1_debounce_ms <= k1_debounce_ms + 1'b1;
        end
    end
    wire k1_button_press = run_armed && !meteor_mode && !restart_pressed && tick_1ms && k1_released && !k1_sync[2] &&
                    k1_debounce_ms == 5'd19;
    wire k1_press=k1_button_press && !angle_keys_owned;

    // External K2 uses P53 and GND, independent of board S2/P87 reset.
    reg [2:0] k2_sync=3'b111;
    reg k2_released=1;
    reg [4:0] k2_debounce_ms=0;
    always @(posedge clk) begin
        k2_sync<={k2_sync[1:0],k2_track_n};
        if(restart_pressed) begin k2_released<=1;k2_debounce_ms<=0;end
        else if(tick_1ms) begin
            if(k2_sync[2]==k2_released) k2_debounce_ms<=0;
            else if(k2_debounce_ms==19) begin
                k2_released<=k2_sync[2];k2_debounce_ms<=0;
            end else k2_debounce_ms<=k2_debounce_ms+1'b1;
        end
    end
    wire k2_button_press=run_armed && !meteor_mode && !restart_pressed && tick_1ms && k2_released && !k2_sync[2] && k2_debounce_ms==19;
    wire k2_press=k2_button_press && !angle_keys_owned;

    // K4 owns the three external keys only while entering an angle. A held
    // key is one press; both press and release must remain stable for20ms.
    reg [2:0] k4_sync=3'b111;
    reg k4_released=1'b1;
    reg [4:0] k4_debounce_ms=0;
    always @(posedge clk) begin
        k4_sync<={k4_sync[1:0],k4_angle_n};
        if(board_restart_pressed) begin k4_released<=1;k4_debounce_ms<=0;end
        else if(tick_1ms) begin
            if(k4_sync[2]==k4_released) k4_debounce_ms<=0;
            else if(k4_debounce_ms==19) begin
                k4_released<=k4_sync[2];k4_debounce_ms<=0;
            end else k4_debounce_ms<=k4_debounce_ms+1'b1;
        end
    end
    wire k4_press=ANGLE_SET_ENABLE && run_armed && !board_restart_pressed &&
        mode_restart_ms==0 && tick_1ms && k4_released && !k4_sync[2] && k4_debounce_ms==19;
    reg angle_setting=0,angle_command_pending=0,angle_dividing=0;
    reg [9:0] angle_degrees=0;
    // K4 has priority over simultaneous K1/K2/K3. Cancel, never execute,
    // unfinished entry on reset, fall, fault, or loss of sensor validity.
    assign angle_keys_owned=ANGLE_SET_ENABLE &&
        (angle_setting || angle_command_pending || angle_dividing || k4_press);
    wire angle_enter=k4_press && !angle_setting && !angle_command_pending &&
        !angle_dividing && balance_valid && !turn_active && !vision_braking;
    wire angle_confirm=k4_press && angle_setting;
    // Read-only telemetry counters; never feed the controller or planner.
    reg [15:0] setting_k1_presses=0,setting_k2_presses=0,setting_k3_presses=0;
    always @(posedge clk) begin
        if(restart_pressed || !balance_valid || angle_enter || angle_confirm) begin
            setting_k1_presses<=0;setting_k2_presses<=0;setting_k3_presses<=0;
        end else if(tick_1ms && angle_setting) begin
            if(k1_button_press && setting_k1_presses!=16'hffff) setting_k1_presses<=setting_k1_presses+1'b1;
            if(k2_button_press && setting_k2_presses!=16'hffff) setting_k2_presses<=setting_k2_presses+1'b1;
            if(k3_button_press && setting_k3_presses!=16'hffff) setting_k3_presses<=setting_k3_presses+1'b1;
        end
    end
    wire [10:0] angle_sum={1'b0,angle_degrees}+
        (k1_button_press ? 11'd100:11'd0)+
        (k2_button_press ? 11'd10:11'd0)+
        (k3_button_press ? 11'd1:11'd0);
    // Sequential unsigned division avoids a large combinational divider or
    // extra DSP: degrees*counts_per_rev*65536/360, full turns retained.
    reg [35:0] angle_dividend=0,angle_quotient=0;
    reg [8:0] angle_remainder=0;
    reg [5:0] angle_bit=0;
    reg signed [47:0] angle_distance_q16=0;
    wire [9:0] angle_trial={angle_remainder,angle_dividend[35]};
    wire angle_subtract=angle_trial>=10'd360;
    wire [9:0] angle_rem_next=angle_subtract ? angle_trial-10'd360:angle_trial;
    wire [35:0] angle_quotient_next={angle_quotient[34:0],angle_subtract};
    // Constant-coefficient shift/add, modulo20 bits exactly as the product.
    // With1040 counts/rev this is only (degrees<<10)+(degrees<<4).
    function [19:0] angle_counts_product(input [9:0] degrees);
        integer bit_number;
        reg [19:0] extended_degrees,accumulator;
        begin
            extended_degrees={10'd0,degrees};accumulator=0;
            for(bit_number=0;bit_number<20;bit_number=bit_number+1)
                if((ARM_COUNTS_PER_REV >> bit_number) & 1)
                    accumulator=accumulator+(extended_degrees << bit_number);
            angle_counts_product=accumulator;
        end
    endfunction
    wire [19:0] angle_count_product=angle_counts_product(angle_degrees);
    wire angle_execute=tick_1ms && balance_valid && angle_command_pending &&
        !angle_dividing && !turn_active && !vision_engaged && !vision_hold_commit;
    always @(posedge clk) begin
        if(restart_pressed || !balance_valid) begin
            angle_setting<=0;angle_degrees<=0;angle_command_pending<=0;
            angle_dividing<=0;angle_distance_q16<=0;
            angle_dividend<=0;angle_quotient<=0;angle_remainder<=0;angle_bit<=0;
        end else begin
            if(angle_enter) begin angle_setting<=1;angle_degrees<=0;end
            else if(angle_confirm) begin
                angle_setting<=0;
                if(angle_degrees!=0) begin
                    angle_command_pending<=1;angle_dividing<=1;angle_bit<=35;
                    angle_dividend<={angle_count_product,16'd0};
                    angle_quotient<=0;angle_remainder<=0;
                end
            end else if(tick_1ms && angle_setting)
                angle_degrees<=angle_sum>=1000 ? angle_sum-11'd1000:angle_sum;
            if(angle_dividing) begin
                angle_dividend<={angle_dividend[34:0],1'b0};
                angle_remainder<=angle_rem_next[8:0];angle_quotient<=angle_quotient_next;
                if(angle_bit==0) begin
                    angle_distance_q16<=$signed({12'd0,angle_quotient_next});
                    angle_dividing<=0;
                end else angle_bit<=angle_bit-1'b1;
            end
            if(angle_execute) angle_command_pending<=0;
        end
    end
    wire [15:0] adc_data;
    wire adc_done;
    ads1115 my_adc(.clk(clk), .scl(scl), .sda(sda),
                   .adc_data(adc_data), .adc_done(adc_done));
    wire signed [31:0] enc_pos;
    wire signed [15:0] enc_speed;
    encoder my_encoder(.clk(clk), .rst_n(!board_restart_pressed), .enc_a(enc_a), .enc_b(enc_b),
                       .position(enc_pos), .speed(enc_speed));

    reg [14:0] ms_div = 0;
    always @(posedge clk) begin
        if (ms_div == 15'd26999) begin
            ms_div <= 0;
            tick_1ms <= 1'b1;
        end else begin
            ms_div <= ms_div + 1'b1;
            tick_1ms <= 1'b0;
        end
    end

    // Delay the ADS1115 completion strobe so the registered data is stable.
    reg [2:0] adc_done_sync = 0;
    always @(posedge clk) adc_done_sync <= {adc_done_sync[1:0], adc_done};
    wire new_adc = adc_done_sync[1] && !adc_done_sync[2];
    // The I2C state machine now uses the system clock with a slow enable.
    // Capture the whole bus after its completion strobe before control uses it.
    reg [15:0] adc_sample = 0;
    wire signed [31:0] raw_angle = $signed({1'b0, adc_sample});
    wire raw_valid = (raw_angle >= ADC_MIN_VALID && raw_angle <= ADC_MAX_VALID);
    reg adc_seen = 0;
    reg check_adc = 0;
    reg [7:0] adc_age_ms = 0;
    reg signed [31:0] last_raw = 0;
    reg sensor_fault = 0;
    wire signed [31:0] adc_step = raw_angle - last_raw;
    wire jump_fault = adc_seen &&
                      (adc_step > ADC_MAX_STEP || adc_step < -ADC_MAX_STEP);
    always @(posedge clk) begin
        check_adc <= new_adc;
        if (new_adc) begin
            adc_sample <= adc_data;
            adc_seen <= 1'b1;
            adc_age_ms <= 0;
        end else if (tick_1ms && adc_age_ms != 8'hff) begin
            adc_age_ms <= adc_age_ms + 1'b1;
        end
        if (check_adc) begin
            last_raw <= raw_angle;
            sensor_fault <= !raw_valid || jump_fault;
        end
    end
    wire adc_ok = adc_seen && !sensor_fault && raw_valid &&
                  (adc_age_ms < ADC_TIMEOUT_MS);

    // A 0/full-scale flash can be followed by in-range but false readings.
    // The catch window and active recovery use separate trust thresholds.
    reg [7:0] catch_good_ms = 0;
    always @(posedge clk) begin
        if (tick_1ms) begin
            // K3 does not reconfigure the ADC. Preserve already-qualified
            // sensor history; only board reset or invalid ADC starts it over.
            if ((METEOR_ENABLE ? board_restart_pressed:restart_pressed) || !adc_ok) catch_good_ms <= 0;
            else if (catch_good_ms != 8'hff)
                catch_good_ms <= catch_good_ms + 1'b1;
        end
    end
    wire catch_sensor_trusted = catch_good_ms >= CATCH_RECOVERY_MS;
    wire recovery_sensor_trusted = catch_good_ms >= RECOVERY_SENSOR_GOOD_MS;

    reg signed [31:0] a0 = 0, a1 = 0, a2 = 0, a3 = 0;
    reg signed [31:0] angle_avg = 0;
    reg filter_ready = 0;
    reg [2:0] filter_count = 0;
    always @(posedge clk) begin
        if (tick_1ms) begin
            if (!adc_ok) begin
                filter_ready <= 0;
                filter_count <= 0;
            end else if (!filter_ready && filter_count == 0) begin
                a0 <= raw_angle; a1 <= raw_angle;
                a2 <= raw_angle; a3 <= raw_angle;
                angle_avg <= raw_angle;
                filter_count <= 1;
            end else begin
                a3 <= a2; a2 <= a1; a1 <= a0; a0 <= raw_angle;
                angle_avg <= (a0 + a1 + a2 + a3) >>> 2;
                if (!filter_ready) begin
                    if (filter_count == 3) filter_ready <= 1'b1;
                    else filter_count <= filter_count + 1'b1;
                end
            end
        end
    end

    reg signed [31:0] arm_start_pos = 0;
    reg signed [31:0] arm_balance_home = 0;
    reg signed [31:0] kick_origin = 0;
    wire signed [31:0] kick_delta = enc_pos - kick_origin;
    wire [31:0] kick_abs = kick_delta < 0 ? -kick_delta : kick_delta;
    reg [31:0] return_start_abs = 0;
    reg kick_dir_known = 0;
    reg kick_enc_positive = 0;
    wire kick_crossed_origin = kick_dir_known &&
        ((kick_delta >= 0) != kick_enc_positive);
    wire signed [31:0] arm_excursion = enc_pos - arm_start_pos;
    wire arm_limit_fault = (ARM_EXCURSION_LIMIT_COUNTS > 0) &&
        (arm_excursion > ARM_EXCURSION_LIMIT_COUNTS ||
         arm_excursion < -ARM_EXCURSION_LIMIT_COUNTS);

    reg [15:0] state_ms = 0;
    reg [15:0] hang_ms = 0;
    reg [15:0] catch_ms = 0;
    reg [15:0] coast_ms = 0;
    reg [1:0] recovery_no_progress=0;
    reg [3:0] fault_reason=0;
    reg kick_sign = 1; // 1: positive kick first; 0: negative first.
    reg [2:0] sample_5ms = 0;
    reg signed [31:0] last_angle_5ms = 0;
    reg signed [31:0] delta_5ms = 0;
    reg signed [31:0] previous_delta_5ms = 0;
    wire near_upright = angle_avg >= TARGET_ANGLE - CATCH_RANGE &&
                        angle_avg <= TARGET_ANGLE + CATCH_RANGE;
    wire slow_enough = delta_5ms <= CATCH_MAX_DELTA_5MS &&
                       delta_5ms >= -CATCH_MAX_DELTA_5MS;
    wire catch_motion_ok =
        (angle_avg >= TARGET_ANGLE - CATCH_OUTWARD_RANGE &&
         angle_avg <= TARGET_ANGLE + CATCH_OUTWARD_RANGE) ||
        (angle_avg < TARGET_ANGLE && delta_5ms >= 0) ||
        (angle_avg > TARGET_ANGLE && delta_5ms <= 0);
    wire catch_ready = !meteor_mode && catch_sensor_trusted && near_upright &&
                       slow_enough && catch_motion_ok;
    wire near_hanging = angle_avg >= HANG_ANGLE - HANG_RANGE &&
                        angle_avg <= HANG_ANGLE + HANG_RANGE;
    wire arm_quiet = enc_speed <= RETRY_MAX_ARM_SPEED &&
                     enc_speed >= -RETRY_MAX_ARM_SPEED;
    wire recovery_arm_ok = enc_speed <= RECOVERY_MAX_ARM_SPEED &&
                           enc_speed >= -RECOVERY_MAX_ARM_SPEED;
    wire recovery_running_arm_ok = enc_speed <= RECOVERY_RUN_MAX_ARM_SPEED &&
                                   enc_speed >= -RECOVERY_RUN_MAX_ARM_SPEED;
    // Require agreement of two independent 5 ms differences before pushing.
    wire recovery_positive = delta_5ms >= RECOVERY_MIN_DELTA_5MS &&
        delta_5ms <= RECOVERY_MAX_DELTA_5MS &&
        previous_delta_5ms >= RECOVERY_MIN_DELTA_5MS &&
        previous_delta_5ms <= RECOVERY_MAX_DELTA_5MS;
    wire recovery_negative = delta_5ms <= -RECOVERY_MIN_DELTA_5MS &&
        delta_5ms >= -RECOVERY_MAX_DELTA_5MS &&
        previous_delta_5ms <= -RECOVERY_MIN_DELTA_5MS &&
        previous_delta_5ms >= -RECOVERY_MAX_DELTA_5MS;
    wire in_recovery_sector = angle_avg >= HANG_ANGLE - selected_recovery_range &&
                              angle_avg <= HANG_ANGLE + selected_recovery_range;
    // Triangular excursion about down, calibrated from down to upright.
    // High ADC on the opposite side of down is NOT a near-upright height.
    // Ignore the electrical gap; never integrate it into a turn count.
    localparam signed [31:0] HALF_TURN_ADC = TARGET_ANGLE - HANG_ANGLE;
    wire signed [31:0] down_offset = angle_avg - HANG_ANGLE;
    wire signed [31:0] wrapped_down_offset = down_offset > HALF_TURN_ADC ?
                                            down_offset - (HALF_TURN_ADC << 1) :
                                            down_offset;
    wire signed [31:0] recovery_height = wrapped_down_offset < 0 ?
                                         -wrapped_down_offset : wrapped_down_offset;
    wire signed [31:0] recovery_height_cap =
        recovery_peak < HALF_TURN_ADC - RECOVERY_MID_HEIGHT_DEFICIT ? RECOVERY_PWM_MAX :
        recovery_peak < HALF_TURN_ADC - RECOVERY_NEAR_HEIGHT_DEFICIT ? RECOVERY_PWM_MID :
        RECOVERY_PWM_NEAR_TOP;
    wire fast_top_pass = adc_ok && filter_ready &&
        catch_good_ms >= RECOVERY_TOP_OBSERVE_MS && near_upright &&
        ((delta_5ms > CATCH_MAX_DELTA_5MS &&
          previous_delta_5ms > CATCH_MAX_DELTA_5MS) ||
         (delta_5ms < -CATCH_MAX_DELTA_5MS &&
          previous_delta_5ms < -CATCH_MAX_DELTA_5MS));
    wire recovery_energy_ok = meteor_mode || (!recovery_energy_hold && !fast_top_pass);
    wire recovery_start_ready = recovery_armed &&
        (meteor_mode || correction_kicks < MAX_CORRECTION_KICKS) &&
        coast_ms >= RECOVERY_COAST_MIN_MS &&
        recovery_sensor_trusted && in_recovery_sector && recovery_arm_ok &&
        recovery_energy_ok && (recovery_positive || recovery_negative);
    // Height feedback selects and latches the next passage's power. It is
    // independent of the 24-attempt counter and survives ADC invalidity.
    // A fast TOP passage pauses further pumping until measured reversal;
    // a naturally fast BOTTOM passage must not starve a low-amplitude swing.
    always @(posedge clk) begin
        if (tick_1ms) begin
            if (restart_pressed || state == ST_WAIT || state == ST_BALANCE) begin
                recovery_energy_hold <= 1'b0;
                recovery_hold_negative <= 1'b0;
                recovery_peak <= 0;
                recovery_power_limit <= RECOVERY_PWM_MAX;
            end else if (adc_ok && filter_ready &&
                         (state == ST_COAST || state == ST_RECOVER)) begin
                if (recovery_sensor_trusted && recovery_height > recovery_peak)
                    recovery_peak <= recovery_height;
                if (state == ST_COAST && recovery_start_ready) begin
                    recovery_power_limit <= recovery_height_cap;
                    recovery_peak <= recovery_height;
                end
                if (fast_top_pass) begin
                    recovery_energy_hold <= 1'b1;
                    // Keep the original rotation direction while holding.
                    // A return through top must not redefine what counts as
                    // a genuine reversal and postpone recovery another half turn.
                    if (!recovery_energy_hold)
                        recovery_hold_negative <= delta_5ms < 0;
                    recovery_peak <= HALF_TURN_ADC;
                end else if (recovery_energy_hold && recovery_sensor_trusted &&
                    (recovery_hold_negative ? recovery_positive : recovery_negative))
                    recovery_energy_hold <= 1'b0;
            end
        end
    end
    // Two differences qualify entry. Once pushing, use the latest measured
    // direction, so a preceding slow sample does not cut a valid fast push.
    wire recovery_direction_ok = kick_sign ?
        (delta_5ms <= -RECOVERY_MIN_DELTA_5MS && delta_5ms >= -RECOVERY_MAX_DELTA_5MS) :
        (delta_5ms >= RECOVERY_MIN_DELTA_5MS && delta_5ms <= RECOVERY_MAX_DELTA_5MS);
    wire recovery_phase_ok = in_recovery_sector && recovery_sensor_trusted &&
                             recovery_running_arm_ok && recovery_direction_ok &&
                             recovery_energy_ok;
    wire pendulum_quiet = delta_5ms <= selected_retry_delta &&
                         delta_5ms >= -selected_retry_delta;
    // K3 may join a moving lower passage after two agreeing5ms samples.
    // No complete stop is required. Full initial kick is reserved for a
    // quiet lower pose, with a bounded angle spread over80ms.
    assign meteor_transition_active=METEOR_ENABLE && (meteor_mode || meteor_exit_pending);
    assign meteor_rest_angle=angle_avg;
    assign meteor_min_next=angle_avg<meteor_rest_min ? angle_avg:meteor_rest_min;
    assign meteor_max_next=angle_avg>meteor_rest_max ? angle_avg:meteor_rest_max;
    assign meteor_motion_start_ready=meteor_transition_active && adc_ok && filter_ready &&
        recovery_sensor_trusted && in_recovery_sector && recovery_arm_ok &&
        (recovery_positive || recovery_negative);
    assign meteor_static_sample_ok=adc_ok && filter_ready && recovery_sensor_trusted && arm_quiet &&
        angle_avg>=HANG_ANGLE-METEOR_STATIC_RANGE && angle_avg<=HANG_ANGLE+METEOR_STATIC_RANGE &&
        delta_5ms<=RETRY_MAX_DELTA_5MS && delta_5ms>=-RETRY_MAX_DELTA_5MS;
    assign meteor_rest_start_ready=meteor_transition_active && meteor_static_sample_ok &&
        meteor_bottom_ms>=METEOR_BOTTOM_STABLE_MS &&
        meteor_max_next-meteor_min_next<=METEOR_STATIC_SPAN;
    wire signed [31:0] meteor_release_command;
    wire meteor_release_active,meteor_release_exhausted;
    meteor_upright_release #(.PWM(METEOR_RELEASE_PWM),.MAX_MS(METEOR_RELEASE_MAX_MS),
        .TRAVEL_COUNTS(METEOR_RELEASE_TRAVEL_COUNTS),.POLARITY(SWING_POLARITY)) meteor_release(
        .clk(clk),.tick_1ms(tick_1ms),.reset(restart_pressed),
        .enable(METEOR_ENABLE && meteor_mode && meteor_wait_bottom && run_armed),
        .sensor_valid(adc_ok && filter_ready && recovery_sensor_trusted),
        .near_top(near_upright),.quiet(pendulum_quiet),.position(enc_pos),
        .command(meteor_release_command),.driving(meteor_release_active),
        .exhausted(meteor_release_exhausted));

    assign balance_valid = run_armed && !restart_pressed && !meteor_mode && !meteor_wait_bottom && adc_ok && filter_ready &&
                         !arm_limit_fault && state == ST_BALANCE &&
                         angle_avg >= TARGET_ANGLE - CENTER_RANGE &&
                         angle_avg <= TARGET_ANGLE + CENTER_RANGE;
    // The position loop follows a moving encoder target while the angle loop
    // remains unchanged. Keep the fractional position so low-speed motion
    // does not advance in whole-count jumps.
    reg signed [47:0] turn_target_q16 = 0;
    reg signed [47:0] turn_goal_q16 = 0;
    reg [15:0] turn_speed_q16 = 0;
    reg turn_positive = 1;
    // Each debounced short press adds120deg. Execute only after a full quiet
    // second since the LAST press, keeping the complete multi-turn distance.
    reg [15:0] k1_pending_steps=0,k1_command_steps=0;
    localparam K1_TIMER_BITS=K1_COLLECT_MS>1 ? $clog2(K1_COLLECT_MS):1;
    reg [K1_TIMER_BITS-1:0] k1_idle_ms=0;
    wire k1_pending=k1_pending_steps!=0;
    localparam signed [47:0] K1_STEP_Q16=($signed(ARM_COUNTS_PER_REV)*48'sd65536)/3;
    localparam K1_HOLD_BITS=K1_LONG_MS>1 ? $clog2(K1_LONG_MS+1):1;
    reg [K1_HOLD_BITS-1:0] k1_hold_ms=0;
    reg k1_gesture=0,k1_jog_command=0;
    reg k1_jog_active=0,k1_jog_braking=0;
    reg [10:0] k1_stop_ms=0;
    reg [7:0] k1_settle_ms=0;
    reg signed [31:0] k1_stop_anchor=0;
    wire k1_long_start=tick_1ms && k1_gesture && !k1_released && !k1_sync[2] &&
        k1_hold_ms==K1_LONG_MS-1 && balance_valid && !angle_keys_owned &&
        !turn_active && !vision_braking && !k2_press;
    wire k1_execute;
    wire k1_manual_command=k1_execute || k1_long_start;
    always @(posedge clk) begin
        if(restart_pressed || !balance_valid || angle_keys_owned || k2_press) begin
            k1_gesture<=0;k1_hold_ms<=0;k1_jog_command<=0;
        end else if(tick_1ms) begin
            if(k1_press && !turn_active && !vision_braking) begin
                k1_gesture<=1;k1_hold_ms<=0;
            end else if(k1_released || k1_long_start) begin
                k1_gesture<=0;k1_hold_ms<=0;
            end else if(k1_gesture) k1_hold_ms<=k1_hold_ms+1'b1;
            if(k1_long_start) k1_jog_command<=1;
            else if(k1_execute) k1_jog_command<=0;
        end
    end
    function signed [47:0] k1_step_product(input [15:0] steps);
        integer bit_number;
        reg [47:0] extended_steps,accumulator;
        begin
            extended_steps={32'd0,steps};accumulator=0;
            for(bit_number=0;bit_number<48;bit_number=bit_number+1)
                if(K1_STEP_Q16[bit_number])
                    accumulator=accumulator+(extended_steps << bit_number);
            k1_step_product=accumulator;
        end
    endfunction
    wire signed [47:0] k1_pending_distance_q16=k1_step_product(k1_pending_steps);
    wire signed [47:0] turn_distance_q16=k1_step_product(k1_command_steps);
    wire signed [47:0] turn_remaining_q16 = turn_positive ?
        turn_goal_q16 - turn_target_q16 :
        turn_target_q16 - turn_goal_q16;
    // Stopping distance must use the actual acceleration, not the old /8.
    // With the production a=16, this constant division synthesizes to /32.
    // The extra step guards the discrete-time boundary.
    wire [31:0] turn_speed_square = turn_speed_q16 * turn_speed_q16;
    localparam [31:0] TURN_BRAKE_DIVISOR = 32'd2 * TURN_ACCEL_Q16;
    wire [31:0] turn_brake_q16 = (turn_speed_square / TURN_BRAKE_DIVISOR) +
                                  {16'd0, turn_speed_q16};
    wire [15:0] turn_accelerated =
        turn_speed_q16 >= TURN_CRUISE_Q16 - TURN_ACCEL_Q16 ?
        TURN_CRUISE_Q16 : turn_speed_q16 + TURN_ACCEL_Q16;
    // A tiny terminal crawl avoids a whole-count target jump at the end.
    wire [15:0] turn_decelerated =
        turn_speed_q16 > 16'd128 + TURN_ACCEL_Q16 ?
        turn_speed_q16 - TURN_ACCEL_Q16 : 16'd128;
    wire [15:0] turn_next_speed =
        turn_remaining_q16 <= $signed({16'd0, turn_brake_q16}) ?
        turn_decelerated : turn_accelerated;
    wire signed [47:0] turn_step_q16 =
        $signed({32'd0, turn_next_speed});
    wire vision_mode,vision_moving,vision_manual_start,vision_target_live;
    assign k1_execute=tick_1ms && balance_valid && !turn_active && !vision_braking &&
        k1_pending && k1_idle_ms==K1_COLLECT_MS-1 && k1_released &&
        !k1_press && !k2_press && !angle_keys_owned;
    always @(posedge clk) begin
        if(restart_pressed || !balance_valid) begin
            k1_pending_steps<=0;k1_command_steps<=0;k1_idle_ms<=0;
        end else if(tick_1ms) begin
            if(turn_active || vision_braking || angle_keys_owned || k1_long_start) begin
                k1_pending_steps<=0;k1_idle_ms<=0;
            end else if(k1_press) begin
                if(k1_pending_steps<K1_MAX_STEPS)k1_pending_steps<=k1_pending_steps+1'b1;
                k1_idle_ms<=0;
            end else if(k2_press) begin
                // An explicit K2 mode change cancels unexecuted input.
                k1_pending_steps<=0;k1_idle_ms<=0;
            end else if(k1_execute) begin
                k1_command_steps<=k1_pending_steps;k1_pending_steps<=0;k1_idle_ms<=0;
            end else if(k1_pending && k1_idle_ms<K1_COLLECT_MS-1) k1_idle_ms<=k1_idle_ms+1'b1;
        end
    end
    wire vision_x_accepted_valid;
    wire [15:0] vision_x_accepted;
    wire signed [31:0] vision_center,vision_velocity_q8;
    wire signed [47:0] vision_position_q16,vision_goal_q16;
    wire signed [16:0] vision_velocity_q16;
    wire signed [47:0] home_q16=$signed({{16{arm_balance_home[31]}},arm_balance_home}) <<< 16;
    wire signed [47:0] vision_relative_q16=vision_position_q16-home_q16;
    generate if(VISION_ENABLE) begin: vision_enabled
        vision_follow #(.ARM_COUNTS_PER_REV(ARM_COUNTS_PER_REV),.TRACK_SIGN(TRACK_SIGN),
            .TRACK_SPAN_DEG(TRACK_SPAN_DEG),
            .X_DEADBAND_PX(TRACK_X_DEADBAND_PX),
            .X_MEDIAN_ENABLE(TRACK_X_MEDIAN_ENABLE),
            .MAX_SPEED_Q16(TRACK_MAX_SPEED_Q16),.ACCEL_Q16(TRACK_ACCEL_Q16)) follow
            (.clk(clk),.reset(restart_pressed),.tick_1ms(tick_1ms),.balance_valid(balance_valid),
             .turn_active(turn_active),.k1_press(k1_manual_command),
             .k2_press((k2_press && !k1_press) || (angle_enter && vision_mode)),.enc_pos(enc_pos),
             .camera_update(camera_update),.camera_valid(camera_x_valid),.camera_x_q8(camera_x_q8),
             .tracking(vision_mode),.braking(vision_braking),.engaged(vision_engaged),
             .moving(vision_moving),.manual_start(vision_manual_start),.center(vision_center),
             .position_q16(vision_position_q16),.goal_q16(vision_goal_q16),
             .velocity_q16(vision_velocity_q16),.velocity_50ms_q8(vision_velocity_q8),
             .target_live(vision_target_live),.accepted_x_q8(vision_x_accepted),
             .accepted_x_valid(vision_x_accepted_valid),.hold_commit(vision_hold_commit));
    end else begin: vision_disabled
        assign vision_mode=0,vision_braking=0,vision_engaged=0,vision_moving=0;
        assign vision_manual_start=0,vision_target_live=0,vision_center=0;
        assign vision_hold_commit=0,vision_x_accepted_valid=0,vision_x_accepted=16'hffff;
        assign vision_position_q16=0,vision_goal_q16=0,vision_velocity_q16=0,vision_velocity_q8=0;
    end endgenerate
    wire signed [31:0] requested_arm_home = (vision_engaged || vision_hold_commit) ?
        $signed(vision_position_q16[47:16]) : arm_balance_home+$signed(turn_target_q16[47:16]);
    // A disturbance never creates a trajectory or relocates the hold goal.
    // Only explicit K1/vision commands supply moving position references.
    wire signed [31:0] moving_arm_home=requested_arm_home;
    // Q16 counts/ms -> Q8 counts/50 ms. The sign follows the commanded turn.
    wire signed [31:0] turn_speed_extended=$signed({16'd0,turn_speed_q16});
    wire signed [31:0] turn_speed_50ms_q8 =
        ((turn_speed_extended <<< 5)+(turn_speed_extended <<< 4)+
         (turn_speed_extended <<< 1)) >>> 8;
    wire signed [31:0] turn_planned_speed_q8 = !turn_active ? 32'sd0 :
        turn_positive ? turn_speed_50ms_q8 : -turn_speed_50ms_q8;
    wire signed [31:0] planned_arm_speed_q8=vision_engaged ? vision_velocity_q8:
        turn_active ? turn_planned_speed_q8 : 32'sd0;
    // K3 deliberately disables balance_valid while throwing the pendulum.
    // Use explicit fault qualification so an intentional K3 swing can blink.
    wire led_fault = !run_armed || restart_pressed || !adc_ok || !filter_ready ||
                     arm_limit_fault || state==ST_FAULT;
    wire led_motion_mode = angle_setting || angle_command_pending || k1_pending || turn_active || vision_engaged ||
                           meteor_mode || meteor_exit_pending;
    reg signed [31:0] led_settle_anchor = 0;
    reg [15:0] led_settle_ms = 0;
    reg led_upright_stable = 1'b0;
    reg signed [31:0] led_angle_min = 0, led_angle_max = 0;
    wire signed [31:0] led_settle_delta = enc_pos - led_settle_anchor;
    wire led_arm_still = led_settle_delta <= LED_SETTLE_BAND_COUNTS &&
                         led_settle_delta >= -LED_SETTLE_BAND_COUNTS;
    wire led_near_upright = angle_avg >= TARGET_ANGLE-LED_UPRIGHT_RANGE_ADC &&
                           angle_avg <= TARGET_ANGLE+LED_UPRIGHT_RANGE_ADC;
    wire signed [31:0] led_angle_min_next = angle_avg<led_angle_min ? angle_avg:led_angle_min;
    wire signed [31:0] led_angle_max_next = angle_avg>led_angle_max ? angle_avg:led_angle_max;
    always @(posedge clk) begin
        if (led_fault || !balance_valid || led_motion_mode || k1_press || k2_press || k3_press) begin
            led_upright_stable <= 1'b0;
            led_settle_anchor <= enc_pos;
            led_settle_ms <= 0;
            led_angle_min <= angle_avg;
            led_angle_max <= angle_avg;
        end else if (tick_1ms) begin
            if (!led_near_upright || !led_arm_still ||
                led_angle_max_next-led_angle_min_next>LED_ANGLE_SPAN_ADC) begin
                led_upright_stable <= 1'b0;
                led_settle_anchor <= enc_pos;
                led_settle_ms <= 0;
                led_angle_min <= angle_avg;
                led_angle_max <= angle_avg;
            end else begin
                led_angle_min <= led_angle_min_next;
                led_angle_max <= led_angle_max_next;
                if (led_settle_ms >= LED_SETTLE_MS - 1'b1)
                    led_upright_stable <= 1'b1;
                else led_settle_ms <= led_settle_ms + 1'b1;
            end
        end
    end
    // Two yellow pulses followed by a longer pause. Preserve the phase across
    // motion-mode handoffs; tracking still blinks with a stationary/lost target.
    reg [15:0] led_blink_ms = 0;
    reg [1:0] led_blink_phase = 0;
    wire led_blink_on = !led_blink_phase[0];
    wire [15:0] led_blink_duration = led_blink_phase==3 ? LED_MODE_PAUSE_MS:LED_MODE_BLINK_MS;
    always @(posedge clk) begin
        if (led_fault || !led_motion_mode) begin
            led_blink_ms <= 0;
            led_blink_phase <= 0;
        end else if (tick_1ms) begin
            if (led_blink_ms >= led_blink_duration - 1'b1) begin
                led_blink_ms <= 0;
                led_blink_phase <= led_blink_phase+1'b1;
            end else led_blink_ms <= led_blink_ms + 1'b1;
        end
    end
    wire [23:0] led_color_grb = led_fault ? 24'h00_20_00 :
        led_motion_mode ? (led_blink_on ? 24'h20_20_00 : 24'h00_00_00) :
        !balance_valid ? 24'h00_20_00 :
        (led_upright_stable ? 24'h20_00_00 : 24'h20_20_00);
    ws2812_status status_led(.clk(clk), .color_grb(led_color_grb),
                             .data(ws2812_data));
    wire signed [31:0] k1_relative_counts=enc_pos-arm_balance_home;
    wire signed [47:0] k1_measured_q16=$signed({{16{k1_relative_counts[31]}},k1_relative_counts}) <<< 16;
    wire signed [31:0] k1_stop_displacement=enc_pos-k1_stop_anchor;
    wire k1_arm_still=k1_stop_displacement<=2 && k1_stop_displacement>=-2;
    wire [15:0] k1_stop_speed=turn_speed_q16>K1_STOP_DECEL_Q16 ?
        turn_speed_q16-K1_STOP_DECEL_Q16:16'd0;
    always @(posedge clk) begin
        if (restart_pressed) begin
            k1_jog_active<=0;k1_jog_braking<=0;k1_stop_ms<=0;k1_settle_ms<=0;k1_stop_anchor<=enc_pos;
            turn_target_q16 <= 0;
            turn_goal_q16 <= 0;
            turn_speed_q16 <= 0;
            turn_active <= 0;
        end else if (!balance_valid) begin
            k1_jog_active<=0;k1_jog_braking<=0;k1_stop_ms<=0;k1_settle_ms<=0;k1_stop_anchor<=enc_pos;
            // A fall or sensor fault cancels the turn. A fresh catch captures
            // a new home and permits a new K1 press.
            turn_target_q16 <= 0;
            turn_goal_q16 <= 0;
            turn_speed_q16 <= 0;
            turn_active <= 0;
        end else if(vision_manual_start) begin
            turn_target_q16<=vision_relative_q16;
            turn_goal_q16<=k1_jog_command ? vision_relative_q16:vision_relative_q16+
                (CW_ENCODER_SIGN>0 ? turn_distance_q16:-turn_distance_q16);
            turn_positive<=CW_ENCODER_SIGN>0;
            turn_speed_q16<=0;turn_active<=!k1_jog_command || !k1_released;
            k1_jog_active<=k1_jog_command && !k1_released;k1_jog_braking<=0;
        end else if(vision_engaged || vision_hold_commit) begin
            k1_jog_active<=0;k1_jog_braking<=0;
            // Seamless handoff: keep the manual planner's hold target equal
            // to the tracking trajectory, including its last braking step.
            turn_target_q16<=vision_relative_q16;turn_goal_q16<=vision_relative_q16;
            turn_speed_q16<=0;turn_active<=0;
        end else if (tick_1ms) begin
            if(k1_jog_active) begin
                if(k1_released || k1_jog_braking) begin
                    // Remove position catch-up during braking; preserve the
                    // upright loop and ramp velocity to zero. Hold the actual
                    // stopping point, never the old forward trajectory.
                    k1_jog_braking<=1;
                    turn_target_q16<=k1_measured_q16;turn_goal_q16<=k1_measured_q16;
                    turn_speed_q16<=k1_stop_speed;
                    if(k1_stop_ms<1200) k1_stop_ms<=k1_stop_ms+1'b1;
                    if(k1_arm_still) begin
                        if(k1_settle_ms<150) k1_settle_ms<=k1_settle_ms+1'b1;
                    end else begin k1_settle_ms<=0;k1_stop_anchor<=enc_pos;end
                    if(k1_stop_speed==0 && (k1_settle_ms>=149 || k1_stop_ms>=1199)) begin
                        k1_jog_active<=0;k1_jog_braking<=0;turn_active<=0;
                    end
                end else begin
                    turn_speed_q16<=turn_accelerated;
                    turn_target_q16<=turn_positive ? turn_target_q16+$signed({32'd0,turn_accelerated}):
                        turn_target_q16-$signed({32'd0,turn_accelerated});
                    turn_goal_q16<=turn_positive ? turn_target_q16+$signed({32'd0,turn_accelerated}):
                        turn_target_q16-$signed({32'd0,turn_accelerated});
                    k1_stop_ms<=0;k1_settle_ms<=0;k1_stop_anchor<=enc_pos;
                end
            end else if(k1_long_start && !vision_engaged) begin
                k1_jog_active<=1;k1_jog_braking<=0;turn_active<=1;
                turn_positive<=CW_ENCODER_SIGN>0;turn_speed_q16<=0;
                k1_stop_ms<=0;k1_settle_ms<=0;k1_stop_anchor<=enc_pos;
            end else if (turn_active) begin
                if (turn_remaining_q16 <= turn_step_q16) begin
                    turn_target_q16 <= turn_goal_q16;
                    turn_speed_q16 <= 0;
                    turn_active <= 0;
                end else begin
                    turn_speed_q16 <= turn_next_speed;
                    turn_target_q16 <= turn_positive ?
                        turn_target_q16 + turn_step_q16 :
                        turn_target_q16 - turn_step_q16;
                end
            end else if (angle_execute) begin
                turn_positive <= CW_ENCODER_SIGN > 0;
                turn_goal_q16 <= turn_target_q16 +
                    (CW_ENCODER_SIGN > 0 ? angle_distance_q16 : -angle_distance_q16);
                turn_speed_q16 <= 0;turn_active <= 1'b1;
            end else if (k1_execute) begin
                turn_positive <= CW_ENCODER_SIGN > 0;
                turn_goal_q16 <= turn_target_q16 +
                    (CW_ENCODER_SIGN > 0 ? k1_pending_distance_q16 : -k1_pending_distance_q16);
                turn_speed_q16 <= 0;
                turn_active <= 1'b1;
            end
        end
    end
    wire signed [16:0] target_offset;
    wire signed [23:0] balance_cmd;
    balance_pid_core #(
        // K3 owns swing pulses and explicitly disables balance_valid. Its
        // unused alternate balance gain path can therefore be constant-folded.
        .METEOR_PROFILE_ENABLE(0),
        .TARGET_ANGLE(TARGET_ANGLE), .PWM_PERIOD(PWM_PERIOD),
        .KP_ANG(KP_ANG), .KI_ANG(KI_ANG), .KD_ANG(KD_ANG),
        .DERIV_MAX(DERIV_MAX), .KP_POS(KP_POS), .KD_POS(KD_POS),
        .MOVE_KD_POS(MOVE_KD_POS),
        .KP_POS_NEAR(KP_POS_NEAR), .POS_SOFT_RANGE(POS_SOFT_RANGE),
        .POS_DEADBAND(POS_DEADBAND),
        .POS_HOLD_I_ENABLE(POS_HOLD_I_ENABLE),
        .QUIET_HOLD_ENABLE(QUIET_HOLD_ENABLE),
        .QUIET_HOLD_QUALIFY_SAMPLES(QUIET_HOLD_QUALIFY_SAMPLES),
        .QUIET_HOLD_ANGLE_SPAN(QUIET_HOLD_ANGLE_SPAN),
        .QUIET_HOLD_EXIT_ANGLE(QUIET_HOLD_EXIT_ANGLE),
        .QUIET_HOLD_EXIT_COUNTS(QUIET_HOLD_EXIT_COUNTS),
        .POS_HOLD_I_DEADBAND(POS_HOLD_I_DEADBAND),
        .POS_HOLD_I_RANGE(POS_HOLD_I_RANGE),
        .POS_HOLD_I_LIMIT_ADC(POS_HOLD_I_LIMIT_ADC),
        .POS_HOLD_I_STEP_MAX(POS_HOLD_I_STEP_MAX),
        .POS_HOLD_I_QUALIFY_SAMPLES(POS_HOLD_I_QUALIFY_SAMPLES),
        .REFERENCE_SETTLING_ENABLE(REFERENCE_SETTLING_ENABLE),
        .BALANCE_TRIM_ADC(BALANCE_TRIM_ADC),
        .POS_P_OFFSET_MAX(POS_P_OFFSET_MAX),
        .MOVE_POS_P_OFFSET_MAX(MOVE_POS_P_OFFSET_MAX),
        .FAST_SPEED_MIN_20MS(FAST_SPEED_MIN_20MS),
        .ADAPTIVE_RETURN_ENABLE(ADAPTIVE_RETURN_ENABLE),
        .RETURN_BOOST_START_COUNTS(RETURN_BOOST_START_COUNTS),
        .RETURN_BOOST_SHIFT(RETURN_BOOST_SHIFT),
        .RETURN_P_OFFSET_MAX(RETURN_P_OFFSET_MAX),
        .RETURN_BRAKE_RANGE_COUNTS(RETURN_BRAKE_RANGE_COUNTS),
        .RETURN_BRAKE_SPEED_Q8(RETURN_BRAKE_SPEED_Q8),
        .PREDICTIVE_BRAKE_ENABLE(PREDICTIVE_BRAKE_ENABLE),
        .RETURN_STOP_MARGIN_COUNTS(RETURN_STOP_MARGIN_COUNTS),
        .RETURN_BRAKE_DISTANCE_SHIFT(RETURN_BRAKE_DISTANCE_SHIFT),
        .RETURN_BRAKE_RAMP_STEP(RETURN_BRAKE_RAMP_STEP),
        .QUICK_RESPONSE_ENABLE(QUICK_RESPONSE_ENABLE),
        .DISTURBANCE_MIN_10MS(DISTURBANCE_MIN_10MS),
        .RESPONSE_TARGET_SLEW(RESPONSE_TARGET_SLEW),
        .POS_VEL_FILTER_SHIFT(POS_VEL_FILTER_SHIFT),
        .MOVE_VEL_FILTER_SHIFT(MOVE_VEL_FILTER_SHIFT),
        .INT_SEP_ERR(INT_SEP_ERR), .ANG_INT_MAX(ANG_INT_MAX),
        .INT_DEADZONE(INT_DEADZONE),
        .TARGET_OFFSET_MAX(POS_OFFSET_LIMIT),
        .MOVE_TARGET_OFFSET_MAX(MOVE_POS_OFFSET_LIMIT), .TARGET_SLEW(TARGET_SLEW),
        .MOVE_TARGET_SLEW(MOVE_TARGET_SLEW),
        .DERIV_FILTER_SHIFT(DERIV_FILTER_SHIFT)
    ) my_balance_pid (
        .clk(clk), .tick_1ms(tick_1ms), .enable(balance_valid),.meteor_mode(meteor_mode),
        .angle_avg(angle_avg), .enc_pos(enc_pos),
        .arm_home(moving_arm_home), .turn_active(turn_active || vision_moving),
        .arm_velocity_target_q8(planned_arm_speed_q8),
        .capture_delta_5ms(delta_5ms),
        .target_offset(target_offset), .angle_out(), .balance_cmd(balance_cmd)
    );
    // Later corrections have bounded extra torque while the arm displacement
    // and per-leg time limits remain unchanged.
    wire signed [31:0] boosted_swing_pwm = SWING_PWM +
        SWING_BOOST_PER_KICK * $signed({24'd0, correction_kicks});
    wire signed [31:0] swing_pwm_now = boosted_swing_pwm > SWING_PWM_MAX ?
        SWING_PWM_MAX : boosted_swing_pwm;
    wire signed [31:0] swing_first = SWING_POLARITY * swing_pwm_now;
    wire signed [31:0] normal_recovery_ramp =
        $signed({16'd0, state_ms}) * RECOVERY_PWM_RAMP_PER_MS;
    wire signed [31:0] recovery_ramp = meteor_mode ?
        ($signed({16'd0,state_ms}) <<< 7) : normal_recovery_ramp;
    wire signed [31:0] selected_power_limit=meteor_mode ? swing_pwm_now:recovery_power_limit;
    wire signed [31:0] recovery_pwm = recovery_ramp > selected_power_limit ?
                                      selected_power_limit : recovery_ramp;
    wire signed [31:0] recovery_first = SWING_POLARITY * recovery_pwm;

    always @(posedge clk) begin
        if (tick_1ms) begin
            if (restart_pressed || !adc_ok || !filter_ready) begin
                sample_5ms <= 0;
                last_angle_5ms <= angle_avg;
                delta_5ms <= 0;
                previous_delta_5ms <= 0;
            end else if (sample_5ms == 4) begin
                sample_5ms <= 0;
                previous_delta_5ms <= delta_5ms;
                delta_5ms <= angle_avg - last_angle_5ms;
                last_angle_5ms <= angle_avg;
            end else sample_5ms <= sample_5ms + 1'b1;

            if (restart_pressed || !run_armed) begin
                state <= !run_armed ? ST_IDLE:
                    (!board_restart_pressed && (k3_press ? meteor_mode:meteor_exit_pending) &&
                     !(near_hanging && pendulum_quiet && arm_quiet)) ? ST_COAST:ST_WAIT;
                correction_kicks <= 0;
                recovery_armed <= 1'b1;
                retry_home_ok <= 1'b1;
                state_ms <= 0; coast_ms <= 0;
                hang_ms <= 0; catch_ms <= 0;
                arm_start_pos <= 0; arm_balance_home <= 0;
                kick_origin <= 0;
                return_start_abs <= 0;
                kick_dir_known <= 0; kick_enc_positive <= 0;
                recovery_no_progress<=0;fault_reason<=0;
            end else if(windmill_owned) begin
                // Dedicated phase pumping owns the actuator. Never capture
                // upright or execute the meteor's large-displacement kicks.
                state<=windmill_fault ? ST_FAULT:ST_COAST;
                fault_reason<=windmill_fault ? 4'd6:4'd0;
                state_ms<=0;coast_ms<=0;hang_ms<=0;catch_ms<=0;
                arm_start_pos<=windmill_center;arm_balance_home<=windmill_center;
                correction_kicks<=0;recovery_armed<=0;
            end else if(windmill_mode && windmill_was_owned)begin
                // Rotation energy fell below qualification. Return directly
                // to proven meteor pumping instead of keeping the arm tight.
                state<=ST_COAST;state_ms<=0;coast_ms<=RECOVERY_COAST_MIN_MS;
                recovery_armed<=1;retry_home_ok<=1;correction_kicks<=0;
                kick_origin<=enc_pos;arm_start_pos<=enc_pos;fault_reason<=0;
            end else if(meteor_wait_bottom) begin
                // Do not use the ordinary top capture while awaiting down.
                // Only the bounded upright-release helper may drive here.
                state<=ST_WAIT;state_ms<=0;coast_ms<=0;hang_ms<=0;catch_ms<=0;
                correction_kicks<=0;recovery_armed<=1;retry_home_ok<=1;
                arm_start_pos<=enc_pos;arm_balance_home<=enc_pos;kick_origin<=enc_pos;
                return_start_abs<=0;kick_dir_known<=0;kick_enc_positive<=0;
                if(meteor_motion_start_ready) begin
                    // Start a phase-matched ramp on this very passage.
                    state<=ST_RECOVER;kick_sign<=recovery_negative;
                    recovery_armed<=0;retry_home_ok<=0;correction_kicks<=1;
                end else if(meteor_rest_start_ready) begin
                    // Quiet enough for the proven two-leg initial swing;
                    // do not add the old500ms wait on top of this check.
                    state<=ST_KICK_1;kick_sign<=1;retry_home_ok<=0;
                end
            // Optional arm excursion limit latches until reset button or power cycle.
            end else if (state != ST_FAULT && state != ST_WAIT && arm_limit_fault) begin
                state <= ST_FAULT;fault_reason<=4'd5;
            end else if (!adc_ok || !filter_ready) begin
                // The electrical blind sector stops motor drive immediately.
                // If a kick was interrupted, preserve its encoder origin and
                // finish the return leg when the ADC is valid again.
                if (state == ST_KICK_1 || state == ST_KICK_2 ||
                    state == ST_COAST || state == ST_RECOVER) begin
                    if (coast_ms >= COAST_TIMEOUT_MS - 1) begin
                        state <= ST_WAIT;
                        coast_ms <= 0;
                    end else begin
                        if (state == ST_KICK_1 || state == ST_KICK_2)
                            state <= ST_KICK_2;
                        else state <= ST_COAST;
                        if (state == ST_RECOVER) recovery_armed <= 1'b0;
                        coast_ms <= coast_ms + 1'b1;
                    end
                end else if (state != ST_FAULT) begin
                    state <= state == ST_BALANCE ? ST_COAST : ST_WAIT;
                    coast_ms <= 0;
                    if (state == ST_BALANCE) begin
                        kick_origin <= enc_pos;
                        arm_start_pos <= enc_pos;
                        recovery_armed <= 1'b1;
                        correction_kicks <= 0;
                    end
                end
                if (state == ST_KICK_1) begin
                    state_ms <= 0;
                    return_start_abs <= kick_abs;
                    if (!kick_dir_known && kick_abs > SWING_RETURN_BAND_COUNTS) begin
                        kick_dir_known <= 1'b1;
                        kick_enc_positive <= (kick_delta >= 0);
                    end
                end else if (state != ST_KICK_2) state_ms <= 0;
                hang_ms <= 0; catch_ms <= 0;
            end else if(meteor_rest_start_ready && (state==ST_WAIT || state==ST_COAST)) begin
                state<=ST_KICK_1;state_ms<=0;coast_ms<=0;hang_ms<=0;catch_ms<=0;
                kick_sign<=1;arm_start_pos<=enc_pos;kick_origin<=enc_pos;
                kick_dir_known<=0;kick_enc_positive<=0;return_start_abs<=0;
                retry_home_ok<=0;recovery_armed<=1;correction_kicks<=0;
            end else if(meteor_motion_start_ready && state==ST_WAIT) begin
                state<=ST_RECOVER;state_ms<=0;coast_ms<=0;hang_ms<=0;catch_ms<=0;
                kick_sign<=recovery_negative;arm_start_pos<=enc_pos;kick_origin<=enc_pos;
                kick_dir_known<=0;retry_home_ok<=0;recovery_armed<=0;
                if(correction_kicks<MAX_CORRECTION_KICKS)correction_kicks<=correction_kicks+1'b1;
            end else begin
                case (state)
                    ST_WAIT: begin
                        state_ms <= 0;
                        coast_ms <= 0;
                        arm_start_pos <= enc_pos;
                        if (catch_ready) begin
                            if (catch_ms >= CATCH_STABLE_MS - 1) begin
                                state <= ST_BALANCE;
                                arm_balance_home <= enc_pos;
                                catch_ms <= 0;
                                retry_home_ok <= 1'b1;
                                correction_kicks <= 0;
                            end else catch_ms <= catch_ms + 1'b1;
                        end else catch_ms <= 0;
                        if (kick_abs <= SWING_RETURN_BAND_COUNTS)
                            retry_home_ok <= 1'b1;
                        if (near_hanging && arm_quiet) begin
                            if (hang_ms >= HANG_STABLE_MS - 1) begin
                                if (retry_home_ok) begin
                                    state <= ST_KICK_1;
                                    state_ms <= 0;
                                    kick_sign <= 1'b1;
                                    arm_start_pos <= enc_pos;
                                    kick_origin <= enc_pos;
                                    kick_dir_known <= 0;
                                    retry_home_ok <= 1'b0;
                                    correction_kicks <= 0;
                                    recovery_armed <= 1'b1;
                                    hang_ms <= 0;
                                end else begin
                                    // A return can stop short of its old origin.
                                    // This rotary arm has no end stop: once both
                                    // arm and pendulum are quiet, rebase here and
                                    // require another full stable interval.
                                    kick_origin <= enc_pos;
                                    retry_home_ok <= 1'b1;
                                    hang_ms <= 0;
                                end
                            end else hang_ms <= hang_ms + 1'b1;
                        end else hang_ms <= 0;
                    end
                    ST_KICK_1: begin
                        if (!kick_dir_known && kick_abs > SWING_RETURN_BAND_COUNTS) begin
                            kick_dir_known <= 1'b1;
                            kick_enc_positive <= (kick_delta >= 0);
                        end
                        if (kick_abs >= SWING_ARM_STEP_COUNTS) begin
                            state <= ST_KICK_2;
                            state_ms <= 0;
                            return_start_abs <= kick_abs;
                        end else if (state_ms >= KICK_MAX_MS - 1) begin
                            // A partial but measured kick may miss the target
                            // displacement after a fall. Return the arm before
                            // retrying; insufficient progress is a hard fault.
                            if (kick_abs >= RETURN_PROGRESS_MIN_COUNTS) begin
                                state <= ST_KICK_2;
                                state_ms <= 0;
                                return_start_abs <= kick_abs;
                            end else begin state <= ST_FAULT;fault_reason<=4'd1;end
                        end else state_ms <= state_ms + 1'b1;
                    end
                    ST_KICK_2: begin
                        if (kick_abs <= SWING_RETURN_BAND_COUNTS ||
                            kick_crossed_origin) begin
                            state <= ST_COAST;
                            state_ms <= 0;
                            coast_ms <= 0;
                            retry_home_ok <= 1'b1;
                            hang_ms <= 0;
                        end else if (kick_abs >= RETURN_MAX_DISPLACEMENT_COUNTS) begin
                            state <= ST_FAULT;fault_reason<=4'd2;
                        end else if (state_ms >= RETURN_MAX_MS - 1) begin
                            // A slow but genuine return is an unsuccessful
                            // swing attempt, not an encoder/motor fault.
                            if (return_start_abs >= RETURN_PROGRESS_MIN_COUNTS &&
                                kick_abs <= return_start_abs - RETURN_PROGRESS_MIN_COUNTS)
                                state <= ST_WAIT;
                            else begin state <= ST_FAULT;fault_reason<=4'd3;end
                        end else state_ms <= state_ms + 1'b1;
                    end
                    ST_COAST: begin
                        // Re-arm only on a new passage: leaving the sector or a
                        // confirmed reversal. Never repeatedly kick one pass.
                        if (!in_recovery_sector ||
                            (kick_sign ? recovery_positive : recovery_negative))
                            recovery_armed <= 1'b1;
                        if (coast_ms >= COAST_TIMEOUT_MS - 1) begin
                            state <= ST_WAIT;
                            coast_ms <= 0;
                            hang_ms <= 0;
                        end else if (near_hanging && arm_quiet && pendulum_quiet &&
                                     hang_ms >= RETRY_SETTLE_MS - 1) begin
                            // The pendulum has stopped near its bottom point.
                            // Re-arm; ST_WAIT still requires another stable
                            // interval before commanding a fresh kick.
                            state <= ST_WAIT;
                            coast_ms <= 0;
                            hang_ms <= 0;
                        end else begin
                            coast_ms <= coast_ms + 1'b1;
                            if (near_hanging && arm_quiet && pendulum_quiet)
                                hang_ms <= hang_ms + 1'b1;
                            else hang_ms <= 0;
                            // Short energy-injection pulses follow the measured
                            // lower-sector phase, not a full blind return kick.
                            if (recovery_start_ready) begin
                                kick_sign <= recovery_negative;
                                state <= ST_RECOVER;
                                recovery_armed <= 1'b0;
                                state_ms <= 0;
                                coast_ms <= 0;
                                arm_start_pos <= enc_pos;
                                kick_origin <= enc_pos;
                                kick_dir_known <= 0;
                                retry_home_ok <= 1'b0;
                                // K3 repeats passages without an artificial
                                // 24-pulse stop; saturate the boost counter.
                                // The ordinary recovery budget is unchanged.
                                if(!meteor_mode || correction_kicks<MAX_CORRECTION_KICKS)
                                    correction_kicks <= correction_kicks + 1'b1;
                                hang_ms <= 0;
                            end
                        end
                    end
                    ST_RECOVER: begin
                        // Stop at sector exit, reversal, speed limit, travel
                        // limit or pulse timeout. Catch below has priority.
                        if (!recovery_phase_ok ||
                            kick_abs >= selected_recovery_step ||
                            state_ms >= selected_recovery_ms - 1) begin
                            state <= ST_COAST;
                            if (state_ms >= selected_recovery_ms - 1 &&
                                kick_abs < RECOVERY_PROGRESS_MIN_COUNTS) begin
                                // A single weak short pulse is not proof of
                                // a jam. Permit two bounded retries around K3
                                // transitions; full-kick no-motion still faults.
                                if(meteor_transition_active && recovery_no_progress<TRANSITION_PULSE_RETRIES) begin
                                    recovery_no_progress<=recovery_no_progress+1'b1;
                                    retry_home_ok<=1;
                                end else begin state<=ST_FAULT;fault_reason<=4'd4;end
                            end else if(kick_abs>=RECOVERY_PROGRESS_MIN_COUNTS) recovery_no_progress<=0;
                            state_ms <= 0;
                            coast_ms <= 0;
                            hang_ms <= 0;
                        end else state_ms <= state_ms + 1'b1;
                    end
                    ST_BALANCE: begin
                        recovery_no_progress<=0;
                        if (angle_avg < TARGET_ANGLE - CENTER_RANGE ||
                            angle_avg > TARGET_ANGLE + CENTER_RANGE) begin
                            state <= ST_COAST;
                            coast_ms <= 0;
                            hang_ms <= 0;
                            arm_start_pos <= enc_pos;
                            kick_origin <= enc_pos;
                            recovery_armed <= 1'b1;
                            correction_kicks <= 0;
                        end
                    end
                    default: begin end // ST_FAULT waits for a user-key restart.
                endcase

                // Catch is checked in all swing states, including during a kick.
                if (state == ST_KICK_1 || state == ST_KICK_2 ||
                    state == ST_COAST || state == ST_RECOVER) begin
                    if (catch_ready) begin
                        if (catch_ms >= CATCH_STABLE_MS - 1) begin
                            state <= ST_BALANCE;
                            arm_balance_home <= enc_pos;
                            catch_ms <= 0;
                            state_ms <= 0;
                            retry_home_ok <= 1'b1;
                            correction_kicks <= 0;
                            recovery_armed <= 1'b1;
                        end else catch_ms <= catch_ms + 1'b1;
                    end else catch_ms <= 0;
                end
            end
        end
    end

    // A full-width comparison avoids the 12-bit magnitude wrap of older RTL.
    windmill_control #(.DOWN_ADC(HANG_ANGLE),.UP_ADC(TARGET_ANGLE),.POLARITY(SWING_POLARITY)) windmill(
        .clk(clk),.reset(restart_pressed),.tick_1ms(tick_1ms),
        .enable(run_armed && windmill_mode),.sensor_valid(adc_ok && filter_ready),
        .angle(angle_avg),.position(enc_pos),.command(windmill_command),
        .center(windmill_center),.fault(windmill_fault),.preparing(windmill_preparing),
        .reversing(windmill_reversing),.acquiring(windmill_acquiring));
    wire signed [31:0] requested_cmd =
        windmill_owned ? windmill_command : meteor_wait_bottom ? meteor_release_command :
        state == ST_KICK_1 ? (kick_sign ? swing_first : -swing_first) :
        state == ST_KICK_2 ? (kick_sign ? -swing_first : swing_first) :
        state == ST_RECOVER ? (kick_sign ? recovery_first : -recovery_first) :
        state == ST_BALANCE ? balance_cmd : 32'sd0;
    wire motor_enable = run_armed && !restart_pressed && adc_ok && filter_ready && !arm_limit_fault &&
                        (windmill_owned ? !windmill_fault : meteor_wait_bottom ? meteor_release_active :
                        (state == ST_KICK_1 || state == ST_KICK_2 ||
                         (state == ST_RECOVER && recovery_phase_ok &&
                          kick_abs < selected_recovery_step &&
                          state_ms < selected_recovery_ms) ||
                         (state == ST_BALANCE && !meteor_mode &&
                          angle_avg >= TARGET_ANGLE - CENTER_RANGE &&
                          angle_avg <= TARGET_ANGLE + CENTER_RANGE))) &&
                        !(state == ST_KICK_2 &&
                          (kick_abs <= SWING_RETURN_BAND_COUNTS || kick_crossed_origin ||
                           kick_abs >= RETURN_MAX_DISPLACEMENT_COUNTS));
    wire signed [31:0] motor_cmd = motor_enable ? requested_cmd : 32'sd0;
    wire [31:0] cmd_mag = motor_cmd[31] ? (~motor_cmd + 32'd1) : motor_cmd;
    wire [11:0] duty = (cmd_mag >= {20'd0, PWM_PERIOD}) ?
                       PWM_PERIOD : cmd_mag[11:0];
    reg [11:0] pwm_cnt = 0;
    reg [11:0] pwm_duty = 0;
    reg pwm_dir = 0;
    wire pwm_wrap = pwm_cnt == PWM_PERIOD - 1'b1;
    wire [11:0] pwm_next = pwm_wrap ? 12'd0 : pwm_cnt + 1'b1;
    always @(posedge clk) begin
        pwm_cnt <= pwm_next;
        if (!motor_enable || duty == 0) begin
            pwma <= 1'b0; ain1 <= 1'b1; ain2 <= 1'b1;
            pwm_duty <= 0; pwm_dir <= 0;
        end else if (pwm_wrap) begin
            // Latch direction and duty together for each complete PWM cycle.
            pwm_duty <= duty;
            pwm_dir <= motor_cmd < 0;
            pwma <= 1'b1;
            if (motor_cmd < 0) begin ain1 <= 1'b0; ain2 <= 1'b1; end
            else begin ain1 <= 1'b1; ain2 <= 1'b0; end
        end else begin
            pwma <= (pwm_next < pwm_duty);
            if (pwm_duty == 0) begin ain1 <= 1'b1; ain2 <= 1'b1; end
            else if (pwm_dir) begin ain1 <= 1'b0; ain2 <= 1'b1; end
            else begin ain1 <= 1'b1; ain2 <= 1'b0; end
        end
    end

    // AA56 v1 diagnostic frame: angle, target, signed command, arm position
    // and independent 50 ms encoder displacement, protected by CRC16.
    reg [7:0] uart_tx_data = 0;
    reg uart_tx_en = 0;
    wire uart_busy;
    uart_tx my_uart(.clk(clk), .data(uart_tx_data), .tx_en(uart_tx_en),
                    .tx_pin(uart_tx_pin), .tx_busy(uart_busy));
    reg [5:0] diag_pos_ms = 0;
    reg signed [31:0] diag_last_pos = 0;
    reg signed [31:0] diag_delta_50ms = 0;
    reg [31:0] diag_uptime_ms=0;
    reg [15:0] diag_k1_presses=0,diag_k2_presses=0,diag_k3_presses=0;
    always @(posedge clk) begin
        if(board_restart_pressed) begin
            diag_uptime_ms<=0;diag_k1_presses<=0;diag_k2_presses<=0;diag_k3_presses<=0;
        end else if(tick_1ms) begin
            diag_uptime_ms<=diag_uptime_ms+1'b1;
            if(k1_press) diag_k1_presses<=diag_k1_presses+1'b1;
            if(k2_press) diag_k2_presses<=diag_k2_presses+1'b1;
            if(k3_press) diag_k3_presses<=diag_k3_presses+1'b1;
        end
    end
    always @(posedge clk) begin
        // Read-only velocity measurement across ALL modes, independent of
        // the PID's own rolling window. Absolute encoder differences avoid
        // a false speed spike when the control home changes at a catch.
        if (restart_pressed) begin
            diag_pos_ms <= 0;
            diag_last_pos <= enc_pos;
            diag_delta_50ms <= 0;
        end else if (tick_1ms) begin
            if (diag_pos_ms == 6'd49) begin
                diag_pos_ms <= 0;
                diag_delta_50ms <= enc_pos - diag_last_pos;
                diag_last_pos <= enc_pos;
            end else diag_pos_ms <= diag_pos_ms + 1'b1;
        end
    end
    // flags[6:4]=state, [3]=filter_ready, [2]=adc_ok,
    // [1]=balance_valid, [0]=balance state.
    wire [7:0] diag_flags = {1'b0, state, filter_ready, adc_ok,
                             balance_valid, (state == ST_BALANCE)};
    wire signed [31:0] diag_arm_position = state == ST_BALANCE ?
        enc_pos - arm_balance_home : enc_pos - kick_origin;
    wire [15:0] diag_command = motor_cmd > 32'sd32767 ? 16'h7fff :
        motor_cmd < -32'sd32768 ? 16'h8000 : motor_cmd[15:0];
    wire [15:0] diag_target = state == ST_BALANCE ?
        TARGET_ANGLE + target_offset : TARGET_ANGLE;
    reg [15:0] sequence_number = 0;
    reg [159:0] packet = 0;
    reg [703:0] vision_packet=0;
    reg [4:0] send_state = 0;
    reg [15:0] crc = 16'hffff;
    reg [9:0] crc_count = 0;
    reg [9:0] crc_last_bit = 10'd159;
    reg crc_active = 0;
    // Read each frozen byte MSB first. Rotate a whole byte only after its
    // eighth CRC bit;20/88 rotations restore the snapshot before sending.
    // Both CRC and UART reuse the same byte-shift wiring and storage.
    wire [7:0] crc_byte=send_state==24 ? packet[159:152]:vision_packet[703:696];
    wire crc_feedback = crc[15] ^ crc_byte[3'd7-crc_count[2:0]];
    wire [15:0] crc_next = {crc[14:0], 1'b0} ^
        (crc_feedback ? 16'h1021 : 16'd0);
    wire [159:0] next_packet = {16'hAA56, 8'd1, diag_flags,
        sequence_number, adc_sample, diag_target, diag_command,
        diag_arm_position, diag_delta_50ms};
    wire [7:0] vision_flags={turn_active,vision_braking,camera_locked,camera_configured,
                            camera_x_valid,vision_target_live,vision_engaged,vision_mode};
    wire signed [31:0] reported_arm_goal=vision_engaged ?
        $signed(vision_goal_q16[47:16]) : moving_arm_home;
    // VTX1 v5:88-byte body (v4 + setting flag/degrees/three counters) + CRC. Both telemetry packets are latched on
    // the SAME clock below, and share sequence_number. These observations
    // never feed back into a controller or change its sampling schedule.
    wire signed [31:0] final_arm_destination=vision_engaged ?
        $signed(vision_goal_q16[47:16]) : turn_active ?
        arm_balance_home+$signed(turn_goal_q16[47:16]) : requested_arm_home;
    wire signed [15:0] applied_pwm_command=!motor_enable ? 16'sd0 :
        pwm_dir ? -$signed({4'd0,pwm_duty}) : $signed({4'd0,pwm_duty});
    wire [703:0] vision_next_packet={32'h56545831,8'd6,vision_flags,
        (camera_x_valid ? camera_x_q8:16'hffff),
        (camera_x_valid ? camera_y_q8:16'hffff),camera_red_pixels,
        (vision_x_accepted_valid ? vision_x_accepted:16'hffff),camera_sequence[15:0],
        (vision_center-arm_balance_home),(reported_arm_goal-arm_balance_home),
        enc_pos,moving_arm_home,final_arm_destination,arm_balance_home,
        diag_delta_50ms,planned_arm_speed_q8,applied_pwm_command,
        diag_k1_presses,diag_k2_presses,diag_uptime_ms,sequence_number,
        camera_error,camera_failed_register,diag_flags,motor_cmd,angle_avg,delta_5ms,
        {k1_pending,fault_reason[2:0],meteor_release_exhausted,meteor_release_active,meteor_wait_bottom,meteor_mode},diag_k3_presses,
        {1'd0,(windmill_mode && windmill_acquiring),windmill_mode,(windmill_mode && windmill_reversing),
         (windmill_mode && windmill_preparing),windmill_fault,windmill_mode,angle_setting},(angle_setting ? {6'd0,angle_degrees}:16'd0),
        (angle_setting ? setting_k1_presses:16'd0),(angle_setting ? setting_k2_presses:16'd0),
        (angle_setting ? setting_k3_presses:16'd0)};
    reg [6:0] vision_index=0;
    reg [21:0] timer_cnt = 0;
    always @(posedge clk) begin
        uart_tx_en <= 0;
        if (crc_active) begin
            crc <= crc_next;
            if(crc_count[2:0]==3'd7) begin
                if(send_state==24) packet<={packet[151:0],packet[159:152]};
                else vision_packet<={vision_packet[695:0],vision_packet[703:696]};
            end
            if (crc_count == crc_last_bit) crc_active <= 0;
            else crc_count <= crc_count + 1'b1;
        end
        case (send_state)
            0: if (timer_cnt >= UART_INTERVAL_CYCLES) begin
                   timer_cnt <= 0;
                   packet <= next_packet;
                   vision_packet <= vision_next_packet;
                   crc <= 16'hffff;
                   crc_count <= 0;
                   crc_last_bit <= 10'd159;
                   crc_active <= 1;
                   sequence_number <= sequence_number + 1'b1;
                   send_state <= 24;
               end else timer_cnt <= timer_cnt + 1'b1;
            24: if (!crc_active) send_state <= 1;
            21: if (!uart_busy && !uart_tx_en) begin
                    uart_tx_data <= crc[15:8]; uart_tx_en <= 1; send_state <= 22;
                end
            22: if (!uart_busy && !uart_tx_en) begin
                    uart_tx_data <= crc[7:0]; uart_tx_en <= 1; send_state <= 23;
                end
            23: if (!uart_busy && !uart_tx_en) send_state <= VISION_ENABLE ? 25:0;
            25: begin
                crc_last_bit<=10'd703;
                crc<=16'hffff;crc_count<=0;crc_active<=1;vision_index<=0;send_state<=26;
            end
            26: if(!crc_active) send_state<=27;
            27: if(!uart_busy && !uart_tx_en) begin
                uart_tx_data<=vision_index<88 ? vision_packet[703:696] :
                              vision_index==88 ? crc[15:8]:crc[7:0];
                uart_tx_en<=1;send_state<=28;
            end
            28: if(!uart_busy && !uart_tx_en) begin
                if(vision_index==89) send_state<=0;
                else begin
                    if(vision_index<87) vision_packet<={vision_packet[695:0],8'd0};
                    vision_index<=vision_index+1'b1;send_state<=27;
                end
            end
            default: if (!uart_busy && !uart_tx_en) begin
                    uart_tx_data <= packet[159:152];
                    packet <= {packet[151:0],8'd0};
                    uart_tx_en <= 1; send_state <= send_state + 1'b1;
                end
        endcase
    end
endmodule

// A standing rotary pendulum can remain balanced by stiction even with PWM=0.
// Supply a bounded startup nudge instead of waiting forever. This is only an
// entry helper; it does not implement the continuous-rotation control law.
module meteor_upright_release #(
    parameter signed [31:0] PWM=32'sd1000,
    parameter MAX_MS=80,
    parameter signed [31:0] TRAVEL_COUNTS=32'sd30,
    parameter POLARITY=1,
    parameter COAST_MS=400,
    parameter MAX_ATTEMPTS=3
)(
    input wire clk,tick_1ms,reset,enable,sensor_valid,near_top,quiet,
    input wire signed [31:0] position,
    output wire signed [31:0] command,
    output wire driving,exhausted
);
    reg active=0;
    reg [7:0] pulse_ms=0;
    reg [15:0] coast_ms=0;
    reg [2:0] attempts=0;
    reg signed [31:0] origin=0;
    wire signed [31:0] displacement=position-origin;
    wire travel_ok=displacement<TRAVEL_COUNTS && displacement> -TRAVEL_COUNTS;
    wire signed [31:0] ramp=$signed({24'd0,pulse_ms}) <<< 5;
    wire signed [31:0] magnitude=ramp>PWM ? PWM:ramp;
    assign driving=enable && !reset && sensor_valid && near_top && active &&
        travel_ok && pulse_ms<MAX_MS;
    assign command=driving ? POLARITY*magnitude:32'sd0;
    assign exhausted=attempts>=MAX_ATTEMPTS && !active && coast_ms>=COAST_MS;
    always @(posedge clk) begin
        if(reset || !enable) begin
            active<=0;pulse_ms<=0;coast_ms<=0;attempts<=0;origin<=position;
        end else if(tick_1ms) begin
            if(active) begin
                if(!sensor_valid || !near_top || !travel_ok || pulse_ms>=MAX_MS-1) begin
                    active<=0;pulse_ms<=0;coast_ms<=0;
                end else pulse_ms<=pulse_ms+1'b1;
            end else if(coast_ms<COAST_MS && attempts!=0) coast_ms<=coast_ms+1'b1;
            else if(sensor_valid && near_top && quiet && attempts<MAX_ATTEMPTS) begin
                active<=1;pulse_ms<=0;origin<=position;attempts<=attempts+1'b1;
            end
        end
    end
endmodule
