// X-only camera target -> acceleration-limited horizontal-arm setpoint.
// All logic runs on clk. tick_1ms is a clock enable, never a derived clock.
module vision_follow #(
    parameter WIDTH=208, ARM_COUNTS_PER_REV=1040, TRACK_SIGN=1,
    parameter TRACK_SPAN_DEG=270,
    parameter X_DEADBAND_PX=5,
    parameter X_MEDIAN_ENABLE=1,
    parameter [15:0] MAX_SPEED_Q16=16'd12000, ACCEL_Q16=16'd16,
    parameter STALE_MS=650, STOP_SETTLE_MS=150, STOP_SETTLE_COUNTS=2,
    parameter STOP_MAX_MS=1200
)(
    input wire clk,reset,tick_1ms,balance_valid,turn_active,k1_press,k2_press,
    input wire signed [31:0] enc_pos,
    input wire camera_update,camera_valid,
    input wire [15:0] camera_x_q8,
    output reg tracking=0,braking=0,
    output wire engaged,moving,manual_start,
    output reg signed [31:0] center=0,
    output reg signed [47:0] position_q16=0,goal_q16=0,
    output reg signed [16:0] velocity_q16=0,
    output wire signed [31:0] velocity_50ms_q8,
    output wire target_live,
    output reg [15:0] accepted_x_q8=16'hffff,
    output reg accepted_x_valid=0,hold_commit=0
);
    localparam [31:0] SPAN_COUNTS=(ARM_COUNTS_PER_REV*TRACK_SPAN_DEG)/360;
    localparam [31:0] X_MAX=(WIDTH-1)*256;
    localparam signed [47:0] HALF_SPAN_Q16=SPAN_COUNTS*48'sd32768;
    reg pending_manual=0,have_mapping=0;
    reg exit_hold=0;
    reg [15:0] stop_ms=0,settle_ms=0;
    reg signed [31:0] stop_anchor=0;
    wire signed [47:0] measured_position_q16=$signed({{16{enc_pos[31]}},enc_pos}) <<< 16;
    wire signed [31:0] stop_displacement=enc_pos-stop_anchor;
    wire arm_still=stop_displacement<=STOP_SETTLE_COUNTS && stop_displacement>=-STOP_SETTLE_COUNTS;
    localparam [15:0] DEADBAND_Q8=X_DEADBAND_PX*256;
    reg [15:0] age_ms=STALE_MS;
    wire frame_event=camera_update;
    wire fresh=camera_valid && camera_x_q8<=X_MAX && age_ms<STALE_MS;
    // Temporal median of three valid sensor frames, BEFORE the deadband.
    // The first frame seeds both taps, preserving immediate acquisition.
    // Subsequent isolated spikes cannot change a stationary target. Real
    // steps need two frames; monotonic motion is delayed by one frame.
    // No timer/heartbeat samples are inserted into this history.
    reg history_valid=0;
    reg [15:0] previous_x=0,older_x=0;
    wire [15:0] history_low=previous_x<older_x ? previous_x:older_x;
    wire [15:0] history_high=previous_x>older_x ? previous_x:older_x;
    wire [15:0] median_x=camera_x_q8<history_low ? history_low :
        camera_x_q8>history_high ? history_high : camera_x_q8;
    wire [15:0] filtered_x=X_MEDIAN_ENABLE && history_valid && fresh ? median_x:camera_x_q8;
    wire [15:0] x_change=filtered_x>=accepted_x_q8 ?
        filtered_x-accepted_x_q8 : accepted_x_q8-filtered_x;
    wire accept_x=!accepted_x_valid || !fresh || x_change>=DEADBAND_Q8;
    assign target_live=tracking && fresh && have_mapping;
    assign engaged=tracking || braking;
    assign moving=velocity_q16!=0;
    // K1 during tracking is queued while its trajectory decelerates. The
    // original five-second turn begins only after the planned speed is zero.
    assign manual_start=tick_1ms && balance_valid && braking &&
                        pending_manual && velocity_q16==0;
    wire signed [31:0] velocity_extended={{15{velocity_q16[16]}},velocity_q16};
    assign velocity_50ms_q8=((velocity_extended <<< 5)+(velocity_extended <<< 4)+
                            (velocity_extended <<< 1)) >>> 8;

    // Mapping: offset = (X/(WIDTH-1) - 0.5) * TRACK_SPAN_DEG.
    // Default 270 degrees = 780 counts total, +/-390 around the center.
    // X is Q8, so a single iterative divide gives a Q8 count offset. There
    // is no large combinational divider in the 1 ms control path.
    reg mapping_busy=0;
    reg [5:0] map_bit=0;
    reg [31:0] dividend=0,quotient=0;
    localparam REM_BITS=$clog2(WIDTH);
    localparam [REM_BITS:0] MAP_DENOMINATOR=WIDTH-1;
    reg [REM_BITS-1:0] remainder=0;
    wire [REM_BITS:0] trial={remainder,dividend[31]};
    wire subtract=trial>=MAP_DENOMINATOR;
    wire [REM_BITS:0] map_remainder=subtract ? trial-MAP_DENOMINATOR:trial;
    wire [31:0] next_quotient={quotient[30:0],subtract};
    wire signed [47:0] mapped_offset=$signed({8'd0,next_quotient,8'd0})-HALF_SPAN_Q16;
    wire signed [47:0] center_q16=$signed({{16{center[31]}},center}) <<< 16;
    wire signed [47:0] mapped_goal=center_q16+
        (TRACK_SIGN>0 ? mapped_offset : -mapped_offset);

    wire signed [47:0] error_q16=goal_q16-position_q16;
    wire [47:0] distance_q16=error_q16<0 ? -error_q16:error_q16;
    wire [16:0] full_speed=velocity_q16<0 ? -velocity_q16:velocity_q16;
    wire [15:0] speed=full_speed[15:0];
    wire [31:0] speed_square=speed*speed;
    localparam BRAKE_DIVISOR=2*ACCEL_Q16;
    // Velocity changes only at 1 ms boundaries. Register its stopping
    // distance on intervening system clocks so the square is not in the
    // velocity -> decision -> 48-bit position feedback path.
    reg [31:0] stopping_distance=0;
    always @(posedge clk) begin
        if(reset || !balance_valid) stopping_distance<=0;
        else stopping_distance<=speed_square/BRAKE_DIVISOR+speed;
    end
    wire toward_goal=(error_q16>=0 && velocity_q16>=0) ||
                     (error_q16<0 && velocity_q16<=0);
    wire brake_now=!target_live ||
        (toward_goal && distance_q16<=stopping_distance);
    wire signed [17:0] desired_speed=brake_now ? 18'sd0 :
        error_q16>=0 ? $signed({2'b0,MAX_SPEED_Q16}) : -$signed({2'b0,MAX_SPEED_Q16});
    wire signed [17:0] speed_difference=desired_speed-velocity_q16;
    wire signed [17:0] next_velocity=speed_difference>$signed({2'b0,ACCEL_Q16}) ?
        velocity_q16+$signed({2'b0,ACCEL_Q16}) :
        speed_difference<-$signed({2'b0,ACCEL_Q16}) ?
        velocity_q16-$signed({2'b0,ACCEL_Q16}) : desired_speed;
    wire signed [47:0] next_position=position_q16+next_velocity;

    always @(posedge clk) begin
        hold_commit<=0;
        if(reset || !balance_valid) begin
            tracking<=0;braking<=0;pending_manual<=0;have_mapping<=0;
            exit_hold<=0;stop_ms<=0;settle_ms<=0;stop_anchor<=enc_pos;
            accepted_x_valid<=0;accepted_x_q8<=16'hffff;
            history_valid<=0;previous_x<=0;older_x<=0;
            age_ms<=STALE_MS;mapping_busy<=0;velocity_q16<=0;
            center<=enc_pos;
            position_q16<=$signed({{16{enc_pos[31]}},enc_pos}) <<< 16;
            goal_q16<=$signed({{16{enc_pos[31]}},enc_pos}) <<< 16;
        end else begin
            if(frame_event) age_ms<=0;
            else if(tick_1ms && age_ms<STALE_MS) age_ms<=age_ms+1'b1;
            if(!fresh) begin
                have_mapping<=0;mapping_busy<=0;accepted_x_valid<=0;
            end
            if(!tracking || !fresh) history_valid<=0;
            if(frame_event && camera_valid && camera_x_q8<=X_MAX && tracking) begin
                previous_x<=camera_x_q8;
                older_x<=history_valid && fresh ? previous_x:camera_x_q8;
                history_valid<=1;
            end
            if(mapping_busy && tracking && fresh) begin
                dividend<=dividend<<1;
                remainder<=map_remainder[REM_BITS-1:0];
                quotient<=next_quotient;
                if(map_bit==31) begin
                    mapping_busy<=0;goal_q16<=mapped_goal;have_mapping<=1;
                end else map_bit<=map_bit+1'b1;
            end
            // Compare the filtered X against the LAST ACCEPTED X, never the previous raw
            // frame. Slow genuine motion can accumulate across the deadband;
            // sub-threshold jitter never changes the mapped goal or anchor.
            // Every received frame still refreshes age_ms above.
            if(frame_event && camera_valid && camera_x_q8<=X_MAX && tracking && accept_x) begin
                dividend<=filtered_x*SPAN_COUNTS;quotient<=0;remainder<=0;
                map_bit<=0;mapping_busy<=1;
                accepted_x_q8<=filtered_x;accepted_x_valid<=1;
            end
            if(tick_1ms) begin
                if(k1_press && tracking) begin
                    tracking<=0;braking<=1;pending_manual<=1;
                    mapping_busy<=0;have_mapping<=0;accepted_x_valid<=0;exit_hold<=0;
                    history_valid<=0;
                end else if(k2_press && tracking) begin
                    tracking<=0;braking<=1;pending_manual<=0;
                    mapping_busy<=0;have_mapping<=0;accepted_x_valid<=0;
                    history_valid<=0;
                    // Cancel the old remote goal immediately. While braking,
                    // use measured position (zero position attraction) and a
                    // ramped velocity target. Latch the actual stopping point.
                    exit_hold<=1;stop_ms<=0;settle_ms<=0;stop_anchor<=enc_pos;
                    position_q16<=measured_position_q16;goal_q16<=measured_position_q16;
                end else if(k2_press && !engaged && !turn_active && !k1_press) begin
                    // The current measured arm position, not its old PID home,
                    // is exactly the midpoint of this new tracking session.
                    tracking<=1;braking<=0;pending_manual<=0;center<=enc_pos;
                    position_q16<=$signed({{16{enc_pos[31]}},enc_pos}) <<< 16;
                    goal_q16<=$signed({{16{enc_pos[31]}},enc_pos}) <<< 16;
                    velocity_q16<=0;have_mapping<=0;mapping_busy<=0;
                    accepted_x_valid<=0;exit_hold<=0;
                    history_valid<=0;
                end else if(exit_hold && braking) begin
                    // Keep the angle/velocity loops active, without asking the
                    // arm to catch up to a stale planned position after K2.
                    velocity_q16<=next_velocity[16:0];
                    position_q16<=measured_position_q16;goal_q16<=measured_position_q16;
                    if(stop_ms<STOP_MAX_MS) stop_ms<=stop_ms+1'b1;
                    if(next_velocity!=0 || !arm_still) begin
                        settle_ms<=0;stop_anchor<=enc_pos;
                    end else if(settle_ms<STOP_SETTLE_MS) settle_ms<=settle_ms+1'b1;
                    if(next_velocity==0 &&
                       (settle_ms>=STOP_SETTLE_MS-1 || stop_ms>=STOP_MAX_MS-1)) begin
                        braking<=0;exit_hold<=0;pending_manual<=0;hold_commit<=1;
                    end
                end else if(engaged) begin
                    velocity_q16<=next_velocity[16:0];
                    if(target_live && distance_q16<=ACCEL_Q16 && speed<=ACCEL_Q16) begin
                        position_q16<=goal_q16;velocity_q16<=0;
                    end else position_q16<=next_position;
                    // A mapping result can arrive on this exact 1 ms edge.
                    // Do not overwrite its new goal with the idle hold value.
                    if(!target_live && next_velocity==0 && !mapping_busy)
                        goal_q16<=position_q16;
                    if(braking && velocity_q16==0) begin
                        braking<=0;pending_manual<=0;goal_q16<=position_q16;
                    end
                end
            end
        end
    end
endmodule
