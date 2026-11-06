// Bottom-only phase pumping for the K3 windmill mode.
//
// The tested response release applied opposite torque near the top. Its capture
// shows the pendulum repeatedly stalling just below upright: the negative top
// impulse removes the energy gained at the bottom, the rotation guard loses
// acquisition, and the meteor actuator has to re-energise the swing. The arm
// was also held so tightly that the position loop cancelled much of the pump.
//
// This revision pumps only in the lower third (positive PWM, the same polarity
// that accelerates the tested forward rotation), coasts over the top, and
// moves the speed brake outside the soft envelope so the pumping motion is not
// braked inside it. Travel protection, two-turn acquisition, per-turn shrink
// and the fixed direction are retained.
module windmill_control #(
    parameter signed [31:0] DOWN_ADC=6360,UP_ADC=20175,POLARITY=1,
    parameter signed [31:0] PUMP_PWM=2600,MAX_PWM=3200,
    parameter signed [31:0] SOFT_COUNTS=60,HARD_COUNTS=160,
    parameter signed [31:0] CENTER_KP=16,ARM_KD=96,
    parameter signed [31:0] PUMP_MS=70,BRAKE_PWM=1600,EDGE_PWM=1200,
    parameter signed [31:0] SLEW_STEP=256,
    parameter signed [31:0] START_SOFT=280,SHRINK_STEP=10,
    // -1 means decreasing ADC away from the electrical wrap, not a claim about
    // clockwise as seen from an unspecified physical viewing side.
    parameter signed [31:0] ROTATION_SIGN=-1,
    // Hardware CSV: positive PWM decreases encoder counts. This sign is
    // independent of POLARITY (pendulum energy injection convention).
    parameter signed [31:0] PWM_ENCODER_SIGN=-1,
    parameter STARTUP_ENABLE=1
)(
    input wire clk,reset,tick_1ms,enable,sensor_valid,
    input wire signed [31:0] angle,position,
    output reg signed [31:0] command=0,
    output reg signed [31:0] center=0,
    output reg fault=0,
    output wire preparing,reversing,acquiring
);
    localparam signed [31:0] HALF=UP_ADC-DOWN_ADC;
    wire signed [31:0] offset=angle-DOWN_ADC;
    wire signed [31:0] wrapped=offset>HALF ? offset-2*HALF:offset;
    wire signed [31:0] height=wrapped<0 ? -wrapped:wrapped;
    reg bottom=0;
    // Register the sector decode on the system clock. One clock of latency is
    // under 40 ns at 27 MHz; it removes angle -> wrap/abs -> phase -> PWM from
    // a single timing path.
    always @(posedge clk) begin
        if(reset || !enable || !sensor_valid) bottom<=0;
        else bottom<=height<HALF/3;
    end
    reg [2:0] sample_ms=0;
    reg [1:0] good_samples=0;
    reg signed [31:0] last_angle=0,last_position=0;
    reg signed [31:0] angular_speed=0,arm_speed=0;
    wire signed [31:0] delta=angle-last_angle;
    wire signed [31:0] delta_abs=delta<0 ? -delta:delta;
    wire signed [31:0] speed_abs=angular_speed<0 ? -angular_speed:angular_speed;
    wire signed [31:0] error=position-center;
    wire signed [31:0] error_abs=error<0 ? -error:error;
    wire phase_valid=good_samples>=1 && speed_abs>=8 && speed_abs<=900;
    wire signed [31:0] directional_speed=ROTATION_SIGN*angular_speed;
    wire guard_acquiring;
    wire signed [31:0] guard_soft;
    wire completed_turn;
    reg signed [31:0] cycle_peak=0;
    wire shrink_ok=cycle_peak+20<guard_soft && error_abs+20<guard_soft &&
        arm_speed>=-16 && arm_speed<=16;
    windmill_rotation_guard #(.DOWN_ADC(DOWN_ADC),.UP_ADC(UP_ADC),.ROTATION_SIGN(ROTATION_SIGN),
        .START_SOFT(START_SOFT),.FINAL_SOFT(SOFT_COUNTS),.SHRINK_STEP(SHRINK_STEP)) rotation_guard(
        .clk(clk),.reset(reset || !enable),.tick_1ms(tick_1ms),
        .valid(sensor_valid && phase_valid),.hold_fault(fault),.angle(angle),
        .directional_speed(directional_speed),.shrink_ok(shrink_ok),
        .completed_turn(completed_turn),.acquiring(guard_acquiring),
        .soft_limit(guard_soft));
    assign acquiring=STARTUP_ENABLE ? guard_acquiring:1'b0;
    wire signed [31:0] active_soft=STARTUP_ENABLE ? guard_soft:SOFT_COUNTS;
    wire signed [31:0] active_hard=active_soft+(HARD_COUNTS-SOFT_COUNTS);
    wire out_of_range=!acquiring && (error>active_hard || error<-active_hard);
    assign reversing=enable && !acquiring && sensor_valid && phase_valid &&
        directional_speed<0;
    assign preparing=1'b0;
    always @(posedge clk) begin
        if(reset || !enable || acquiring)cycle_peak<=0;
        else if(tick_1ms)begin
            if(completed_turn)cycle_peak<=error_abs;
            else if(error_abs>cycle_peak)cycle_peak<=error_abs;
        end
    end

    reg [7:0] pulse_ms=0;
    reg [8:0] quiet_ms=0;
    reg [7:0] seed_ms=0;
    // pulse_ms resets only after leaving the lower third. A down-crossing must
    // not split the impulse into two short pieces; one continuous bottom pulse
    // per passage transfers more energy for the same command budget.
    always @(posedge clk)begin
        if(reset || !enable || acquiring)begin
            pulse_ms<=0;quiet_ms<=0;seed_ms<=0;
        end else if(tick_1ms)begin
            if(!bottom)pulse_ms<=0;
            else if(phase_valid && sensor_valid && !fault && pulse_ms!=255)
                pulse_ms<=pulse_ms+1'b1;
            if(speed_abs<8 && good_samples>=2)begin
                if(quiet_ms<150)quiet_ms<=quiet_ms+1'b1;
                else seed_ms<=seed_ms==239 ? 0:seed_ms+1'b1;
            end else begin
                quiet_ms<=0;seed_ms<=0;
            end
        end
    end
    // Positive PWM is the direction that accelerated the tested forward
    // rotation at the bottom (the meteor release uses the same polarity).
    // The top sector deliberately coasts: there is no negative impulse there.
    wire pump_active=bottom && phase_valid && sensor_valid && !fault &&
        (pulse_ms<PUMP_MS);
    wire signed [31:0] pump=pump_active ? PUMP_PWM:32'sd0;
    wire seed_active=quiet_ms>=150 && seed_ms<80 && bottom;
    wire signed [31:0] seed=seed_active ? 32'sd1000:32'sd0;
    wire signed [31:0] phase_next=pump_active ? pump:seed;
    reg signed [31:0] phase_request=0;
    always @(posedge clk)begin
        if(reset || !enable || !sensor_valid || acquiring)phase_request<=0;
        else phase_request<=phase_next;
    end
    // Position feedback is in encoder counts; the phase request is converted
    // to encoder direction once here and back to PWM once at the actuator.
    // The stronger gains need a few system-clock pipeline stages: the 27MHz
    // path from encoder position through the constant multiply, centering
    // shift, damping sum and output limiters was about 58ns when left
    // combinational. Each stage below is a single add/multiply; the resulting
    // latency is a few hundred nanoseconds, far below the 1ms control tick.
    reg signed [31:0] feedback_error=0;
    reg signed [31:0] position_scaled=0,velocity_scaled=0;
    reg signed [31:0] position_feedback=0,velocity_feedback=0;
    reg signed [31:0] softened_position=0,feedback=0;
    reg signed [31:0] candidate_q=0,speed_limited_q=0;
    reg signed [31:0] inward_q=0,encoder_limited_q=0,limited_q=0;
    always @(posedge clk)begin
        if(reset || !enable || !sensor_valid)begin
            feedback_error<=0;position_scaled<=0;velocity_scaled<=0;
            position_feedback<=0;velocity_feedback<=0;softened_position<=0;feedback<=0;
            candidate_q<=0;speed_limited_q<=0;inward_q<=0;
            encoder_limited_q<=0;limited_q<=0;
        end else begin
            feedback_error<=error;
            position_scaled<=CENTER_KP*feedback_error;
            velocity_scaled<=ARM_KD*arm_speed;
            position_feedback<=-position_scaled;
            velocity_feedback<=-velocity_scaled;
            softened_position<=active_soft>160 ?
                position_feedback>>>1:position_feedback;
            feedback<=softened_position+velocity_feedback;
            candidate_q<=feedback+PWM_ENCODER_SIGN*phase_request;
            // Brake only outside the soft envelope. The old half-soft gate was
            // cancelling the very arm motion the phase pump needs inside it.
            speed_limited_q<=
                error>active_soft && arm_speed>=4 && candidate_q>0 ? -BRAKE_PWM:
                error< -active_soft && arm_speed<=-4 && candidate_q<0 ? BRAKE_PWM:candidate_q;
            inward_q<=error>active_soft ?
                (speed_limited_q> -EDGE_PWM ? -EDGE_PWM:speed_limited_q):
                error< -active_soft ? (speed_limited_q<EDGE_PWM ? EDGE_PWM:speed_limited_q):speed_limited_q;
            encoder_limited_q<=inward_q>MAX_PWM ? MAX_PWM:
                inward_q< -MAX_PWM ? -MAX_PWM:inward_q;
            limited_q<=PWM_ENCODER_SIGN*encoder_limited_q;
        end
    end
    wire signed [31:0] applied_encoder_direction=PWM_ENCODER_SIGN*command;
    reg signed [31:0] demand_pwm=0;
    always @(posedge clk)begin
        if(reset || !enable || !sensor_valid)demand_pwm<=0;
        else demand_pwm<=limited_q;
    end
    wire signed [31:0] change=demand_pwm-command;
    always @(posedge clk)begin
        if(reset || !enable)begin
            center<=position;command<=0;fault<=0;sample_ms<=0;good_samples<=0;
            last_angle<=angle;last_position<=position;angular_speed<=0;arm_speed<=0;
        end else if(tick_1ms)begin
            if(out_of_range)fault<=1;
            if(fault || out_of_range || !sensor_valid)begin
                command<=0;sample_ms<=0;good_samples<=0;angular_speed<=0;arm_speed<=0;
                last_angle<=angle;last_position<=position;
            end else begin
                if(sample_ms==4)begin
                    sample_ms<=0;last_angle<=angle;last_position<=position;
                    arm_speed<=position-last_position;
                    // Reject wrap/gap jumps; require fresh valid samples.
                    if(delta_abs>900)begin
                        angular_speed<=0;good_samples<=0;
                    end else begin
                        angular_speed<=delta;
                        if(good_samples<2)good_samples<=good_samples+1'b1;
                    end
                end else sample_ms<=sample_ms+1'b1;
                if(change>SLEW_STEP)command<=command+SLEW_STEP;
                else if(change< -SLEW_STEP)command<=command-SLEW_STEP;
                else command<=demand_pwm;
                if(good_samples<1)command<=0;
                if(!acquiring && ((error>active_soft && applied_encoder_direction>0) ||
                   (error< -active_soft && applied_encoder_direction<0)))command<=0;
            end
            if(acquiring)begin
                center<=position;command<=0;fault<=0;
            end
        end
    end
endmodule

// Qualify rotations by an ordered sequence of four REAL valid-angle sectors,
// not by integrating the ADC wrap/gap into a fictitious turn. Negative ADC
// direction: down -> low-before-gap -> up -> middle -> down. Reverse motion
// cancels the sequence. The sensor's blind gap preserves progress briefly.
module windmill_rotation_guard #(
    parameter signed [31:0] DOWN_ADC=6360,UP_ADC=20175,ROTATION_SIGN=-1,
    parameter signed [31:0] START_SOFT=280,FINAL_SOFT=60,SHRINK_STEP=10,
    parameter integer TURN_TIMEOUT_MS=1800
)(input wire clk,reset,tick_1ms,valid,hold_fault,shrink_ok,
  input wire signed [31:0] angle,directional_speed,
  output reg acquiring=1,output wire completed_turn,
  output reg signed [31:0] soft_limit=START_SOFT);
    localparam signed [31:0] MID_ADC=(DOWN_ADC+UP_ADC)/2;
    wire down=angle>=DOWN_ADC-1200 && angle<=DOWN_ADC+1200;
    wire low=angle>=1500 && angle<=3500;
    wire up=angle>=UP_ADC-1500 && angle<=UP_ADC+1500;
    wire middle=angle>=MID_ADC-1000 && angle<=MID_ADC+1000;
    // For positive-direction calibration the checkpoint order is reversed.
    wire first=ROTATION_SIGN<0 ? low:middle;
    wire third=ROTATION_SIGN<0 ? middle:low;
    reg [2:0] step=0;
    reg [1:0] consecutive=0;
    reg [11:0] age_ms=0;
    reg [5:0] reverse_ms=0;
    reg fit_turn=0;
    assign completed_turn=tick_1ms && valid && directional_speed>=8 && step==4 && down;
    always @(posedge clk) begin
        if(reset) begin
            acquiring<=1;soft_limit<=START_SOFT;step<=0;consecutive<=0;age_ms<=0;
            reverse_ms<=0;fit_turn<=0;
        end else if(tick_1ms && !hold_fault) begin
            if(age_ms<TURN_TIMEOUT_MS)age_ms<=age_ms+1'b1;
            if(valid && directional_speed< -8)begin
                step<=0;consecutive<=0;
                if(reverse_ms<40)reverse_ms<=reverse_ms+1'b1;
            end else if(valid)reverse_ms<=0;
            if(age_ms>=TURN_TIMEOUT_MS-1 || reverse_ms>=39)begin
                acquiring<=1;soft_limit<=START_SOFT;step<=0;consecutive<=0;
                age_ms<=0;reverse_ms<=0;fit_turn<=0;
            end else if(valid && directional_speed>=8) begin
                case(step)
                    0: if(down)begin step<=1;age_ms<=0;end
                    1: if(first)step<=2;
                    2: if(up && directional_speed>=60)step<=3;
                    3: if(third)step<=4;
                    4: if(down)begin
                        step<=1;age_ms<=0;
                        if(consecutive<2)consecutive<=consecutive+1'b1;
                        if(acquiring && consecutive>=1)acquiring<=0;
                        else if(!acquiring && soft_limit>FINAL_SOFT)begin
                            if(shrink_ok && fit_turn)begin
                                soft_limit<=soft_limit-FINAL_SOFT>SHRINK_STEP ?
                                    soft_limit-SHRINK_STEP:FINAL_SOFT;
                                fit_turn<=0;
                            end else fit_turn<=shrink_ok;
                        end
                    end
                    default:step<=0;
                endcase
            end
        end
    end
endmodule
