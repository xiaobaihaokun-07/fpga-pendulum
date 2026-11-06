// Independent integrated project: proven swing-up/balance + K1 manual turn +
// K2 X-only vision following. Camera never owns PWM or the fast balance loop.
module pendulum_vision_top #(
    parameter TRACK_SIGN=1,
    parameter TRACK_SPAN_DEG=270,
    parameter TRACK_X_DEADBAND_PX=5,
    parameter TRACK_X_MEDIAN_ENABLE=1,
    parameter [15:0] TRACK_MAX_SPEED_Q16=16'd12000,
    parameter [15:0] TRACK_ACCEL_Q16=16'd16
)(
    input wire clk,key1,key2,k1_turn_n,k2_track_n,k3_meteor_n,k4_angle_n,
    output wire scl,inout wire sda,input wire enc_a,enc_b,
    output wire ain1,ain2,pwma,uart_tx_pin,ws2812_data,
    inout wire cam_scl,cam_sda,output wire cam_reset_n,cam_pwdn,
    input wire cam_pclk,cam_vs,cam_hs,input wire [7:0] cam_d
);
    reg [1:0] camera_reset_sync=0;
    always @(posedge clk) camera_reset_sync<={camera_reset_sync[0],key1 || key2};
    wire camera_configured,camera_locked,camera_valid,camera_update;
    wire [15:0] camera_x,camera_y,camera_count,chip_id,failed_register;
    wire [31:0] camera_sequence;
    wire [7:0] camera_error;
    camera_x_frontend camera(.clk(clk),.reset(camera_reset_sync[1]),
        .cam_scl(cam_scl),.cam_sda(cam_sda),.cam_reset_n(cam_reset_n),.cam_pwdn(cam_pwdn),
        .cam_pclk(cam_pclk),.cam_vs(cam_vs),.cam_hs(cam_hs),.cam_d(cam_d),
        .configured(camera_configured),.locked(camera_locked),.x_valid(camera_valid),
        .update(camera_update),.x_q8(camera_x),.y_q8(camera_y),.red_pixels(camera_count),
        .chip_id(chip_id),.failed_register(failed_register),
        .sequence_number(camera_sequence),.error(camera_error));
    balance_top #(.ANGLE_SET_ENABLE(1),.METEOR_ENABLE(1),.VISION_ENABLE(1),.TRACK_SIGN(TRACK_SIGN),.TRACK_SPAN_DEG(TRACK_SPAN_DEG),
        .TRACK_X_DEADBAND_PX(TRACK_X_DEADBAND_PX),
        .TRACK_X_MEDIAN_ENABLE(TRACK_X_MEDIAN_ENABLE),
        .TRACK_MAX_SPEED_Q16(TRACK_MAX_SPEED_Q16),.TRACK_ACCEL_Q16(TRACK_ACCEL_Q16)) control
        (.clk(clk),.key1(key1),.key2(key2),.k1_turn_n(k1_turn_n),.k2_track_n(k2_track_n),.k3_meteor_n(k3_meteor_n),
         .k4_angle_n(k4_angle_n),
         .camera_update(camera_update),.camera_x_valid(camera_valid),
         .camera_configured(camera_configured),.camera_locked(camera_locked),
         .camera_x_q8(camera_x),.camera_y_q8(camera_y),.camera_error(camera_error),
         .camera_failed_register(failed_register),
         .camera_red_pixels(camera_count),.camera_sequence(camera_sequence),
         .scl(scl),.sda(sda),.enc_a(enc_a),.enc_b(enc_b),
         .ain1(ain1),.ain2(ain2),.pwma(pwma),.uart_tx_pin(uart_tx_pin),.ws2812_data(ws2812_data));
endmodule
