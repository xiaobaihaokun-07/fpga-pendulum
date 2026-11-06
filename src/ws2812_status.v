// Drive the single onboard WS2812B-2020 from the 27 MHz FPGA clock.
// One GRB frame is 24 bits, most-significant bit first. At 27 MHz each bit
// lasts 32 clocks: 0 = 9 high + 23 low, 1 = 22 high + 10 low.
// A long low interval between frames latches the color and permits updates.
module ws2812_status #(
    parameter integer RESET_CYCLES = 270000 // 10 ms at 27 MHz; >280 us minimum
) (
    input wire clk,
    input wire [23:0] color_grb,
    output reg data = 1'b0
);
    localparam ST_RESET = 1'b0, ST_SEND = 1'b1;
    reg state = ST_RESET;
    reg [18:0] reset_count = 0;
    reg [4:0] bit_count = 0;
    reg [4:0] bit_clock = 0;
    reg [23:0] payload = 0;

    always @(posedge clk) begin
        if (state == ST_RESET) begin
            data <= 1'b0;
            if (reset_count == RESET_CYCLES - 1) begin
                reset_count <= 0;
                payload <= color_grb;
                bit_count <= 0;
                bit_clock <= 0;
                state <= ST_SEND;
            end else reset_count <= reset_count + 1'b1;
        end else begin
            data <= bit_clock < (payload[23] ? 5'd22 : 5'd9);
            if (bit_clock == 5'd31) begin
                bit_clock <= 0;
                payload <= {payload[22:0], 1'b0};
                if (bit_count == 5'd23) begin
                    bit_count <= 0;
                    state <= ST_RESET;
                end else bit_count <= bit_count + 1'b1;
            end else bit_clock <= bit_clock + 1'b1;
        end
    end
endmodule
