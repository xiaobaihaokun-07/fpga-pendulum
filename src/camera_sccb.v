// Read: 0x78 + 16-bit address, STOP, 0x79 + byte; write: 0x78 + address + value.
// Every transmitted byte must be acknowledged. SCL/SDA are open drain.
// With TICK_CYCLES >= 3 and no stretching, data bits take five ticks:
// two low (FALL -> SETUP -> RISE), three high (RISE wait -> HIGH -> FALL).
// At 27 MHz / 270 this is approximately 20 kHz, not 50 or 25 kHz.
// Exact 0/1 checks deliberately reject unknown/unpulled bus levels in RTL.
// The target Gowin flow synthesizes comparisons against known constants.
module camera_sccb #(
    parameter integer TICK_CYCLES=270,
    parameter integer TIMEOUT_CYCLES=2700000
)(
    input wire clk, input wire reset, input wire start,
    input wire [15:0] reg_addr, input wire write_mode, input wire [7:0] write_data,
    inout wire scl, inout wire sda,
    output reg busy=0, output reg done=0,
    output reg [7:0] data=0, output reg [3:0] error=0
);
    reg scl_low=0,sda_low=0;
    assign scl=scl_low ? 1'b0 : 1'bz;
    assign sda=sda_low ? 1'b0 : 1'bz;
    reg [1:0] scl_sync=3,sda_sync=3;
    always @(posedge clk) begin
        scl_sync<={scl_sync[0],scl}; sda_sync<={sda_sync[0],sda};
    end
    reg [31:0] divider=0,watchdog=0;
    wire tick=divider==TICK_CYCLES-1;
    reg [15:0] address_latched=0;
    reg write_latched=0;
    reg [7:0] value_latched=0;
    reg [7:0] tx=0,rx=0;
    reg [2:0] bit_index=7;
    reg [1:0] byte_index=0,stop_action=0;
    reg nack=0;
    reg [3:0] clear_count=0;
    localparam [5:0] IDLE=0,BUS_CHECK=1,CLEAR_LOW=2,CLEAR_RISE=3,
        CLEAR_HIGH=4,CLEAR_FALL=5,START_SETUP=6,START_LOW=7,START_FALL=8,
        TX_SETUP=9,TX_RISE=10,TX_HIGH=11,TX_FALL=12,
        ACK_SETUP=13,ACK_RISE=14,ACK_SAMPLE=15,ACK_FALL=16,
        STOP_LOW=17,STOP_RISE=18,STOP_RELEASE=19,STOP_DONE=20,
        RX_SETUP=21,RX_RISE=22,RX_SAMPLE=23,RX_FALL=24,
        NACK_SETUP=25,NACK_RISE=26,NACK_HIGH=27,NACK_FALL=28;
    reg [5:0] state=IDLE;
    always @(posedge clk) begin
        done<=0;
        if(reset) begin
            state<=IDLE; scl_low<=0; sda_low<=0;
            busy<=0; done<=0; data<=0; error<=0;
            divider<=0; watchdog<=0;
        end else begin
            divider<=tick ? 0 : divider+1'b1;
            if(state==IDLE && start) begin
                address_latched<=reg_addr; write_latched<=write_mode; value_latched<=write_data;
                byte_index<=0; tx<=8'h78;
                bit_index<=7; rx<=0; error<=0; busy<=1;
                watchdog<=0; divider<=0; state<=BUS_CHECK;
            end else if(busy && watchdog==TIMEOUT_CYCLES-1) begin
                // Release the bus even when SCL cannot rise; never fabricate data.
                scl_low<=0; sda_low<=0; busy<=0; done<=1;
                error<=2; state<=IDLE;
            end else begin
                watchdog<=busy ? watchdog+1'b1 : 0;
                if(tick) case(state)
                    IDLE: begin scl_low<=0; sda_low<=0; end
                    BUS_CHECK: begin
                        scl_low<=0; sda_low<=0;
                        if(scl_sync[1]===1'b1) begin
                            if(sda_sync[1]===1'b1) state<=START_SETUP;
                            else begin clear_count<=0; state<=CLEAR_LOW; end
                        end
                    end
                    CLEAR_LOW: begin scl_low<=1; sda_low<=0; state<=CLEAR_RISE; end
                    CLEAR_RISE: begin
                        scl_low<=0;
                        if(scl_sync[1]===1'b1) state<=CLEAR_HIGH;
                    end
                    CLEAR_HIGH: state<=CLEAR_FALL;
                    CLEAR_FALL: begin
                        scl_low<=1;
                        if(clear_count==8) begin stop_action<=2; state<=STOP_LOW; end
                        else begin clear_count<=clear_count+1'b1; state<=CLEAR_LOW; end
                    end
                    START_SETUP: begin scl_low<=0; sda_low<=0; state<=START_LOW; end
                    START_LOW: begin sda_low<=1; state<=START_FALL; end
                    START_FALL: begin scl_low<=1; state<=TX_SETUP; end
                    TX_SETUP: begin sda_low<=!tx[bit_index]; state<=TX_RISE; end
                    TX_RISE: begin
                        scl_low<=0;
                        if(scl_sync[1]===1'b1) state<=TX_HIGH;
                    end
                    TX_HIGH: state<=TX_FALL;
                    TX_FALL: begin
                        scl_low<=1;
                        if(bit_index==0) state<=ACK_SETUP;
                        else begin bit_index<=bit_index-1'b1; state<=TX_SETUP; end
                    end
                    ACK_SETUP: begin sda_low<=0; state<=ACK_RISE; end
                    ACK_RISE: begin
                        scl_low<=0;
                        if(scl_sync[1]===1'b1) state<=ACK_SAMPLE;
                    end
                    ACK_SAMPLE: begin
                        nack<=sda_sync[1]!==1'b0;
                        if(sda_sync[1]!==1'b0) error<={2'b00,byte_index}+4'd3;
                        state<=ACK_FALL;
                    end
                    ACK_FALL: begin
                        scl_low<=1;
                        if(nack) begin stop_action<=0; state<=STOP_LOW; end
                        else if(byte_index==2 && !write_latched) begin stop_action<=1; state<=STOP_LOW; end
                        else if(byte_index==3 && write_latched) begin stop_action<=0; state<=STOP_LOW; end
                        else if(byte_index==3) begin bit_index<=7; rx<=0; state<=RX_SETUP; end
                        else begin
                            tx<=byte_index==0 ? address_latched[15:8] :
                                byte_index==1 ? address_latched[7:0] : value_latched;
                            byte_index<=byte_index+1'b1; bit_index<=7; state<=TX_SETUP;
                        end
                    end
                    RX_SETUP: begin sda_low<=0; state<=RX_RISE; end
                    RX_RISE: begin
                        scl_low<=0;
                        if(scl_sync[1]===1'b1) state<=RX_SAMPLE;
                    end
                    RX_SAMPLE: begin rx<={rx[6:0],sda_sync[1]}; state<=RX_FALL; end
                    RX_FALL: begin
                        scl_low<=1;
                        if(bit_index==0) state<=NACK_SETUP;
                        else begin bit_index<=bit_index-1'b1; state<=RX_SETUP; end
                    end
                    NACK_SETUP: begin sda_low<=0; state<=NACK_RISE; end
                    NACK_RISE: begin
                        scl_low<=0;
                        if(scl_sync[1]===1'b1) state<=NACK_HIGH;
                    end
                    NACK_HIGH: state<=NACK_FALL;
                    NACK_FALL: begin scl_low<=1; stop_action<=0; state<=STOP_LOW; end
                    STOP_LOW: begin scl_low<=1; sda_low<=1; state<=STOP_RISE; end
                    STOP_RISE: begin
                        scl_low<=0;
                        if(scl_sync[1]===1'b1) state<=STOP_RELEASE;
                    end
                    STOP_RELEASE: begin sda_low<=0; state<=STOP_DONE; end
                    STOP_DONE: begin
                        if(stop_action==1) begin
                            byte_index<=3; bit_index<=7; tx<=8'h79; state<=START_SETUP;
                        end else if(stop_action==2 && sda_sync[1]===1'b1) state<=BUS_CHECK;
                        else begin
                            if(stop_action==2) error<=1;
                            else if(error==0) data<=rx;
                            busy<=0; done<=1; state<=IDLE;
                        end
                    end
                    default: begin state<=IDLE; busy<=0; scl_low<=0; sda_low<=0; end
                endcase
            end
        end
    end
endmodule
