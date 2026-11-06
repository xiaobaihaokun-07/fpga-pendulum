// ADS1115 at 0x48: AIN0-GND, +/-4.096V, continuous 860 SPS.
// Open-drain I2C, separate setup/high phases, ACK on every transmitted byte.
module ads1115 #(
    parameter integer TICK_CYCLES=135,
    parameter [15:0] POWER_WAIT_TICKS=2000,
    parameter [15:0] CONVERSION_WAIT_TICKS=400,
    parameter [15:0] RETRY_WAIT_TICKS=200
)(input wire clk, output wire scl, inout wire sda,
  output reg [15:0] adc_data=0, output reg adc_done=0);
    reg scl_low=0, sda_low=0;
    assign scl=scl_low ? 1'b0 : 1'bz;
    assign sda=sda_low ? 1'b0 : 1'bz;
    reg [15:0] tick_count=0;
    wire bus_tick=tick_count==TICK_CYCLES-1;
    always @(posedge clk) tick_count<=bus_tick ? 16'd0 : tick_count+1'b1;
    localparam [5:0] WAIT_BUS=0,START=1,START_LOW=2,
        TX_SETUP=3,TX_RISE=4,TX_HIGH=5,TX_FALL=6,
        ACK_SETUP=7,ACK_RISE=8,ACK_SAMPLE=9,ACK_FALL=10,
        RX_SETUP=11,RX_RISE=12,RX_SAMPLE=13,RX_FALL=14,
        MACK_SETUP=15,MACK_RISE=16,MACK_HIGH=17,MACK_FALL=18,
        STOP_LOW=19,STOP_RISE=20,STOP_RELEASE=21,STOP_DONE=22,
        CLEAR_LOW=23,CLEAR_HIGH=24;
    reg [5:0] state=WAIT_BUS;
    // transaction: config=0, conversion pointer=1, conversion read=2.
    reg [1:0] transaction=0,byte_index=0,stop_action=0;
    reg [2:0] bit_index=7;
    reg [7:0] tx_byte=8'h90,rx_byte=0;
    reg [15:0] rx_word=0,wait_ticks=POWER_WAIT_TICKS;
    reg read_low_byte=0,ack_failed=0,configured=0;
    reg [3:0] clear_count=0;
    always @(posedge clk) begin
        adc_done<=0;
        if(bus_tick) case(state)
            WAIT_BUS: begin
                scl_low<=0; sda_low<=0;
                if(wait_ticks!=0) wait_ticks<=wait_ticks-1'b1;
                else if(sda!==1'b1) begin
                    configured<=0; clear_count<=0; stop_action<=0; state<=CLEAR_LOW;
                end else begin
                    transaction<=configured ? 1 : 0;
                    byte_index<=0; bit_index<=7; tx_byte<=8'h90; state<=START;
                end
            end
            START: begin sda_low<=1; state<=START_LOW; end
            START_LOW: begin scl_low<=1; state<=TX_SETUP; end
            TX_SETUP: begin sda_low<=!tx_byte[bit_index]; state<=TX_RISE; end
            TX_RISE: begin scl_low<=0; state<=TX_HIGH; end
            TX_HIGH: state<=TX_FALL;
            TX_FALL: begin
                scl_low<=1;
                if(bit_index==0) state<=ACK_SETUP;
                else begin bit_index<=bit_index-1'b1; state<=TX_SETUP; end
            end
            ACK_SETUP: begin sda_low<=0; state<=ACK_RISE; end
            ACK_RISE: begin scl_low<=0; state<=ACK_SAMPLE; end
            ACK_SAMPLE: begin ack_failed<=sda!==1'b0; state<=ACK_FALL; end
            ACK_FALL: begin
                scl_low<=1;
                if(ack_failed) begin configured<=0; stop_action<=0; state<=STOP_LOW; end
                else if(transaction==2) begin
                    read_low_byte<=0; bit_index<=7; state<=RX_SETUP;
                end else if(transaction==0 && byte_index<3) begin
                    case(byte_index)
                        0: tx_byte<=8'h01;
                        1: tx_byte<=8'hC2;
                        default: tx_byte<=8'hE3;
                    endcase
                    byte_index<=byte_index+1'b1; bit_index<=7; state<=TX_SETUP;
                end else if(transaction==1 && byte_index==0) begin
                    tx_byte<=0; byte_index<=1; bit_index<=7; state<=TX_SETUP;
                end else begin stop_action<=transaction==0 ? 1 : 2; state<=STOP_LOW; end
            end
            RX_SETUP: begin sda_low<=0; state<=RX_RISE; end
            RX_RISE: begin scl_low<=0; state<=RX_SAMPLE; end
            RX_SAMPLE: begin rx_byte[bit_index]<=sda; state<=RX_FALL; end
            RX_FALL: begin
                scl_low<=1;
                if(bit_index==0) state<=MACK_SETUP;
                else begin bit_index<=bit_index-1'b1; state<=RX_SETUP; end
            end
            MACK_SETUP: begin sda_low<=!read_low_byte; state<=MACK_RISE; end
            MACK_RISE: begin scl_low<=0; state<=MACK_HIGH; end
            MACK_HIGH: state<=MACK_FALL;
            MACK_FALL: begin
                scl_low<=1;
                if(!read_low_byte) begin
                    rx_word[15:8]<=rx_byte; read_low_byte<=1; bit_index<=7; state<=RX_SETUP;
                end else begin rx_word[7:0]<=rx_byte; stop_action<=3; state<=STOP_LOW; end
            end
            STOP_LOW: begin scl_low<=1; sda_low<=1; state<=STOP_RISE; end
            STOP_RISE: begin scl_low<=0; state<=STOP_RELEASE; end
            STOP_RELEASE: begin sda_low<=0; state<=STOP_DONE; end
            STOP_DONE: case(stop_action)
                1: begin configured<=1; wait_ticks<=CONVERSION_WAIT_TICKS; state<=WAIT_BUS; end
                2: begin transaction<=2; tx_byte<=8'h91; bit_index<=7; state<=START; end
                3: begin adc_data<=rx_word; adc_done<=1; wait_ticks<=0; state<=WAIT_BUS; end
                default: begin wait_ticks<=RETRY_WAIT_TICKS; state<=WAIT_BUS; end
            endcase
            // Recover a slave left mid-byte; abort if the bus remains held low.
            CLEAR_LOW: begin scl_low<=1; sda_low<=0; state<=CLEAR_HIGH; end
            CLEAR_HIGH: begin
                scl_low<=0;
                if(clear_count==8) state<=STOP_LOW;
                else begin clear_count<=clear_count+1'b1; state<=CLEAR_LOW; end
            end
            default: begin configured<=0; stop_action<=0; state<=STOP_LOW; end
        endcase
    end
endmodule
