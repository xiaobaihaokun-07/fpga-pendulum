// Low-clock DVP receiver: every counter and RAM write uses the system clock.
// At nominal PCLK 2.4 MHz / system 27 MHz there are about 11 samples per
// camera cycle. Two equal synchronized PCLK samples qualify a transition;
// a single sampled spike/dip cannot create an extra byte clock enable.
// This is not a voltage translator and does not repair long/missing pulses.
module camera_dvp_sampler(
    input wire clk,input wire reset,
    input wire pclk,input wire vs,input wire hs,input wire [7:0] data,
    input wire sample_falling,
    output reg sample_ce=0,output reg sampled_vs=0,sampled_hs=0,
    output reg [7:0] sampled_data=0
);
    reg [1:0] pclk_sync=0,vs_sync=0,hs_sync=0;
    reg [7:0] data_meta=0,data_sync=0;
    reg candidate=0,qualified=0;
    always @(posedge clk) begin
        if(reset) begin
            pclk_sync<=0; vs_sync<=0; hs_sync<=0;
            data_meta<=0; data_sync<=0; candidate<=0; qualified<=0;
            sample_ce<=0; sampled_vs<=0; sampled_hs<=0; sampled_data<=0;
        end else begin
            pclk_sync<={pclk_sync[0],pclk};
            vs_sync<={vs_sync[0],vs}; hs_sync<={hs_sync[0],hs};
            data_meta<=data; data_sync<=data_meta;
            candidate<=pclk_sync[1];
            sample_ce<=0;
            if(pclk_sync[1]==candidate && candidate!=qualified) begin
                qualified<=candidate;
                if(candidate!=sample_falling) begin
                    sample_ce<=1;
                    sampled_vs<=vs_sync[1]; sampled_hs<=hs_sync[1];
                    sampled_data<=data_sync;
                end
            end
        end
    end
endmodule
