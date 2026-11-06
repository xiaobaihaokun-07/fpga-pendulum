// Polarity is accepted only after a complete HEIGHT-row two-byte/pixel segment.
// Stats retain BYTE lengths: floor(bytes/2) can hide a missing/extra byte.
// In camera_preview the legacy pclk port is driven by the 27 MHz system clk.
// Keep asynchronous assertion and the two-clock release guard; this reset
// style is intentional and is not an additional raw-camera clock domain.
module camera_sync_detect #(parameter integer WIDTH=320,HEIGHT=240)(
    input wire pclk,input wire reset,input wire enable,input wire sample_ce,
    input wire vs,input wire href,
    output reg locked=0,output reg vs_image_high=0,href_high=1,
    output wire vs_blank,href_valid,
    output reg [15:0] observed_width=0,observed_height=0,
    output reg [191:0] observed_stats=0,
    output reg observation_toggle=0,input wire observation_ack
);
    reg [1:0] enable_sync=0,release_sync=0,ack_sync=0;
    always @(posedge pclk or posedge reset) begin
        if(reset) begin enable_sync<=0; release_sync<=0; ack_sync<=0; end
        else begin
            enable_sync<={enable_sync[0],enable}; release_sync<={release_sync[0],1'b1};
            ack_sync<={ack_sync[0],observation_ack};
        end
    end
    reg previous_vs=0,previous_href=0,segment_started=0,run_started=0;
    reg [15:0] interval=0;
    reg [15:0] good[0:1],rows[0:1],minimum[0:1],maximum[0:1],odd[0:1],short_runs[0:1];
    reg [1:0] bad_frames=0;
    localparam [15:0] HEIGHT_U16=HEIGHT;
    function [15:0] increment(input [15:0] value);
        increment=value==16'hffff ? value : value+16'd1;
    endfunction
    // An HS run can end on the very same edge as VS. Count it before the
    // previous segment is evaluated, instead of losing its last row.
    wire closing=segment_started && run_started && href!=previous_href;
    wire last_low=closing && !previous_href;
    wire last_high=closing && previous_href;
    wire long_run=interval>=WIDTH;
    wire exact_run=interval==WIDTH*2;
    wire [15:0] gl=last_low && exact_run ? increment(good[0]) : good[0];
    wire [15:0] gh=last_high && exact_run ? increment(good[1]) : good[1];
    wire [15:0] rl=last_low && long_run ? increment(rows[0]) : rows[0];
    wire [15:0] rh=last_high && long_run ? increment(rows[1]) : rows[1];
    wire [15:0] sl=last_low && !long_run ? increment(short_runs[0]) : short_runs[0];
    wire [15:0] sh=last_high && !long_run ? increment(short_runs[1]) : short_runs[1];
    wire low_complete=gl==HEIGHT_U16 && rl==HEIGHT_U16 && sl==0;
    wire high_complete=gh==HEIGHT_U16 && rh==HEIGHT_U16 && sh==0;
    wire [15:0] ld=rl>HEIGHT_U16 ? rl-HEIGHT_U16 : HEIGHT_U16-rl;
    wire [15:0] hd=rh>HEIGHT_U16 ? rh-HEIGHT_U16 : HEIGHT_U16-rh;
    wire selected=gh>gl || (gh==gl && hd<ld);
    wire selected_last=selected ? last_high : last_low;
    wire [15:0] selected_rows=selected ? rh : rl;
    wire [15:0] selected_good=selected ? gh : gl;
    wire [15:0] selected_min=selected_last && long_run && interval<minimum[selected] ? interval : minimum[selected];
    wire [15:0] selected_max=selected_last && long_run && interval>maximum[selected] ? interval : maximum[selected];
    wire [15:0] selected_odd=selected_last && long_run && interval[0] ? increment(odd[selected]) : odd[selected];
    wire [15:0] selected_short=selected ? sh : sl;
    assign vs_blank=vs ^ vs_image_high;
    assign href_valid=href == href_high;
    integer k;
    always @(posedge pclk or posedge reset) begin
        if(reset) begin
            locked<=0; vs_image_high<=0; href_high<=1; bad_frames<=0;
            previous_vs<=0; previous_href<=0; interval<=0; segment_started<=0; run_started<=0;
            observed_width<=0; observed_height<=0; observed_stats<=0; observation_toggle<=0;
            for(k=0;k<2;k=k+1) begin
                good[k]<=0; rows[k]<=0; minimum[k]<=16'hffff; maximum[k]<=0; odd[k]<=0; short_runs[k]<=0;
            end
        end else if(release_sync[1] && (sample_ce || !enable_sync[1])) begin
            previous_vs<=vs; previous_href<=href;
            if(!enable_sync[1]) begin
                locked<=0; interval<=0; segment_started<=0; run_started<=0; bad_frames<=0;
                for(k=0;k<2;k=k+1) begin
                    good[k]<=0; rows[k]<=0; minimum[k]<=16'hffff; maximum[k]<=0; odd[k]<=0; short_runs[k]<=0;
                end
            end else if(vs!=previous_vs) begin
                if(segment_started) begin
                    if(!locked && (low_complete ^ high_complete)) begin
                        locked<=1; vs_image_high<=previous_vs; href_high<=high_complete; bad_frames<=0;
                    end else if(locked && previous_vs==vs_image_high) begin
                        if(href_high ? high_complete : low_complete) bad_frames<=0;
                        else if(bad_frames==2) begin locked<=0; bad_frames<=0; end
                        else bad_frames<=bad_frames+1'b1;
                    end
                    // Handshake keeps the entire diagnostic bundle immutable.
                    if(observation_toggle==ack_sync[1] && selected_rows>=HEIGHT/4) begin
                        observed_width<=selected_max>>1; observed_height<=selected_rows;
                        observed_stats<={selected_min,selected_max,selected_good,selected_rows,
                            selected_odd,selected_short,
                            {12'b0,selected_last,1'b1,selected,previous_vs},
                            {11'b0,(low_complete && high_complete),(selected_short!=0),
                             (selected_odd!=0),(selected_rows!=HEIGHT_U16),(selected_good!=HEIGHT_U16)},
                            gl,gh,rl,rh};
                        observation_toggle<=!observation_toggle;
                    end
                end
                segment_started<=1; run_started<=href!=previous_href; interval<=1;
                for(k=0;k<2;k=k+1) begin
                    good[k]<=0; rows[k]<=0; minimum[k]<=16'hffff; maximum[k]<=0; odd[k]<=0; short_runs[k]<=0;
                end
            end else if(href!=previous_href) begin
                if(closing) begin
                    if(exact_run) good[previous_href]<=increment(good[previous_href]);
                    if(long_run) begin
                        rows[previous_href]<=increment(rows[previous_href]);
                        if(interval<minimum[previous_href]) minimum[previous_href]<=interval;
                        if(interval>maximum[previous_href]) maximum[previous_href]<=interval;
                        if(interval[0]) odd[previous_href]<=increment(odd[previous_href]);
                    end else short_runs[previous_href]<=increment(short_runs[previous_href]);
                end
                run_started<=1; interval<=1;
            end else if(interval!=16'hffff) interval<=interval+1'b1;
        end
    end
endmodule
