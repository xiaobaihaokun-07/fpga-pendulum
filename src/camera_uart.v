module camera_uart #(parameter integer BAUD_DIV=234)(
    input wire clk,input wire reset,input wire send,input wire [7:0] data,
    output reg tx=1,output reg busy=0
);
    reg [15:0] count=0;
    reg [3:0] bit_index=0;
    reg [9:0] shift=10'h3ff;
    always @(posedge clk) begin
        if(reset) begin tx<=1; busy<=0; count<=0; bit_index<=0; end
        else if(!busy) begin
            tx<=1;
            if(send) begin
                shift<={1'b1,data,1'b0}; tx<=0; busy<=1; count<=0; bit_index<=0;
            end
        end else if(count==BAUD_DIV-1) begin
            count<=0;
            if(bit_index==9) begin busy<=0; tx<=1; end
            else begin
                shift<={1'b1,shift[9:1]}; tx<=shift[1]; bit_index<=bit_index+1'b1;
            end
        end else count<=count+1'b1;
    end
endmodule
