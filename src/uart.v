module uart_tx (
    input  wire clk,        // 27MHz 系统时钟
    input  wire [7:0] data, // 要发送的 8 位数据
    input  wire tx_en,      // 发送使能信号 (给高电平触发一次发送)
    output reg  tx_pin,     // 物理发送引脚
    output reg  tx_busy     // 忙碌标志 (1: 正在发送, 0: 空闲)
);
    // 27MHz / 115200 波特率 ≈ 234
    parameter BAUD_DIV = 234; 
    
    reg [15:0] baud_cnt = 0;
    reg [3:0]  bit_cnt = 0;
    reg [9:0]  tx_shift_reg = 10'b11_1111_1111; // 包含起始位0，数据位，停止位1
    reg [2:0]  state = 0;

    always @(posedge clk) begin
        case (state)
            0: begin // 空闲等待触发
                tx_pin <= 1'b1;
                tx_busy <= 1'b0;
                if (tx_en) begin
                    // 装载数据：停止位(1) + 数据(8bit) + 起始位(0)
                    tx_shift_reg <= {1'b1, data, 1'b0}; 
                    tx_busy <= 1'b1;
                    baud_cnt <= 0;
                    bit_cnt <= 0;
                    state <= 1;
                end
            end
            1: begin // 发送数据的 10 个 bit
                tx_pin <= tx_shift_reg[0]; // 最低位先发
                if (baud_cnt >= BAUD_DIV - 1) begin
                    baud_cnt <= 0;
                    tx_shift_reg <= {1'b1, tx_shift_reg[9:1]}; // 数据右移
                    if (bit_cnt >= 9) state <= 0; // 10个bit发完回到空闲
                    else bit_cnt <= bit_cnt + 1;
                end else begin
                    baud_cnt <= baud_cnt + 1;
                end
            end
        endcase
    end
endmodule