module encoder (
    input  wire clk,         // 27MHz 系统时钟
    input  wire rst_n,       // 复位信号 (低有效)
    input  wire enc_a,       // 编码器 A 相输入
    input  wire enc_b,       // 编码器 B 相输入
    
    output reg signed [31:0] position, // 实时累计位置 (脉冲数)
    output reg signed [15:0] speed     // 实时速度 (脉冲数/100ms)
);

    // ====================================================
    // 1. 防亚稳态与边沿提取 (打三拍)
    // ====================================================
    // 外部输入的信号是不稳定的，直接用会导致状态机崩溃，必须用寄存器打拍同步
    reg [2:0] a_r;
    reg [2:0] b_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_r <= 3'b000;
            b_r <= 3'b000;
        end else begin
            a_r <= {a_r[1:0], enc_a}; // 移位寄存器
            b_r <= {b_r[1:0], enc_b};
        end
    end

    // ====================================================
    // 2. 4 倍频状态机解码 (判断正反转并累计位置)
    // ====================================================
    // a_r[2]/b_r[2] 是上一个状态， a_r[1]/b_r[1] 是当前状态
    reg signed [31:0] pos_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_cnt <= 0;
        end else begin
            case ({a_r[2], b_r[2], a_r[1], b_r[1]})
                // 出现以下四种状态跳变，说明是正转 (位置 +1)
                4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00: pos_cnt <= pos_cnt + 1;
                // 出现以下四种状态跳变，说明是反转 (位置 -1)
                4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00: pos_cnt <= pos_cnt - 1;
                // 其他情况 (没转动，或者毛刺引起的非法跳变)
                default: pos_cnt <= pos_cnt;
            endcase
        end
    end

    // ====================================================
    // 3. M法测速 (每 100ms 采样一次速度)
    // ====================================================
    // 速度 = 单位时间内的脉冲变化量
    reg [21:0] speed_timer; // 27MHz * 0.1s = 2,700,000
    reg signed [31:0] last_pos;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            speed_timer <= 0;
            speed <= 0;
            position <= 0;
            last_pos <= 0;
        end else begin
            position <= pos_cnt; // 实时向外输出位置

            // 100ms 定时器
            if (speed_timer >= 22'd2_700_000 - 1) begin
                speed_timer <= 0;
                // 当前位置减去上一次的位置，就是这 100ms 内走过的脉冲数 (有正负号)
                speed <= pos_cnt - last_pos; 
                last_pos <= pos_cnt; // 记录当前位置，留给下一次用
            end else begin
                speed_timer <= speed_timer + 1;
            end
        end
    end

endmodule