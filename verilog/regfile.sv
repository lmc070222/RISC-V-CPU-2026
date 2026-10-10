// regfile: 双读单写寄存器堆，写穿（同拍写可读出新值），x0 恒 0
module regfile #(
    parameter int NUM_REGS    = 32,
    parameter int READ_PORTS  = 2,   // v1 固定实现 2 读端口
    parameter int WRITE_PORTS = 1    // v1 固定实现 1 写端口
) (
    input  logic        clk,
    input  logic        reset,
    input  logic [4:0]  rs1_addr,
    input  logic [4:0]  rs2_addr,
    output logic [31:0] rs1_data,
    output logic [31:0] rs2_data,
    input  logic        we,
    input  logic [4:0]  wa,
    input  logic [31:0] wd
);

    logic [31:0] rf [0:NUM_REGS-1];

    // 写穿：we 且地址命中且非 x0 时直接透传 wd；x0 恒读 0
    assign rs1_data = (we && (wa == rs1_addr) && (rs1_addr != 5'd0)) ? wd
                    : (rs1_addr == 5'd0) ? 32'd0 : rf[rs1_addr];
    assign rs2_data = (we && (wa == rs2_addr) && (rs2_addr != 5'd0)) ? wd
                    : (rs2_addr == 5'd0) ? 32'd0 : rf[rs2_addr];

    always_ff @(posedge clk) begin
        if (we && (wa != 5'd0)) begin
            rf[wa] <= wd;
        end
    end

endmodule
