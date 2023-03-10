/* ***********************************************
 *  COSE222 Lab #1
 *
 *  Module: testbench including clock and reset_b signals (tb_.sv)
 *  -
 *
 *  Author: Gunjae Koo (gunjaekoo@korea.ac.kr)
 *
 **************************************************
 */

`timescale 1ns/1ps
`define CLK_T 10

module tb_regfile ();

    logic           clk, reset_b;
    logic           reg_write;
    logic [4:0]     rs1, rs2, rd;
    logic [63:0]    rd_din, rs1_dout, rs2_dout; 

    initial clk = 1'b1;
    always #(`CLK_T/2) clk = ~clk;

    initial begin
        clk = 1'b1;
        reset_b = 1'b0;
        repeat (2) @ (posedge clk);
        #(1) reset_b = 1'b1;
    end

    initial begin
        #(3) reg_write = 1'b1; rd = 5'b00000; rd_din = 0; rs1 = 5'b00000; rs2 = 5'b11111;
        repeat (32) begin
            #(10) rd++; rd_din++;
        end

        #(5) reg_write = 1'b0; rd = 5'b00000;
        repeat (32) begin
            while (rs1 < 5'b11111) begin
                #(5) rs1++;
            end
            #(5) rs1 = 5'b00000; rs2--;
        end
    end
    
    regfile dut (clk, rs1, rs2, rd, rd_din, reg_write, rs1_dout, rs2_dout);
endmodule