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

module tb_alu ();

    logic           clk, reset_b;
    logic           zero;
    logic [3:0]     alu_control;
    logic [63:0]    in1, in2, result;

    initial clk = 1'b1;
    always #(`CLK_T/2) clk = ~clk;

    initial begin
        clk = 1'b1;
        reset_b = 1'b0;
        repeat (2) @ (posedge clk);
        #(1) reset_b = 1'b1;
    end

    // change needed
    initial begin
        in1 = 64'hffffffffffffffff; in2 = 64'h0000000000000000;
        #(3) repeat (100) begin
            #(5) alu_control = 4'b0000;
            #(5) alu_control = 4'b0001;
            #(5) alu_control = 4'b0010;
            #(5) alu_control = 4'b0110; in1 -= 500; in2 += 300;
        end
    end

    alu dut (in1, in2, alu_control, result, zero);
endmodule