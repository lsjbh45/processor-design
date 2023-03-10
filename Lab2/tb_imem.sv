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

module tb_imem ();

    logic           clk, reset_b;
    logic [9:0]     addr;
    logic [31:0]    dout;

    initial clk = 1'b1;
    always #(`CLK_T/2) clk = ~clk;

    initial begin
        clk = 1'b1;
        reset_b = 1'b0;
        repeat (2) @ (posedge clk);
        #(1) reset_b = 1'b1;
    end

    initial begin
        #(3) addr = 10'h000;
        while(addr < 10'h3ff) begin
            #(10) addr++;
        end
    end

    imem dut (addr, dout);
endmodule