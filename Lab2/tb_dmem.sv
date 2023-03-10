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

module tb_dmem ();

    logic           clk, reset_b;
    logic           mem_read, mem_write;
    logic [9:0]     addr;
    logic [63:0]    din, dout;

    initial clk = 1'b1;
    always #(`CLK_T/2) clk = ~clk;

    initial begin
        clk = 1'b1;
        reset_b = 1'b0;
        repeat (2) @ (posedge clk);
        #(1) reset_b = 1'b1;
    end

    initial begin
        #(3) mem_write = 1; mem_read = 0; addr = 0; din = 0; dout = 0;
        repeat (10) begin
            while (addr < 10'h00f) begin
                #(5) addr++; din++;
            end
            #(5) addr = 0; din++; mem_write = ~mem_write; mem_read = ~mem_read;
            if (mem_read) #(5) din++;
        end
    end

    dmem dut (clk, addr, din, mem_read, mem_write, dout);
endmodule