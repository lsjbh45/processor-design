`timescale 1ns/1ps

module tb_and2();
    logic [1:0] a, b, c;
    
    initial begin
        a = 2'b00; b = 2'b00;
        #(10) a = 2'b11; b = 2'b00;
        #(10) a = 2'b11; b = 2'b01;
        #(10) a = 2'b11; b = 2'b10;
        #(10) a = 2'b11; b = 2'b11;
    end
    
    and2 dut (a, b, c);
endmodule
