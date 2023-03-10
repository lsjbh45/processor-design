`timescale 1ns/1ps

module tb_add2();
    logic [7:0] a, b, c;
    
    initial begin
        a = 8'h00; b = 8'hff;
        while(a < 8'hff) begin
            #(10) a++; b--;
        end
    end
    
    add2 dut (a, b, c);
endmodule


