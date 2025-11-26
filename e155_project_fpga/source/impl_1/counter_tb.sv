/* 
Name: Emily Kendrick
Email: ekendrick@hmc.edu
Date created: 11/26/25
Testbench for the counter module
*/

module counter_tb();

	logic clk, reset;
	logic [23:0] num;
	logic counted;
	
counter dut(clk, reset, num, counted);

// generate clock
	always 
		begin
			// period = 10 ticks
			clk = 1; #5;
			clk = 0; #5;
		end
	
	initial begin
		reset = 1;
		num = 24'd5; #22;
		reset = 0; 	 #10;
		#30;
	end


endmodule