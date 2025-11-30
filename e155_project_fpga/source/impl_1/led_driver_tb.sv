// led_driver_tb
// Testbench for led_driver fsm
// Author: Emily Kendrick
// Email: ekendrick@hmc.edu
// Date: 11/26/25

module led_driver_tb();
	logic clk, rst;
	logic start, load;
	logic [23:0] rgb;
	logic to_light, done;
	
	led_driver dut(clk, rst, rgb, load, to_light, done);
	
	always
		begin
			clk = 1; #5; 
			clk = 0; #5;
		end
	
	initial begin
		#12; 
		rst = 1;
		rgb = {8'd0, 8'd206, 8'd255};
		load = 1;
		#12; 
		rst = 0; 
		#10;
		load = 0;
		#1000;
	end
endmodule
		