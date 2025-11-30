// led_string_tb
// Testbench for led_string fsm
// Author: Emily Kendrick
// Email: ekendrick@hmc.edu
// Date: 11/30/25

module led_string_tb();
	logic clk, rst;
	logic [23:0] rgb;
	logic to_light;
	
	led_string dut(clk, rst, rgb, to_light);
	
	always
		begin
			clk = 1; #5; 
			clk = 0; #5;
		end
	
	initial begin
		#12; 
		rst = 1;
		rgb = {8'd0, 8'd206, 8'd255};
		#12; 
		rst = 0; 
		#1000;
	end
endmodule
		