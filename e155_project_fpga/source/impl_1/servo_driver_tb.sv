// servo_driver_tb
// Testbench for servo_driver fsm
// Author: Emily Kendrick
// Email: ekendrick@hmc.edu
// Date: 12/3/25

module servo_driver_tb();
	logic clk, rst;
	logic [9:0] angle;
	logic servo_signal;
	

	servo_driver dut(clk, rst, angle, servo_signal);
	
	
	always
		begin
			clk = 1; #5; 
			clk = 0; #5;
		end
	
	initial begin
		rst = 1;
		angle = 10'd180;
		#25; 
		rst = 0; 
		#1000;
	end
endmodule
		