// led_string_tb
// Testbench for led_string fsm
// Author: Emily Kendrick
// Email: ekendrick@hmc.edu
// Date: 11/30/25

module led_string_tb();
	logic clk, rst;
	logic [23:0] rgb;
	logic to_light;
	logic [23:0] color;
	logic [3455:0] color_string;
	
	assign color = {8'd0, 8'd206, 8'd255};
	assign color_string = {24{{3{color}}, {3{24'd0}}}};
	led_string dut(clk, rst, color_string, to_light);
	
	
	always
		begin
			clk = 1; #5; 
			clk = 0; #5;
		end
	
	initial begin
		rst = 1;
		#25; 
		rst = 0; 
		#1000;
	end
endmodule
		