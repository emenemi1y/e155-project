// led_shifter_tb
// Testbench for led_shifter fsm
// Author: Emily Kendrick
// Email: ekendrick@hmc.edu
// Date: 12/2/25

module led_shifter_tb();
	logic clk, rst;
	logic to_light;
	logic [23:0] color;
	logic [3455:0] color_string;
	
	assign color = {8'd0, 8'd206, 8'd255};
	assign color_string = {24{{3{color}}, {3{24'd0}}}};
	led_shifter dut(clk, rst, color_string, to_light);
	
	
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
		