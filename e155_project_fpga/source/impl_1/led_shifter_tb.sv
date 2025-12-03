// led_shifter_tb
// Testbench for led_shifter fsm
// Author: Emily Kendrick
// Email: ekendrick@hmc.edu
// Date: 12/2/25

module led_shifter_tb();
	logic clk, rst;
	logic to_light;
	logic [23:0] color1, color2;
	logic [143:0] color_string;
	
	assign color1 = {8'd0, 8'd206, 8'd255};
	assign color2 = {8'd127, 8'd50, 8'd168};
	assign color_string ={color1, color1, color1,    // 3× color1
         color2, color2, color2};    // 3× color2
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
		