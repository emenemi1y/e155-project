// led_driver_tb
// Testbench for led_driver fsm
// Author: Emily Kendrick
// Email: ekendrick@hmc.edu
// Date: 11/26/25

module led_driver_tb();
	logic clk, rst;
	logic start, load;
	logic [143:0] color_string;
	logic [23:0] color1, color2;
	assign color1 = {8'd0, 8'd206, 8'd255};
	assign color2 = {8'd127, 8'd50, 8'd168};
	assign color_string ={color1, color1, color1,    
         color2, color2, color2};   
	logic to_light, done;
	
	led_driver dut(clk, rst, 1'b0, color_string, load, to_light, done);
	
	always
		begin
			clk = 1; #5; 
			clk = 0; #5;
		end
	
	initial begin
		#12; 
		// Reset LEDs and set color
		rst = 1;
		load = 1;
		#12; 
		rst = 0; 
		#10;
		load = 0;
		#1000;
	end
endmodule
		