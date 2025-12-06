// Emily Kendrick
// 12/4/25
// ekendrick@hmc.edu
// Top level module for controlling LEDs and servo and communicating with the MCU

module top (input song_start,				// Signal sent from the MCU indicating that a card has been tapped and a song is starting
			output logic servo_signal,		// PWM signal that gets sent to the servo
			output logic to_light			// PWM signal that gets sent to the LED string
			);
			
	logic clk; 
	
	// 24 MHz clock 
	HSOSC #(.CLKHF_DIV(2'b01))
		hf_osc(.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(clk));	
				
	logic load, reset;
	logic [23:0] color1, color2;
	logic [143:0] color_string;
	logic [143:0] color_string_shift;
	
	// Color string for sending one color pattern to the lights (since getting colors from the MCU is untested)
	assign color_string = {{8'd28, 8'd252, 8'd3}, {8'd159, 8'd245, 8'd47}, {8'd245, 8'd252, 8'd23}, {8'd252, 8'd92, 8'd23},
							{8'd141,8'd23,8'd252}, {8'd23, 8'd157, 8'd252}};
	
	// LED shifter 
	logic shifter_reset;
	led_shifter led_shifter1(clk, reset, 1'b1, color_string, to_light);
	
	// Servo logic
	logic [9:0] angle; 
	//song_timing song_timing1(clk, reset, song_start, angle);
	//servo_driver servo(clk, reset, angle, servo_signal);
	
endmodule
		
		