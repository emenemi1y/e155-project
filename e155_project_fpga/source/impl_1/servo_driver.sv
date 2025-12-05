// Emily Kendrick
// servo_driver.sv
// ekendrick@hmc.edu
// 12/3/25
// Servo PWM logic -- -90 degrees corresponds to 1 ms / 20 ms PWM, 0 degrees is 1.5 ms / 20 ms PWM, 90 degrees is 2 ms / 20 ms PWM

module servo_driver(input logic clk, reset,
				    input logic [9:0] angle,  // angle 0 to 180, except 0 corresponds to -90
					output logic servo_signal // Output logic to the servo
					);
					
		
		// Define state variables
		typedef enum logic [2:0] {start, high, low} statetype;
		statetype state, nextstate;
		
		// Counters
		logic counter_reset_high, counter_reset_low;
		logic counted_high, counted_low;
		logic [31:0] high_num, low_num;
		
		counter counter_high(clk, counter_reset_high, high_num, counted_high);
		counter counter_low(clk, counter_reset_low, low_num, counted_low);
		
		// Next state flip flop
		always_ff @(posedge clk) 
			if (reset) state <= start;
			else state <= nextstate;
				
		logic [31:0] high_num_r, low_num_r;

		always_ff @(posedge clk) begin
			if (reset) begin
				high_num <= 32'd24000;
				low_num  <= 32'd456000;
			end else begin
				high_num <= angle * 32'd133 + 32'd24000; 
				low_num  <= 32'd480000 - high_num;
			end
		end
		
		// Next state logic
		always_comb
			case(state)
				start:					nextstate = high;
				high: if(counted_high)	nextstate = low;
					  else 				nextstate = high;
				low:  if(counted_low)	nextstate = high;
					  else 				nextstate = low;
				default:				nextstate = start;
			endcase
		
		// Counter variables and output logic
		always_comb begin
			counter_reset_high = (state != high);
			counter_reset_low = (state != low);
			servo_signal = (state == high);
		end
		
endmodule

