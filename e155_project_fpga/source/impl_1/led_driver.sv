// Emily Kendrick
// 11/24/25
// ekendrick@hmc.edu
// FSM for the LED light strip

module led_driver (
	input logic clk,			 	// clk
	input logic rst, rst_leds,		// reset signal and signal to indicate that the leds should be reset
	input logic [143:0] rgb,		// 144 bits (corresponding to 6 LEDs) to send to the LEDs (24 bits: 8 bits for green + 8 bits for red + 8 bits for blue)
	input logic load,				// Load = start shifting rgb bits 
	output logic to_light, done);	// to_light is the output PWM signal, done indicates that all 144 bits have been shifted through

	// FSM definitions
	typedef enum logic [4:0] {init, shift, T1H, T1L, T0H, T0L, finish, next, hold} statetype;
	statetype state, nextstate;
	
	// Counter definitions for waiting during PWM cycle
	logic [31:0] num_high, num_low;
	logic counter_reset_high, counted_high;
	logic counter_reset_low, counted_low;
	
	counter counter_high(clk, counter_reset_high, num_high, counted_high);
	counter counter_low(clk, counter_reset_low, num_low, counted_low);
	
	// RGB definitions
	logic [143:0] rgb_shift; 
	logic [10:0] rgb_count;
	logic update_bits;
	
	// Nextstate flip flop
	always_ff @(posedge clk) begin
		if (rst) begin
			state <= init;
		end
		else begin 
			state <= nextstate;
		end
	end
	
	// Set timing for PWM signals
	always_ff @(posedge clk) begin
		if (rst) begin
			if (rgb[23]) num_high <= 24'd38;
			else num_high <= 24'd40;
		end
		else begin
			if (state == T1H) num_high <= 24'd28;
			if (state == T1L) num_low <= 24'd29;
			if (state == T0H) num_high <= 24'd11;
			if (state == T0L) num_low <= 24'd46; 
		end
	end
	
	// Shift the rgb signal
	always_ff @(posedge clk) begin
		if (rst | update_bits) begin
			rgb_shift <= rgb;
			rgb_count <= 0;
		end
		else begin
			if (nextstate == shift) begin
				rgb_shift <= {rgb_shift[142:0], 1'b0};
				if (done) rgb_count <= 10'b0;
				rgb_count <= rgb_count + 10'd1;
			end
			else begin 
				rgb_shift <= rgb_shift;
				rgb_count <= rgb_count;
			end
		end
	end
	
	// Next state logic
	always_comb 
		case(state) 
			init:		if (~load)					nextstate = init;
						else if (rgb_shift[143]) 	nextstate = T1H;
						else                    	nextstate = T0H;
			T1H:		if (~counted_high)			nextstate = T1H;
						else						nextstate = T1L;
			T1L:		if (~counted_low) 			nextstate = T1L;
						else 						nextstate = shift;
			
			T0H:		if (~counted_high)			nextstate = T0H;
						else 						nextstate = T0L;
			T0L:		if (~counted_low)			nextstate = T0L;
						else 						nextstate = shift;
			
			shift:		if (rgb_count == 10'd144)	nextstate = finish;
						else						nextstate = hold;
			finish:		 							nextstate = next;
			next: 		if (~load)					nextstate = next;
						else						nextstate = init;
			hold:		if (rgb_shift[143])      	nextstate = T1H;
						else						nextstate = T0H;
			default:								nextstate = init;
						 
		endcase
		
		
	// Output logic
	always_comb begin
		done = (state == finish);
		counter_reset_low = (state != T1L) && (state != T0L);
		counter_reset_high = (state != T1H) && (state != T0H);
		to_light = ((state == T1H) | (state == T0H));
		if (rst_leds) to_light = 0;
		update_bits = (state == next);
	end

endmodule
	



