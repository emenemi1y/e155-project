// Emily Kendrick
// 12/2/25
// ekendrick@hmc.edu
// FSM to load bits to LED strip and shift them one by one 

module led_shifter (
	input logic clk,
	input logic rst,
	input logic go,				// Go signal from MCU to start shifting lights
	input logic [143:0] rgb,	// 6 LEDs worth of color data
	output logic to_light		// Output signal to the LEDs
	);
	
	// Single_led driver (takes one string of 144 bits and sends them to the LED string)
	logic load, done, rst_leds;
	logic [143:0] rgb_shifted;
	led_driver single_led(clk, rst, rst_leds, rgb_shifted, load, to_light, done);
	
	// FSM definitions
	typedef enum logic [3:0] {start, load_bits, hold, finish, next, shift, delay} statetype;
	statetype state, nextstate;
	
	logic [9:0] led_num;
	
	// Counter for delay for resetting LEDs
	logic delay_reset, delay_done;
	logic [31:0] delay_num;
	assign delay_num = 32'd720000;
	counter delay_counter(clk, delay_reset, delay_num, delay_done);
	
	// Nextstate flip flop
	always_ff @(posedge clk) begin
		if (rst) begin
			state <= start;
		end
		else begin
			state <= nextstate;
		end
	end
		
	// Shift RGB string (so it looks like the lights are "sliding" across the strip)
	always_ff @(posedge clk) begin
		if (rst) begin
			led_num <= 0;
		end
		else begin
			if (state == start) rgb_shifted <= rgb;
			else begin 
				if (nextstate == next) led_num <= led_num + 10'd1;
				if (nextstate == shift) begin 
					led_num <= 10'd0; 
					rgb_shifted <= {rgb_shifted[119:0], rgb_shifted[143:120]};
				end
			end
		end
	end
	
	// Nextstate logic 
	always_comb begin
		case(state) 
			start:	 if (go)				 nextstate = load_bits;
					 else    				 nextstate = start;
			load_bits: 					 	 nextstate = hold;
			hold:	 if (done) 				 nextstate = next;
					 else 	  				 nextstate = hold;
			next: 	 if (led_num == 10'd24)  nextstate = shift;
					 else					 nextstate = load_bits;
			shift:							 nextstate = delay;
			delay:	 if (delay_done)		 nextstate = load_bits;
					 else					 nextstate = delay;
			finish:	 					     nextstate = finish;
			default: 						 nextstate = start;
										
		endcase
	end
		
	// Output logic
	always_comb begin
		load = (state == load_bits) | (state == hold);
		rst_leds = (state == delay);
		delay_reset = (state != delay);
	end

endmodule
	
