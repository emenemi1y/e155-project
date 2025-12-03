// Emily Kendrick
// 12/2/25
// ekendrick@hmc.edu
// FSM to load bits to LED strip and shift them one by one 

module led_shifter (
	input logic clk,
	input logic rst,
	input logic [143:0] rgb,
	output logic to_light);
	
	logic load, done;
	led_driver single_led(clk, rst, rgb, load, to_light, done);
	
	typedef enum logic [3:0] {start, load_bits, hold, finish, next} statetype;
	statetype state, nextstate;
	
	logic [9:0] led_num; 
	logic inc_num;
	
	always_ff @(posedge clk) begin
		if (rst) begin
			state <= start;
		end
		else begin
			state <= nextstate;
		end
	end
		
	always_ff @(negedge clk) begin
		if (rst) begin
			led_num <= 0;
		end
		else begin
			if (inc_num) led_num <= led_num + 10'd1;
		end
	end
	
	always_comb 
		case(state) 
			start:							 nextstate = load_bits;
			load_bits: 					 	 nextstate = hold;
			hold:	 if (done) 				 nextstate = next;
					 else 	  				 nextstate = hold;
			next: 	 if (led_num == 10'd24) nextstate = finish;
					 else					 nextstate = load_bits;
			finish:	 					     nextstate = finish;
			default: 						 nextstate = start;
										
		endcase
		
		
	// Output logic
	always_comb begin
		load = (state == load_bits) | (state == hold);
		inc_num = (state == next);
		
	end

endmodule
	
/*
module led_shifter(input logic clk, shifter_reset, 
				   input logic [143:0] rgb_string,
				   output logic to_light);
				   

		// led and rotation counters
		logic [9:0] led_num; // 0 -> 5
		
		// Counter for reset
		logic counter_reset, reset_done;
		counter counter_for_reset(clk, counter_reset, 24'd2400, reset_done);
		
		logic update_bits, load;
		logic [143:0] rgb;
		logic [143:0] current_led;
		led_driver led_driver1(clk, shifter_reset, rgb, load, to_light, update_bits);	
		
		// Define states
		typedef enum logic [3:0] {start, load_bits, hold, finish, next, delay} statetype;
		statetype state, nextstate;
		// logic [143:0] rgb_string_shift;
		
		// Nextstate
		always_ff @(posedge clk) begin
			if (shifter_reset) begin
				state <= start;				
			end
			else begin
				state <= nextstate;
			end
		end
		
		always_ff @(negedge clk) begin
			if (shifter_reset | state == start) led_num <= 10'd0;
			else if (state == next) begin
				if (led_num >= 10'd143) led_num <= 10'd0;
				else led_num <= led_num + 10'd1;
			end
		end
					
				
		
		// Nextstate logic
		always_comb 
			case(state) 
				start:					 nextstate = load_bits;
				load_bits:			     nextstate = hold;
				hold:	if (update_bits) nextstate = next;
						else			 nextstate = hold;
				next:	if (led_num == 10'd24) nextstate = finish;
						else				nextstate = load_bits;
				delay:
						if (reset_done)  nextstate = load_bits;
						else			 nextstate = delay;
				finish: 				 nextstate = finish;
				default:				 nextstate = start;
			endcase
			
		// Output logic
		always_comb begin
			counter_reset = (state != delay);
			load = (state == load_bits) | (state == hold);
		end

endmodule
*/