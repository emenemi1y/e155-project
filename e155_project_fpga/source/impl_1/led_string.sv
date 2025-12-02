// Emily Kendrick
// 11/30/25
// ekendrick@hmc.edu
// FSM to load bits to entire LED light strip 

module led_string (
	input logic clk,
	input logic rst,
	input logic [3455:0] led_string,
	output logic to_light);
	
	logic load, done;
	logic [23:0] rgb;
	led_driver single_led(clk, rst, rgb, load, to_light, done);
	
	typedef enum logic [4:0] {start, load_bits, hold, finish, next, update_string, reset} statetype;
	statetype state, nextstate;
	
	logic [9:0] led_num; 
	logic inc_num;
	logic [3455:0] led_string_shift;
	
	// Reset counter:
	logic counter_reset_rst, reset_done;
	counter counter_reset(clk, counter_reset_rst, 24'd2400, reset_done);
	
	always_ff @(posedge clk) begin
		if (rst) begin
			state <= start;
			led_string_shift <= led_string;
			rgb <= led_string[3455-23+:24];
			led_num <= 0;
		end
		else begin
			if (state == start) begin
				led_string_shift <= led_string;
				rgb <= led_string[3455-23+:24];
			end
			else rgb <= rgb;
				
			if (nextstate == update_string) begin
				led_string_shift <= {led_string_shift[3455-24:0], led_string_shift[3455-24+:24]};
				
			end
			else led_string_shift <= led_string_shift;
				
			if (nextstate == next) begin
				rgb <= led_string[3455-24*led_num-24+:24];
				led_num <= led_num + 10'd1;
			end
			else begin 
				rgb <= rgb;
				led_num <= led_num;
			end
			
			if (nextstate == reset) led_num <= 0;
				
				
			state <= nextstate;
			
		end
	end
	/*	
	always_ff @(negedge clk) begin
		if (rst) begin
			led_num <= 0;
		end
		else begin
			if (inc_num) led_num <= led_num + 10'd1;
		end
	end
	*/
	
	always_comb 
		case(state) 
			start:							 nextstate = load_bits;
			load_bits: 					 	 nextstate = hold;
			hold:	 if (done) 				 nextstate = next;
					 else 	  				 nextstate = hold;
			next: 	 if (led_num == 10'd144) nextstate = update_string;
					 else					 nextstate = load_bits;
			update_string:	 			     nextstate = reset;
			reset:	 if (reset_done)		 nextstate = load_bits;
					 else                    nextstate = reset;
			default: 						 nextstate = start;
										
		endcase
		
		
	// Output logic
	always_comb begin
		load = (state == load_bits) | (state == hold);
		inc_num = (state == next);
		counter_reset_rst = (state != reset);
		
	end

endmodule
	





