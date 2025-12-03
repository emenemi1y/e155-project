// Emily Kendrick
// 11/30/25
// ekendrick@hmc.edu
// FSM to load bits to entire LED light strip 

module led_string (
	input logic clk,
	input logic rst,
	input logic [3455:0] rgb_string, 
	input logic reset_done,
	output logic to_light, update_bits);
	
	logic load, starting;
	logic [3455:0] rgb;
	led_driver single_led(clk, rst, rgb[3455 -: 24], load, to_light, next_led);
	
	typedef enum logic [3:0] {start, load_bits, hold, finish, next} statetype;
	statetype state, nextstate;
	
	logic [9:0] led_num; 
	logic inc_num;
	
	always_ff @(posedge clk) begin
		if (rst) begin
			state <= start;
			rgb <= rgb_string;
		end
		else begin
			state <= nextstate;
		end
		if (starting)
			rgb <= rgb_string;
		else if (state == next) 
			rgb <= {rgb[3455 - 24:0], rgb[3455 -: 24]};
		else rgb <= rgb;
	end
		
	always_ff @(negedge clk) begin
		if (rst) begin
			led_num <= 0;
		end
		else begin
			if (inc_num) led_num <= led_num + 10'd1;
			if (state == finish) led_num <= 10'd0;
		end
			
	end
	
	always_comb 
		case(state) 
			start:							 nextstate = load_bits;
			load_bits: 					 	 nextstate = hold;
			hold:	 if (next_led) 			 nextstate = next;
					 else 	  				 nextstate = hold;
			next: 	 if (led_num == 10'd144) nextstate = finish;
					 else					 nextstate = load_bits;
			finish:	 if (reset_done)	     nextstate = start;
					 else 					 nextstate = finish;
			default: 						 nextstate = start;
										
		endcase
		
		
	// Output logic
	always_comb begin
		load = (state == load_bits) | (state == hold);
		inc_num = (state == next);
		starting = (state == start);
		update_bits = (state == finish);
	end

endmodule






