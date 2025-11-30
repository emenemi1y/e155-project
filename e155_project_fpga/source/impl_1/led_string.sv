// Emily Kendrick
// 11/30/25
// ekendrick@hmc.edu
// FSM to load bits to entire LED light strip 

module led_string (
	input logic clk,
	input logic rst,
	input logic [23:0] rgb,
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
			next: 	 if (led_num == 10'd144) nextstate = finish;
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
	





