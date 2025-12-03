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
	
	// Convert 3456-bit input to RAM
	localparam int NUM_PIXELS = 144;
	logic [23:0] mem [0:NUM_PIXELS - 1]; // 144 lights, each 24 bits
	
	// Rotation base index
	logic [7:0] base_head;
	logic [7:0] idx;
	logic [7:0] addr;
	logic [8:0] sum_addr;
	
	// FSM variables
	typedef enum logic [2:0] {start, load_bits, hold, finish, next} statetype;
	statetype state, nextstate;
	
	logic next_led;
	logic load;
	logic inc_num;
	
	logic [9:0] led_num;

	
	// Unpack at reset: 
	
	always_ff @(posedge clk) begin
		if (rst) begin
			for (int i = 0; i < NUM_PIXELS; i = i + 1)
				// Unpack MSB first
				mem[i] <= rgb_string[3455 - i * 24 -: 24];
		end
		else if (state == start) begin
			// Load new frame into memory when a new frame is starting 
			for (int i = 0; i < NUM_PIXELS; i = i + 1)
				mem[i] <= rgb_string[3455 - i * 24 -: 24];
		end
	end
	
	// Address computing
	assign sum_addr = base_head + idx;
	always_comb begin
		if (sum_addr >= NUM_PIXELS)
			addr = sum_addr - NUM_PIXELS;
		else
			addr = sum_addr[7:0];
	end
	
	// Current pixel output
	logic [23:0] pixel_out;
	assign pixel_out = mem[addr];
	
	led_driver single_led(clk, rst, pixel_out, load, to_light, next_led);


	// state register and sequential behavior
	always_ff @(posedge clk) begin
		if (rst) begin
			state <= start;
			base_head <= 0;
			idx <= 0;
			led_num <= 0;
			update_bits <= 1'b0;
		end
		else begin
			state <= nextstate;
			if (state == load_bits && next_led) begin
				if (idx == NUM_PIXELS - 1)
					idx <= 0;
				else
					idx <= idx + 1;
				
				led_num <= led_num + 1;
				
			end
			// When full frame finished increment base head once
			if (led_num == NUM_PIXELS) begin
				led_num <= 0;
				base_head <= (base_head == NUM_PIXELS - 1) ? 0 : (base_head + 1);
				update_bits <= 1;
			end else begin
				update_bits <= 1'b0;
			end
		end
	end
	/*	
	always_ff @(negedge clk) begin
		if (rst) begin
			led_num <= 0;
		end
		else begin
			if (inc_num) led_num <= led_num + 10'd1;
			if (state == finish) led_num <= 10'd0;
		end
			
	end
	*/
	// Next state logic and load control 
	
	always_comb begin
		
		case(state) 
			start:							 nextstate = load_bits;
			load_bits: 					 	 nextstate = hold;
			hold:	 if (next_led) 			 nextstate = next;
					 else 	  				 nextstate = hold;
			next: 	 if (led_num == NUM_PIXELS) nextstate = finish;
					 else					 nextstate = load_bits;
			finish:	 if (reset_done)	     nextstate = start;
					 else 					 nextstate = finish;
			default: 						 nextstate = start;
										
		endcase
	end
		
		
	// Output logic
	always_comb begin
		load = (state == load_bits) | (state == hold);
		inc_num = (state == next);
		update_bits = (state == finish);
	end

endmodule






