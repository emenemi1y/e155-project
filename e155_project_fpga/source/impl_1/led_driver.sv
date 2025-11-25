// Emily Kendrick
// 11/24/25
// ekendrick@hmc.edu
// FSM for the LED light strip

module led_driver (
	input logic clk,
	input logic rst,
	input logic start, 
	input logic [23:0] rgb,
	input logic load,
	output logic to_light, done);

	typedef enum logic [3:0] {init, shift, T1H, T1L, T0H, T0L, finish} statetype;
	statetype state, nextstate;
	
	logic [23:0] num_high, num_low;
	logic counter_reset_high, counted_high;
	logic counter_reset_low, counted_low;
	// Counter for waiting 
	counter counter_high(clk, counter_reset_high, num_high, counted_high);
	counter counter_low(clk, counter_reset_low, num_low, counted_low);
	
	logic [23:0] rgb_shift; 
	logic [4:0] rgb_count;
	logic shift_bit;
	
	always_ff @(posedge clk) begin
		if (rst) begin
			state <= init;
			rgb_shift <= rgb;
			rgb_count <= 0;
		end
		else begin 
			if (shift) begin 
				rgb_shift <= {rgb[22:0], 1'b0};
				rgb_count <= rgb_count + 4'd1;
			end
			else begin
				rgb_shift <= rgb_shift;
				rgb_count <= rgb_count;
			end
			state <= nextstate;
		end
	end
	
	always_comb 
		case(state) 
			init:		if (~load)				nextstate = init;
						else if (rgb_shift[23]) nextstate = T1H;
						else                    nextstate = T0H;
			T1H:		if (~counted_high)		nextstate = T1H;
						else					nextstate = T1L;
			T1L:		if (~counted_low) 		nextstate = T1L;
						else 					nextstate = shift;
			
			T0H:		if (~counted_high)		nextstate = T0H;
						else 					nextstate = T0L;
			T0L:		if (~counted_low)		nextstate = T0L;
						else 					nextstate = shift;
			
			shift:		if (rgb_count == 5'd24) nextstate = finish;
						else					nextstate = init;
			finish:		if (~load)				nextstate = finish;
						else					nextstate = init;
		endcase
		
		
	// Output logic
	always_comb begin
		done = (state == shift && rgb_count == 5'd24);
		counter_reset_low = (~(state != T1L) && ~(state != T0L));
		counter_reset_high = (~(state == T1H) && ~(state == T0H));
		if (state == T1H) num_high = 38;
		if (state == T1L) num_low = 21;
		if (state == T0H) num_high = 40;
		if (state == T0L) num_low = 19;
		shift_bit = (state == shift);
	end

endmodule
	





