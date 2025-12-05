// Emily Kendrick
// song_timing.sv
// ekendrick@hmc.edu
// 12/3/25
// Handles song timing based on communcation with MCU

module song_timing(input logic clk, reset, song_start, // Song start is a signal from the MCU that goes high when a card is detected
				    output logic [9:0] angle 			// angle 0 to 180, except 0 corresponds to -90
				   );
					
		
		// Define state variables
		typedef enum logic [3:0] {start, down, up, hold, down_again} statetype;
		statetype state, nextstate;
		
		// Counters
		logic counter_reset;
		logic counted;
		logic [31:0] count_num;
		assign count_num = 32'd48000000;
		
		counter counter1(clk, counter_reset, count_num, counted);
		
		
		
		// Next state flip flop
		always_ff @(posedge clk) 
			if (reset) state <= start;
			else state <= nextstate;
		
		// Next state logic
		always_comb
			case(state)
				start: 					nextstate = down;
				down: if (song_start)	nextstate = up;
					  else 				nextstate = down;
				up:  					nextstate = hold;
				hold: if (counted)      nextstate = down;
					  else              nextstate = hold;
				default:				nextstate = start;
			endcase
		
		// Counter variables and output logic
		always_comb begin
			counter_reset = (state != hold);
			if ((state == up) | (state == hold)) angle = 10'd0;
			else angle = 10'd180;
		end
		
endmodule

