// Emily Kendrick
// 12/2/25
// ekendrick@hmc.edu
// FSM to load bits to LED strip and shift them one by one 

module led_shifter(input logic clk, rst, 
				   input logic [3455:0] rgb_string,
				   output logic to_light);
				   
		localparam int NUM_PIXELS = 144;
		logic [23:0] mem [0:NUM_PIXELS - 1]; // 144 lights, each 24 bits
		logic [3455:0] shifted_frame;
		// Unpack at reset: 
		always_ff @(posedge clk) begin
			if (rst) begin
				for (int i = 0; i < 144; i++)
					mem[i] <= rgb_string[i*24 +: 24];
			end
		end
		
		// Memory:
		logic [7:0] head;        // 0...143 pointer
		logic update_bits;
		// Rotation each frame
		always_ff @(posedge clk) begin
			if (rst) begin
				head <= 0;
			end else if (update_bits) begin
				head <= (head == NUM_PIXELS - 1) ? 0 : head + 1;
			end
		end
		
		// Build rotated 3456 bit frame
		always_comb begin
			for (int i = 0; i < NUM_PIXELS; i++) begin
				int src = head + i;
				if (src > NUM_PIXELS) src -= NUM_PIXELS;
				shifted_frame[i*24 +: 24] = mem[src];
			end
		end
		
		// LED string driver
		logic counter_reset, reset_done;
		counter counter_for_reset(clk, counter_reset, 24'd10000, reset_done);

		led_string led_string1(clk, rst, shifted_frame, reset_done, to_light, update_bits);	
		
		// Define states
		typedef enum logic [3:0] {start, play, shift, delay} statetype;
		statetype state, nextstate;
		
		always_ff @(posedge clk) begin
			if (rst) state <= start;
			else state <= nextstate;
		end

		// Nextstate logic
		always_comb 
			case(state) 
				start:					 nextstate = play;
				play:	if (update_bits) nextstate = shift;
						else			 nextstate = play;
				shift:					 nextstate = delay;
				delay:
						if (reset_done)  nextstate = play;
						else			 nextstate = delay;
				default:				 nextstate = start;
			endcase
			
		// Output logic
		always_comb begin
			counter_reset = (state != delay);
			
		end

endmodule
