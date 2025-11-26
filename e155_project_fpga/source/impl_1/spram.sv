// spram256 user modules //

module spram #( 
				localparam WIDTH=16,
				localparam DEPTH=16384,
				localparam ADDRW=$clog2(DEPTH) ) (
				
			    input logic clk,
			    input logic [3:0] we,
			    input logic [ADDRW-1:0] addr,
			    input logic [WIDTH-1:0] data_in,
			    output logic [WIDTH-1:0] data_out);

		SB_SPRAM256KA ramfn_inst1(
		 .DATAIN(data_in),
		 .ADDRESS(addr),
		 .MASKWREN(we),
		 .WREN(|we), // write enable -- if writing a bit at least one of we (for maskwren) will be high
		 .CHIPSELECT(1'b1),
		 .CLOCK(clk),
		 .STANDBY(1'b0),
		 .SLEEP(1'b0),
		 .POWEROFF(1'b1),
		 .DATAOUT(data_out)
		);
	

/*
		typedef enum logic [3:0] {start, write, stop_write, read, inc, hold} statetype;
		statetype state, nextstate; 
		
		logic to_read;
		
		always_ff @(posedge clk) begin
			if (rst) state <= start;
			else state <= nextstate;
		end
	
		// Next state logic
		always_comb begin
			case(state) 
				start:			nextstate = write;
				write:			nextstate = stop_write;
				stop_write:		nextstate = read;
				inc: 			nextstate = hold;
				hold:			nextstate = write;
			endcase
		end
		
		// Output logic
		always_comb
			next = (state == inc);
*/
				
endmodule