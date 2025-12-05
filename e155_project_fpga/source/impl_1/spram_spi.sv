// spram_spi (untested)

module spram_spi(	input logic sck,  // sck from the MCU
					input logic clk,  // FPGA clock
					input  logic sdi, // Serial data in
					input logic ce,   // Chip enable
					output logic next // Indicate ready for next 16 bts
				 );

	
    logic [15:0] lightString;
               
    // On the positive edge of the clock, shift in bits during chip enable
    always_ff @(posedge sck)
		if (ce) {lightString} = {lightString[14:0], sdi};
		else lightString <= lightString;

	// Initialize SPRAM
	logic [13:0] addr;
	logic [15:0] data_in, data_out;
	logic wren;
	SP256K spram (.AD(addr), .DI(data_in), .MASKWE(4'b1111), .WE(wren), .CS(1'b1),
				   .CK (clk), .STDBY(1'b0), .SLEEP(1'b0), .PWROFF_N(1'b1), .DO(data_out));

	// Initialize FSM
	typedef enum logic [3:0] {ready, hold, store, inc, init} statetype;
	statetype state, nextstate;
	
	// Next state flip flop, save incoming data to lightString
	always_ff @(posedge clk) begin
		state <= nextstate;
		data_in <= lightString;
	end

	// Increment address:
	always_ff @(posedge clk) begin
		if (state == init) addr <= 14'b0;
		else if (state == inc) addr <= addr + 14'd1;
		else addr <= addr;
	end

	// Next state logic
	always_comb
		case (state)
			init: 					nextstate = ready;
			ready: 	if(ce) 			nextstate = hold;
					else 			nextstate = ready;
			hold: 	if(ce) 			nextstate = hold;
					else 			nextstate = store;
			store: 					nextstate = inc;
			inc: 					nextstate = ready;
	endcase

	// Output logic
	always_comb begin
		wren = (state == store);
		next = (state == ready);
	end

 
endmodule
