module top (input logic clk, 
			input logic [15:0] data_in,
			input logic reset,
			output logic [15:0] data_out, 
			output logic light_out);
			
	logic start, load, done, to_light;
	logic [23:0] rgb;
	led_driver led_driver1(clk, reset, start, rgb, load, done, to_light);
	/*		
	typedef enum logic [3:0] {start, setVal, writeEn, hold, writeDis, read, change};
	statetype state, nextstate;
	
	logic incVals;
	logic [3:0] we;
	logic [13:0] addr;   
	
	spram mem(clk, we, addr, data_in, data_out);
	
	always_ff @(posedge clk)
		if (~reset) state <= start;
		else state <= nextstate;
			
	// Next state logic
	always_comb 
		case(state)
			start:		nextstate = setVal;
			setVal: 	nextstate = writeEn;
			writeEn:	nextstate = hold;
			hold: 		nextstate = writeDis;
			read: 		nextstate = setVal;
			default: 	nextstate = start;
		endcase
		
	always_comb begin
		incVals = (state == change) | (state == setVals);
	*/
	
endmodule
		
		