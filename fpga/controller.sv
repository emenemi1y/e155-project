module controller(input logic clk,
				  output logic incCount);
	
	typedef enum logic [1:0] {loadInTemp, loadInTemp2, saveToSpram} statetype;
	statetype state, nextstate;
	always_ff @(posedge clk) begin
		state <= nextstate;
	end
	
	always_comb begin 
		case (state) 
			loadInTemp:  		nextstate = loadInTemp2;
			loadInTemp2:		nextstate = saveToSpram;
			saveToSpram:		nextstate = loadInTemp;
			
			default: nextstate = loadInTemp;
		endcase
	end
		
	always_comb begin
		case(state)
			loadInTemp:		incCount = 1'b0;
			loadInTemp2:	incCount = 1'b0;
			saveToSpram:	incCount = 1'b1;
			default: incCount = 1'b0;
		endcase
	end
	
endmodule