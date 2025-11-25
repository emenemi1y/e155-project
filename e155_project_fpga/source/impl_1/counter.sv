/* 
Name: Emily Kendrick
Email: ekendrick@hmc.edu
Date created: 11/24/25
Counts to a certain value given by "num". 
Counted goes high when the counter reaches the "num" value
*/

module counter(
	input logic clk, reset,
	input logic [23:0] num,
	output logic counted
);

	logic [23:0] count;

	// counter
	always_ff @(posedge clk) begin
		if (reset == 1) begin
			count <= 24'b0;
			counted <= 0;
		end
		else begin
			if (count == num) begin
				count <= 24'b0;
				counted <= 1;
			end
			else begin 
				count <= count + 2'b1;
				counted <= 0;
			end
		end
	end

endmodule