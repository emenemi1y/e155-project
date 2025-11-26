/*
Name: Emily Kendrick
Date created: 11/26/25
Email: ekendrick@hmc.edu
spram_tb.sv
Testbench for spram module
*/

module spram_tb();
	
	logic clk, we;
	logic [13:0] addr;
	logic [15:0] data_in, data_out;
	spram dut(clk, we, addr, data_in, data_out);
	
	// generate clock
	always 
		begin
			// period = 10 ticks
			clk = 1; #5;
			clk = 0; #5;
		end
	
	initial begin
		we = 0;
		data_in = 15'd4567;
		addr = 14'd1;
		#22; 
		we = 1;
		#22; 
		we = 0;
		#50;
		data_in = 15'd5;
		addr = 14'd2;
		#10;
		we = 1;
		#30;
		we = 0;
		#20;
		addr = 14'd1;
		#30;
	end
endmodule