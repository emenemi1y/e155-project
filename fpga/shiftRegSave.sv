// Nina Jobanputra
// 12/3/24
// This should shift the incoming spi data 8 bits in at a time

module shiftRegSave (
    input clk,
	input logic load,
	input  logic sdi,
    output reg [15:0] parallel_out // 8-bit output register
);

always @(posedge clk) begin
    if (!load) begin // Asynchronous reset
        parallel_out <= 16'b0; // Reset the register to all zeros
    end else begin
        // Shift right: parallel_out[7] gets serial_in, parallel_out[6] gets parallel_out[7], etc.
        parallel_out <= {sdi, parallel_out[15:1]}; 
    end
end

endmodule


/*module ShiftRegister (
    input clk,
    input reset,
    input data_in,
    output reg [7:0] shift_reg_out
);

always @(posedge clk or posedge reset) begin
    if (reset) begin
        shift_reg_out <= 8'b0; // Reset the register to all zeros
    end else begin
        // Shift existing data to the left and introduce new data_in at the LSB
        shift_reg_out <= {shift_reg_out[6:0], data_in}; 
    end
end

endmodule */
