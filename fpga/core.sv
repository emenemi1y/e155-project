/////////////////////////////////////////////
// aes
//   Top level module with SPI interface and SPI core
/////////////////////////////////////////////

module aes(input  logic clk,
           input  logic sck, 
           input  logic sdi,
           output logic sdo,
           input  logic load);
                    
        logic [7:0] parallel_out;
    // use an FSM here:
	// One stage would use the spio module to load 8 bits of data into it
	// and then second stage takes the last stages output and stores it into another 8 bit register
	// Last stage stores it into SPRAM
    sipo_shift_register (sck, load, sdi, sdo, parallel_out);  
endmodule
endmodule

/////////////////////////////////////////////
// aes_spi
//   SPI interface.  Shifts in key and plaintext
//   Captures ciphertext when done, then shifts it out
//   Tricky cases to properly change sdo on negedge clk
/////////////////////////////////////////////

module aes_spi(input  logic sck, 
               input  logic sdi,
               output logic sdo,
               input  logic done,
               output logic [431:0] stringLight,
               input  logic [431:0] cyphertext);

    logic         sdodelayed, wasdone;
    logic [127:0] cyphertextcaptured;
               
    // assert load
    // apply 256 sclks to shift in key and plaintext, starting with plaintext[127]
    // then deassert load, wait until done
    // then apply 128 sclks to shift out cyphertext, starting with cyphertext[127]
    // SPI mode is equivalent to cpol = 0, cpha = 0 since data is sampled on first edge and the first
    // edge is a rising edge (clock going from low in the idle state to high).
    always_ff @(posedge sck)
        if (!wasdone)  {cyphertextcaptured, plaintext, key} = {cyphertext, plaintext[126:0], key, sdi};
        else           {cyphertextcaptured, plaintext, key} = {cyphertextcaptured[126:0], plaintext, key, sdi}; 
    
    // sdo should change on the negative edge of sck
    always_ff @(negedge sck) begin
        wasdone = done;
        sdodelayed = cyphertextcaptured[126];
    end
    
    // when done is first asserted, shift out msb before clock edge
    assign sdo = (done & !wasdone) ? cyphertext[127] : sdodelayed;
endmodule


module sipo_shift_register (
    input sck,
	input logic load,
	input  logic sdi,
    output logic sdo,
    output reg [7:0] parallel_out // 8-bit output register
);

always @(posedge clk or negedge rst_n) begin
    if (!load) begin // Asynchronous reset
        parallel_out <= 8'b0; // Reset the register to all zeros
    end else begin
        // Shift right: parallel_out[7] gets serial_in, parallel_out[6] gets parallel_out[7], etc.
        parallel_out <= {serial_in, parallel_out[7:1]}; 
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
