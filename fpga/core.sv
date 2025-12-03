/////////////////////////////////////////////
// aes
//   Top level module with SPI interface and SPI core
/////////////////////////////////////////////

module core(input  logic sck, 
           input  logic sdi,
           output logic sdo,
           input  logic load);
                    
        logic [7:0] tempReg1, tempReg2;
		logic [3:0] counter;
		logic incCount;
		logic [13:0] addr;
		assign addr  = 14'b00000000000000;
		
    // use an FSM here:
	// One stage would use the spio module to load 8 bits of data into it
	// and then second stage takes the last stages output and stores it into another 8 bit register
	// Last stage stores it into SPRAM
	
	always_ff @(posedge sck)
		if (load==0) begin
			counter = 31'b0;
			tempReg2 <= 8'b0;
		end else if (incCount == 6'd1) begin
			counter = counter + 1;
		end else begin
			counter = counter;
			tempReg2 <= tempReg1;
		end
		
	controller FSM(sck, incCount);
    sipo_shift_register spiToReg(sck, load, sdi, tempReg1);  
	SP256K spram (.AD(addr), .DI(tempReg2), .MASKWE(4'b1111), .WE(incCount), .CS(1'b1), .CK(sck), .STDBY(1'b0), .SLEEP(1'b0), .PWROFF_N(1'b1), .DO(sdo));
	
endmodule

/////////////////////////////////////////////
// aes_spi
//   SPI interface.  Shifts in key and plaintext
//   Captures ciphertext when done, then shifts it out
//   Tricky cases to properly change sdo on negedge clk
/////////////////////////////////////////////
/*
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
endmodule */
