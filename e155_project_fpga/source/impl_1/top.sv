module top (output logic to_light);
			
	logic clk;
	logic [13:0] addr;
	logic [15:0] data_in, data_out;
	logic wren;
	
	
	// 24 MHz clock 
	HSOSC #(.CLKHF_DIV(2'b01))
		hf_osc(.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(clk));	
	
	// Instantiate SPRAM
	SP256K spram (.AD(addr), .DI(data_in), .MASKWE(4'b1111), .WE(wren), .CS(1'b1), .CK(clk), .STDBY(1'b0), .SLEEP(1'b0), .PWROFF_N(1'b1), .DO(data_out));

	
			
	logic load, reset;
	logic [23:0] color1, color2;
	logic [3455:0] color_string;
	logic [3455:0] color_string_shift;
		
	assign color1 = {8'd0, 8'd206, 8'd255};
	assign color2 = {8'd127, 8'd50, 8'd168};
	assign color_string = {
    24 {
        {color1, color1, color1,    // 3× color1
         color2, color2, color2}    // 3× color2
    }
};
	
	led_shifter led_shifter1(clk, reset, color_string, to_light);
	

	
	
	
	/*
	typedef enum logic [5:0] {init, set_data, load_data, write_disable, read_data, next, go} statetype;
	statetype state, nextstate;
	
	always_ff @(posedge clk)
		state <= nextstate;
		
	always_ff @(negedge clk) begin
		if (state == init) begin 
			data_count <= 4'd0;
			read_count <= 4'd0;
			addr <= 14'd0;
			rgb <= 24'd0;
		end
		else if (state == write_disable) begin 
			data_count <= data_count + 4'd1;
			read_count <= read_count;
			addr <= addr + 4'd1;
		end
		else if (state == next) begin
			data_count <= data_count;
			read_count <= read_count + 4'd1;
			if (addr == 14'd1) addr <= addr - 4'd1;
			else addr <= addr;
		end
		else begin 
			data_count <= data_count;
			addr <= addr;
		end
		
		if ((state == read_data) & (data_count == 4'd0)) rgb[15:0] <= data_out;
		else if ((state == read_data) & (data_count = 4'd1)) rgb[23:16] <= data_out[7:0];
		else rgb <= rgb;
		
		
	end
	
	always_comb
		case(state)
			init:			nextstate = set_data;
			set_data:		nextstate = load_data;
			load_data:		nextstate = write_disable;
			write_disable: 	if (data_count == 4'd1) nextstate = read_data;
							else					nextstate = set_data;
			read_data:		nextstate = next;
			next:     		if (read_count == 4'd1) nextstate = go;
							else nextstate = read_data;
			go: 		   	nextstate = go;
			default: 	   	nextstate = init;
		endcase
	
	always_comb begin
		rgb_start = {8'd0, 8'd206, 8'd255};
		if (((state == set_data) | (state == load_data)) & (data_count == 4'd0)) data_in = rgb_start[15:0];
		else if (((state == set_data) | (state == load_data)) & (data_count == 4'd1)) data_in = {8'd0, rgb_start[23:16]};
		else data_in = 16'd0;
		
		
		reset = (state != go);
		wren = (state == load_data);
	end
	*/	
	
	//led_driver led_driver1(clk, reset, rgb, load, done, to_light, hold);
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
		
		