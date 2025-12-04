module top (input song_start,
			output logic servo_signal,
			output logic to_light);
			
	logic clk;
	//logic [13:0] addr;
	//logic [15:0] data_in, data_out;
	//logic wren;
	 
	
	// 24 MHz clock 
	HSOSC #(.CLKHF_DIV(2'b01))
		hf_osc(.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(clk));	
	
	// Instantiate SPRAM
	// SP256K spram (.AD(addr), .DI(data_in), .MASKWE(4'b1111), .WE(wren), .CS(1'b1), .CK(clk), .STDBY(1'b0), .SLEEP(1'b0), .PWROFF_N(1'b1), .DO(data_out));

	
			
	logic load, reset;
	logic [23:0] color1, color2;
	logic [143:0] color_string;
	logic [143:0] color_string_shift;
		
	assign color_string = {{8'd28, 8'd252, 8'd3}, {8'd159, 8'd245, 8'd47}, {8'd245, 8'd252, 8'd23}, {8'd252, 8'd92, 8'd23},
							{8'd141,8'd23,8'd252}, {8'd23, 8'd157, 8'd252}};
	//assign color2 = {8'd127, 8'd50, 8'd168};
	//assign color_string ={color1, color1, color1,    // 3x color1
         //color2, color2, color2};    // 3x color2
	
	logic shifter_reset;
	led_shifter led_shifter1(clk, reset, song_start, color_string, to_light);
	
	
	logic [9:0] angle;
	song_timing song_timing1(clk, reset, song_start, angle);
	servo_driver servo(clk, reset, angle, servo_signal);
	
	/*
	typedef enum logic [2:0] {start, go} statetype;
	statetype state, nextstate;
	always_ff @(posedge clk) begin
		state <= nextstate;
	end
	always_comb 
		case (state) 
			start:  nextstate = go;
			go:		nextstate = go;
			default: nextstate = start;
	
		endcase
		
	always_comb 
		shifter_reset = (state == start);
	
	
	logic load, done, reset;
	logic [3:0] data_count, read_count;
	logic [23:0] rgb, rgb_start;
	
	led_shifter led_string1(clk, reset, {8'd0, 8'd206, 8'd255}, to_light);
	*/
	
endmodule
		
		