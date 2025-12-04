// Emily Kendrick
// update_rgb_string.sv
// Updates the rgb string when a card is tapped 

module update_rgb_string(input logic clk,
						 input logic [5:0] id,
						 input logic song_start,
						 output logic [143:0] rgb);
						 
	
	
	logic [143:0] new_led_string;
	
	always_ff @(posedge clk)
		if (song_start) rgb <= new_led_string;
		else rgb <= rgb;
	
	always_comb
		case(id)
			6'd1: new_led_string = {{8'd28, 8'd252, 8'd3}, {8'd159, 8'd245, 8'd47}, {8'd245, 8'd252, 8'd23}, {8'd252, 8'd92, 8'd23}, {8'd141,8'd23,8'd252}, {8'd23, 8'd157, 8'd252}};
			6'd2: new_led_string = {{8'd127, 8'd50, 8'd168}, {8'd127, 8'd50, 8'd168}, {8'd127, 8'd50, 8'd168}, {8'd159, 8'd245, 8'd47}, {8'd159, 8'd245, 8'd47}, {8'd159, 8'd245, 8'd47}};
		endcase
		
endmodule
						 