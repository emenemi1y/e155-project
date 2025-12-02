// Emily Kendrick
// ekendrick@hmc.edu
// 12/1/25
// Use led_driver and led_string to flash the led 

module flash_led(
	input logic clk,
	input logic rst,
	input logic [23:0] rgb,
	output logic to_light);