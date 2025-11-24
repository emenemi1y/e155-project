module led_driver #(LED_COUNT = 144) // The number of LEDs in the chain
(
	input clk,
	input rst,
	input update,
	input color[24],
	input clear,
	output data,
	output next_pixel,
	output reset,
	output done
) {


