module led_memory(
    input  logic        clk,
    input  logic        we,
    input  logic [7:0]  addr,
    input  logic [23:0] din,
    output logic [23:0] dout
);

    (* ram_style="block" *) logic [23:0] mem [0:143];

    always_ff @(posedge clk) begin
        if (we)
            mem[addr] <= din;
        dout <= mem[addr];
    end
endmodule