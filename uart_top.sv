module uart_top (
    input  wire        CLOCK_50,
    input  wire [3:0]  KEY,
    output wire [9:0]  LEDR,

    output wire [6:0]  HEX0,
    output wire [6:0]  HEX1,
    output wire [6:0]  HEX2,
    output wire [6:0]  HEX3,
    output wire [6:0]  HEX4,
    output wire [6:0]  HEX5,

    input  wire        UART_TX,   // PIN_E9 (PC->FPGA)
    output wire        UART_RX    // PIN_D9 (FPGA->PC)
);

    wire rst = ~KEY[0];

    wire [7:0] rx_tdata;
    wire       rx_tvalid;
    wire       rx_tready;

    reg  [7:0] tx_data;
    reg        tx_valid;
    wire       tx_tready;

    wire tx_busy, rx_busy, rx_overrun_error, rx_frame_error;

    uart #(.DATA_WIDTH(8)) uart_inst (
        .clk(CLOCK_50),
        .rst(rst),

        .i_data_s(tx_data),
        .i_valid_s(tx_valid),
        .o_ready_s(tx_tready),

        .o_data_m(rx_tdata),
        .o_valid_m(rx_tvalid),
        .i_ready_m(rx_tready),

        .i_rxd(UART_TX),
        .o_txd(UART_RX),

        .o_tx_busy(tx_busy),
        .o_rx_busy(rx_busy),
        .o_rx_overrun_error(rx_overrun_error),
        .o_rx_frame_error(rx_frame_error)
    );

    // =========================
    // RX: any byte -> flash LEDR0
    // =========================
    reg [23:0] led_timer;
    always @(posedge CLOCK_50) begin
        if (rst) begin
            led_timer <= 24'd0;
        end else begin
            if (rx_tvalid) begin
                led_timer <= 24'd5_000_000; // ~0.1s
            end else if (led_timer != 0) begin
                led_timer <= led_timer - 1'b1;
            end
        end
    end

	 //debug
    assign LEDR[0] = (led_timer != 0);
    assign LEDR[1] = tx_busy;
    assign LEDR[2] = rx_frame_error;
    assign LEDR[3] = rx_overrun_error;
    assign LEDR[8:4] = 5'b0;
    assign LEDR[9] = ~rst;

    reg key1_d;
    wire key1_pulse = key1_d & ~KEY[1];

    always @(posedge CLOCK_50) begin
        if (rst) key1_d <= 1'b1;
        else     key1_d <= KEY[1];
    end

    // =========================
    // TX message sender
    // =========================
    function [7:0] msg_byte(input [5:0] i);
        begin
            case (i)
                0:  msg_byte = "H";
                1:  msg_byte = "e";
                2:  msg_byte = "l";
                3:  msg_byte = "l";
                4:  msg_byte = "o";
                5:  msg_byte = ",";
                6:  msg_byte = "W";
                7:  msg_byte = "o";
                8:  msg_byte = "r";
                9:  msg_byte = "l";
                10: msg_byte = "d";
					 11: msg_byte = "!";
					 12: msg_byte = "-";
                13: msg_byte = "D";
                14: msg_byte = "E";
                15: msg_byte = "1";
                16: msg_byte = "-";
                17: msg_byte = "S";
                18: msg_byte = "o";
                19: msg_byte = "C";
                20: msg_byte = "!";
                21: msg_byte = 8'h0D; // \r
                22: msg_byte = 8'h0A; // \n
                default: msg_byte = 8'h00;
            endcase
        end
    endfunction
    localparam MSG_LEN = 23;

    reg        sending;
    reg [5:0]  idx;

    always @(posedge CLOCK_50) begin
        if (rst) begin
            tx_data  <= 8'd0;
            tx_valid <= 1'b0;
            sending   <= 1'b0;
            idx       <= 6'd0;
        end else begin
            if (tx_valid && tx_tready) begin
                tx_valid <= 1'b0;
            end

            if (!sending && key1_pulse) begin
                sending <= 1'b1;
                idx     <= 6'd0;
            end

            if (sending) begin
                if (!tx_valid && tx_tready) begin
                    tx_data  <= msg_byte(idx);
                    tx_valid <= 1'b1;

                    if (idx == MSG_LEN-1) sending <= 1'b0;
                    else                  idx <= idx + 1'b1;
                end
            end
        end
    end

    // =========================
    // 6-digit 7-seg: show received ASCII digits '0'..'9'
    // =========================
    reg [3:0] dig0, dig1, dig2, dig3, dig4, dig5;
    reg       has0, has1, has2, has3, has4, has5;

    always @(posedge CLOCK_50) begin
        if (rst) begin
            dig0 <= 4'd0; dig1 <= 4'd0; dig2 <= 4'd0; dig3 <= 4'd0; dig4 <= 4'd0; dig5 <= 4'd0;
            has0 <= 1'b0; has1 <= 1'b0; has2 <= 1'b0; has3 <= 1'b0; has4 <= 1'b0; has5 <= 1'b0;
        end else if (rx_tvalid) begin
            if (rx_tdata >= 8'h30 && rx_tdata <= 8'h39) begin
                // shift old digits towards HEX5, put newest on HEX0
                dig5 <= dig4; has5 <= has4;
                dig4 <= dig3; has4 <= has3;
                dig3 <= dig2; has3 <= has2;
                dig2 <= dig1; has2 <= has1;
                dig1 <= dig0; has1 <= has0;
                dig0 <= rx_tdata[3:0]; has0 <= 1'b1;
            end
        end
    end

    // active-low 7-seg decoder (a,b,c,d,e,f,g)
    function [6:0] seg7(input [3:0] d);
        begin
            case (d)
                4'd0: seg7 = 7'b1000000;
                4'd1: seg7 = 7'b1111001;
                4'd2: seg7 = 7'b0100100;
                4'd3: seg7 = 7'b0110000;
                4'd4: seg7 = 7'b0011001;
                4'd5: seg7 = 7'b0010010;
                4'd6: seg7 = 7'b0000010;
                4'd7: seg7 = 7'b1111000;
                4'd8: seg7 = 7'b0000000;
                4'd9: seg7 = 7'b0010000;
                default: seg7 = 7'b1111111;
            endcase
        end
    endfunction

    assign HEX0 = has0 ? seg7(dig0) : 7'b1111111;
    assign HEX1 = has1 ? seg7(dig1) : 7'b1111111;
    assign HEX2 = has2 ? seg7(dig2) : 7'b1111111;
    assign HEX3 = has3 ? seg7(dig3) : 7'b1111111;
    assign HEX4 = has4 ? seg7(dig4) : 7'b1111111;
    assign HEX5 = has5 ? seg7(dig5) : 7'b1111111;

endmodule
