module clk_gen(
  input clk_100MHz,
  input rst,
  output clk_25MHz,
  output clk_250MHz,
  output mmcm_lock

);

wire clkfb;
wire pll_lock;

MMCME2_BASE #(
       .BANDWIDTH("OPTIMIZED"), // Jitter programming (OPTIMIZED, HIGH, LOW)
       .DIVCLK_DIVIDE(1),       // Master division value (1-106)
       .CLKFBOUT_MULT_F(10.0),  // Multiply value for all CLKOUT (2.000-64.000)
       .CLKFBOUT_PHASE(0.0),    // Phase offset in degrees of CLKFB (-360.000-360.000)
       .CLKIN1_PERIOD(10.000),  // Input clock period in ns to ps resolution (i.e. 33.333 is 30 MHz)

      //CLKOUT0_DIVIDE - CLKOUT6_DIVIDE: Divide amount for each CLKOUT (1-128)
       .CLKOUT0_DIVIDE_F(4.0),  // Divide amount for CLKOUT0 (1.000-128.000)
       .CLKOUT1_DIVIDE(40),
       .CLKOUT2_DIVIDE(1),
       .CLKOUT3_DIVIDE(1),
       .CLKOUT4_DIVIDE(1),
       .CLKOUT5_DIVIDE(1),
       .CLKOUT6_DIVIDE(1),

      //CLKOUT0_DUTY_CYCLE - CLKOUT6_DUTY_CYCLE: Duty cycle for each CLKOUT (0.01-0.99)
       .CLKOUT0_DUTY_CYCLE(0.5),
       .CLKOUT1_DUTY_CYCLE(0.5),
       .CLKOUT2_DUTY_CYCLE(0.5),
       .CLKOUT3_DUTY_CYCLE(0.5),
       .CLKOUT4_DUTY_CYCLE(0.5),
       .CLKOUT5_DUTY_CYCLE(0.5),
       .CLKOUT6_DUTY_CYCLE(0.5),

      //CLKOUT0_PHASE - CLKOUT6_PHASE: Phase offset for each CLKOUT (-360.000-360.000)
       .CLKOUT0_PHASE(0.0),
       .CLKOUT1_PHASE(0.0),
       .CLKOUT2_PHASE(0.0),
       .CLKOUT3_PHASE(0.0),
       .CLKOUT4_PHASE(0.0),
       .CLKOUT5_PHASE(0.0),
       .CLKOUT6_PHASE(0.0),
               
       .CLKOUT4_CASCADE("FALSE"), // Cascade CLKOUT4 counter with CLKOUT6 (FALSE, TRUE)
       .REF_JITTER1(0.0),         // Reference input jitter in UI (0.000-0.999)
       .STARTUP_WAIT("FALSE")     // Delays DONE until MMCM is locked (FALSE, TRUE)
    ) 
    u_MMCME2_BASE(
       .CLKOUT0(clk_250MHz),       // 1-bit output: CLKOUT0
       .CLKOUT1(clk_25MHz),        // 1-bit output: CLKOUT2
       .CLKFBOUT(clkfb),          // 1-bit output: Feedback clock
       .LOCKED(pll_lock),         // 1-bit output: LOCK
       .CLKFBIN(clkfb),            // 1-bit input: Feedback clock
.CLKIN1(clk_100MHz),
    .PWRDWN(1'b0)       
    );


assign mmcm_lock = pll_lock;

endmodule

module TMDS_encoder(
	input clk,
	input [7:0] VD,  // video data (red, green or blue)
	input [1:0] CD,  // control data
	input VDE,  // video data enable, to choose between CD (when VDE=0) and VD (when VDE=1)
	output reg [9:0] TMDS = 0
    );

    wire [3:0] Nb1s = VD[0] + VD[1] + VD[2] + VD[3] + VD[4] + VD[5] + VD[6] + VD[7];
    wire XNOR = (Nb1s>4'd4) || (Nb1s==4'd4 && VD[0]==1'b0);
    wire [8:0] q_m = {~XNOR, q_m[6:0] ^ VD[7:1] ^ {7{XNOR}}, VD[0]};

    reg [3:0] balance_acc = 0;
    wire [3:0] balance = q_m[0] + q_m[1] + q_m[2] + q_m[3] + q_m[4] + q_m[5] + q_m[6] + q_m[7] - 4'd4;
    wire balance_sign_eq = (balance[3] == balance_acc[3]);
    wire invert_q_m = (balance==0 || balance_acc==0) ? ~q_m[8] : balance_sign_eq;
    wire [3:0] balance_acc_inc = balance - ({q_m[8] ^ ~balance_sign_eq} & ~(balance==0 || balance_acc==0));
    wire [3:0] balance_acc_new = invert_q_m ? balance_acc-balance_acc_inc : balance_acc+balance_acc_inc;
    wire [9:0] TMDS_data = {invert_q_m, q_m[8], q_m[7:0] ^ {8{invert_q_m}}};
    wire [9:0] TMDS_code = CD[1] ? (CD[0] ? 10'b1010101011 : 10'b0101010100) : (CD[0] ? 10'b0010101011 : 10'b1101010100);

    always @(posedge clk) TMDS <= VDE ? TMDS_data : TMDS_code;
    always @(posedge clk) balance_acc <= VDE ? balance_acc_new : 4'h0;
endmodule

module HDMI_test(
	input clk,  // 24MHz
	input rst,
	// output [2:0] TMDSp,
	// output TMDSp_clock,
  output [2:0] hdmi_tx_p, //hdmi output signals (positives) (blue, green, red)
  output [2:0] hdmi_tx_n, //hdmi output signals (negatives) (blue, green, red)
  output hdmi_clk_p, 
  output hdmi_clk_n //differential hdmi clock
    );

    ////////////////////////////////////////////////////////////////////////
    wire clk_TMDS;
    wire pixclk;
    wire mmcm_lock;

    clk_gen clkgenInstance(.clk_100MHz(clk),
                    .rst(rst),
                    .clk_250MHz(clk_TMDS),
                    .clk_25MHz(pixclk),
                    .mmcm_lock(mmcm_lock));


    ////////////////////////////////////////////////////////////////////////
    reg [9:0] CounterX, CounterY;
    reg hSync, vSync, DrawArea;
    always @(posedge pixclk) DrawArea <= (CounterX<640) && (CounterY<480);

    always @(posedge pixclk) begin
      if (!mmcm_lock) CounterX <= 10'b0;
      else CounterX <= (CounterX==799) ? 0 : CounterX+1;
      end

    always @(posedge pixclk) begin
      
      if (!mmcm_lock) CounterY <= 10'b0;
      else if(CounterX==799) CounterY <= (CounterY==524) ? 0 : CounterY+1;

    end

    always @(posedge pixclk) hSync <= (CounterX>=656) && (CounterX<752);
    always @(posedge pixclk) vSync <= (CounterY>=490) && (CounterY<492);

    ////////////////////////////////////////////////////////////////////////
    wire [7:0] W = {8{CounterX[7:0]==CounterY[7:0]}};
    wire [7:0] A = {8{CounterX[7:5]==3'h2 && CounterY[7:5]==3'h2}};
    reg [7:0] red, green, blue;
    always @(posedge pixclk) red <= ({CounterX[5:0] & {6{CounterY[4:3]==~CounterX[4:3]}}, 2'b00} | W) & ~A;
    always @(posedge pixclk) green <= (CounterX[7:0] & {8{CounterY[6]}} | W) & ~A;
    always @(posedge pixclk) blue <= CounterY[7:0] | W | A;

    ////////////////////////////////////////////////////////////////////////
    wire [9:0] TMDS_red, TMDS_green, TMDS_blue;
    TMDS_encoder encode_R(.clk(pixclk), .VD(red  ), .CD(2'b00)        , .VDE(DrawArea), .TMDS(TMDS_red));
    TMDS_encoder encode_G(.clk(pixclk), .VD(green), .CD(2'b00)        , .VDE(DrawArea), .TMDS(TMDS_green));
    TMDS_encoder encode_B(.clk(pixclk), .VD(blue ), .CD({vSync,hSync}), .VDE(DrawArea), .TMDS(TMDS_blue));

    ////////////////////////////////////////////////////////////////////////
    reg [3:0] TMDS_mod10=0;  // modulus 10 counter
    reg [9:0] TMDS_shift_red=0, TMDS_shift_green=0, TMDS_shift_blue=0;
    reg TMDS_shift_load=0;
    always @(posedge clk_TMDS) TMDS_shift_load <= (TMDS_mod10==4'd9);

    always @(posedge clk_TMDS)
    begin
        TMDS_shift_red   <= TMDS_shift_load ? TMDS_red   : TMDS_shift_red  [9:1];
        TMDS_shift_green <= TMDS_shift_load ? TMDS_green : TMDS_shift_green[9:1];
        TMDS_shift_blue  <= TMDS_shift_load ? TMDS_blue  : TMDS_shift_blue [9:1];	
        TMDS_mod10 <= (TMDS_mod10==4'd9) ? 4'd0 : TMDS_mod10+4'd1;
    end

    // assign TMDSp[2] = TMDS_shift_red;
    // assign TMDSp[1] = TMDS_shift_green;
    // assign TMDSp[0] = TMDS_shift_blue;
    // assign TMDSp_clock = pixclk;


  OBUFDS OBUFDS_blue (.I(TMDS_shift_blue[0]), .O(hdmi_tx_p[0]), .OB(hdmi_tx_n[0]));
  OBUFDS OBUFDS_green(.I(TMDS_shift_green[0]), .O(hdmi_tx_p[1]), .OB(hdmi_tx_n[1]));
  OBUFDS OBUFDS_red  (.I(TMDS_shift_red[0]), .O(hdmi_tx_p[2]), .OB(hdmi_tx_n[2]));
  OBUFDS OBUFDS_clock(.I(pixclk), .O(hdmi_clk_p), .OB(hdmi_clk_n));

endmodule
