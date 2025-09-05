module block_sprite #(
  parameter WIDTH=128, HEIGHT=128, COLOR=24'hFF_FF_FF)(
  input [10:0] hcount_in,
  input [9:0] vcount_in,
  input [10:0] x_in,
  input [9:0]  y_in,
  output reg [7:0] red_out,
  output reg [7:0] green_out,
  output reg [7:0] blue_out);

  wire in_sprite;
  assign in_sprite = ((hcount_in >= x_in && hcount_in < (x_in + WIDTH)) &&
                      (vcount_in >= y_in && vcount_in < (y_in + HEIGHT)));
  always@(*) begin
    if (in_sprite)begin
      red_out = COLOR[23:16];
      green_out = COLOR[15:8];
      blue_out = COLOR[7:0];
    end else begin
      red_out = 0;
      green_out = 0;
      blue_out = 0;
    end
  end
endmodule


// file: clk_wiz_0.v
//
// (c) Copyright 2008 - 2013 Xilinx, Inc. All rights reserved.
//
// This file contains confidential and proprietary information
// of Xilinx, Inc. and is protected under U.S. and
// international copyright and other intellectual property
// laws.
//
// DISCLAIMER
// This disclaimer is not a license and does not grant any
// rights to the materials distributed herewith. Except as
// otherwise provided in a valid license issued to you by
// Xilinx, and to the maximum extent permitted by applicable
// law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
// WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
// AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
// BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
// INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
// (2) Xilinx shall not be liable (whether in contract or tort,
// including negligence, or under any other theory of
// liability) for any loss or damage of any kind or nature
// related to, arising under or in connection with these
// materials, including for any direct, or any indirect,
// special, incidental, or consequential loss or damage
// (including loss of data, profits, goodwill, or any type of
// loss or damage suffered as a result of any action brought
// by a third party) even if such damage or loss was
// reasonably foreseeable or Xilinx had been advised of the
// possibility of the same.
//
// CRITICAL APPLICATIONS
// Xilinx products are not designed or intended to be fail-
// safe, or for use in any application requiring fail-safe
// performance, such as life-support or safety devices or
// systems, Class III medical devices, nuclear facilities,
// applications related to the deployment of airbags, or any
// other applications that could lead to death, personal
// injury, or severe property or environmental damage
// (individually and collectively, "Critical
// Applications"). Customer assumes the sole risk and
// liability of any use of Xilinx products in Critical
// Applications, subject only to applicable laws and
// regulations governing limitations on product liability.
//
// THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
// PART OF THIS FILE AT ALL TIMES.
//
//----------------------------------------------------------------------------
// User entered comments
//----------------------------------------------------------------------------
// None
//
//----------------------------------------------------------------------------
//  Output     Output      Phase    Duty Cycle   Pk-to-Pk     Phase
//   Clock     Freq (MHz)  (degrees)    (%)     Jitter (ps)  Error (ps)
//----------------------------------------------------------------------------
// clk_pixel__74.25000______0.000______50.0______337.616____322.999
// clk_tmds__371.25000______0.000______50.0______258.703____322.999
//
//----------------------------------------------------------------------------
// Input Clock   Freq (MHz)    Input Jitter (UI)
//----------------------------------------------------------------------------
// __primary_________100.000____________0.010

`timescale 1ps/1ps

module hdmi_clk_wiz_720p

 (// Clock in ports
  // Clock out ports
  output        clk_pixel,
  output        clk_tmds,
  // Status and control signals
  input         reset,
  output        locked,
  input         clk_ref
 );
  // Input buffering
  //------------------------------------
wire clk_ref_clk_wiz_0;
wire clk_in2_clk_wiz_0;
  IBUF clkin1_ibufg
   (.O (clk_ref_clk_wiz_0),
    .I (clk_ref));




  // Clocking PRIMITIVE
  //------------------------------------

  // Instantiation of the MMCM PRIMITIVE
  //    * Unused inputs are tied off
  //    * Unused outputs are labeled unused

  wire        clk_pixel_clk_wiz_0;
  wire        clk_tmds_clk_wiz_0;
  wire        clk_out3_clk_wiz_0;
  wire        clk_out4_clk_wiz_0;
  wire        clk_out5_clk_wiz_0;
  wire        clk_out6_clk_wiz_0;
  wire        clk_out7_clk_wiz_0;

  wire [15:0] do_unused;
  wire        drdy_unused;
  wire        psdone_unused;
  wire        locked_int;
  wire        clkfbout_clk_wiz_0;
  wire        clkfbout_buf_clk_wiz_0;
  wire        clkfboutb_unused;
    wire clkout0b_unused;
   wire clkout1b_unused;
   wire clkout2_unused;
   wire clkout2b_unused;
   wire clkout3_unused;
   wire clkout3b_unused;
   wire clkout4_unused;
  wire        clkout5_unused;
  wire        clkout6_unused;
  wire        clkfbstopped_unused;
  wire        clkinstopped_unused;
  wire        reset_high;

  MMCME2_ADV
  #(.BANDWIDTH            ("OPTIMIZED"),
    .CLKOUT4_CASCADE      ("FALSE"),
    .COMPENSATION         ("ZHOLD"),
    .STARTUP_WAIT         ("FALSE"),
    .DIVCLK_DIVIDE        (5),
    .CLKFBOUT_MULT_F      (37.125),
    .CLKFBOUT_PHASE       (0.000),
    .CLKFBOUT_USE_FINE_PS ("FALSE"),
    .CLKOUT0_DIVIDE_F     (10.000),
    .CLKOUT0_PHASE        (0.000),
    .CLKOUT0_DUTY_CYCLE   (0.500),
    .CLKOUT0_USE_FINE_PS  ("FALSE"),
    .CLKOUT1_DIVIDE       (2),
    .CLKOUT1_PHASE        (0.000),
    .CLKOUT1_DUTY_CYCLE   (0.500),
    .CLKOUT1_USE_FINE_PS  ("FALSE"),
    .CLKIN1_PERIOD        (10.000))
  mmcm_adv_inst
    // Output clocks
   (
    .CLKFBOUT            (clkfbout_clk_wiz_0),
    // .CLKFBOUTB           (clkfboutb_unused),
    .CLKOUT0             (clk_pixel_clk_wiz_0),
    // .CLKOUT0B            (clkout0b_unused),
    .CLKOUT1             (clk_tmds_clk_wiz_0),
    // .CLKOUT1B            (clkout1b_unused),
    // .CLKOUT2             (clkout2_unused),
    // .CLKOUT2B            (clkout2b_unused),
    // .CLKOUT4             (clkout4_unused),
    // .CLKOUT5             (clkout5_unused),
    // .CLKOUT3             (clkout3_unused),
    // .CLKOUT3B            (clkout3b_unused),

    // .CLKOUT6             (clkout6_unused),
     // Input clock control
    .CLKFBIN             (clkfbout_buf_clk_wiz_0),
    .CLKIN1              (clk_ref_clk_wiz_0),
    .CLKIN2              (1'b0),
     // Tied to always select the primary input clock
    .CLKINSEL            (1'b1),
    // Ports for dynamic reconfiguration
    .DADDR               (7'h0),
    .DCLK                (1'b0),
    .DEN                 (1'b0),
    .DI                  (16'h0),
    .DO                  (do_unused),
    .DRDY                (drdy_unused),
    .DWE                 (1'b0),
    // Ports for dynamic phase shift
    .PSCLK               (1'b0),
    .PSEN                (1'b0),
    .PSINCDEC            (1'b0),
    .PSDONE              (psdone_unused),
    // Other control and status signals
    .LOCKED              (locked_int),
    .CLKINSTOPPED        (clkinstopped_unused),
    .CLKFBSTOPPED        (clkfbstopped_unused),
    .PWRDWN              (1'b0),
    .RST                 (reset_high));
  assign reset_high = reset;

  assign locked = locked_int;
// Clock Monitor clock assigning
//--------------------------------------
 // Output buffering
  //-----------------------------------

  BUFG clkf_buf
   (.O (clkfbout_buf_clk_wiz_0),
    .I (clkfbout_clk_wiz_0));

  BUFG clkout1_buf
   (.O   (clk_pixel),
    .I   (clk_pixel_clk_wiz_0));

  BUFG clkout2_buf
   (.O   (clk_tmds),
    .I   (clk_tmds_clk_wiz_0));

endmodule

`timescale 1ns / 1ps

module tmds_serializer (
    input wire clk_pixel_in,
    input wire clk_5x_in,
    input wire rst_in,
    input wire [9:0] tmds_in,
    output tmds_out
);
  wire [1:0] linker;

  // this is requried for OSERDESE2 to work
  reg pwup_rst = 1'b1;
  always @(posedge clk_pixel_in)begin
      pwup_rst <= 1'b0;
  end

  OSERDESE2 #(
      .DATA_RATE_OQ("DDR"),
      .DATA_RATE_TQ("SDR"),
      .DATA_WIDTH(10),
      .SERDES_MODE("MASTER"),
      .TRISTATE_WIDTH(1),
      .TBYTE_CTL("FALSE"),
      .TBYTE_SRC("FALSE")
  ) primary (
      .OQ(tmds_out),
      .OFB(),
      .TQ(),
      .TFB(),
      .SHIFTOUT1(),
      .SHIFTOUT2(),
      .TBYTEOUT(),
      .CLK(clk_5x_in),
      .CLKDIV(clk_pixel_in),
      .D1(tmds_in[0]),
      .D2(tmds_in[1]),
      .D3(tmds_in[2]),
      .D4(tmds_in[3]),
      .D5(tmds_in[4]),
      .D6(tmds_in[5]),
      .D7(tmds_in[6]),
      .D8(tmds_in[7]),
      .TCE(1'b0),
      .OCE(1'b1),
      .TBYTEIN(1'b0),
      .RST(rst_in || pwup_rst),
      .SHIFTIN1(linker[0]),
      .SHIFTIN2(linker[1]),
      .T1(1'b0),
      .T2(1'b0),
      .T3(1'b0),
      .T4(1'b0)
  );
  OSERDESE2 #(
      .DATA_RATE_OQ("DDR"),
      .DATA_RATE_TQ("SDR"),
      .DATA_WIDTH(10),
      .SERDES_MODE("SLAVE"),
      .TRISTATE_WIDTH(1),
      .TBYTE_CTL("FALSE"),
      .TBYTE_SRC("FALSE")
  ) secondary (
      .OQ(),
      .OFB(),
      .TQ(),
      .TFB(),
      .SHIFTOUT1(linker[0]),
      .SHIFTOUT2(linker[1]),
      .TBYTEOUT(),
      .CLK(clk_5x_in),
      .CLKDIV(clk_pixel_in),
      .D1(1'b0),
      .D2(1'b0),
      .D3(tmds_in[8]),
      .D4(tmds_in[9]),
      .D5(1'b0),
      .D6(1'b0),
      .D7(1'b0),
      .D8(1'b0),
      .TCE(1'b0),
      .OCE(1'b1),
      .TBYTEIN(1'b0),
      .RST(rst_in || pwup_rst),
      .SHIFTIN1(1'b0),
      .SHIFTIN2(1'b0),
      .T1(1'b0),
      .T2(1'b0),
      .T3(1'b0),
      .T4(1'b0)
  );
endmodule

module test_pattern_generator(
  input wire [1:0] sel_in,
  input wire [10:0] hcount_in,
  input wire [9:0] vcount_in,
  output reg [7:0] red_out,
  output reg [7:0] green_out,
  output reg [7:0] blue_out
  );


  always @(*) begin

    case(sel_in)
        2'b00: begin
            
            red_out = 8'hFF;
            green_out = 8'h00;
            blue_out = 8'h00;

        end

        2'b01: begin

            if (hcount_in < 11'd400) begin
                red_out = 8'hFF;
                green_out = 8'h00;
                blue_out = 8'h00;
            end

            else if (hcount_in < 11'd800) begin
                red_out = 8'h00;
                green_out = 8'hFF;
                blue_out = 8'h00;
            end

            else begin
                red_out = 8'h00;
                green_out = 8'h00;
                blue_out = 8'hFF;
            end

        end

        2'b10: begin

            if (vcount_in < 11'd300) begin
                red_out = 8'hFF;
                green_out = 8'h00;
                blue_out = 8'h00;
            end

            else if (vcount_in < 11'd600) begin
                red_out = 8'h00;
                green_out = 8'hFF;
                blue_out = 8'h00;
            end

            else begin
                red_out = 8'h00;
                green_out = 8'h00;
                blue_out = 8'hFF;
            end

        end

        2'b11: begin

            red_out = hcount_in % 256;
            green_out = vcount_in % 256;
            blue_out = 8'hF0;

        end

    endcase

  end
endmodule

module tm_choice (
  input wire [7:0] data_in,
  output reg [8:0] qm_out
);

localparam OPTION1 = 1'b0;
localparam OPTION2 = 1'b1;

// check option 0 or option 1 //

reg [3:0] number_of_ones;
reg option;
integer i;

always @(*) begin

  number_of_ones = 1'b0;
  for (i = 0; i < 8; i = i + 1) begin
    number_of_ones = number_of_ones + data_in[i];
  end

end


always @(*) begin

  if ((number_of_ones > 3'd4) || ((number_of_ones == 3'd4) && (data_in[0] == 1'b0))) begin
    option = OPTION2;
  end

  else begin
    option = OPTION1;
  end

end

// produce output based on option //

always @(*) begin

  if (option == OPTION1) begin

    qm_out[0] = data_in[0];
    qm_out[1] = qm_out[0] ^ data_in[1];
    qm_out[2] = qm_out[1] ^ data_in[2];
    qm_out[3] = qm_out[2] ^ data_in[3];
    qm_out[4] = qm_out[3] ^ data_in[4];
    qm_out[5] = qm_out[4] ^ data_in[5];
    qm_out[6] = qm_out[5] ^ data_in[6];
    qm_out[7] = qm_out[6] ^ data_in[7];
    qm_out[8] = 1'b1;

  end

  else begin

    qm_out[0] = data_in[0];
    qm_out[1] = qm_out[0] ~^ data_in[1];
    qm_out[2] = qm_out[1] ~^ data_in[2];
    qm_out[3] = qm_out[2] ~^ data_in[3];
    qm_out[4] = qm_out[3] ~^ data_in[4];
    qm_out[5] = qm_out[4] ~^ data_in[5];
    qm_out[6] = qm_out[5] ~^ data_in[6];
    qm_out[7] = qm_out[6] ~^ data_in[7];
    qm_out[8] = 1'b0;

  end


end




endmodule //end tm_choice

`timescale 1ns / 1ps

module tmds_encoder(
	  input wire clk_in,
	  input wire rst_in,
	  input wire [7:0] data_in,  // video data (red, green or blue)
	  input wire [1:0] control_in, //for blue set to {vs,hs}, else will be 0
	  input wire ve_in,  // video data enable, to choose between control or video signal0	 
    output reg [9:0] tmds_out
);

wire [8:0] q_m;

	 
tm_choice mtm(
  .data_in(data_in),
  .qm_out(q_m)
);

wire qm8rev;

assign qm8rev = ~q_m[8]; // hack 

reg [4:0] tally; // can be positive or negative based on MSB //

reg [3:0] number_of_ones;
reg [3:0] number_of_zeros;
integer i;

always @(*) begin

  number_of_ones = 4'b0;
  number_of_zeros = 4'b0;

  for (i = 0; i < 8; i = i + 1) begin
    number_of_ones = number_of_ones + q_m[i];
    number_of_zeros = number_of_zeros + ~(q_m[i]);
  end

end


always @(posedge clk_in) begin

  if (rst_in) begin

    tmds_out <= 10'b0;
    tally <= 5'b0;

  end
  else if (ve_in == 1'b0) begin

    case (control_in)
      2'b00: tmds_out <= 10'b1101010100;
      2'b01: tmds_out <= 10'b0010101011;
      2'b10: tmds_out <= 10'b0101010100;
      2'b11: tmds_out <= 10'b1010101011;

    endcase

    tally <= 5'b0;
  end

  else begin // just follow the HDMI original diagram //

    if ((tally == 5'b0) || (number_of_ones == number_of_zeros)) begin

      // update tmds //
      tmds_out[9] <= ~q_m[8];
      tmds_out[8] <= q_m[8];

      for (i = 0; i < 8; i = i + 1) begin
        tmds_out[i] <= (q_m[8] ? q_m[i] : ~q_m[i]);
      end

      // update tally //
      if (q_m[8] == 1'b0) begin
        tally <= tally + (number_of_zeros - number_of_ones);
      end
      else begin
        tally <= tally + (number_of_ones - number_of_zeros);
      end

    end

    else begin 

      if (((tally[4] == 1'b0) && (number_of_ones > number_of_zeros)) || 
          ((tally[4] == 1'b1) && (number_of_zeros > number_of_ones))) begin

        tmds_out[9] <= 1'b1;
        tmds_out[8] <= q_m[8];

        for (i = 0; i < 8; i = i + 1) begin
          tmds_out[i] <= ~q_m[i];
        end

        tally <= tally + q_m[8] + q_m[8] + (number_of_zeros - number_of_ones);

      end

      else begin

        tmds_out[9] <= 1'b0;
        tmds_out[8] <= q_m[8];

        for (i = 0; i < 8; i = i + 1) begin
          tmds_out[i] <= q_m[i];
        end 

        // tally <= tally - (~q_m[8]) - ~(q_m[8]) + (number_of_ones - number_of_zeros); // this does not work for some reason //
        tally <= tally - qm8rev - qm8rev + (number_of_ones - number_of_zeros);

      end
    end
  end
end

	 
endmodule
	 

   module video_sig_gen
#(
  parameter ACTIVE_H_PIXELS = 1280,
  parameter H_FRONT_PORCH = 110,
  parameter H_SYNC_WIDTH = 40,
  parameter H_BACK_PORCH = 220,
  parameter ACTIVE_LINES = 720,
  parameter V_FRONT_PORCH = 5,
  parameter V_SYNC_WIDTH = 5,
  parameter V_BACK_PORCH = 20,
  parameter FPS = 60)
(
  input wire pixel_clk_in,
  input wire rst_in,
  output [11:0] hcount_out,
  output [10:0] vcount_out,
  output vs_out, //vertical sync out
  output hs_out, //horizontal sync out
  output ad_out,
  output nf_out, //single cycle enable signal
  output [5:0] fc_out // frame
); 

localparam TOTAL_PIXELS = ACTIVE_H_PIXELS + H_FRONT_PORCH + H_SYNC_WIDTH + H_BACK_PORCH;
localparam TOTAL_LINES = ACTIVE_LINES + V_FRONT_PORCH + V_SYNC_WIDTH + V_BACK_PORCH;


reg [11:0] hcount;
reg [10:0] vcount;


// increase hcount at every clock cycle  //
// and vcount every time hcount is maxed //
always @(posedge pixel_clk_in) begin
  
  if (rst_in) begin
    hcount <= 0;
    vcount <= 0;
  end

  else begin
    if (hcount == TOTAL_PIXELS) begin
      hcount <= 12'b0;

      if (vcount == TOTAL_LINES) begin
        vcount <= 11'b0;
      end
      else begin
        vcount <= vcount + 1'b1; 
      end
    end

    else begin
      hcount <= hcount + 1'b1; 
    end
  end

end

assign hcount_out = hcount;
assign vcount_out = vcount;

assign ad_out = (hcount < ACTIVE_H_PIXELS) && (vcount < ACTIVE_LINES);
assign vs_out = (vcount < ACTIVE_LINES + V_FRONT_PORCH + V_SYNC_WIDTH) && (vcount >= ACTIVE_LINES + V_FRONT_PORCH);
assign hs_out = (hcount < ACTIVE_H_PIXELS + H_FRONT_PORCH + H_SYNC_WIDTH) && (hcount >= ACTIVE_H_PIXELS + H_FRONT_PORCH);

assign nf_out = (hcount == ACTIVE_H_PIXELS) && (vcount == ACTIVE_LINES); // single-cycle signal //


// increase frame counter every time signal nf_out is raised //

reg [5:0] frame_counter;

always @(posedge pixel_clk_in) begin
  
  if (rst_in || frame_counter == FPS) begin
    frame_counter <= 0;
  end

  else if (nf_out) begin
    frame_counter <= frame_counter + 1'b1;
  end

end

assign fc_out = frame_counter;


endmodule

module pong (
  input wire pixel_clk_in,
  input wire rst_in,
  input wire [1:0] control_in,
  input wire [3:0] puck_speed_in,
  input wire [3:0] paddle_speed_in,
  input wire nf_in,
  input wire [10:0] hcount_in,
  input wire [9:0] vcount_in,
  output  [7:0] red_out,
  output  [7:0] green_out,
  output  [7:0] blue_out
  );

  //use these params!
  localparam PADDLE_WIDTH = 16;
  localparam PADDLE_HEIGHT = 128;
  localparam PUCK_WIDTH = 128;
  localparam PUCK_HEIGHT = 128;
  localparam GAME_WIDTH = 1280;
  localparam GAME_HEIGHT = 720;

  reg [10:0] puck_x, paddle_x; //puck x location, paddle x location
  reg [9:0] puck_y, paddle_y; //puck y location, paddle y location
  wire [7:0] puck_r,puck_g,puck_b; //puck red, green, blue (from block sprite)
  wire [7:0] paddle_r,paddle_g,paddle_b; //paddle colors from its block sprite)

  reg dir_x, dir_y; //use for direction of movement: 1 going positive, 0 going negative


  wire up, down; //up down from buttons
  reg game_over; //signal to indicate game over (0 on game reset, 1 during play)
  assign up = control_in[1]; //up control
  assign down = control_in[0]; //down control

  block_sprite #(.WIDTH(PADDLE_WIDTH), .HEIGHT(PADDLE_HEIGHT))
  paddle(
    .hcount_in(hcount_in),
    .vcount_in(vcount_in),
    .x_in(paddle_x),
    .y_in(paddle_y),
    .red_out(paddle_r),
    .green_out(paddle_g),
    .blue_out(paddle_b));

  block_sprite #(.WIDTH(PUCK_WIDTH), .HEIGHT(PUCK_HEIGHT))
  puck(
    .hcount_in(hcount_in),
    .vcount_in(vcount_in),
    .x_in(puck_x),
    .y_in(puck_y),
    .red_out(puck_r),
    .green_out(puck_g),
    .blue_out(puck_b));

  assign red_out = puck_r|paddle_r; //merge color contributions from puck and paddle
  assign green_out =  puck_g | paddle_g; //merge color contribuations from puck and paddle
  assign blue_out = puck_b | paddle_b; //merge color contributsion from puck and paddle

  wire puck_overlap; //one bit signal indicating if puck and paddle overlap
  //this signal should be one when puck is red in the video included in lab.
  //make signal be derived combinationally. you will need to figure this out
  //remember numbers are not signed here...so there's no such thing as negative

  // above signal not used in my implementation. puck_past_paddle is used instead //

  wire puck_past_paddle;

  assign puck_past_paddle = ((paddle_x + PADDLE_WIDTH > puck_x) && 
                            ((paddle_y + PADDLE_HEIGHT < puck_y) || (puck_y + PUCK_HEIGHT < paddle_y)));
  

  always @(posedge pixel_clk_in)begin
    if (rst_in)begin
      //start puck in center of screen
      puck_x <= GAME_WIDTH/2-PUCK_WIDTH/2;
      puck_y <= GAME_HEIGHT/2 - PUCK_HEIGHT/2;
      dir_x <= hcount_in[0]; //start at pseudorandom direction
      dir_y <= hcount_in[1]; //start with pseudorandom direction
      //start paddle in center of left half of screen
      paddle_x <= 12'b0;
      paddle_y <= GAME_HEIGHT/2 - PADDLE_HEIGHT/2;
      game_over <= 1'b0;
    end else begin
      if (~game_over)begin
        //your reg here.
        
        if (nf_in) begin // all action happens when the frame changes //

          // paddle movement //

          if (up) begin

            if (paddle_y < paddle_speed_in) begin // if the paddle would collide with upper bound in next frame //

              paddle_y <= 10'b0; // prevent it from going upwards and off-sceen //

            end

            else begin
              
              paddle_y <= paddle_y - paddle_speed_in; // moves upwards normally //
            end

          end

          else if (down) begin

            if (paddle_y + PADDLE_HEIGHT + paddle_speed_in >= GAME_HEIGHT) begin // if the paddle would collide with lower bound in next frame //

              paddle_y <= GAME_HEIGHT - PADDLE_HEIGHT; // prevent it from going downwards and off-sceen //

            end

            else begin
              
              paddle_y <= paddle_y + paddle_speed_in; // moves downwards normally //

            end

          end

          // puck (x direction) //

          if (dir_x) begin // moves to the right //

            if (puck_x + PUCK_WIDTH + puck_speed_in >= GAME_WIDTH) begin // if the puck would collide with right bound //
                dir_x <= 1'b0;                     // change direction (to left)       // 
                puck_x <= GAME_WIDTH - PUCK_WIDTH; // prevent it from going off-screen //     
            end 
            else begin
                puck_x <= puck_x + puck_speed_in; // move normally //
            end

          end
          else begin // moves to the left //
            
            if (puck_x - puck_speed_in <= paddle_x + PADDLE_WIDTH) begin // if it hits the paddle //

              // if the puck would collide with left bound //
              if (!(paddle_y + PADDLE_HEIGHT < puck_y) && !(puck_y + PUCK_HEIGHT < paddle_y)) begin
                dir_x <= 1'b1;                          // change direction (to right) //
                puck_x <= paddle_x + PADDLE_WIDTH;      // prevent it from going off-screen //

              end
              else begin // game over if it does not hit the paddle //
                game_over <= 1'b1;
              end
            end 
            else begin
                puck_x <= puck_x - puck_speed_in; // move normally //
            end

          end

          // puck (y direction) //

          if (dir_y) begin // moves to the bottom //

            if (puck_y + PUCK_HEIGHT + puck_speed_in >= GAME_HEIGHT) begin
                dir_y <= 1'b0;                          
                puck_y <= GAME_HEIGHT - PUCK_HEIGHT;    
            end 
            else begin
                puck_y <= puck_y + puck_speed_in;
            end

          end
          else begin // moves to the top //

            if (puck_y < puck_speed_in) begin
              dir_y <= 1'b1;                          
              puck_y <= 10'd0;                        
            end 
            else begin
                puck_y <= puck_y - puck_speed_in;
            end

          end

          if (puck_past_paddle) game_over <= 1'b1;
        end

      end
    end
  end
  
endmodule

 
module top_level(
  input wire clk_100mhz, //crystal reference clock
  input wire [15:0] sw, //all 16 input slide switches
  input wire [3:0] btn, //all four momentary button switches
  output [15:0] led, //16 green output LEDs (located right above switches)
  output [2:0] rgb0, //rgb led
  output [2:0] rgb1, //rgb led
  output [2:0] hdmi_tx_p, //hdmi output signals (positives) (blue, green, red)
  output [2:0] hdmi_tx_n, //hdmi output signals (negatives) (blue, green, red)
  output hdmi_clk_p, hdmi_clk_n //differential hdmi clock
  );
 
  assign led = sw; //to verify the switch values
  //shut up those rgb LEDs (active high):
  assign rgb1 = 0;
  assign rgb0 = 0;
 
  //have btn[0] control system reset
  wire sys_rst;
  assign sys_rst = btn[0]; //reset is btn[0]
  wire game_rst;
  assign game_rst = btn[1]; //reset is btn[1]
 
  wire clk_pixel, clk_5x; //clock lines
  wire locked; //locked signal (we'll leave unused but still hook it up)
 
  //clock manager...creates 74.25 Hz and 5 times 74.25 MHz for pixel and TMDS
  hdmi_clk_wiz_720p mhdmicw (
      .reset(0),
      .locked(locked),
      .clk_ref(clk_100mhz),
      .clk_pixel(clk_pixel),
      .clk_tmds(clk_5x));
 
  wire [10:0] hcount; //hcount of system!
  wire [9:0] vcount; //vcount of system!
  wire hor_sync; //horizontal sync signal
  wire vert_sync; //vertical sync signal
  wire active_draw; //ative draw! 1 when in drawing region.0 in blanking/sync
  wire new_frame; //one cycle active indicator of new frame of info!
  wire [5:0] frame_count; //0 to 59 then rollover frame counter
 
  //written by you previously! (make sure you include in your hdl)
  //default instantiation so making signals for 720p
  video_sig_gen mvg(
      .pixel_clk_in(clk_pixel),
      .rst_in(sys_rst),
      .hcount_out(hcount),
      .vcount_out(vcount),
      .vs_out(vert_sync),
      .hs_out(hor_sync),
      .ad_out(active_draw),
      .nf_out(new_frame),
      .fc_out(frame_count));
 
 reg [7:0] red, green, blue; //red green and blue pixel values for output
  wire [7:0] tp_r, tp_g, tp_b; //color values as generated by test_pattern module
  wire [7:0] pg_r, pg_g, pg_b;//color values as generated by pong game(part 2)
 
  //comment out in checkoff 1 once you know you have your video pipeline working:
  //these three colors should be a nice pink (6.205 sidebar) color on full screen .
  /*
  assign tp_r = 8'hFF;
  assign tp_g = 8'h40;
  assign tp_b = 8'h7A;
  */
 
  //uncomment the test pattern generator for the latter portion of part 1
  //and use it to drive tp_r,g, and b once you know that your video
  //pipeline is working (by seeing the 6.205 pink color)
  
  test_pattern_generator mtpg(
      .sel_in(sw[1:0]),
      .hcount_in(hcount),
      .vcount_in(vcount),
      .red_out(tp_r), 
      .green_out(tp_g),
      .blue_out(tp_b));
  
 
  //uncomment for last part of lab!:

  pong my_pong (
      .pixel_clk_in(clk_pixel),
      .rst_in(game_rst),
      .control_in(btn[3:2]),
      .puck_speed_in(sw[15:12]),
      .paddle_speed_in(sw[11:8]),
      .nf_in(new_frame),
      .hcount_in(hcount),
      .vcount_in(vcount),
      .red_out(pg_r),
      .green_out(pg_g),
      .blue_out(pg_b));

 
  always@(*) begin
    if (~sw[2])begin //if switch 3 switched use shapes signal from part 2, else defaults
      red = tp_r;
      green = tp_g;
      blue = tp_b;
    end else begin
      red = pg_r;
      green = pg_g;
      blue = pg_b;
    end
  end
 
  wire [9:0] tmds_10b [0:2]; //output of each TMDS encoder!
  wire tmds_signal [2:0]; //output of each TMDS serializer!
 
  //three tmds_encoders (blue, green, red)
  //MISSING two more tmds encoders (one for green and one for blue)
  //note green should have no control signal like red
  //the blue channel DOES carry the two sync signals:
  //  * control_in[0] = horizontal sync signal
  //  * control_in[1] = vertical sync signal
 
  tmds_encoder tmds_red(
      .clk_in(clk_pixel),
      .rst_in(sys_rst),
      .data_in(red),
      .control_in(2'b0),
      .ve_in(active_draw),
      .tmds_out(tmds_10b[2]));

  tmds_encoder tmds_green(
      .clk_in(clk_pixel),
      .rst_in(sys_rst),
      .data_in(green),
      .control_in(2'b0),
      .ve_in(active_draw),
      .tmds_out(tmds_10b[1]));

  tmds_encoder tmds_blue(
      .clk_in(clk_pixel),
      .rst_in(sys_rst),
      .data_in(blue),
      .control_in({vert_sync, hor_sync}),
      .ve_in(active_draw),
      .tmds_out(tmds_10b[0]));
 
  //three tmds_serializers (blue, green, red):
  //MISSING: two more serializers for the green and blue tmds signals.
  tmds_serializer red_ser(
      .clk_pixel_in(clk_pixel),
      .clk_5x_in(clk_5x),
      .rst_in(sys_rst),
      .tmds_in(tmds_10b[2]),
      .tmds_out(tmds_signal[2]));

  tmds_serializer green_ser(
      .clk_pixel_in(clk_pixel),
      .clk_5x_in(clk_5x),
      .rst_in(sys_rst),
      .tmds_in(tmds_10b[1]),
      .tmds_out(tmds_signal[1]));

  tmds_serializer blue_ser(
      .clk_pixel_in(clk_pixel),
      .clk_5x_in(clk_5x),
      .rst_in(sys_rst),
      .tmds_in(tmds_10b[0]),
      .tmds_out(tmds_signal[0]));
 
  //output buffers generating differential signals:
  //three for the r,g,b signals and one that is at the pixel clock rate
  //the HDMI receivers use recover wire coupled with the control signals asserted
  //during blanking and sync periods to synchronize their faster bit clocks off
  //of the slower pixel clock (so they can recover a clock of about 742.5 MHz from
  //the slower 74.25 MHz clock)
  OBUFDS OBUFDS_blue (.I(tmds_signal[0]), .O(hdmi_tx_p[0]), .OB(hdmi_tx_n[0]));
  OBUFDS OBUFDS_green(.I(tmds_signal[1]), .O(hdmi_tx_p[1]), .OB(hdmi_tx_n[1]));
  OBUFDS OBUFDS_red  (.I(tmds_signal[2]), .O(hdmi_tx_p[2]), .OB(hdmi_tx_n[2]));
  OBUFDS OBUFDS_clock(.I(clk_pixel), .O(hdmi_clk_p), .OB(hdmi_clk_n));
 
endmodule // top_level

