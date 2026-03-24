module FFT (
    input  wire              clk,
    input  wire              reset,
    input  wire              in_valid,
    input  wire signed [11:0] din_r,
    input  wire signed [11:0] din_i,
    output wire              out_valid,
    output reg  signed [15:0] dout_r,
    output reg  signed [15:0] dout_i
);

    integer i;

    // ----------------------------------------------------------------
    // Result buffers – 8 complex bins in natural (un-scrambled) order
    // ----------------------------------------------------------------
    reg signed [15:0] result_r    [0:7];
    reg signed [15:0] result_i    [0:7];
    reg signed [15:0] result_r_ns [0:7];
    reg signed [15:0] result_i_ns [0:7];
    reg signed [15:0] next_dout_r, next_dout_i;

    // count_y : counts valid outputs arriving from stage-2
    // y_1     : count_y delayed by one (points at the current output slot)
    reg  [3:0] count_y, next_count_y;
    wire [3:0] y_1;
    reg  [3:0] y_1_delay;

    // Input sign-extended to Q8 fixed-point  (12-bit → 24-bit with 8 frac bits)
    reg signed [23:0] din_r_reg, din_i_reg;
    wire [23:0] din_r_wire, din_i_wire;
    assign din_r_wire = din_r_reg;
    assign din_i_wire = din_i_reg;

    // Registered in_valid (1-cycle pipeline delay before stage 1)
    reg in_valid_reg;

    // Stage-3 trivial butterfly state machine
    reg        r2_valid, next_r2_valid;   // registered radix_no2 outvalid
    reg [1:0]  no3_state;                 // state fed into trivial radix_no3
    reg        s3_count, next_s3_count;   // 1-bit: tracks even/odd butterfly half

    // Output handshake
    reg over, next_over;                  // all 8 bins captured
    reg assign_out, next_out_valid;

    assign out_valid = assign_out;
    assign y_1       = (count_y > 4'd0) ? (count_y - 4'd1) : 4'd0;

    // ================================================================
    //  Stage 1 – shift_4 / ROM_4 / radix_no1
    //  Twiddle factors: W_8^0, W_8^1, W_8^2, W_8^3  (from ROM_4)
    // ================================================================
    wire [1:0]  rom4_state;
    wire [23:0] rom4_w_r,    rom4_w_i;
    wire [23:0] shift4_r,    shift4_i;
    wire [23:0] r1_delay_r,  r1_delay_i;
    wire [23:0] r1_op_r,     r1_op_i;
    wire        r1_outvalid;

    radix2 radix_no1 (
        .state   (rom4_state),
        .din_a_r (shift4_r),     .din_a_i (shift4_i),
        .din_b_r (din_r_wire),   .din_b_i (din_i_wire),
        .w_r     (rom4_w_r),     .w_i     (rom4_w_i),
        .op_r    (r1_op_r),      .op_i    (r1_op_i),
        .delay_r (r1_delay_r),   .delay_i (r1_delay_i),
        .outvalid(r1_outvalid)
    );

    shift_4 shift4_inst (
        .clk(clk), .reset(reset),
        .in_valid(in_valid_reg),
        .din_r   (r1_delay_r),   .din_i   (r1_delay_i),
        .dout_r  (shift4_r),     .dout_i  (shift4_i)
    );

    ROM_4 rom4_inst (
        .clk     (clk),
        .in_valid(in_valid_reg),
        .reset   (reset),
        .w_r     (rom4_w_r),     .w_i     (rom4_w_i),
        .state   (rom4_state)
    );

    // ================================================================
    //  Stage 2 – shift_2 / ROM_2 / radix_no2
    //  Twiddle factors: W_4^0 = 1,  W_4^1 = -j  (from ROM_2)
    // ================================================================
    wire [1:0]  rom2_state;
    wire [23:0] rom2_w_r,    rom2_w_i;
    wire [23:0] shift2_r,    shift2_i;
    wire [23:0] r2_delay_r,  r2_delay_i;
    wire [23:0] r2_op_r,     r2_op_i;
    wire        r2_outvalid;

    radix2 radix_no2 (
        .state   (rom2_state),
        .din_a_r (shift2_r),     .din_a_i (shift2_i),
        .din_b_r (r1_op_r),      .din_b_i (r1_op_i),
        .w_r     (rom2_w_r),     .w_i     (rom2_w_i),
        .op_r    (r2_op_r),      .op_i    (r2_op_i),
        .delay_r (r2_delay_r),   .delay_i (r2_delay_i),
        .outvalid(r2_outvalid)
    );

    shift_2 shift2_inst (
        .clk(clk), .reset(reset),
        .in_valid(r1_outvalid),
        .din_r   (r2_delay_r),   .din_i   (r2_delay_i),
        .dout_r  (shift2_r),     .dout_i  (shift2_i)
    );

    ROM_2 rom2_inst (
        .clk     (clk),
        .in_valid(r1_outvalid),
        .reset   (reset),
        .w_r     (rom2_w_r),     .w_i     (rom2_w_i),
        .state   (rom2_state)
    );

    // ================================================================
    //  Stage 3 – trivial butterfly (W^0 = 1, no multiplication needed)
    //  Uses the same radix2 module with w_r = 256 (= 1.0 in Q8 × 256),
    //  w_i = 0.  State is driven by the no3_state machine below.
    // ================================================================
    wire [23:0] shift1_r,    shift1_i;
    wire [23:0] r3_delay_r,  r3_delay_i;
    wire [23:0] out_r,       out_i;     // final butterfly outputs

    radix2 radix_no3 (
        .state   (no3_state),
        .din_a_r (shift1_r),     .din_a_i (shift1_i),
        .din_b_r (r2_op_r),      .din_b_i (r2_op_i),
        .w_r     (24'd256),      .w_i     (24'd0),
        .op_r    (out_r),        .op_i    (out_i),
        .delay_r (r3_delay_r),   .delay_i (r3_delay_i),
        .outvalid(/* unused */)
    );

    shift_1 shift1_inst (
        .clk(clk), .reset(reset),
        .in_valid(r2_outvalid),
        .din_r   (r3_delay_r),   .din_i   (r3_delay_i),
        .dout_r  (shift1_r),     .dout_i  (shift1_i)
    );

    // ================================================================
    //  Sequential registers
    // ================================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            din_r_reg    <= 0;
            din_i_reg    <= 0;
            in_valid_reg <= 0;
            s3_count     <= 0;
            r2_valid     <= 0;
            count_y      <= 0;
            assign_out   <= 0;
            over         <= 0;
            dout_r       <= 0;
            dout_i       <= 0;
            y_1_delay    <= 0;
            for (i = 0; i <= 7; i = i + 1) begin
                result_r[i] <= 0;
                result_i[i] <= 0;
            end
        end else begin
            // Sign-extend 12-bit input and shift left 8 bits for Q8 headroom
            din_r_reg    <= {{4{din_r[11]}}, din_r, 8'b0};
            din_i_reg    <= {{4{din_i[11]}}, din_i, 8'b0};
            in_valid_reg <= in_valid;
            s3_count     <= next_s3_count;
            r2_valid     <= next_r2_valid;
            count_y      <= next_count_y;
            assign_out   <= next_out_valid;
            over         <= next_over;
            y_1_delay    <= y_1;
            dout_r       <= next_dout_r;
            dout_i       <= next_dout_i;
            for (i = 0; i <= 7; i = i + 1) begin
                result_r[i] <= result_r_ns[i];
                result_i[i] <= result_i_ns[i];
            end
        end
    end

    // ================================================================
    //  Stage-3 state machine + output counter
    // ================================================================
    always @(*) begin
        // Register radix_no2's outvalid one cycle ahead of stage-3 processing
        next_r2_valid = r2_outvalid;

        // s3_count alternates 0/1 while r2_valid is high,
        // selecting state=01 (first butterfly half) vs state=10 (twiddle half)
        if (r2_valid) next_s3_count = s3_count + 1'b1;
        else          next_s3_count = s3_count;

        if      (r2_valid && s3_count == 1'b0) no3_state = 2'b01;
        else if (r2_valid && s3_count == 1'b1) no3_state = 2'b10;
        else                                   no3_state = 2'b00;

        // Count each valid stage-2 output (triggers collection of FFT bins)
        if (r2_outvalid) next_count_y = count_y + 4'd1;
        else             next_count_y = count_y;

        // Drive dout from the result buffer using a one-cycle delayed index
        if (next_out_valid) begin
            next_dout_r = result_r[y_1_delay[2:0]];
            next_dout_i = result_i[y_1_delay[2:0]];
        end else begin
            next_dout_r = dout_r;
            next_dout_i = dout_i;
        end
    end

    // ================================================================
    //  Output bit-reversal reordering
    //
    //  The radix-2 DIF pipeline emits bins in a scrambled order.
    //  This case statement maps the time-slot index (y_1) to the
    //  natural FFT bin index so that result[k] = X[k].
    //
    //  Mapping (derived by pattern extension from the 32-pt design):
    //    y_1 :  0   1   2   3   4   5   6   7
    //    bin :  7   3   1   5   0   4   2   6
    //
    //  NOTE: verify this mapping with a functional simulation against
    //  a reference FFT before using in production.
    // ================================================================
    always @(*) begin
        next_over = over;
        for (i = 0; i <= 7; i = i + 1) begin
            result_r_ns[i] = result_r[i];
            result_i_ns[i] = result_i[i];
        end

        if (next_over == 1'b1) next_out_valid = 1'b1;
        else                   next_out_valid = assign_out;

        if (over != 1'b1) begin
            case (y_1[2:0])
            3'd0: begin
                result_r_ns[7] = out_r[23:8];
                result_i_ns[7] = out_i[23:8];
            end
            3'd1: begin
                result_r_ns[3] = out_r[23:8];
                result_i_ns[3] = out_i[23:8];
            end
            3'd2: begin
                result_r_ns[1] = out_r[23:8];
                result_i_ns[1] = out_i[23:8];
            end
            3'd3: begin
                result_r_ns[5] = out_r[23:8];
                result_i_ns[5] = out_i[23:8];
            end
            3'd4: begin
                result_r_ns[0] = out_r[23:8];
                result_i_ns[0] = out_i[23:8];
            end
            3'd5: begin
                result_r_ns[4] = out_r[23:8];
                result_i_ns[4] = out_i[23:8];
            end
            3'd6: begin
                result_r_ns[2] = out_r[23:8];
                result_i_ns[2] = out_i[23:8];
            end
            3'd7: begin
                result_r_ns[6] = out_r[23:8];
                result_i_ns[6] = out_i[23:8];
                next_over = 1'b1;   // all 8 bins captured
            end
            endcase
        end
    end

endmodule




module shift_4(
		input wire clk,
		input wire reset,
		input wire in_valid,
		input wire signed [23:0] din_r,
		input wire signed [23:0] din_i,
		output wire signed [23:0] dout_r,
		output wire signed [23:0] dout_i
	);
	integer i ;
	reg [95:0] shift_reg_r ;
	reg [95:0] shift_reg_i ;
	reg [95:0] tmp_reg_r ;
	reg [95:0] tmp_reg_i ;
	reg [3:0] counter_4,next_counter_4;
	reg valid,next_valid;

	assign dout_r    = shift_reg_r[95:72];
	assign dout_i    = shift_reg_i[95:72];

	always@(posedge clk or posedge reset)begin
		if(reset)begin
			shift_reg_r <= 0;
			shift_reg_i <= 0;
			counter_4  <= 0;
			valid <= 0;
		end
		else 
		if (in_valid)begin
			counter_4        <= next_counter_4;
			shift_reg_r      <= (tmp_reg_r<<24) + din_r;
			shift_reg_i      <= (tmp_reg_i<<24) + din_i;
			valid            <= in_valid;
		end else if(valid)begin
			counter_4        <= next_counter_4;
			shift_reg_r      <= (tmp_reg_r<<24) + din_r;
			shift_reg_i      <= (tmp_reg_i<<24) + din_i;
			valid            <= next_valid;        
		end
	end
	
		always@(*)begin
		next_counter_4 = counter_4 + 3'd1;
		tmp_reg_r = shift_reg_r;
		tmp_reg_i = shift_reg_i;
		next_valid = valid;
	end


endmodule


module shift_2(
		input wire clk,
		input wire reset,
		input wire in_valid,
		input wire signed [23:0] din_r,
		input wire signed [23:0] din_i,
		output wire signed [23:0] dout_r,
		output wire signed [23:0] dout_i
	);
	integer i ;
	reg [47:0] shift_reg_r ;
	reg [47:0] shift_reg_i ;
	reg [47:0] tmp_reg_r ;
	reg [47:0] tmp_reg_i ;
	reg [2:0] counter_2,next_counter_2;
	reg valid,next_valid;

	assign dout_r    = shift_reg_r[47:24];
	assign dout_i    = shift_reg_i[47:24];

	always@(posedge clk or posedge reset)begin
		if(reset)begin
			shift_reg_r <= 0;
			shift_reg_i <= 0;
			counter_2  <= 0;
			valid <= 0;
		end
		else 
		if (in_valid)begin
			counter_2        <= next_counter_2;
			shift_reg_r      <= (tmp_reg_r<<24) + din_r;
			shift_reg_i      <= (tmp_reg_i<<24) + din_i;
			valid            <= in_valid;
		end else if(valid)begin
			counter_2        <= next_counter_2;
			shift_reg_r      <= (tmp_reg_r<<24) + din_r;
			shift_reg_i      <= (tmp_reg_i<<24) + din_i;
			valid            <= next_valid;
		end
	end
	
		always@(*)begin
		next_counter_2 = counter_2 + 2'd1;
		tmp_reg_r = shift_reg_r;
		tmp_reg_i = shift_reg_i;
		next_valid = valid;
	end

endmodule



module shift_1(
		input wire clk,
		input wire reset,
		input wire in_valid,
		input signed [23:0] din_r,
		input signed [23:0] din_i,
		output signed [23:0] dout_r,
		output signed [23:0] dout_i
	);
	integer i ;
	reg [23:0] shift_reg_r ;
	reg [23:0] shift_reg_i ;
	reg [23:0] tmp_reg_r ;
	reg [23:0] tmp_reg_i ;
	reg [1:0] counter_1,next_counter_1;
	reg valid,next_valid;

	assign dout_r    = shift_reg_r[23:0];
	assign dout_i    = shift_reg_i[23:0];

	always@(posedge clk or posedge reset)begin
		if(reset)begin
			shift_reg_r <= 0;
			shift_reg_i <= 0;
			counter_1  <= 0;
			valid      <= 0;
		end
		else 
		if (in_valid)begin
			counter_1        <= next_counter_1;
			shift_reg_r      <= (tmp_reg_r<<24) + din_r;
			shift_reg_i      <= (tmp_reg_i<<24) + din_i;
			valid            <= in_valid;
		end else if(valid)begin
			counter_1        <= next_counter_1;
			shift_reg_r      <= (tmp_reg_r<<24) + din_r;
			shift_reg_i      <= (tmp_reg_i<<24) + din_i;
			valid            <= next_valid;
		end
	end
	
		always@(*)begin
		next_counter_1 = counter_1 + 2'd1;
		tmp_reg_r = shift_reg_r;
		tmp_reg_i = shift_reg_i;
		next_valid = valid;
	end

endmodule



module radix2(
		input wire [1:0] state,
		input wire signed [23:0] din_a_r,
		input wire signed [23:0] din_a_i,
		input wire signed [23:0] din_b_r,//a
		input wire signed [23:0] din_b_i,//b
		input wire signed [23:0] w_r,//c
		input wire signed [23:0] w_i,//d
		output reg signed[23:0] op_r,
		output reg signed[23:0] op_i,
		output reg signed[23:0] delay_r,
		output reg signed[23:0] delay_i,
		output reg outvalid
	);

	reg signed [41:0] inter,mul_r,mul_i;//was 27
	reg signed [23:0] a,b,c,d;

	always@(*)begin
		op_r = 0;
		op_i = 0;
		delay_r = din_b_r;
		delay_i = din_b_i;
		case(state)
		2'b00:begin
		//waiting
		delay_r = din_b_r;
		delay_i = din_b_i;
		outvalid = 1'b0;
		end
		2'b01:begin
		//first half
		a = din_a_r + din_b_r;
		b = din_a_i + din_b_i;
		
		c = (din_a_r - din_b_r);//a-b
		d = (din_a_i - din_b_i);//a-b

		op_r = a;
		op_i = b;
		delay_r = c;
		delay_i = d;
		outvalid = 1'b1;
		end
		2'b10:begin
		//second half
		a = din_a_r;
		b = din_a_i;
		delay_r = din_b_r;
		delay_i = din_b_i;

		inter = b * (w_r - w_i); //b(c-d)
		mul_r  = w_r * (a - b) + inter;
		mul_i  = w_i * (a + b) + inter;

		op_r = (mul_r[31:8]);
		op_i = (mul_i[31:8]);
		outvalid = 1'b1;
		end
		2'b11:begin
		//disable
		outvalid = 1'b0;
		end
		default:begin
		delay_r = din_b_r;
		delay_i = din_b_i;
		end
		endcase
		
	end


endmodule




module ROM_4(
		input wire clk,
		input wire in_valid,
		input wire reset,
		output reg [23:0] w_r,
		output reg [23:0] w_i,
		output reg[1:0] state
	);

	reg valid,next_valid;
	reg [5:0] count,next_count;
	reg [2:0] s_count,next_s_count;

	always@(posedge clk or posedge reset)begin
		if(reset)begin
			count <= 0;
			s_count <= 0;
		end
		else begin
			count <= next_count;
			s_count <= next_s_count;
		end
	end
	
		always @(*) begin
		if(in_valid || valid)
		begin 
			next_count = count + 1;
			next_s_count = s_count;
		end
		else begin
			next_count = count;
			next_s_count = s_count;  
		end

		if (count<6'd4) 
			state = 2'd0;
		else if (count >= 6'd4 && s_count < 3'd4)begin
			state = 2'd1;
			next_s_count = s_count + 1;
		end
		else if (count >= 6'd4 && s_count >= 3'd4)begin
			state = 2'd2;
			next_s_count = s_count + 1;
		end
		case(s_count)
		3'd4: begin
			w_r = 24'b 00000000_00000001_00000000;
			w_i = 24'b 00000000_00000000_00000000;
			end
		3'd5: begin
			w_r = 24'b 00000000_00000000_10110101;
			w_i = 24'b 11111111_11111111_01001011;
			end
		3'd6: begin
			w_r = 24'b 00000000_00000000_00000000;
			w_i = 24'b 11111111_11111111_00000000;
			end
		3'd7: begin
			w_r = 24'b 11111111_11111111_01001011;
			w_i = 24'b 11111111_11111111_01001011;
			end
		default: begin
			w_r = 24'b 00000000_00000001_00000000;
			w_i = 24'b 00000000_00000000_00000000;
			end
		endcase
	end
endmodule


module ROM_2(
		input wire clk,
		input wire in_valid,
		input wire reset,
		output reg [23:0] w_r,
		output reg [23:0] w_i,
		output reg[1:0] state
	);

	reg valid,next_valid;
	reg [5:0] count,next_count;
	reg [1:0] s_count,next_s_count;

	always@(posedge clk or posedge reset)begin
		if(reset)begin
			count <= 0;
			s_count <= 0;
		end
		else begin
			count <= next_count;
			s_count <= next_s_count;
		end
	end
	
		always @(*) begin
		state = 2'd0;
		if(in_valid || valid)
		begin 
			next_count = count + 1;
			next_s_count = s_count;
		end
		else begin
			next_count = count;
			next_s_count = s_count;  
		end

		if (count<6'd2) 
			state = 2'd0;
		else if (count >= 6'd2 && s_count < 2'd2)begin
			state = 2'd1;
			next_s_count = s_count + 1;
		end else if (count >= 6'd2 && s_count >= 2'd2)begin
			state = 2'd2;
			next_s_count = s_count + 1;
		end
		case(s_count)
		2'd2: begin
			w_r = 24'b 00000000_00000001_00000000;
			w_i = 24'b 00000000_00000000_00000000;
			end
		2'd3: begin
			w_r = 24'b 00000000_00000000_00000000;
			w_i = 24'b 11111111_11111111_00000000;
			end
		default: begin
			w_r = 24'b 00000000_00000001_00000000;
			w_i = 24'b 00000000_00000000_00000000;
			end
		endcase
	end
endmodule
