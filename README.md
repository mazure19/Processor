# Processor

# FullDataPath

//PC

module PC(reset, PC2, PC1);

input reset; 

input [31:0] PC2;

output reg [31:0] PC1;

always @(*) begin

if (reset)

PC1 = 8'd0;

else

PC1 = PC2;

end

endmodule 

//4-BitAdder

module Adder_PC( PC1, PC2);

input [31:0] PC1;

output  [31:0] PC2;


assign PC2 = PC1 + 8'd4;


endmodule 

//ROM

module ROM(address, out);

input [31:0] address;

output [31:0] out;

reg [31:0] result;

assign out = result;

always @(address) begin

case (address)

8'd00: result = 32'h00000000;

8'd04: result = 32'h00100713;

8'd08: result = 32'h00b76463;

8'd12: result = 32'h00008067;

8'd16: result = 32'h006a803;

8'd20: result = 32'h00068613;

8'd24: result = 32'h00070793;

8'd28: result = 32'hffc62883;

8'd32: result = 32'h01185a63;

8'd36: result = 32'h01162023;

8'd40: result = 32'hfff78793;

8'd44: result = 32'hffc60613;

8'd48: result = 32'hfe0796e3;

8'd52: result = 32'h00279793;

8'd56: result = 32'h00f507b3;

8'd60: result = 32'h0107a023;

8'd64: result = 32'h00170713;

8'd68: result = 32'h00468693;

8'd72: result = 32'hfc1ff06f;

endcase

end

endmodule 


//InstructionDecoder

module InstrDecoder(Is, rd, rs1, rs2, imm);

output [4:0] rd, rs1, rs2;

output [11:0] imm;

input[31:0]Is;

assign rd = Is[11:7];

assign rs1 = Is[19:15];

assign rs2 = Is[24:20];

assign imm = Is[31:20];

/*always @(*)

begin

end*/

endmodule 


//RegisterFile

module RegFile(rs1, rs2, rd, imm, out0, out1);

input [4:0] rs1, rs2, rd;

input [11:0] imm;

output [31:0] out0, out1;

reg [31:0] result0, result1;

assign out0 = result0;

assign out1 = result1;

always @(*)

begin

	result0 = rs1;
  
	result1 = rs2;
	
end

endmodule 

//ALU

module ALU(Data0, Data1, sel, Cin, ALU_OP, Cout, out, status);

output [31:0] out;

output [3:0] status;

output Cout;

reg [31:0] result;

assign out = result;

input [31:0] Data0, Data1;

input [2:0] sel, ALU_OP;

input  Cin;

wire [31:0] I0,I1,I2,I3,I4,I5,I6,I7;

wire [31:0] Data1_inv,ADD_SUB;

wire  Cin_inv;

wire Cout1,Cout2;

assign Data1_inv = ~Data1;

assign Cin_inv = Cin;


 Adder_ALU uut_add(Data0, Data1, Cin, I0,Cout1);
 
 Adder_ALU uut_sub(Data0, Data1_inv, Cin_inv, I1,Cout2);
 
 
 //generate other operations
 
always @(*)

begin

	case(sel)
	
	3'b000: //ADD and SUB	
  
		case(ALU_OP)
		
		1'b000: //ADD
    
			result = I0;
			
		1'b001: //SUB
    
			result = I1;
			
		endcase
	
	3'b001: //xor
  
		result = Data0 ^ Data1;
		
	3'b010: //and
  
		result = Data0 & Data1;
		
	3'b011: //or
  
		result = Data0 | Data1;
		
	3'b010: //nor
  
		result = ~(Data0 | Data1);
		
	3'b101: //shift left
  
		result = Data0<<1;
		
	3'b110: //shift right
  
		result = Data0>>1;
		
	default: result = Data0 + Data1;
  
	endcase
  
 end
 
 endmodule


//RAM

module RAM(input [7:0] adrs, input [7:0] data, output [7:0] out);
reg [7:0] mem[0:63];

reg [7:0] result;

assign out = result;

always@(*)

begin

	result = data;
  
end

endmodule 


//FullRAM

module FullRAM(a, b, c, d, in, csin, out32, out1, out2, out3, out4, out5, out6, out7, out8, out9, out10, out11, out12, out13, out14, out15, out16);

input [7:0] a, b, c, d, in;

input [1:0] csin;

output [31:0] out32;

output [7:0] out1;

output [7:0] out2;

output [7:0] out3;

output [7:0] out4;

output [7:0] out5;

output [7:0] out6;

output [7:0] out7;

output [7:0] out8;

output [7:0] out9;

output [7:0] out10;

output [7:0] out11;

output [7:0] out12;

output [7:0] out13;

output [7:0] out14;

output [7:0] out15;

output [7:0] out16;

reg [31:0] result;

assign out32 = result;

RAM row1_1(a, in, out1);

RAM row1_2(b, in, out2);

RAM row1_3(c, in, out3);

RAM row1_4(d, in, out4);

RAM row2_1(a, in, out5);

RAM row2_2(b, in, out6);

RAM row2_3(c, in, out7);

RAM row2_4(d, in, out8);

RAM row3_1(a, in, out9);

RAM row3_2(b, in, out10);

RAM row3_3(c, in, out11);

RAM row3_4(d, in, out12);

RAM row4_1(a, in, out13);

RAM row4_2(b, in, out14);

RAM row4_3(c, in, out15);

RAM row4_4(d, in, out16);

always @(*)

begin

	if (csin == 2'd0)
  
		result = {out1, out2, out3, out4};
    
	if(csin == 2'd1)
  
		result = {out5, out6, out7, out8};
    
	if(csin == 2'd2)
  
		result = {out9, out10, out11, out12};
    
	if(csin == 2'd3)
  
  result = {out13, out14, out15, out16}; 
  
end 

endmodule 


//AdderALU

module Adder_ALU(Data0, Data1, Cin, out3,Cout);

input [31:0] Data0, Data1;

output [31:0] out3;

input Cin;

output Cout;

assign {Cout, out3} = Cin + Data0 + Data1;

endmodule 


//MAIN FILE

module TestFile(rst, sel, ALU_OP, ALUsrc, MRW, csin, FullOut);

input [1:0] csin;

input [7:0] MRW;

input [2:0] sel, ALU_OP;

input rst, ALUsrc;

output [31:0] FullOut;

wire [31:0] in0, PC_out, ROM_out, ALU_out, RAM_out, Reg_out0, Reg_out1;

wire [4:0] rs1, rs2, rd;

wire [11:0] imm;

wire [7:0] out1, out2, out3, out4, out5, out6, out7, out8, out9, out10, out11, out12, out13, out14, out15, out16;

wire [3:0] status;

wire Cout;

reg [31:0] result;

assign FullOut = result;

PC pc0(rst, in0, PC_out);

Adder_PC adderpc(PC_out, in0);

ROM rom0(PC_out, ROM_out);

InstrDecoder ROM_Decoder(ROM_out, rd, rs1, rs2, imm);

RegFile regfile(rs1, rs2, RAM_out[4:0], imm, Reg_out0, Reg_out1);

ALU alu(Reg_out0, Reg_out1, sel, ALUsrc, ALU_OP, Cout, ALU_out, status);

FullRAM ram(ALU_out[7:0], ALU_out[15:8], ALU_out[23:16], ALU_out[31:24], MRW, csin, RAM_out, out1, out2, out3, out4, out5, out6, out7, out8, out9, out10, out11, out12, out13, out14, out15, out16);

always @(*)

begin

	result = RAM_out;

end

endmodule

# ControlUnit

//Control Unit

module ControlUnit(reset, clock, inst, status, MEM_RW, RW, ALU_src, ALU_op, PC_src, imm_sel, WB, C0);

input reset, clock;

output reg C0;

output reg ALU_src;

output reg PC_src;

output reg MEM_RW;

output reg RW;

output reg WB;

output reg [3:0] ALU_op;

output reg [1:0] imm_sel;

input [31:0] inst;

input [3:0] status;

//Rtype

always @(*)

if (reset ==0)

case (inst[6:0])

7'b0110011: begin

C0 = 1'b0;

ALU_src = 1'b0;

RW = 1'b1;

MEM_RW = 1'b1;

WB = 1'b1;

PC_src = 1'b0;

imm_sel = 2'b00;

ALU_op = {inst[30], inst [14:12]};

end

//Itype 

7'b0010011: begin

C0 = 1'b0;

ALU_src = 1'b1;

RW = 1'b1;

MEM_RW = 1'b1;

WB = 1'b1;

PC_src = 1'b0;

imm_sel = 2'b01;

ALU_op = {inst[30], inst [14:12]};

end

//LW

7'b0000011: begin

C0 = 1'b0;

ALU_src = 1'b1;

RW = 1'b1;

MEM_RW = 1'b1;

WB = 1'b0;

PC_src = 1'b0;

imm_sel = 2'b01;

ALU_op = 4'b0000;

end

//SW

7'b0100011: begin

C0 = 1'b0;

ALU_src = 1'b1;

RW = 1'b1;

MEM_RW = 1'b1;

WB = 1'b1;

PC_src = 1'b0;

imm_sel = 2'b01;

ALU_op = 4'b0000;

end

//Btype

7'b1100011: begin

C0 = 1'b0;

ALU_src = 1'b1;

RW = 1'b1;

MEM_RW = 1'b1;

WB = 1'b1;

imm_sel = 2'b01;

case (inst [14:12])

3'b000: begin

PC_src = status[0]? 1'b1 :1'b0;

end

3'b000: begin

PC_src = status[1]? 1'b1 :1'b0;

end

endcase

begin

ALU_op = 4'b0000;

C0 = 1'b0;

end

end

endcase

endmodule 

//Testbench

module ControlUnit_tb();

reg [31:0]inst;

reg reset;

reg clock;

reg [3:0]status;


wire C0;

wire PC_src;

wire [3:0]ALU_op;

wire [1:0]imm_sel;

wire WB;

wire ALU_src;

wire REG_RW;

wire MEM_RW;

ControlUnit dut(reset, clock, inst, status, MEM_RW, REG_RW, ALU_src, ALU_op,
 PC_src, imm_sel, WB, C0);

 

always begin

#5 clock= ~clock;

end


initial begin

clock = 0;

reset = 1;

#10 reset = 0;

status = 4'b0000;

#10 inst = 32'b00000000010000000000000010010011;

#10 inst = 32'b00000000001000001000000110110011;

#10 inst = 32'b00000000000100000000000000100011;

#10 inst = 32'b01000000001000010000010001100011;

end

endmodule
