module Control_Unit(opcode , Branch, Jump , ImmSrc , ResultSrc , ALUOp , MemWrite , ALUSrc , RegWrite);
input [6:0] opcode;
output reg Branch , Jump , MemWrite , ALUSrc , RegWrite;
output reg [1:0] ImmSrc , ALUOp ,  ResultSrc;
always @(*) begin 
case (opcode)
7'b0110011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b0_00_xx_1_0_0_10_0;
7'b0000011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b1_01_00_1_0_0_00_0;
7'b0100011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b1_xx_01_0_1_0_00_0;
7'b1100011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b0_xx_10_0_0_1_01_0;
7'b0010011 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'b1_00_00_1_0_0_10_0;
7'b1101111 : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'bx_10_11_1_0_0_xx_1;
default    : {ALUSrc , ResultSrc , ImmSrc , RegWrite , MemWrite , Branch , ALUOp , Jump} = 11'bx_xx_xx_x_x_x_xx_x;
endcase
end
endmodule
module ALU_Control(ALUOp , Funct3 , Funct7 , op , Operation);
input [1:0] ALUOp;
input [2:0] Funct3;
input Funct7 , op;
output reg [2:0] Operation;

always @(*) begin
case(ALUOp)
2'b00: Operation = 3'b000;  // add for load/store
2'b01: Operation = 3'b001;  // sub for branch
2'b10: begin               // R-type/I-type
case(Funct3)
3'b000: Operation = (op && Funct7) ? 3'b001 : 3'b000; // sub if R-type and Funct7=1, add otherwise
 3'b010: Operation = 3'b101; // slt
                3'b110: Operation = 3'b011; // or
                3'b111: Operation = 3'b010; // and
                default: Operation = 3'b000;
            endcase
        end
        default: Operation = 3'b000;
    endcase
end
endmodule
module CU(Zero , opcode , Funct3 , Funct7 , PCSrc , ResultSrc , MemWrite , ALUSrc , ImmSrc , RegWrite , Operation);
input Zero;
input [6:0] opcode ,Funct7;
input [2:0] Funct3;
output  PCSrc , MemWrite , ALUSrc , RegWrite;
output  [1:0] ImmSrc , ResultSrc;
output  [2:0] Operation;
wire Branch , Jump;
wire [1:0] ALUOp;
Control_Unit c(opcode , Branch , Jump , ImmSrc , ResultSrc , ALUOp , MemWrite , ALUSrc , RegWrite);
ALU_Control  a(ALUOp , Funct3 , Funct7[5] , opcode[5] , Operation);
assign PCSrc = Jump | (Zero & Branch);
endmodule



module registerFile (
    input wire clk,             // Clock signal
    input wire RegWrite,        // Write enable signal
    input wire [4:0] RS1,       // Source register 1 (5 bits, selects one of 32 registers)
    input wire [4:0] RS2,       // Source register 2 (5 bits, selects another of 32 registers)
    input wire [4:0] RD,        // Destination register (5 bits, selects the register to write to)
    input wire [31:0] WriteData, // Data to be written to register RD (32 bits wide)
    output wire [31:0] ReadData1, // Data read from register RS1 (32 bits wide)
    output wire [31:0] ReadData2  // Data read from register RS2 (32 bits wide)
);

    // 32 registers, each 32 bits wide
    reg [31:0] Registers [31:0];  

    // Initialize registers with random values
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1)
            Registers[i] = i;  // Initialize with random values for simulation
    end

    // Read operation (asynchronous)
    assign ReadData1 = Registers[RS1];  // Read register RS1
    assign ReadData2 = Registers[RS2];  // Read register RS2

    // Write operation (synchronous, on positive clock edge)
    always @(posedge clk) begin
        if (RegWrite) begin
            Registers[RD] <= WriteData;  
        end
    end

endmodule



module alu(a, b, op, res, zero);
input [31:0] a, b; 
input [2:0] op;
output reg zero;
output reg [31:0] res;

always @(*) begin
    case(op)
        3'b000: res = a + b;    // ADD
        3'b001: res = a - b;    // SUB
        3'b101: res = a < b;    // SLT
        3'b011: res = a | b;    // OR
        3'b010: res = a & b;    // AND
        default: res = 32'b0;
    endcase
    
    zero = (res == 32'b0) ? 1'b1 : 1'b0;
end
endmodule



module imm_data_gen(instruction , ImmSrc , imm_data);
input [31:0] instruction;
input [1:0] ImmSrc;
output reg [31:0] imm_data;
always @(*) begin
    case(ImmSrc)
        2'b00: imm_data = {{20{instruction[31]}}, instruction[31:20]};
        2'b01: imm_data = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
        2'b10: imm_data = {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0};
        2'b11: imm_data = {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};
        default: imm_data = 32'b0; 
    endcase
end
endmodule


module Data_Memory(
    input wire [31:0] Mem_Addr,    
    input wire [31:0] Write_Data,  
    input wire clk,                
    input wire MemWrite,           
    output wire [31:0] Read_Data   
);

    reg [7:0] memory [63:0];       

    integer i;
    initial begin
        for (i = 0; i < 64; i = i + 1)
            memory[i] = i;
    end

    assign Read_Data = {memory[Mem_Addr + 3], memory[Mem_Addr + 2], memory[Mem_Addr + 1], memory[Mem_Addr]};

    always @(posedge clk) begin
        if (MemWrite) begin
            {memory[Mem_Addr + 3], memory[Mem_Addr + 2], memory[Mem_Addr + 1], memory[Mem_Addr]} = Write_Data;
        end
    end
endmodule




module Program_Counter (clk , rst , PC_In , PC_Out);
input clk , rst;
input [31:0] PC_In;
output reg [31:0] PC_Out;
always @(posedge clk) begin
if (rst)
PC_Out <= 0;
else
PC_Out <= PC_In;
end
endmodule 


module adder (a , b , out);
input [31:0] a , b;
output [31:0] out;
assign out = a + b ; 
endmodule


module Instruction_Memory(Inst_Address , Instruction); 
input wire [31:0] Inst_Address;   
output reg [31:0] Instruction; 
reg [7:0] memory [15:0];
    initial begin
        memory[0] = 8'b00000011;
        memory[1] = 8'b10100011;
        memory[2] = 8'b11000100;
        memory[3] = 8'b11111111;
        memory[4] = 8'b00100011;
        memory[5] = 8'b10100100;
        memory[6] = 8'b01100100;
        memory[7] = 8'b00000000;
        memory[8] = 8'b00110011;
        memory[9] = 8'b11100010;
        memory[10] = 8'b01100010;
        memory[11] = 8'b00000000;

        memory[12] = 8'b11100011;
        memory[13] = 8'b00001010;
        memory[14] = 8'b01000010;
        memory[15] = 8'b11111110;
    end


always @(*) begin
        Instruction = {memory[Inst_Address + 3], memory[Inst_Address + 2], memory[Inst_Address + 1], memory[Inst_Address]};
    end 
endmodule


module mux32bit(a,b,c,sel,out);
input [31:0] a,b,c;
input [1:0] sel;
output reg [31:0] out; 
always @(*) begin 
case(sel)
2'b00 : out <= a;
2'b01 : out <= b;
2'b10 : out <= c;
default: out <=32'b0;
endcase
end
endmodule


module mux32bit2(a,b,sel,out);
input [31:0] a,b;
input  sel;
output reg [31:0] out; 
always @(*) begin 
case(sel)
2'b0 : out <= a;
2'b1 : out <= b;
default: out <=32'b0;
endcase
end
endmodule





