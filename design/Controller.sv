`timescale 1ns / 1ps

module Controller (
    //Input
    input logic [6:0] Opcode,
    //7-bit opcode field from the instruction

    //Outputs
    output logic ALUSrc,
    //0: The second ALU operand comes from the second register file output (Read data 2); 
    //1: The second ALU operand is the sign-extended, lower 16 bits of the instruction.
    output logic MemtoReg,
    //0: The value fed to the register Write data input comes from the ALU.
    //1: The value fed to the register Write data input comes from the data memory.
    output logic RegWrite, //The register on the Write register input is written with the value on the Write data input 
    output logic [1:0] ALUop,  //00: LW/SW; 01:Branch; 10: Rtype
    output logic jumpreg, //0: jump register is not taken; 1: jump register is taken
    output logic Branch,  //0: branch is not taken; 1: branch is taken
    output logic jump, //0: jump is not taken; 1: jump is taken
    output logic MemRead,  //Data memory contents designated by the address input are put on the Read data output
    output logic MemWrite, //Data memory contents designated by the address input are replaced by the value on the Write data input.
    output logic Halt //0: halt is not taken; 1: halt is taken
);

  logic [6:0] R_TYPE, I_TYPE, LW, SW, BR, JAL, JALR, HALT;

  assign R_TYPE = 7'b0110011;  //add,and
  assign I_TYPE = 7'b0010011;  //immediate aritmetico
  assign LW = 7'b0000011;  //load
  assign JAL = 7'b1101111;  //jump
  assign JALR = 7'b1100111;  //jump register
  assign HALT= 7'b1111111;  //halt
  assign SW = 7'b0100011;  //store
  assign BR = 7'b1100011;  //branchs condicionais
    
  assign ALUSrc = (Opcode == LW || Opcode == SW || Opcode == I_TYPE || Opcode == JALR);
  assign MemtoReg = (Opcode == LW);
  assign RegWrite = (Opcode == R_TYPE || Opcode == I_TYPE ||Opcode == LW || Opcode == JAL || Opcode == JALR);
  assign MemRead = (Opcode == LW);
  assign MemWrite = (Opcode == SW);
  assign ALUop[0] = (Opcode == BR || Opcode == JALR);
  assign ALUop[1] = (Opcode == R_TYPE || Opcode == I_TYPE || Opcode == JALR);
  assign Branch = (Opcode == BR);
  assign jump = (Opcode == JAL) || (Opcode == JALR);
  assign jumpreg = (Opcode == JALR);
  assign Halt = (Opcode == HALT);

endmodule
