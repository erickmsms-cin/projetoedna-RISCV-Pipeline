`timescale 1ns / 1ps

module riscv #(
    parameter DATA_W = 32
) (
    input logic clk,
    reset,  // sinal de clock e de reset
    output logic [31:0] WB_Data,  // resultado da alu
    output logic [4:0] reg_num,
    output logic [31:0] reg_data,
    output logic reg_write_sig,
    output logic wr,
    output logic rd,
    output logic [8:0] addr,
    output logic [DATA_W-1:0] wr_data,
    output logic [DATA_W-1:0] rd_data
);

  logic [6:0] opcode;
  logic ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, jump, jumpreg, Halt;
  logic [1:0] ALUop_Reg;
  logic [6:0] Funct7;
  logic [2:0] Funct3;
  logic [3:0] Operation;
  logic [1:0] ALUop;

  Controller c (
      opcode,
      ALUSrc,
      MemtoReg,
      RegWrite,
      MemRead,
      MemWrite,
      ALUop,
      Branch,
      jump,
      jumpreg,
      Halt
  );

  ALUController ac (
      ALUop_Reg,
      Funct7,
      Funct3,
      Operation
  );

  Datapath dp (
      clk,
      reset,
      RegWrite,
      jump,
      MemtoReg,
      ALUSrc,
      MemWrite,
      MemRead,
      Branch,
      jumpreg,
      Halt,
      ALUop,
      Operation,
      opcode,
      Funct7,
      Funct3,
      ALUop_Reg,
      WB_Data,
      reg_num,
      reg_data,
      reg_write_sig,
      wr,
      rd,
      addr,
      wr_data,
      rd_data
  );

endmodule
