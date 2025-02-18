`timescale 1ns / 1ps

module BranchUnit #(
    parameter PC_W = 9
) (
    input logic [PC_W-1:0] Cur_PC,
    input logic [31:0] Imm,
    input logic jump,
    input logic Branch,
    input logic Halt,
    input logic jumpreg,
    input logic [31:0] AluResult,
    output logic [31:0] PC_Imm,
    output logic [31:0] PC_Four,
    output logic [31:0] BrPC,
    output logic PcSel
);

  logic Branch_Sel;
  logic [31:0] PC_Full;
  logic Halt_selector; 

  assign PC_Full = {23'b0, Cur_PC}; 
  assign Halt_selector = Halt;
  assign PC_Imm = (jumpreg) ? AluResult : (PC_Full + Imm); 
  assign PC_Four = PC_Full + 32'b100; 
  assign Branch_Sel = jump || (Branch && AluResult[0]);  // Se for zero eh branch, se for um, nao pega o branch

  assign BrPC = (Branch_Sel) ? PC_Imm : ((Halt_selector) ? PC_Imm : 32'b0);  // Branch -> PC+Imm   // Senao, ignora Brpc
  assign PcSel = Branch_Sel || jump || Halt_selector;  

endmodule
