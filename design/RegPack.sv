package Pipe_Buf_Reg_PKG;
  // Reg do tipo A
  typedef struct packed {
    logic [8:0]  Curr_Pc;
    logic [31:0] Curr_Instr;
    
  } if_id_reg;

  // Tipo B
  typedef struct packed {
    logic        ALUSrc;
    logic        MemtoReg;
    logic        RegWrite;
    logic        MemRead;
    logic        MemWrite;
    logic        Branch;
    logic        jump;
    logic        jumpreg;
    logic        Halt;
    logic [1:0]  ALUOp;
    logic [4:0]  RS_Two;
    logic [4:0]  rd;
    logic [31:0] ImmG;
    logic [2:0]  func3;
    logic [6:0]  func7;
    logic [31:0] Curr_Instr;
    logic [8:0]  Curr_Pc;
    logic [31:0] RD_One;
    logic [31:0] RD_Two;
    logic [4:0]  RS_One;
        
  } id_ex_reg;

  // Tipo C
  typedef struct packed {
    logic        RegWrite;
    logic        jump;
    logic        MemtoReg;
    logic        MemRead;
    logic        MemWrite;
    logic [31:0] Pc_Imm;
    logic [31:0] Pc_Four;
    logic [31:0] Imm_Out;
    logic [31:0] Alu_Result;
    logic [31:0] RD_Two;
    logic [4:0]  rd;
    logic [2:0]  func3;
    logic [6:0]  func7;
    logic [31:0] Curr_Instr;
    
  } ex_mem_reg;

  // Tipo D
  typedef struct packed {
    logic        RegWrite;
    logic        jump;
    logic        MemtoReg;
    logic [31:0] Pc_Imm;
    logic [31:0] Pc_Four;
    logic [31:0] Imm_Out;
    logic [31:0] Alu_Result;
    logic [31:0] MemReadData;
    logic [4:0]  rd;
    logic [31:0] Curr_Instr;
    
  } mem_wb_reg;
endpackage
