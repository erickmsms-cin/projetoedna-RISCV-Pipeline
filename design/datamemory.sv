`timescale 1ns / 1ps

module datamemory #(
    parameter DM_ADDRESS = 9,
    parameter DATA_W = 32
) (
    input logic clk,
    input logic MemRead,  // vem da unidade de controle
    input logic MemWrite,  // vem da unidade de controle
    input logic [DM_ADDRESS - 1:0] a,  // Endereço de leitura/escrita - 9 bits menos significativos da saída da ALU
    input logic [DATA_W - 1:0] wd,  // Dados para escrita
    input logic [2:0] Funct3,  // bits 12 a 14 da instrução
    output logic [DATA_W - 1:0] rd  // Dados lidos
);


  logic [31:0] raddress;
  logic [31:0] waddress;
  logic [31:0] Datain;
  logic [31:0] Dataout;
  logic [ 3:0] Wr;

  Memoria32Data mem32 (
      .raddress(raddress),
      .waddress(waddress),
      .Clk(~clk),
      .Datain(Datain),
      .Dataout(Dataout),
      .Wr(Wr)
  );

  always_ff @(*) begin
    raddress = {{22{1'b0}}, a};
    waddress = {{22{1'b0}}, {a[8:2], {2{1'b0}}}};
    Datain = wd;
    Wr = 4'b0000;

    if (MemRead) begin
      case (Funct3)
        3'b010: begin //LW
          rd <= Dataout;
        end
        3'b000: begin  //LB
          rd <= {24'b0, Dataout[7:0]};
        end
        3'b100: begin  //LBU
          rd <= {1'b0, Dataout[30:0]};
        end
        3'b001: begin  //LH
          rd <= {16'b0, Dataout[15:0]};
        end
        default: rd <= Dataout;
      endcase
    end else if (MemWrite) begin
      case (Funct3)
        3'b010: begin  //SW
          Wr <= 4'b1111;
          Datain <= wd;
        end
        3'b000:  begin //SB
          Wr <= 4'b1111;
          Datain <= wd[7:0];
        end
        3'b001:  begin //SH
          Wr <= 4'b1111;
          Datain <= wd[15:0];
        end
        default: begin
          Wr <= 4'b1111;
          Datain <= wd;
        end
      endcase
    end
  end

endmodule
