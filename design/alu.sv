`timescale 1ns / 1ps

module alu#(
        parameter DATA_WIDTH = 32,
        parameter OPCODE_LENGTH = 4
        )
        (
                input logic [DATA_WIDTH-1:0]    SrcA,
                input logic [DATA_WIDTH-1:0]    SrcB,

                input logic [OPCODE_LENGTH-1:0]    Operation,
                output logic[DATA_WIDTH-1:0] ALUResult
        );
    
        always_comb
        begin
            case(Operation)
            4'b0000:        // AND
                    ALUResult = SrcA & SrcB;
            4'b0001:        // OR
                    ALUResult = SrcA | SrcB;
            4'b0010:        // ADD e ADDI
                    ALUResult = SrcA + SrcB; 
            4'b0011:        // SUB
                    ALUResult = SrcA - SrcB;
            4'b0100:        // XOR
                    ALUResult = SrcA ^ SrcB;
            4'b1000: // BEQ
    		if (SrcA == SrcB)
        	ALUResult = 1;
    		else 
        	ALUResult = 0;
	    4'b1001: // Be Not Equal (bne)
    		if (SrcA != SrcB)
        	ALUResult = 1;
    		else 
        	ALUResult = 0;
	    4'b1010: // Be Less than (blt)
    		if (SrcA < SrcB)
        	ALUResult = 1;
    		else 
        	ALUResult = 0;
	    4'b1011: // Greater than or equal
    		if (SrcA >= SrcB)
        	ALUResult = 1;
    		else 
        	ALUResult = 0;
            4'b1100:        // Shift left (SLLI)
                    ALUResult = SrcA << SrcB;
            4'b1101:        // Shift right (SRLI)
                    ALUResult = SrcA >> SrcB;
            4'b1110: // Slt e Slti
    		if (SrcA < SrcB) 
        	ALUResult = 1;
    		else 
        	ALUResult = 0;
            4'b1111:        //SRAI
                ALUResult = SrcA >>> SrcB[4:0];
            default:
                    ALUResult = 0;
            endcase
        end
endmodule
