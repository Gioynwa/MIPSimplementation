`include "constants.h"

/************** Main control in ID pipe stage  *************/
module control_main(output reg RegDst,
                output reg Branch,  
                output reg MemRead,
                output reg MemWrite,  
                output reg MemToReg,  
                output reg ALUSrc,  
                output reg RegWrite,  
                output reg [1:0] ALUcntrl,  
                input [5:0] opcode,
                output reg Jump);

  always @(*) 
   begin
     case (opcode)
      `R_FORMAT: 
          begin 
            RegDst = 1'b1;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            MemToReg = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b1;
            Branch = 2'b00;         
            ALUcntrl  = 2'b10; // R          
            Jump = 1'b0;
          end
       `LW :   
           begin 
            RegDst = 1'b0;
            MemRead = 1'b1;
            MemWrite = 1'b0;
            MemToReg = 1'b1;
            ALUSrc = 1'b1;
            RegWrite = 1'b1;
            Branch = 1'b0;
            ALUcntrl  = 2'b00; // add
            Jump = 1'b0;
           end
        `SW :   
           begin 
            RegDst = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b1;
            MemToReg = 1'b0;
            ALUSrc = 1'b1;
            RegWrite = 1'b0;
            Branch = 1'b0;
            ALUcntrl  = 2'b00; // add
            Jump = 1'b0;
           end
       `BEQ:  
           begin 
            RegDst = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            MemToReg = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b0;
            Branch = 1'b1;
            ALUcntrl = 2'b01; // sub
            Jump = 1'b0;
           end
       `BNE:
           begin
            RegDst = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            MemToReg = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b0;
            Branch = 1'b1;
            ALUcntrl = 2'b01; // sub
            Jump = 1'b0; 
           end 
       `ADDI:
	   begin
	    RegDst = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            MemToReg = 1'b0;
            ALUSrc = 1'b1;
            RegWrite = 1'b1;
            Branch = 1'b0;
            ALUcntrl = 2'b00;
            Jump = 1'b0;
           end
       `JUMP :
           begin 
	    RegDst = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            MemToReg = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b0;
            Branch = 1'b0;
            ALUcntrl = 2'b00;
            Jump = 1'b1;
	   end
       default:
           begin
            RegDst = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            MemToReg = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b0;
            ALUcntrl = 2'b00; 
            Branch = 1'b0;
            Jump = 0;
           end
      endcase
    end // always
endmodule


module PCSrc_set(input [5:0] opcode,
                 input Zero,
                 input Branch,
                 output reg PCSrc);
/***********************************************/
always@(*) begin

   if ((opcode == `BNE) && (~Zero & Branch))
      PCSrc = 1'b1;

   else if ((opcode == `BEQ) && (Zero & Branch))
      PCSrc = 1'b1;

   else
      PCSrc = 1'b0;
end
/**********************************************/
endmodule


/**************** Module for Bypass Detection in EX pipe stage goes here  *********/
 module  control_bypass_ex(input Branch,  //prepei na proste8ei kai katw
                       output reg [1:0] bypassA,
                       output reg [1:0] bypassB,
                       input [4:0] idex_rs,
                       input [4:0] idex_rt,
                       input [4:0] exmem_rd,
                       input [4:0] memwb_rd,
                       input       exmem_regwrite,
                       input       memwb_regwrite);

  
  // EX/MEM MEM/WB equations

/****************souloupwma kai pros8hkh twn Branch*****************/
    always@(*)
      begin  
	bypassA = 2'b00;
	bypassB = 2'b00; 
        if((exmem_regwrite == 1'b1) & (exmem_rd != 4'b0) & (exmem_rd == idex_rs)) begin
          bypassA = 2'b10;
        end
        if((exmem_regwrite == 1'b1) & (exmem_rd != 4'b0) & (exmem_rd == idex_rt)) begin
          bypassB = 2'b10;
	end
	if((memwb_regwrite == 1'b1) & (memwb_rd != 4'b0) & (memwb_rd == idex_rs) & ((exmem_rd != idex_rs) | (exmem_regwrite == 1'b0)))
        begin  
	  bypassA = 2'b01;
        end
        if((memwb_regwrite == 1'b1) & (memwb_rd != 4'b0) & (memwb_rd == idex_rt) & ((exmem_rd != idex_rt) | (exmem_regwrite == 1'b0)))
        begin  
	  bypassB = 2'b01;
	end
    end
endmodule          
                       

/**************** Module for Stall Detection in ID pipe stage goes here  *********/
module ctrl_stall(input  Branch,
               input IDEX_MemRead,
               output reg bubble_idex, 
               input [4:0] idex_rt,
               input [4:0] ifid_rt,
               input [4:0] ifid_rs
             );

always@(*)
     begin
       if(IDEX_MemRead&&((idex_rt == ifid_rs) || (idex_rt == ifid_rt)))
         begin 
	 bubble_idex = 1;
       	 end
        else 
         bubble_idex = 0;
      end
endmodule          
                       
/************** control for ALU control in EX pipe stage  *************/
module control_alu(output reg [3:0] ALUOp,                  
               input [1:0] ALUcntrl,
               input [5:0] func);

  always @(ALUcntrl or func)  
    begin
      case (ALUcntrl)
        2'b10: 
           begin
             case (func)
              6'b100000: ALUOp = 4'b0010; // add
              6'b100010: ALUOp = 4'b0110; // sub
              6'b100100: ALUOp = 4'b0000; // and
              6'b100101: ALUOp = 4'b0001; // or
              6'b100111: ALUOp = 4'b1100; // nor
              6'b101010: ALUOp = 4'b0111; // slt
	      6'b000000: ALUOp = 4'b0100; // sll
              6'b000100: ALUOp = 4'b0101; // sllv  
              default: ALUOp = 4'b0000;       
             endcase 
          end   
        2'b00: 
              ALUOp = 4'b0010; // add
        2'b01: 
              ALUOp = 4'b0110; // sub
        default:
              ALUOp = 4'b0000; //nop
     endcase
    end
endmodule
