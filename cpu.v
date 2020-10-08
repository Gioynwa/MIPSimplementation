/***********************************************************************************************/
/*********************************  MIPS 5-stage pipeline implementation ***********************/
/***********************************************************************************************/

module cpu(input clock, input reset);
 reg [31:0] PC; 
 reg [31:0] IFID_PCplus4;
 reg [31:0] IFID_instr;
 reg [31:0] IDEX_rdA, IDEX_rdB, IDEX_signExtend, IDEX_PCplus4;
 reg [4:0]  IDEX_instr_rt, IDEX_instr_rs, IDEX_instr_rd, IDEX_shamt;                            
 reg        IDEX_RegDst, IDEX_ALUSrc;
 reg [1:0]  IDEX_ALUcntrl;
 reg IDEX_Branch, EXMEM_Branch, MEMWB_Branch;
 reg        IDEX_MemRead, IDEX_MemWrite; 
 reg        IDEX_MemToReg, IDEX_RegWrite;           
 reg [4:0]  EXMEM_RegWriteAddr, EXMEM_instr_rd, EXMEM_instr_rt, EXMEM_shamt; 
 reg [31:0] EXMEM_ALUOut, EXMEM_PCplus4, EXMEM_signExtend;
 reg        EXMEM_Zero;
 reg [31:0] EXMEM_MemWriteData;
 reg        EXMEM_MemRead, EXMEM_MemWrite, EXMEM_RegWrite, EXMEM_MemToReg;
 reg [31:0] MEMWB_DMemOut;
 reg [4:0]  MEMWB_RegWriteAddr, MEMWB_instr_rd, MEMWB_instr_rt, MEMWB_shamt; 
 reg [31:0] MEMWB_ALUOut;
 reg        MEMWB_MemToReg, MEMWB_RegWrite;   
 reg [5:0] IDEX_opcode, EXMEM_opcode, MEMWB_opcode;            
 wire [31:0] instr, ALUInA, ALUInB, ALUOut, rdA, rdB, signExtend, DMemOut, wRegData, PCIncr, ALUInBtemp, MemWriteData, PCplus4;
 wire Zero, RegDst, MemRead, MemWrite, MemToReg, ALUSrc, RegWrite, Jump;
 wire PCSrc;
 wire [5:0] opcode, func;
 wire [4:0] instr_rs, instr_rt, instr_rd, RegWriteAddr, shamt;
 wire [3:0] ALUOp;
 wire [1:0] ALUcntrl;
 wire [15:0] imm;
 wire [1:0] bypassA, bypassB; 
 wire [25:0] jumpAddress;
 reg [25:0] IFID_jumpAddress,IDEX_jumpAddress;
 wire bubble_idex;
 wire Branch;
 wire [31:0] temp;

/***************** Instruction Fetch Unit (IF)  ****************/
assign temp = {IFID_PCplus4[31:28], (jumpAddress << 2)};
assign PCplus4 = PC+4;

 always @(posedge clock or negedge reset) begin

    if (reset == 1'b0)
       PC <= -1;

    else if (PC == -1)
       PC <= 0;

    else if (bubble_idex == 1'b0) begin

        if (Jump == 1)  begin //jump
            PC <= temp;
        end
        else if (PCSrc == 1)  //branches
            PC <= EXMEM_PCplus4 + (EXMEM_signExtend << 2);
  
        else
            PC <= PCplus4;
    end
    else if(bubble_idex == 1'b1) 
	PC <= PC;    
  end //always


  // IFID pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0)     
      begin
       IFID_PCplus4 <= 32'b0;    
       IFID_instr <= 32'b0;
      end 
    else     
      begin
       IFID_PCplus4 <= PCplus4;
       if(bubble_idex == 1'b0)
         IFID_instr <= instr;
       if((PCSrc == 1'b1)||(Jump == 1'b1))
	 IFID_instr <= 32'b0;
      end
  end
  
// Instruction memory 1KB
Memory cpu_IMem(clock, reset, 1'b1, 1'b0, PC>>2, 32'b0, instr);
 
  
  
  
  
/***************** Instruction Decode Unit (ID)  ****************/
assign opcode = IFID_instr[31:26];
assign func = IFID_instr[5:0];
assign instr_rs = IFID_instr[25:21];
assign instr_rt = IFID_instr[20:16];
assign instr_rd = IFID_instr[15:11];
assign imm = IFID_instr[15:0];
assign shamt = IFID_instr[10:6];
assign jumpAddress = IFID_instr[25:0];
assign signExtend = {{16{imm[15]}}, imm};





// Register file
RegFile cpu_regs(clock, reset, 
                  instr_rs, 
                  instr_rt, 
                  MEMWB_RegWriteAddr, 
                  MEMWB_RegWrite, 
                  wRegData, 
                  rdA, 
                  rdB);

  // IDEX pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if ((reset == 1'b0)||(PCSrc == 1)||(Jump == 1))
      begin
       IDEX_opcode <= 5'b0;
       IDEX_rdA <= 32'b0;    
       IDEX_rdB <= 32'b0;
       IDEX_signExtend <= 32'b0;
       IDEX_jumpAddress <= 26'b0;
       IDEX_instr_rd <= 5'b0;
       IDEX_instr_rs <= 5'b0;
       IDEX_instr_rt <= 5'b0;
       IDEX_RegDst <= 1'b0;
       IDEX_ALUcntrl <= 2'b0;
       IDEX_ALUSrc <= 1'b0;
       IDEX_Branch <= 1'b0;
       IDEX_MemRead <= 1'b0;
       IDEX_MemWrite <= 1'b0;
       IDEX_MemToReg <= 1'b0;                  
       IDEX_RegWrite <= 1'b0;
       IDEX_shamt <= 5'b0;
       IDEX_PCplus4 <= 32'b0;
      end 
    else if(bubble_idex == 1'b0)
      begin
       IDEX_opcode <= opcode;	
       IDEX_rdA <= rdA;
       IDEX_rdB <= rdB;
       IDEX_signExtend <= signExtend;
       IDEX_jumpAddress <= jumpAddress;
       IDEX_instr_rd <= instr_rd;
       IDEX_instr_rs <= instr_rs;
       IDEX_instr_rt <= instr_rt;
       IDEX_RegDst <= RegDst;
       IDEX_ALUcntrl <= ALUcntrl;
       IDEX_ALUSrc <= ALUSrc;
       IDEX_Branch <= Branch;
       IDEX_MemRead <= MemRead;
       IDEX_MemWrite <= MemWrite;
       IDEX_MemToReg <= MemToReg;                  
       IDEX_RegWrite <= RegWrite;
       IDEX_shamt <= shamt;
       IDEX_PCplus4 <= IFID_PCplus4;
       end
    else if(bubble_idex == 1'b1)
      begin
       IDEX_RegDst <= 1'b0;
       IDEX_ALUcntrl <= 2'b00;
       IDEX_ALUSrc <= 1'b0;
       IDEX_Branch <= 1'b0;
       IDEX_MemRead <= 1'b0;
       IDEX_MemWrite <= 1'b0;
       IDEX_MemToReg <= 1'b0;                  
       IDEX_RegWrite <= 1'b0;
      end
  end

 
// Main Control Unit 
control_main control_main (RegDst,
                  Branch,
                  MemRead,
                  MemWrite,
                  MemToReg,
                  ALUSrc,
                  RegWrite,
                  ALUcntrl,
                  opcode,
                  Jump);
                  
                  
// Instantiation of Control Unit that generates stalls goes here
ctrl_stall bubble(Branch,IDEX_MemRead, bubble_idex, IDEX_instr_rt, instr_rt, instr_rs);


/********************************* Execution Unit (EX)******************************/

/*************************Forwarding Unit*************************/
assign ALUInA = 
          (bypassA == 2'b00) ? IDEX_rdA :
          (bypassA == 2'b01) ? wRegData :
          (bypassA == 2'b10) ? EXMEM_ALUOut :
          'bx;
          
assign ALUInBtemp = 
          (bypassB == 2'b00) ? IDEX_rdB :
          (bypassB == 2'b01) ? wRegData :
          (bypassB == 2'b10) ? EXMEM_ALUOut :         
          'bx;
               
assign ALUInB = (IDEX_ALUSrc == 1'b0) ? ALUInBtemp : IDEX_signExtend;
/********************************************************************/


//  ALU
ALU cpu_alu(ALUOut, Zero, ALUInA, ALUInB, ALUOp, IDEX_shamt);

//set PCSrc signal
PCSrc_set set(EXMEM_opcode, EXMEM_Zero, EXMEM_Branch, PCSrc);

assign RegWriteAddr = (IDEX_RegDst==1'b0) ? IDEX_instr_rt : IDEX_instr_rd;

 // EXMEM pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if ((reset == 1'b0)||(PCSrc == 1'b1))     
      begin
       EXMEM_instr_rt <= 5'b0;
       EXMEM_opcode <= 5'b0;
       EXMEM_ALUOut <= 32'b0;    
       EXMEM_RegWriteAddr <= 5'b0;
       EXMEM_MemWriteData <= 32'b0;
       EXMEM_Zero <= 1'b0;
       EXMEM_Branch <= 1'b0;
       EXMEM_MemRead <= 1'b0;
       EXMEM_MemWrite <= 1'b0;
       EXMEM_MemToReg <= 1'b0;                  
       EXMEM_RegWrite <= 1'b0;
       EXMEM_shamt <= 5'b0;
       EXMEM_signExtend <= 32'b0;
       EXMEM_PCplus4 <= 32'b0;
      end 
    else 
      begin
       EXMEM_instr_rt <= IDEX_instr_rt;	
       EXMEM_opcode <= IDEX_opcode;
       EXMEM_ALUOut <= ALUOut;    
       EXMEM_RegWriteAddr <= RegWriteAddr;
       EXMEM_MemWriteData <= ALUInBtemp;
       EXMEM_Zero <= Zero;
       EXMEM_Branch <= IDEX_Branch;
       EXMEM_MemRead <= IDEX_MemRead;
       EXMEM_MemWrite <= IDEX_MemWrite;
       EXMEM_MemToReg <= IDEX_MemToReg;                  
       EXMEM_RegWrite <= IDEX_RegWrite;
       EXMEM_shamt <= IDEX_shamt;
      // passing instr_rd
       EXMEM_instr_rd <= RegWriteAddr;
       EXMEM_PCplus4 <= IDEX_PCplus4;
       EXMEM_signExtend <= IDEX_signExtend;
      end
  end
  
  // ALU control
  control_alu control_alu(ALUOp, IDEX_ALUcntrl, IDEX_signExtend[5:0]);
  
   // Instantiation of control logic for Forwarding goes here
  control_bypass_ex forwarding(Branch,
                              bypassA, 
                              bypassB, 
                              IDEX_instr_rs, 
                              IDEX_instr_rt,
                              EXMEM_instr_rd, 
                              MEMWB_instr_rd, 
                              EXMEM_RegWrite, 
                              MEMWB_RegWrite);

  
  
  
/***************** Memory Unit (MEM)  ****************/  

// Data memory 1KB
Memory cpu_DMem(clock, reset, EXMEM_MemRead, EXMEM_MemWrite, EXMEM_ALUOut, EXMEM_MemWriteData, DMemOut);

// MEMWB pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0)
      begin
       MEMWB_instr_rt <= 5'b0;
       MEMWB_opcode <= 5'b0;
       MEMWB_DMemOut <= 32'b0;    
       MEMWB_ALUOut <= 32'b0;
       MEMWB_RegWriteAddr <= 5'b0;
       MEMWB_MemToReg <= 1'b0;                  
       MEMWB_RegWrite <= 1'b0;
       MEMWB_Branch <= 1'b0;
      end 
    else 
      begin
       MEMWB_instr_rt <= EXMEM_instr_rt;
       MEMWB_opcode <= EXMEM_opcode;
       MEMWB_DMemOut <= DMemOut;
       MEMWB_ALUOut <= EXMEM_ALUOut;
       MEMWB_RegWriteAddr <= EXMEM_RegWriteAddr;
       MEMWB_MemToReg <= EXMEM_MemToReg;                  
       MEMWB_RegWrite <= EXMEM_RegWrite;
       MEMWB_instr_rd <= EXMEM_instr_rd;
       MEMWB_shamt <= EXMEM_shamt;
       MEMWB_Branch <= EXMEM_Branch;
      end
  end
  
/***************** WriteBack Unit (WB)  ****************/  
assign wRegData = (MEMWB_MemToReg == 1'b0) ? MEMWB_ALUOut : MEMWB_DMemOut;


endmodule
