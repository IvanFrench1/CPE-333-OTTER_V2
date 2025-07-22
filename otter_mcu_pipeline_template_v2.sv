`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/04/2019 04:32:12 PM
// Design Name: 
// Module Name: PIPELINED_OTTER_CPU
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

  typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP       = 7'b0110011,
           SYSTEM   = 7'b1110011
 } opcode_t;
        
typedef struct packed{
    opcode_t opcode;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;
    logic [4:0] rd_addr;
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] alu_fun;
    logic memWrite;
    logic memRead2;
    logic regWrite;
    logic [1:0] rf_wr_sel;
    logic [2:0] mem_type;  //sign, size
    logic [31:0] pc;
} instr_t;

module OTTER_MCU(input CLK,
                input INTR,
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
); 

    wire [31:0] pc_out, pc_plus_4;
    wire [31:0] jalr, branch, jal;
    logic pc_write_en;
    logic [2:0] pc_source_sel;


    wire [6:0] opcode;
    wire [31:0] pc, pc_value, next_pc, jalr_pc, branch_pc, jump_pc, int_pc,A,B,
        I_immed,S_immed,U_immed,aluBin,aluAin,aluResult,rfIn,csr_reg, mem_data;
    
    wire [31:0] IR;
    wire memRead1,memRead2;
    
    wire pcWrite,regWrite,memWrite, op1_sel,mem_op,IorD,pcWriteCond,memRead;
    wire [1:0] opB_sel, rf_sel, wb_sel, mSize;
    logic [1:0] pc_sel;
    wire [3:0]alu_fun;
    wire opA_sel;
    
    logic br_lt,br_eq,br_ltu;

              
//==== Instruction Fetch ===========================================
    
     logic [31:0] if_de_pc; //register
     logic [31:0] if_de_ir; //register
    
    //instantiate pc
    PC PC_unit (
        .CLK(CLK),
        .RST(RST),
        .PC_WRITE(pc_write_en), 
        .PC_SOURCE(pc_source_sel),  
        .JALR(jalr),      
        .BRANCH(branch),
        .JAL(jal),
        .MTVEC(32'b0),  
        .MEPC(32'b0), 
        .PC_OUT(pc_out),
        .PC_OUT_INC(pc_plus_4)
    );

    assign pc_write_en = 1'b1;
    assign pc_source_sel = 3'b000; //selects pc outplus4 in pcmux 

     
     always_ff@(posedge CLK) begin
        if (RST) begin
            if_de_pc <= 32'b0;
            if_de_ir <= 32'b0;
        end else begin
            if_de_pc <= pc_out
            if_de_ir <= IR;
        end
     end
     
<<<<<<< HEAD
=======
     assign pcWrite = 1'b1; 	//Hardwired high, assuming now hazards
     assign memRead1 = 1'b1; 	//Fetch new instruction every cycle
     
>>>>>>> 3e83621aa1c01fbec2d6b266f6dcd6ba1fd3ec4c
	//send if_de_pc to reg module



     
//==== Instruction Decode ===========================================
    logic [31:0] de_ex_opA;
    logic [31:0] de_ex_opB;
    logic [31:0] de_ex_rs2;

    logic [31:0] rs1_data, rs2_data;
    logic [31:0] imm_i, imm_s, imm_b, imm_u, imm_j;
    logic [31:0] final_immediate;
    logic [3:0] alu_fun;
    logic alu_srca;
    logic [1:0] alu_srcb;
    logic [2:0] cu_pc_source; 
    logic [1:0] rf_wr_sel;
    logic br_eq, br_lt, br_ltu;

    logic [6:0]  id_opcode = if_de_ir[6:0];
    logic [2:0]  id_funct3 = if_de_ir[14:12];
    logic id_ir_30 = if_de_ir[30];


    instr_t de_ex_inst, de_inst;

    //instantiate
    ImmediateGenerator IMMED_GEN (
        .IR(ir[24:0]), 
        .U_TYPE(imm_u), 
        .I_TYPE(imm_i), 
        .S_TYPE(imm_s), 
        .B_TYPE(imm_b), 
        .J_TYPE(imm_j)
    );
    CU_DCDR decoder (
        .IR_30(ir_30), 
        .IR_OPCODE(opcode), 
        .IR_FUNCT(funct3),
        .BR_EQ(br_eq), 
        .BR_LT(br_lt), 
        .BR_LTU(br_ltu),
        .ALU_FUN(alu_fun), 
        .ALU_SRCA(alu_srca), 
        .ALU_SRCB(alu_srcb),
        .PC_SOURCE(cu_pc_source), 
        .RF_WR_SEL(rf_wr_sel)
    );
    BCG branch_cond_gen (
        .RS1(rs1_data), 
        .RS2(rs2_data), 
        .BR_EQ(br_eq), 
        .BR_LT(br_lt), 
        .BR_LTU(br_ltu)
    );
    REG_FILE reg_file (
    .CLK(CLK), 
    .EN(wb_reg_write_en), //write enable writeback 
    .ADR1(if_de_ir[19:15]),
    .ADR2(if_de_ir[24:20]),
    .WA(wb_rd_addr), //write address writeback
    .WD(wb_write_data), //write data writeback
    .RS1(rs1_data),//output read port1
    .RS2(rs2_data)//output read port 2
    );

    always_comb begin
        case (id_opcode)
            `LOAD, `OP_IMM, `JALR: final_immediate = imm_i;
            `STORE:                final_immediate = imm_s;
            `BRANCH:               final_immediate = imm_b;
            `LUI, `AUIPC:          final_immediate = imm_u;
            `JAL:                  final_immediate = imm_j;
            default:               final_immediate = 32'b0;
        endcase
    end

    opcode_t OPCODE;
    assign OPCODE_t = opcode_t'(opcode);


    assign de_inst.alu_fun=alu_fun;
    assign de_inst.rf_wr_sel=rf_wr_sel;
    
    assign de_inst.rs1_addr=if_de_ir[19:15];
    assign de_inst.rs2_addr=if_de_ir[24:20];
    assign de_inst.rd_addr=if_de_ir[11:7];
    assign de_inst.opcode=OPCODE;
    assign de_inst.pc=if_de_pc;

    logic [31:0] de_ex_rs1_data, de_ex_rs2_data, de_ex_immediate;

    always_ff @(posedge CLK) begin
        if(RESET) begin
            de_ex_rs1_data  <= 32'b0;
            de_ex_rs2_data  <= 32'b0;
            de_ex_immediate <= 32'b0;
        end else begin
            de_ex_rs1_data  <= rs1_data;
            de_ex_rs2_data  <= rs2_data;
            de_ex_immediate <= final_immediate;
        end
    end

    always_ff @(posedge CLK) begin
                de_ex_inst <= de_inst;
    end
	//Send de_ex_inst into IMMED_GEN/FSM, then it should go out to ALU.

<<<<<<< HEAD
=======
     always_ff @(posedge CLK) begin
                de_ex_inst <= de_inst;
     end
	//Send de_ex_inst into IMMED_GEN/FSM, then it should go out to ALU.

>>>>>>> 3e83621aa1c01fbec2d6b266f6dcd6ba1fd3ec4c
	
//==== Execute ======================================================
     logic [31:0] ex_mem_rs2;
     logic ex_mem_aluRes = 0;
     instr_t ex_mem_inst;
     logic [31:0] opA_forwarded;
     logic [31:0] opB_forwarded;
     
     // Creates a RISC-V ALU
    OTTER_ALU ALU (de_ex_inst.alu_fun, de_ex_opA, de_ex_opB, aluResult); // the ALU
     
	always_ff(@posedge CLK) begin
		ex_mem_rs2 <= ex_mem_inst
	end



//==== Memory ======================================================
     
     
    assign IOBUS_ADDR = ex_mem_aluRes;
    assign IOBUS_OUT = ex_mem_rs2;
    assign de_ex_inst.ir = 32'b0;
    assign IOBUS_WR = ex_mem_writeMem; //write enable for stores

    instr_t mem_wb_inst;
    logic [31:0] ex_mem_aluRes;
    logic [31:0] mem_read_data;
    
    always_ff @(posedge CLK) begin
        if (RESET) begin
            mem_wb_inst <= 0;
            ex_mem_aluRes <= 32'b0;
        end else begin
            mem_wb_inst<= ex_mem_inst;
            ex_mem_aluRes<= ex_mem_aluRes;
        end
    end
     
//==== Write Back ==================================================
     
    FourMux writeback_mux(
        .SEL(mem_wb_inst.rf_wr_sel),
        .ZERO(mem_wb_inst.pc + 4),
        .ONE(32'b0),               
        .TWO(mem_read_data),
        .THREE(mem_wb_alu_result),
        .OUT(wb_write_data)
    );
 
       
            
endmodule