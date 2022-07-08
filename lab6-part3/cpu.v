`timescale 1ns/100ps

module cpu_tb;

    reg CLK, RESET;
    wire [31:0] PC;
    wire [31:0] INSTRUCTION;

    wire Write, Read, MEMWrite, MEMRead;
    wire [7:0] READDATA, ADDRESS, WRITEDATA;
    wire [5:0] MEMADDRESS;
    wire [31:0] MEMWRITEDATA;
    wire [31:0] MEMREADDATA;
    wire BUSYWAIT, MEMBUSYWAIT;
    wire INSTRUCT_READ, INSTRUCT_BUSYWAIT, INSTCACHE_BUSYWAIT, DCACHE_BUSYWAIT;
    wire [5 : 0] INSTRUCT_ADDRESS;
    wire [127 : 0] INSTRUCT_READDATA;

    assign BUSYWAIT = (INSTCACHE_BUSYWAIT || DCACHE_BUSYWAIT);

    /* 
    ------------------------
     SIMPLE INSTRUCTION MEM
    ------------------------
    
    
    // TODO: Initialize an array of registers (8x1024) named 'instr_mem' to be used as instruction memory
    reg [7:0] instr_mem[0:1023];

    // TODO: Create combinational logic to support CPU instruction fetching, given the Program Counter(PC) value 
    //       (make sure you include the delay for instruction fetching here)

    // always @(PC) begin
    //     #2
    //     INSTRUCTION = {instr_mem[PC+3], instr_mem[PC+2], instr_mem[PC+1], instr_mem[PC]};
    // end
    
    initial
    begin
        // Initialize instruction memory with the set of instructions you need execute on CPU
        
        // METHOD 1: manually loading instructions to instr_mem
        // {instr_mem[10'd3], instr_mem[10'd2], instr_mem[10'd1], instr_mem[10'd0]} = 32'b0000000000000700000000000000101;
        // {instr_mem[10'd7], instr_mem[10'd6], instr_mem[10'd5], instr_mem[10'd4]} = 32'b0000000000000070000000000001001;
        // {instr_mem[10'd11], instr_mem[10'd10], instr_mem[10'd9], instr_mem[10'd8]} = 32'b00000070000017000007000000010;
        
        // METHOD 2: loading instr_mem content from instr_mem.mem file
        //$readmemb("instr_mem.mem", instr_mem);
    end

    */
    
    /* 
    -----
     CPU
    -----
    */
    // instantiate cpu
    cpu mycpu(PC, INSTRUCTION, CLK, RESET, Read, Write, ADDRESS, WRITEDATA, READDATA, BUSYWAIT);

    // instantiate instruction memory'
    instruction_memory d1(CLK, INSTRUCT_READ, INSTRUCT_ADDRESS, INSTRUCT_READDATA, INSTRUCT_BUSYWAIT);

    // instantiate instruction cache
    instruction_cache ich(CLK, RESET, PC[9 : 2], INSTCACHE_BUSYWAIT, INSTRUCT_READ, INSTRUCT_ADDRESS, INSTRUCT_READDATA, INSTRUCT_BUSYWAIT, INSTRUCTION);

    // instantiate data memory
    data_memory d2(CLK, RESET, MEMRead, MEMWrite, MEMADDRESS, MEMWRITEDATA, MEMREADDATA, MEMBUSYWAIT);

    // instantiate data cache
    dcache ch(CLK, RESET, Read, Write, ADDRESS, WRITEDATA, READDATA, DCACHE_BUSYWAIT, MEMRead, MEMWrite, MEMADDRESS, MEMWRITEDATA, MEMREADDATA, MEMBUSYWAIT);

    initial
    begin
    
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("cpu_wavedata.vcd");
		$dumpvars(0, cpu_tb);
        
        CLK = 1'b0;
        RESET = 1'b0;
        
        // TODO: Reset the CPU (by giving a pulse to RESET signal) to start the program execution
        #2
        RESET = 1'b1;

        #4
        RESET= 1'b0;
        
        // finish simulation after some time
        #4000
        $finish;
        
    end
    
    // clock signal generation
    always
        #4 CLK = ~CLK;
        

endmodule


module cpu(PC, INSTRUCTION, CLK, RESET, ReadFromMem, WriteToMem, ALURESULT, OUT1, ReadData, WAIT);
    output reg [31:0] PC;
    input [31:0] INSTRUCTION;
    input CLK, RESET;

    wire [7:0] OUT2, NEGATIVE, OUT3, OUT4, OUT6, OUT5, RESULT;
    reg [2:0] DESTINATION, SOURCE1, SOURCE2, SEL;
    reg [1:0] ShiftSEL;
    reg [7:0] BRANCH;
    reg WRITE, isNEGATIVE, isIMMEDIATE, isBRANCH, isJUMP, isNotEqlBRANCH, isMem;
    wire [31:0] NEXT, EXTENDED, OFFSET, BRANCHto, PCbranch, PCNEXT;
    wire Zero, AndOut1, AndOut2, OrOut;

    output reg WriteToMem, ReadFromMem;
    input [7:0] ReadData;
    input WAIT;

    output [7:0] OUT1, ALURESULT;



    //instantiate all modules
    pc_adder a1(PC, NEXT);
    reg_file a2(RESULT, OUT1, OUT2, DESTINATION, SOURCE1, SOURCE2, WRITE & !WAIT, CLK, RESET);
    twosCompliment t1(OUT2, NEGATIVE); // convert positive value to negative value
    selectMUX m1(NEGATIVE, OUT2, OUT3, isNEGATIVE); //choose negative value for sub and positive value for add
    selectMUX m2(INSTRUCTION[7:0], OUT3, OUT4, isIMMEDIATE); //choose immediate value or register value
    alu a3(OUT1, OUT4, ALURESULT, SEL, Zero, ShiftSEL);
    and_gate a4(Zero, isBRANCH, AndOut1); //and gate to decide whether it is beq or not
    and_gate a5(~Zero, isNotEqlBRANCH, AndOut2); // and gate for decide whether it is bne or not 
    or_gate o1(AndOut1, AndOut2, OrOut); // final output of above and gates connected to the or gate and it connected as selector to selector to pc mux
    PCmux m3(BRANCHto, NEXT, PCbranch, OrOut); // and gate output used to select next PC value by using mux
    branch_adder b1(NEXT, OFFSET, BRANCHto); // shifted value give as a input to the new adder for calculate PC value
    PCmux m4(BRANCHto, PCbranch, PCNEXT, isJUMP); // this mux is used to select PC value when instruction is jump or branch
    find_offset f1(INSTRUCTION[23:16], OFFSET); //calculate offset
    selectMUX m5(ReadData, ALURESULT, RESULT,  isMem); //mux for select value to write


     always @(posedge CLK) begin
            if(WAIT == 1'b0) begin
                #1 PC = PCNEXT; //pc update
                if(RESET) PC = 32'd0; // if the reset is high PC should be zero  
            end
          
     end
     
    always@(negedge WAIT) begin
        ReadFromMem = 0;
        WriteToMem = 0;
    end

    always @(INSTRUCTION) begin
        //assign instructions to regiter file inputs
        DESTINATION = INSTRUCTION[18:16];
        SOURCE1 = INSTRUCTION[10:8];
        SOURCE2 = INSTRUCTION[2:0];
  
        //cheack opcde and decode
        #1
        case(INSTRUCTION[31:24]) 
            8'd0 : //loadi
                begin
                    WRITE = 1'b1; //set write to high because in load instruction need to write value to ragister 
                    isNEGATIVE = 1'b0; // here negative is low because no need subtract operation
                    isIMMEDIATE = 1'b1; // we need immidiate value to write register so immidiate signal set to high
                    isJUMP = 1'b0;
                    isBRANCH = 1'b0;
                    SEL = 3'b000; //alu select value is set to do forward operation
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b0;

                end
            8'd1 : //mov
                begin
                    WRITE = 1'b1; // move instruction need to move register value to another register it is also write operation  
                    isNEGATIVE = 1'b0; // here negative just set to zero. It is not useful
                    isIMMEDIATE = 1'b0; // immidiate signal aslo zero because we read register value
                    isJUMP = 1'b0;
                    isBRANCH = 1'b0;
                    SEL = 3'b000; // set alu operation to forward
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b0;
                end
            8'd2 : //add
                begin
                    WRITE = 1'b1; // in addition we need write result in to register so write set to high
                    isNEGATIVE = 1'b0; // we are doing add instruction, no need negative value so negative signal is set to low
                    isIMMEDIATE = 1'b0; // we noly need register values so immidiate signal also low
                    isJUMP = 1'b0;
                    isBRANCH = 1'b0;
                    SEL = 3'b001; // set alu operation to add operation
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b0;
                end
            8'd3 : //sub
                begin
                    WRITE = 1'b1; // in subtraction also we need write result in to register so write set to high
                    isNEGATIVE = 1'b1; // we need 2's complement value of second operand then set negative signal to high
                    isIMMEDIATE = 1'b0; // only need register values then immidiate signal is low      
                    isJUMP = 1'b0;
                    isBRANCH = 1'b0;
                    SEL = 3'b001; // set alu operation to add
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b0;
                end
            8'd4 : //and
                begin
                    WRITE = 1'b1; // after and operation we have to write result into a register
                    isNEGATIVE = 1'b0; // no need negative values
                    isIMMEDIATE = 1'b0; // o need immidiate values   
                    isJUMP = 1'b0;
                    isBRANCH = 1'b0;
                    SEL = 3'b010; // set alu to do and operation
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b0;
                end
            8'd5 : //or
                begin
                    WRITE = 1'b1; // or operaton result need to write in to register so write is set to high
                    isNEGATIVE = 1'b0; //no need negative values
                    isIMMEDIATE = 1'b0; // no need immidiate values
                    isJUMP = 1'b0;
                    isBRANCH = 1'b0;
                    SEL = 3'b011; // set alu to do or operation
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b0;
                end
            8'd6 : //j
                begin
                    WRITE = 1'b0; // no need to write into register so write is set to low
                    isNEGATIVE = 1'b0; //no need negative values
                    isIMMEDIATE = 1'b0; // no need immidiate values 
                    isJUMP = 1'b1; // in jump instruction we need to set jump signal to high
                    isBRANCH = 1'b0; // but here branch is low
                    SEL = 3'b000; //no need alu operation
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b0;
                end
            8'd7 : //beq
                begin
                    WRITE = 1'b0; // no need to write into register so write is set to low 
                    isNEGATIVE = 1'b1; // make second operand to negative value
                    isIMMEDIATE = 1'b0; // here we dont need immidiate values we only use registers
                    isJUMP = 1'b0; // jump is low
                    isBRANCH = 1'b1; // set branch signal to high
                    SEL = 3'b001; //set alu to do sub operation
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b0;
                    
                end
            8'd8 : //lwd
                begin
                    WRITE = 1'b1; // no need to write into register so write is set to low 
                    isNEGATIVE = 1'b0; // make second operand to negative value
                    isIMMEDIATE = 1'b0; // here we dont need immidiate values we only use registers
                    isJUMP = 1'b0; // jump is low
                    isBRANCH = 1'b0; // set branch signal to high
                    SEL = 3'b000; //set alu to do sub operation
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b1;
                    ReadFromMem = 1'b1;
                    WriteToMem = 1'b0;
                    
                end
            8'd9 : //lwi
                begin
                    WRITE = 1'b1; // no need to write into register so write is set to low 
                    isNEGATIVE = 1'b0; // make second operand to negative value
                    isIMMEDIATE = 1'b1; // here we dont need immidiate values we only use registers
                    isJUMP = 1'b0; // jump is low
                    isBRANCH = 1'b0; // set branch signal to high
                    SEL = 3'b000; //set alu to do sub operation
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b1;
                    ReadFromMem = 1'b1;
                    WriteToMem = 1'b0;
                    
                end
            8'd10 : //swd
                begin
                    WRITE = 1'b0; // no need to write into register so write is set to low 
                    isNEGATIVE = 1'b0; // make second operand to negative value
                    isIMMEDIATE = 1'b0; // here we dont need immidiate values we only use registers
                    isJUMP = 1'b0; // jump is low
                    isBRANCH = 1'b0; // set branch signal to high
                    SEL = 3'b000; //set alu to do sub operation
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b0;
                    ReadFromMem = 1'b0;
                    WriteToMem = 1'b1;
                    
                end
            8'd11 : //swi
                begin
                    WRITE = 1'b0; // no need to write into register so write is set to low 
                    isNEGATIVE = 1'b0; // make second operand to negative value
                    isIMMEDIATE = 1'b1; // here we dont need immidiate values we only use registers
                    isJUMP = 1'b0; // jump is low
                    isBRANCH = 1'b0; // set branch signal to high
                    SEL = 3'b000; //set alu to do sub operation
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b0;
                    ReadFromMem = 1'b0;
                    WriteToMem = 1'b1;
                    
                end
            8'd12 : //sll
                begin
                    WRITE = 1'b1; // final result should be write to given register 
                    isNEGATIVE = 1'b0; // no need negative values
                    isIMMEDIATE = 1'b1; // need immidiate value
                    isJUMP = 1'b0; // no need to jump
                    isBRANCH = 1'b0; // no need branch
                    SEL = 3'b100; //set alu to do shift operation
                    ShiftSEL = 2'b01; // logical left shift
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b0;
                    
                end
            8'd13 : //srl
                begin
                    WRITE = 1'b1; // final result should be write to given register 
                    isNEGATIVE = 1'b0; // no need negative values
                    isIMMEDIATE = 1'b1; // need immidiate value
                    isJUMP = 1'b0; // jump is low
                    isBRANCH = 1'b0; // branch signal should be low
                    SEL = 3'b100; //set alu to do shift operation
                    ShiftSEL = 2'b00; // logical right shift
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b0;
                end

            8'd14 : //sra
                begin
                    WRITE = 1'b1; // final result should be write to given register 
                    isNEGATIVE = 1'b0; // no need negative values
                    isIMMEDIATE = 1'b1; // need immidiate value
                    isJUMP = 1'b0; // jump is low
                    isBRANCH = 1'b0; //branch signal should be low
                    SEL = 3'b100; //set alu to do shift operation
                    ShiftSEL = 2'b10; // logical left shift
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b0;
                end
            8'd15 : //ror
                begin
                    WRITE = 1'b1; // final result should be write to given register  
                    isNEGATIVE = 1'b0; // no need negative values
                    isIMMEDIATE = 1'b1; // need immidiate value
                    isJUMP = 1'b0; // jump is low
                    isBRANCH = 1'b0; // branch signal should be low
                    SEL = 3'b100; //set alu to do shift operation
                    ShiftSEL = 2'b11; // rotate right
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b0;
                end
            8'd16 : //bne
                begin
                    WRITE = 1'b0; // no need to write into register so write is set to low 
                    isNEGATIVE = 1'b1; // make second operand to negative value
                    isIMMEDIATE = 1'b0; // here we dont need immidiate values we only use registers
                    isJUMP = 1'b0; // jump is low
                    isBRANCH = 1'b0; // branch signal should be low
                    SEL = 3'b001; //set alu to do sub operation
                    isNotEqlBRANCH = 1'b1; 
                    isMem = 1'b0;
                end

            8'd17 : //mul
                begin
                    WRITE = 1'b1; // write should be high because final 
                    isNEGATIVE = 1'b0; // no need negative values
                    isIMMEDIATE = 1'b0; // here we dont need immidiate values we only use registers
                    isJUMP = 1'b0; // jump is low
                    isBRANCH = 1'b0; // branch signal should be low
                    SEL = 3'b101; //set alu to do sub operation
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b0;
                end
        endcase
    
    end   

endmodule


//reg_file module
module reg_file(IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET);
    
    reg [7:0] Registers[0:7]; //8 bit wide element array with depth of 8

    //declaration of inputs and ouputs
    input [7:0] IN;
    input [2:0] OUT1ADDRESS, OUT2ADDRESS, INADDRESS;
    input CLK, RESET, WRITE;
    output [7:0] OUT1, OUT2;

    integer i;

    //read asynchronously and load values into OUT1 and OUT2 
    assign #2 OUT1 = Registers[OUT1ADDRESS];
    assign #2 OUT2 = Registers[OUT2ADDRESS];

    //always block only teriggerd in the positive edge of clock
    always@(posedge CLK)
    begin
        if(WRITE & !RESET) begin// if write port is set to high and reset is low then IN port is write into register named as INADDRESS
           #1 Registers[INADDRESS] <= IN;
           
        end
        if(RESET) begin // if reset is high, all values in registers are written as 0
            #1
            for(i = 0; i < 8; i = i + 1) begin // make for loop for access each element in an array
                 Registers[i] <= 8'd0;
            end
        end

    end
     initial begin
     #4000 $display($time, "%d %d %d %d %d %d %d %d\n", Registers[0], Registers[1], Registers[2], Registers[3], Registers[4], Registers[5], Registers[6], Registers[7]);
     end
endmodule



module alu(DATA1, DATA2, RESULT, SELECT, ZERO, ShiftOp);

  //declaration for inputs and output
  input [7:0] DATA1;
  input signed[7:0] DATA2;
  input [2:0] SELECT;
  input [1:0] ShiftOp;
  output reg signed[7:0] RESULT;
  output reg ZERO;

  //declaration of wires that hold vlues of each operation
  wire [7:0]a, b, c, d, e, f;

  //instantiate modules
  FORWARD ins1(DATA2, a);
  ADD ins2(DATA1, DATA2, b);
  AND ins5(DATA1, DATA2, c);
  OR ins6(DATA1, DATA2, d);
  SHIFT ins7(DATA1, DATA2, e, ShiftOp);
  MULTIPLY ins8(DATA1, DATA2, f);

  always@(b) begin
    if(b == 0) ZERO = 1; //if subtraction module output equals to zero then set ZERO signal to high
  else ZERO = 0; // else set it to low
  end

    always@(DATA1, DATA2, SELECT)
    begin
      case(SELECT)
      //doing corresponding operation for each select value 
      3'b000 : assign RESULT = a; // RESULT = DATA2
      3'b001 : assign RESULT = b; //RESULT = DATA1 + DATA2
      3'b010 : assign RESULT = c; //RESULT = DATA1 & DATA2
      3'b011 : assign RESULT = d; //RESULT = DATA1 | DATA2
      3'b100 : assign RESULT = e; // RESULT = SHIFTED RESULT
      3'b101 : assign RESULT = f; // RESULT = DATA1 x DATA2
      default: assign RESULT = 8'd0; //in default RESULT is 0
      endcase

    end
    
endmodule

//FORWARD module
module FORWARD(DATA, OUT);

  //declare input and output
  input [7:0] DATA;
  output [7:0] OUT;

  assign #1 OUT = DATA;
endmodule


//module for addition
 module ADD(DATA1, DATA2, OUT);
//declare inputs and output
   input [7:0] DATA1;
   input [7:0] DATA2;
   output [7:0] OUT;

  assign #2 OUT = DATA1 + DATA2;

 endmodule


//module for and operation
module AND(DATA1, DATA2, OUT);
//declare inputs and output
   input [7:0] DATA1;
   input [7:0] DATA2;
   output [7:0] OUT;

  assign #1 OUT = DATA1 & DATA2;

 endmodule

//module for or operation
module OR(DATA1, DATA2, OUT);
  //declare inputs and output
   input [7:0] DATA1;
   input [7:0] DATA2;
   output [7:0] OUT;

  assign #1 OUT = DATA1 | DATA2;

 endmodule

 //module for 2's compliment
module twosCompliment(DATA, OUT);
   //declare input and output
   input [7:0] DATA;
   output [7:0] OUT;

   assign #1 OUT = ~DATA + 1;
endmodule

module SHIFT(DATA, ShiftValue, OUT, shiftOP);
    input [7:0]DATA, ShiftValue;
    output reg[7:0] OUT;
    input [1:0] shiftOP;

    always@(*) begin
        #1
        case(shiftOP)
        2'b00 : begin //logical right shift
            case(ShiftValue)
            8'd0 : OUT = DATA;
            8'd1 : OUT = {{1{1'b0}}, DATA[7:1]};
            8'd2 : OUT = {{2{1'b0}}, DATA[7:2]};
            8'd3 : OUT = {{3{1'b0}}, DATA[7:3]};
            8'd4 : OUT = {{4{1'b0}}, DATA[7:4]};
            8'd5 : OUT = {{5{1'b0}}, DATA[7:5]};
            8'd6 : OUT = {{6{1'b0}}, DATA[7:6]};
            8'd7 : OUT = {{7{1'b0}}, DATA[7:7]};
            default : OUT = {7{1'b0}};  
            endcase
            end

        2'b01 : begin //logical left shift
            case(ShiftValue)
            8'd0 : OUT = DATA;
            8'd1 : OUT = {DATA[6:0], {1{1'b0}}};
            8'd2 : OUT = {DATA[5:0], {2{1'b0}}};
            8'd3 : OUT = {DATA[4:0], {3{1'b0}}};
            8'd4 : OUT = {DATA[3:0], {4{1'b0}}};
            8'd5 : OUT = {DATA[2:0], {5{1'b0}}};
            8'd6 : OUT = {DATA[1:0], {6{1'b0}}};
            8'd7 : OUT = {DATA[0:0], {7{1'b0}}};
            default : OUT = {7{1'b0}};  
            endcase
            end

        2'b10 : begin //arithmetic right shift
            case(ShiftValue)
            8'd0 : OUT = DATA;
            8'd1 : OUT = {{1{DATA[7]}}, DATA[7:1]};
            8'd2 : OUT = {{2{DATA[7]}}, DATA[7:2]};
            8'd3 : OUT = {{3{DATA[7]}}, DATA[7:3]};
            8'd4 : OUT = {{4{DATA[7]}}, DATA[7:4]};
            8'd5 : OUT = {{5{DATA[7]}}, DATA[7:5]};
            8'd6 : OUT = {{6{DATA[7]}}, DATA[7:6]};
            8'd7 : OUT = {{7{DATA[7]}}, DATA[7:7]};
            default : OUT = {7{DATA[7]}}; 
            endcase
            end
        2'b11 : begin //rotate shift
            case(ShiftValue%8)
            8'd0 : OUT = DATA;
            8'd1 : OUT = {DATA[0], DATA[7:1]};
            8'd2 : OUT = {DATA[1:0], DATA[7:2]};
            8'd3 : OUT = {DATA[2:0], DATA[7:3]};
            8'd4 : OUT = {DATA[3:0], DATA[7:4]};
            8'd5 : OUT = {DATA[4:0], DATA[7:5]};
            8'd6 : OUT = {DATA[5:0], DATA[7:6]};
            8'd7 : OUT = {DATA[6:0], DATA[7:7]};
            endcase
            end
        endcase
    end
endmodule

module MULTIPLY(DATA1, DATA2, OUT);
    input [7:0] DATA1, DATA2;
    output [7:0] OUT;
    wire [7:0] p0, p1, p2, p3, p4, p5, p6, p7;

       assign p0 = (DATA2[0] == 1) ? {DATA1} : 8'd0;
       assign p1 = (DATA2[1] == 1) ? {DATA1, 1'd0} : 8'd0;
       assign p2 = (DATA2[2] == 1) ? {DATA1, 2'd0} : 8'd0;
       assign p3 = (DATA2[3] == 1) ? {DATA1, 3'd0} : 8'd0;
       assign p4 = (DATA2[4] == 1) ? {DATA1, 4'd0} : 8'd0;
       assign p5 = (DATA2[5] == 1) ? {DATA1, 5'd0} : 8'd0;
       assign p6 = (DATA2[6] == 1) ? {DATA1, 6'd0} : 8'd0;
       assign p7 = (DATA2[7] == 1) ? {DATA1, 7'd0} : 8'd0;

       assign #1 OUT = p0 + p1 + p2 + p3 + p4 + p5 + p6 + p7;

endmodule

//and gate module
module and_gate(IN1, IN2, OUT);
    input IN1, IN2;
    output OUT;

    and(OUT, IN1, IN2);

endmodule

//or gate module
module or_gate(IN1, IN2, OUT);
    input IN1, IN2;
    output OUT;

    or(OUT, IN1, IN2);

endmodule

//just test
module branch_adder(pcNext, in, out);
    input [31:0] pcNext, in;
    output [31:0] out;

    assign #2 out = pcNext + in;

endmodule

//module for find next pc value
module pc_adder(IN, OUT);
    input [31:0] IN; // input address/ current instruction
    output [31:0] OUT; // output address/ next instruction

    assign #1 OUT = IN + 32'd4;

endmodule

module find_offset(IN, OUT);
    input [7:0] IN;
    output [31:0] OUT;

    assign OUT = {{24{IN[7]}}, IN[5:0], {2{1'b0}}}; // sign extended and shifted
endmodule


//MUX
module selectMUX(IN1, IN2, OUT, SELECT);
    input [7:0] IN1, IN2;
    input SELECT;
    output reg [7:0] OUT;

    always@(*) begin
        case(SELECT)
            1'b1: OUT <= IN1;
            1'b0: OUT <= IN2;
        endcase
    end
endmodule

//mux module for select right pc value
module PCmux(IN1, IN2, OUT, SELECT);
    input [31:0] IN1, IN2;
    input SELECT;
    output reg [31:0] OUT;

    always@(*) begin
        case(SELECT)
            1'b1: OUT <= IN1;
            1'b0: OUT <= IN2;
        endcase
    end
endmodule


module dcache (clock,reset,read,write,address,writedata,readdata,busywait,mem_read,mem_write,mem_address,mem_writedata,mem_readdata,mem_busywait);

    input  clock, reset,write,read,mem_busywait;
    input [7:0] writedata,address;
    input [31:0] mem_readdata;

    output reg mem_write,mem_read,busywait;
    output reg [5:0] mem_address;
    output reg [7:0] readdata;
    output reg [31:0] mem_writedata;
   
   reg readaccess,writeaccess,dirty,valid;
   reg [1:0] offset;
   reg [2:0] index,tag;
   reg [36:0] cache_memory [7:0];
   reg [31:0] block;
   wire hit, eql;
   reg [2:0] address_tag;
   reg cache_update;
  
   //array for store data
   reg [31:0] data_block_array [7:0];

   //array for store tag values
   reg [2:0] tag_array [7:0];

   //array for store valid bit
   reg validbit_array [7:0];

   //array for store dirty bit values
   reg dirtybit_array[7:0];

//Cache access signals are generated based on the read and write signals given to the cache
    always @(write, read) begin
        busywait = (write || read)? 1 : 0;
        readaccess = (read && !write)? 1 : 0;
        writeaccess = (!read && write)? 1 : 0;
    end



    always @(*) begin
        if(readaccess || writeaccess) begin
            #1
            address_tag = address[7:5];
            index = address[4:2];
            offset = address[1:0];

            block = data_block_array[index];
            tag = tag_array[index];
            dirty = dirtybit_array[index];
            valid = validbit_array[index];
        end
    end


    comparator c1(tag, address_tag, eql); //comparator
    and_gate ag(eql, valid, hit); //check comparator output and valid bit

    //data word is selected based on the offset from the block 
    always@(block,offset) 
    begin

    #1
        case(offset)
                2'b00 : readdata = block[7:0]; 
                2'b01 : readdata = block[15:8];
                2'b10 : readdata = block[23:16];
                2'b11 : readdata = block[31:24];

        endcase 
    end



    always @(posedge clock)begin 
        
    //if hit is successful therefore we can give directly answer to cpu without futher stalling 

    if(readaccess && hit)begin 

        readaccess = 1'b0;
        busywait= 1'b0;

    end

    else if(writeaccess && hit)begin 

        //deassert busywait signal
        busywait = 1'b0;
                
        #1
        case(offset)
                
            //Dataword is written to the cache based on the offset
            2'b00	:	data_block_array[index][7:0]   = writedata;
            2'b01 	:	data_block_array[index][15:8]  = writedata;
            2'b10 	:	data_block_array[index][23:16] = writedata;
            2'b11   :	data_block_array[index][31:24] = writedata;
            
        endcase

        //set dirtybit = 1
        dirtybit_array[index] = 1'b1;

        //set validbit = 1  
        validbit_array[index] = 1'b1;

        //to prevent accessing memory for writing        	
        writeaccess = 1'b0;
                
            
        end
    end

    always @ (cache_update) begin
    	#1
        if(cache_update) begin

            data_block_array[index] = mem_readdata;	//write a data block to the cache
            tag_array[index] = address_tag ;	//update tag
            dirtybit_array[index] = 1'd0;	//dirty bit=0 since we are writing a data in cache which is also in memory
            validbit_array[index] = 1'd1;	//valid bit
            
        end
    end


    parameter IDLE = 3'b000, MEM_READ = 3'b001, MEM_WRITE=3'b010, CACHE_UPDATE=3'b011;
    reg [2:0] state, next_state;

    // combinational next state logic
    always @(*)
    begin
        case (state)
            IDLE:

                //if the existing block is not dirty ,the missing block should be fetched from memory
                if ((read || write) && !dirty && !hit)               
                    next_state = MEM_READ;        //memory read

                //if the existing block is dirty ,that block must be written back to the memory before fetching the missing block
                else if ((read ||write) && dirty && !hit)            
                    next_state = MEM_WRITE;       //write-back

                
                else
                    next_state = IDLE;
            
            

            MEM_READ:
                if (!mem_busywait)
                    next_state = CACHE_UPDATE;
                else    
                    next_state = MEM_READ;


           MEM_WRITE:
             if (!mem_busywait)
                    next_state = MEM_READ;	//after memory writing,start the memory reading
                else    
                    next_state = MEM_WRITE;	//write back to the memory

            //next state after writing in cache from data received from memory is IDLE		
	        CACHE_UPDATE:
              
                next_state = IDLE;
               
        endcase
    end

    // combinational output logic
    always @(*)
    begin
        case(state)

        //contol signals of cache control during read and write is 0
            IDLE:
            begin
                mem_read      = 0;
                mem_write     = 0;
                mem_address   = 6'dx;
                mem_writedata = 32'dx;
                cache_update = 0;    
            end
         

         //during a read miss we assert the mem_read to read from data memory
            MEM_READ: 
            begin
                mem_read      = 1;
                mem_write     = 0;
                mem_address   = {tag,index};
                mem_writedata = 32'dx;
                busywait      = 1;
                cache_update = 0; 
               
            end


           	MEM_WRITE: 
            begin
                mem_read  = 1'd0;
                mem_write = 1'd1;
                mem_address ={tag,index};	//data block address from the cache
                mem_writedata =block;
                busywait=1;
                cache_update = 0;
            end
			
			CACHE_UPDATE: 
            begin
                mem_read = 1'd0;
                mem_write = 1'd0;
                mem_address = 6'dx;
                mem_writedata = 32'dx;
                busywait= 1'd1;
                cache_update = 1;
			
            end
                   
        endcase
    end

    integer j;
    // sequential logic for state transitioning 
    always @(posedge clock, reset)
    
    begin
        if(reset)
        begin
            state = IDLE;
            busywait=1'b0;

            //reset the cache after 1 time unit delay
             #1
             for( j=0;j<8;j=j+1)
            begin
                data_block_array[j] = 32'd0;
                tag_array[j] = 3'b0;
                dirtybit_array[j] = 1'b0;
                validbit_array[j] = 1'b0;
            end
        end
        
        else begin
            state = next_state;
         end
    end


endmodule



//comparator
module comparator(IN1, IN2, OUT);

    input [2:0] IN1, IN2;
    output OUT;
    wire w0, w1, w2;

    xnor(w0, IN1[0], IN2[0]);
    xnor(w1, IN1[1], IN2[1]);
    xnor(w2, IN1[2], IN2[2]);

    assign #0.9 OUT = w0 & w1 & w2;  // tag comparison with latency #0.9 units

endmodule


//main memory
module data_memory(
	clock,
    reset,
    read,
    write,
    address,
    writedata,
    readdata,
	busywait
);
input				clock;
input           	reset;
input           	read;
input           	write;
input[5:0]      	address;
input[31:0]     	writedata;
output reg [31:0]	readdata;
output reg      	busywait;

//Declare memory array 256x8-bits 
reg [7:0] memory_array [255:0];

//Detecting an incoming memory access
reg readaccess, writeaccess;
always @(read, write)
begin
	busywait = (read || write)? 1 : 0;
	readaccess = (read && !write)? 1 : 0;
	writeaccess = (!read && write)? 1 : 0;
end

//Reading & writing
always @(posedge clock)
begin
	if(readaccess)
	begin
		readdata[7:0]   = #40 memory_array[{address,2'b00}];
		readdata[15:8]  = #40 memory_array[{address,2'b01}];
		readdata[23:16] = #40 memory_array[{address,2'b10}];
		readdata[31:24] = #40 memory_array[{address,2'b11}];
		busywait = 0;
		readaccess = 0;
	end
	if(writeaccess)
	begin
		memory_array[{address,2'b00}] = #40 writedata[7:0];
		memory_array[{address,2'b01}] = #40 writedata[15:8];
		memory_array[{address,2'b10}] = #40 writedata[23:16];
		memory_array[{address,2'b11}] = #40 writedata[31:24];
		busywait = 0;
		writeaccess = 0;
	end
end

integer i;
//Reset memory
always @(posedge reset)
begin
    if (reset)
    begin
        for (i=0;i<256; i=i+1)
            memory_array[i] = 0;
        
        busywait = 0;
		readaccess = 0;
		writeaccess = 0;
    end
end
//initial begin
    //#700
    //for(i=0;i<35; i=i+1) 
        //$display($time,"%d %d", i, memory_array[i]);
//end
endmodule

/*
Module	: 256x8-bit instruction memory (16-Byte blocks)
Author	: Isuru Nawinne
Date		: 10/06/2020

Description	:

This file presents a primitive instruction memory module for CO224 Lab 6 - Part 3
This memory allows instructions to be read as 16-Byte blocks
*/

module instruction_memory(
	clock,
	read,
    address,
    readinst,
	busywait
);
input				clock;
input				read;
input[5:0]			address;
output reg [127:0]	readinst;
output	reg			busywait;

reg readaccess;

//Declare memory array 1024x8-bits 
reg [7:0] memory_array [1023:0];

//Initialize instruction memory
initial
begin
	busywait = 0;
	readaccess = 0;
	
    // Sample program given below. You may hardcode your software program here, or load it from a file:
    // {memory_array[10'd3],  memory_array[10'd2],  memory_array[10'd1],  memory_array[10'd0]}  = 32'b00000000000001000000000000011001; // loadi 4 #25
    // {memory_array[10'd7],  memory_array[10'd6],  memory_array[10'd5],  memory_array[10'd4]}  = 32'b00000000000001010000000000100011; // loadi 5 #35
    // {memory_array[10'd11], memory_array[10'd10], memory_array[10'd9],  memory_array[10'd8]}  = 32'b00000010000001100000010000000101; // add 6 4 5
    // {memory_array[10'd15], memory_array[10'd14], memory_array[10'd13], memory_array[10'd12]} = 32'b00000000000000010000000001011010; // loadi 1 90
    // {memory_array[10'd19], memory_array[10'd18], memory_array[10'd17], memory_array[10'd16]} = 32'b00000011000000010000000100000100; // sub 1 1 4

    // loading instr_mem content from instr_mem.mem file
    $readmemb("instr_mem.mem", memory_array);

end

//Detecting an incoming memory access
always @(read)
begin
    busywait = (read)? 1 : 0;
    readaccess = (read)? 1 : 0;
end

//Reading
always @(posedge clock)
begin
	if(readaccess)
	begin
		readinst[7:0]     = #40 memory_array[{address,4'b0000}];
		readinst[15:8]    = #40 memory_array[{address,4'b0001}];
		readinst[23:16]   = #40 memory_array[{address,4'b0010}];
		readinst[31:24]   = #40 memory_array[{address,4'b0011}];
		readinst[39:32]   = #40 memory_array[{address,4'b0100}];
		readinst[47:40]   = #40 memory_array[{address,4'b0101}];
		readinst[55:48]   = #40 memory_array[{address,4'b0110}];
		readinst[63:56]   = #40 memory_array[{address,4'b0111}];
		readinst[71:64]   = #40 memory_array[{address,4'b1000}];
		readinst[79:72]   = #40 memory_array[{address,4'b1001}];
		readinst[87:80]   = #40 memory_array[{address,4'b1010}];
		readinst[95:88]   = #40 memory_array[{address,4'b1011}];
		readinst[103:96]  = #40 memory_array[{address,4'b1100}];
		readinst[111:104] = #40 memory_array[{address,4'b1101}];
		readinst[119:112] = #40 memory_array[{address,4'b1110}];
		readinst[127:120] = #40 memory_array[{address,4'b1111}];
		busywait = 0;
		readaccess = 0;
	end
end
 
endmodule

/*
--------------------INSTRUCTION CACHE---------------------
*/
module instruction_cache(clock, reset, pc_address, busywait, instrMem_read, instrMem_address, instrMem_readdata, instrMem_busywait, instruction);
    // declare the ports
    input clock, reset, instrMem_busywait;
    input [7 : 0] pc_address;     
    input [127 : 0] instrMem_readdata;

    output reg [5 : 0] instrMem_address;
    output reg instrMem_read, busywait;
    output [31 : 0] instruction;

    reg [127 : 0] instr_cache [0 : 7];  // 128 byte instruction cashe memory

    reg [127 : 0] cache_data;       // store the cache data according to the address
    reg [2 : 0] tag_array [0 : 7];  // store the tag for each data
    reg valid_array [0 : 7];        // store the validity bit for each data

    wire hit, tagMatch;
    reg valid, cache_update;
    reg [2 : 0] tag; 
    wire [2 : 0] index;

    /*
    .....combinational part.....
    */

    // check the pc_address is positive or negative
    always @(pc_address) 
    begin
        busywait = 1;  // stall pc to read data from instruction cache
        
    end

    // get the index of the intruction with latency of #1 time unit
    assign #1 index = pc_address[4 : 2];

    always @(*) 
    begin
        #1;
        cache_data = instr_cache[index];  // get the data from instruction cache
        tag = tag_array[index];           // get the tag from tag array
        valid = valid_array[index];       // get the valid bit from valid array
    end
    
    comparator compareTag(tag, pc_address[7 : 5], tagMatch);  // compare the tags using comparator module(the latency for the tag comparison was included in this module)
    and_gate gate1(tagMatch, valid, hit);                      // hit deciding using AND gate

    // read the requested instruction, according to the offset
    assign #1 instruction = ((pc_address[1 : 0] == 2'b00) && hit)? cache_data[31:0]:
                            ((pc_address[1 : 0] == 2'b01) && hit)? cache_data[63:32]:
                            ((pc_address[1 : 0] == 2'b10) && hit)? cache_data[95:64]:cache_data[127:96];

    //if hit is true, de assert the busywait on posedge clock
    always @(posedge clock)
    begin
        #1;
        if(hit)
            busywait = 0;
    end

    /* Instruction Cache Controller FSM Start */

    //update inst_cache with the newly fetched instruction block with a #1 time delay 
    always @(cache_update) 
    begin
        #1
        if(cache_update)
        begin
        instr_cache[index] = instrMem_readdata;
        tag_array[index] = pc_address[7 : 5];
        valid_array[index] = 1;
        end
    end


    parameter IDLE = 2'b00, MEM_READ = 2'b01, UPDATE_CACHE = 2'b10;
    reg [2:0] state, next_state;

    //combinational next state logic
    always @(*)
    begin
    case(state)
        IDLE:
            if(!hit)
                next_state = MEM_READ;          // go to mem read state 
            else
                next_state = IDLE;

        MEM_READ:
            if(!instrMem_busywait)
                next_state = UPDATE_CACHE;      // when busywait for fetching instruction block from memory de-asserts go to update cache
            else
                next_state = MEM_READ;

        UPDATE_CACHE:
            next_state = IDLE;
    endcase
    end

    //combinational output logic
    always @(state)
    begin
    case(state)
        IDLE:
        begin
        instrMem_read = 0;
        instrMem_address = 6'bx; 
        cache_update = 0; 
        end

        MEM_READ:
        begin                             // access instruction memory address by sending read signal and address
        instrMem_read = 1;        
        instrMem_address = {pc_address[7 : 2]};
        cache_update = 0; 
        end

        UPDATE_CACHE:
        begin
        instrMem_read = 0;
        instrMem_address = 6'bx;
        cache_update = 1; 
        end

    endcase
    end

    integer i;
    // sequential logic for state transitioning 
    always @(posedge clock, reset)
    begin
        if(reset)
        begin
            state = IDLE;
            busywait = 1'b0;

            //reset the cache after 1 time unit delay
            #1
            for(i = 0; i < 8; i = i + 1)
            begin
                instr_cache[i] = 128'd0;
                tag_array[i] = 3'b0;
                valid_array[i] = 1'b0;
            end
        end
        
        else 
        begin
            state = next_state;
        end
    end

    /* Instruction Cache FSM End*/


endmodule
