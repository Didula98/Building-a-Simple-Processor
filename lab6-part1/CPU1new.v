module cpu_tb;

    reg CLK, RESET;
    wire [31:0] PC;
    reg [31:0] INSTRUCTION;

    wire Write, Read;
    wire [7:0] READDATA, ADDRESS, WRITEDATA;
    wire BUSYWAIT;

    /* 
    ------------------------
     SIMPLE INSTRUCTION MEM
    ------------------------
    */
    
    // TODO: Initialize an array of registers (8x1024) named 'instr_mem' to be used as instruction memory
    reg [7:0] instr_mem[0:1023];

    // TODO: Create combinational logic to support CPU instruction fetching, given the Program Counter(PC) value 
    //       (make sure you include the delay for instruction fetching here)

    always @(PC) begin
        #2
        INSTRUCTION = {instr_mem[PC+3], instr_mem[PC+2], instr_mem[PC+1], instr_mem[PC]};
    end
    
    initial
    begin
        // Initialize instruction memory with the set of instructions you need execute on CPU
        
        // METHOD 1: manually loading instructions to instr_mem
        // {instr_mem[10'd3], instr_mem[10'd2], instr_mem[10'd1], instr_mem[10'd0]} = 32'b00000000000001000000000000000101;
        // {instr_mem[10'd7], instr_mem[10'd6], instr_mem[10'd5], instr_mem[10'd4]} = 32'b00000000000000100000000000001001;
        // {instr_mem[10'd11], instr_mem[10'd10], instr_mem[10'd9], instr_mem[10'd8]} = 32'b00000010000001100000010000000010;
        
        // METHOD 2: loading instr_mem content from instr_mem.mem file
        $readmemb("instr_mem.mem", instr_mem);
    end
    
    /* 
    -----
     CPU
    -----
    */
    cpu mycpu(PC, INSTRUCTION, CLK, RESET, Read, Write, ADDRESS, WRITEDATA, READDATA, BUSYWAIT);

    data_memory d1(CLK, RESET, Read, Write, ADDRESS, WRITEDATA, READDATA, BUSYWAIT);

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
        #250
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
                    WRITE = 1'b1; // need to write into register 
                    isNEGATIVE = 1'b0; // no need negative values
                    isIMMEDIATE = 1'b0; // here we dont need immidiate values we only use registers
                    isJUMP = 1'b0; // jump is low
                    isBRANCH = 1'b0; //branch is low
                    SEL = 3'b000; //set alu to forward operation
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b1; //select data from memory
                    ReadFromMem = 1'b1;
                    WriteToMem = 1'b0;
                    
                end
            8'd9 : //lwi
                begin
                    WRITE = 1'b1; // need to write into register
                    isNEGATIVE = 1'b0; // no need negative values
                    isIMMEDIATE = 1'b1; // here we dont need immidiate values we only use registers
                    isJUMP = 1'b0; // jump is low
                    isBRANCH = 1'b0; // branch is low
                    SEL = 3'b000; //set alu to forward operation
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b1; //select data from memory
                    ReadFromMem = 1'b1;
                    WriteToMem = 1'b0;
                    
                end
            8'd10 : //swd
                begin
                    WRITE = 1'b0; // no need to write into register so write is set to low 
                    isNEGATIVE = 1'b0; // no need negative values
                    isIMMEDIATE = 1'b0; // here we dont need immidiate values we only use registers
                    isJUMP = 1'b0; // jump is low
                    isBRANCH = 1'b0; // branch is low
                    SEL = 3'b000; //set alu to forward operation
                    isNotEqlBRANCH = 1'b0;
                    isMem = 1'b0;
                    ReadFromMem = 1'b0;
                    WriteToMem = 1'b1;
                    
                end
            8'd11 : //swi
                begin
                    WRITE = 1'b0; // no need to write into register so write is set to low 
                    isNEGATIVE = 1'b0; // no need negative values
                    isIMMEDIATE = 1'b1; // here we dont need immidiate values we only use registers
                    isJUMP = 1'b0; // jump is low
                    isBRANCH = 1'b0; // branch is low
                    SEL = 3'b000; //set alu to forward operation
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
     #250 $display($time, "%d %d %d %d %d %d %d %d\n", Registers[0], Registers[1], Registers[2], Registers[3], Registers[4], Registers[5], Registers[6], Registers[7]);
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
module branch_adder(pcNext, offset, out);
    input [31:0] pcNext, offset;
    output [31:0] out;

    assign #2 out = pcNext + offset;

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



//memory module
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
input           clock;
input           reset;
input           read;
input           write;
input[7:0]      address;
input[7:0]      writedata;
output reg [7:0]readdata;
output reg      busywait;

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
        readdata = #40 memory_array[address];
        busywait = 0;
		readaccess = 0;
    end
    if(writeaccess)
	begin
        memory_array[address] = #40 writedata;
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
initial begin
    #250
    for(i=0;i<20; i=i+1) 
        $display($time,"%d %d", i, memory_array[i]);
end
endmodule