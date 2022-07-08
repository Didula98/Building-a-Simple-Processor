/*
 * LAB 05
 * PART IV : FLOW CONTROL INSTRUCTIONS
 * GROUP 30
 */

module cpu_tb;

    reg CLK, RESET;
    wire [31:0] PC;
    reg [31:0] INSTRUCTION;
    
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
    cpu mycpu(PC, INSTRUCTION, CLK, RESET);

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
        #100
        $finish;
        
    end
    
    // clock signal generation
    always
        #4 CLK = ~CLK;
        

endmodule


module cpu(PC, INSTRUCTION, CLK, RESET);
    output reg [31:0] PC;
    input [31:0] INSTRUCTION;
    input CLK, RESET;

    wire [7:0] OUT1, OUT2, NEGATIVE, OUT3, OUT4, RESULT;
    reg [2:0] DESTINATION, SOURCE1, SOURCE2, SEL;
    reg [7:0] BRANCH;
    reg WRITE, isNEGATIVE, isIMMEDIATE, isBRANCH, isJUMP;
    wire [31:0] NEXT, EXTENDED, SHIFTED, BRANCHto, PCbranch, PCNEXT;
    wire Zero, AndOut;



    //instantiate all modules
    pc_adder a1(PC, NEXT);
    reg_file a2(RESULT, OUT1, OUT2, DESTINATION, SOURCE1, SOURCE2, WRITE, CLK, RESET);
    twosCompliment t1(OUT2, NEGATIVE); // convert positive value to negative value
    selectMUX m1(NEGATIVE, OUT2, OUT3, isNEGATIVE); //choose negative value for sub and positive value for add
    selectMUX m2(INSTRUCTION[7:0], OUT3, OUT4, isIMMEDIATE); //choose immediate value or register value 
    alu a3(OUT1, OUT4, RESULT, SEL, Zero);
    and_gate a4(Zero, isBRANCH, AndOut); //and gate to decide wether it is branch or not
    PCmux m3(BRANCHto, NEXT, PCbranch, AndOut); // and gate output used to select next PC value by using mux
    sign_exteded s1(INSTRUCTION[23:16], EXTENDED); // before find the offset we need to sign extend given value to 32 bit
    shift s2(EXTENDED, SHIFTED); // after extended it should be mutiply by 4. To do that we shift it by 2 
    branch_adder b1(NEXT, SHIFTED, BRANCHto); // shifted value give as a input to the new adder for calculate PC value
    PCmux m4(BRANCHto, PCbranch, PCNEXT, isJUMP); // this mux is used to select PC value when instruction is jump or branch

     always @(posedge CLK) begin
         #1 PC = PCNEXT; //pc update
         if(RESET) PC = 32'd0; // if the reset is high PC should be zero  
     end


    always @(INSTRUCTION) begin
        //assign instructions to regiter file inputs
        DESTINATION = INSTRUCTION[18:16];
        SOURCE1 = INSTRUCTION[10:8];
        SOURCE2 = INSTRUCTION[2:0];
  
        //cheack opcde and decode
        #1
        case(INSTRUCTION[31:24]) 
            8'd0 : 
                begin
                    WRITE = 1; //set write to high because in load instruction need to write value to ragister 
                    isNEGATIVE = 1'b0; // here negative is low because no need subtract operation
                    isIMMEDIATE = 1'b1; // we need immidiate value to write register so immidiate signal set to high
                    isJUMP = 1'b0;
                    isBRANCH = 1'b0;
                    SEL = 3'b000; //alu select value is set to do forward operation

                end
            8'd1 : 
                begin
                    WRITE = 1; // move instruction need to move register value to another register it is also write operation  
                    isNEGATIVE = 1'b0; // here negative just set to zero. It is not useful
                    isIMMEDIATE = 1'b0; // immidiate signal aslo zero because we read register value
                    isJUMP = 1'b0;
                    isBRANCH = 1'b0;
                    SEL = 3'b000; // set alu operation to forward
                    
                end
            8'd2 : 
                begin
                    WRITE = 1; // in addition we need write result in to register so write set to high
                    isNEGATIVE = 1'b0; // we are doing add instruction, no need negative value so negative signal is set to low
                    isIMMEDIATE = 1'b0; // we noly need register values so immidiate signal also low
                    isJUMP = 1'b0;
                    isBRANCH = 1'b0;
                    SEL = 3'b001; // set alu operation to add operation
                    
                end
            8'd3 : 
                begin
                    WRITE = 1; // in subtraction also we need write result in to register so write set to high
                    isNEGATIVE = 1'b1; // we need 2's complement value of second operand then set negative signal to high
                    isIMMEDIATE = 1'b0; // only need register values then immidiate signal is low      
                    isJUMP = 1'b0;
                    isBRANCH = 1'b0;
                    SEL = 3'b001; // set alu operation to add
                    
                end
            8'd4 : 
                begin
                    WRITE = 1; // after and operation we have to write result into a register
                    isNEGATIVE = 1'b0; // no need negative values
                    isIMMEDIATE = 1'b0; // o need immidiate values   
                    isJUMP = 1'b0;
                    isBRANCH = 1'b0;
                    SEL = 3'b010; // set alu to do and operation
                    
                end
            8'd5 : 
                begin
                    WRITE = 1; // or operaton result need to write in to register so write is set to high
                    isNEGATIVE = 1'b0; //ne need negative values
                    isIMMEDIATE = 1'b0; // no need immidiate values
                    isJUMP = 1'b0;
                    isBRANCH = 1'b0;
                    SEL = 3'b011; // set alu to do or operation
                    
                end
            8'd6 :
                begin
                    WRITE = 0; // no need to write into register so write is set to low
                    isNEGATIVE = 1'b0; //ne need negative values
                    isIMMEDIATE = 1'b0; // no need immidiate values 
                    isJUMP = 1'b1; // in jump instruction we need to set jump signal to high
                    isBRANCH = 1'b0; // but here branch is low
                    SEL = 3'b000; //no need alu operation
                end
            8'd7 :
                begin
                    WRITE = 0; // no need to write into register so write is set to low 
                    isNEGATIVE = 1'b1; // make second operand to negative value
                    isIMMEDIATE = 1'b0; // here we dont need immidiate values we only use registers
                    isJUMP = 1'b0; // jump is low
                    isBRANCH = 1'b1; // set branch signal to high
                    SEL = 3'b001; //set alu to do sub operation
                    
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
            for(i = 0; i < 8; i += 1) begin // make for loop for access each element in an array
                 Registers[i] <= 8'd0;
            end
        end

    end

endmodule



module alu(DATA1, DATA2, RESULT, SELECT, ZERO);

  //declaration for inputs and output
  input [7:0] DATA1;
  input signed[7:0] DATA2;
  input [2:0] SELECT;
  output reg signed[7:0] RESULT;
  output reg ZERO;

  //declaration of wires that hold vlues of each operation
  wire [7:0]a, b, c, d;

  //instantiate modules
  FORWARD ins1(DATA2, a);
  ADD ins2(DATA1, DATA2, b);
  AND ins5(DATA1, DATA2, c);
  OR ins6(DATA1, DATA2, d);

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


//and gate module
module and_gate(IN1, IN2, OUT);
    input IN1, IN2;
    output OUT;

    and(OUT, IN1, IN2);

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

//module for sign extend
module sign_exteded(IN, OUT);
    input [7:0] IN; //8 bit input
    output [31:0] OUT; //32 bit output

    assign OUT[7:0] = IN[7:0];
    assign OUT[31:8] = {24{IN[7]}};
    initial begin
    end
endmodule

//module for shifting
module shift(IN, OUT);
    input signed[31:0] IN;
    output signed[31:0] OUT;

    assign OUT = IN << 2; // multiply by 4

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