/*
 * LAB 05: BUILDING A SIMPLE PROCESSOR
 * PART I: ALU 
 * GROUP 30
 */

// creating the test bench for alu module
module alu_tb;
    // inputs
    reg [7 : 0] DATA1 = 5, DATA2 = 10;
    reg [2 : 0] SEL;
    // outputs
    wire [7 : 0] RESULT;
   
    // instantiate the alu module
    ALU alu(DATA1, DATA2, RESULT, SEL);

    // initial block
    initial 
    begin

        // print the results
        #5 SEL = 3'b000; // FORWARD
        $monitor ($time ," DATA1 = %d  DATA2 = %d  RESULT = %d", DATA1, DATA2, RESULT); 

        #5 SEL = 3'b001; // ADD
        // change the values in DATA1 and DATA2
        #5;
        SEL = 3'b100; // default, RESULT = 0
        DATA1 = 20;
        DATA2 = 30;
        #5 SEL = 3'b010; // AND
        #5 SEL = 3'b011; // OR


        #5 SEL = 3'b001; // ADD
        #5;
        // change the values in DATA1 and DATA2
        DATA1 = 30;
        DATA2 = 40;

    end

endmodule


// creating the ALU module
module ALU(DATA1, DATA2, RESULT, SELECT);
    // declaring the ports
    // inputs
    input [7 : 0] DATA1, DATA2;
    input [2 : 0] SELECT;
    // outputs
    output reg [7 : 0] RESULT;
    wire [7 : 0] fwd_out, add_out, and_out, or_out; // to get outputs from the instances created by the function modules

    // instantiate function modules
    FORWARD fwd(DATA2, fwd_out);
    ADD add(DATA1, DATA2, add_out);
    AND and_(DATA1, DATA2, and_out);
    OR or_(DATA1, DATA2, or_out);

    // always block
    always @(*) 
    begin
        case (SELECT)
            3'b000 : RESULT <= fwd_out;
            3'b001 : RESULT <= add_out;
            3'b010 : RESULT <= and_out;
            3'b011 : RESULT <= or_out;
            default: RESULT <= 0;
        endcase
        
    end
endmodule

// FORWARD module
module FORWARD(DATA2, fwd_out);
    // declaring the ports
    input [7 : 0] DATA2;
    output [7 : 0] fwd_out;

    assign #1 fwd_out = DATA2; // for loadi, mov instructions
endmodule

// ADD module
module ADD(DATA1, DATA2, add_out);
    // declaring the ports
    input [7 : 0] DATA1;
    input [7 : 0] DATA2;
    output [7 : 0] add_out;

    assign #2 add_out = DATA1 + DATA2; // for add, sub instructions
endmodule

// AND module
module AND(DATA1, DATA2, and_out);
    // declaring the ports
    input [7 : 0] DATA1;
    input [7 : 0] DATA2;
    output [7 : 0] and_out;

    assign #1 and_out = DATA1 & DATA2; // for and instruction

endmodule

// OR module
module OR(DATA1, DATA2, or_out);
    // declaring the ports
    input [7 : 0] DATA1;
    input [7 : 0] DATA2;
    output [7 : 0] or_out;

    assign #1 or_out = DATA1 | DATA2; // for or instruction

endmodule