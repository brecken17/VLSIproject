module VectorProcessingV3 #(
    parameter VECTOR_LENGTH = 4,     
    parameter DATA_WIDTH = 32,        
    parameter NUM_REGISTERS = 32  
)(
    `ifdef USE_POWER_PINS
        inout vccd1;
        inout vssd1;
    `endif
    input wire clk,
    input wire rst_n,
    
    input wire enable,                
    input wire [6:0] funct,           
    input wire [4:0] vs1,             
    input wire [4:0] vs2,             
    input wire [4:0] vr,              
    input wire start_op,              
    output reg op_done,               
    
    input wire [DATA_WIDTH-1:0] mem_data_in,
    output reg [DATA_WIDTH-1:0] mem_data_out,
    output reg [31:0] mem_addr,
    output reg mem_read,
    output reg mem_write,
    
    input wire [24:0] vl            
);
endmodule
