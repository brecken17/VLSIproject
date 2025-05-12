module VectorProcessingV3 #(
    parameter VECTOR_LENGTH = 4,      // Number of elements per vector
    parameter DATA_WIDTH = 32,        // Width of each element
    parameter NUM_REGISTERS = 32      // Vector registers
)(
    `ifdef USE_POWER_PINS
        inout vccd1;
        inout vssd1;
    `endif
    input wire clk,
    input wire rst_n,
    
    input wire enable,                
    input wire [6:0] funct,           // RISC-V funct field
    input wire [4:0] vs1,             // Source vector register 1
    input wire [4:0] vs2,             // Source vector register 2
    input wire [4:0] vr,              // Result vector register
    input wire start_op,              // Start operation signal
    output reg op_done,               // Operation complete signal
    
    input wire [DATA_WIDTH-1:0] mem_data_in,
    output reg [DATA_WIDTH-1:0] mem_data_out,
    output reg [31:0] mem_addr,
    output reg mem_read,
    output reg mem_write,
    
    input wire [24:0] vl             // Current vector length
);

    
    // Vector operations
    localparam VADD       = 7'b0000000;  // Vector addition
    localparam VSUB       = 7'b0000001;  // Vector subtraction
    localparam VMUL       = 7'b0000010;  // Vector multiplication
    
    // Memory operations
    localparam VLE        = 7'b1000000;  // Vector load
    localparam VSE        = 7'b0100000;  // Vector store
    
    // Vector register file
    reg [DATA_WIDTH-1:0] vector_reg [0:NUM_REGISTERS-1][0:VECTOR_LENGTH-1];
    
    // Pipeline stages
    reg [6:0] pipe_funct;
    reg [4:0] pipe_vs1, pipe_vs2, pipe_vr;
    reg pipe_valid;
    reg [31:0] pipe_vl;  // Vector length for current operation
    
    // Temporary registers for computation
    reg [DATA_WIDTH-1:0] src1_data [0:VECTOR_LENGTH-1];
    reg [DATA_WIDTH-1:0] src2_data [0:VECTOR_LENGTH-1];
    reg [DATA_WIDTH-1:0] result_data [0:VECTOR_LENGTH-1];
    
    // State machine
    localparam IDLE      = 2'b00;
    localparam EXECUTE   = 2'b01;
    localparam WRITEBACK = 2'b10;
    localparam MEMORY_OP = 2'b11;
    reg [1:0] state;
    
    // Memory operation counters
    reg [31:0] mem_counter;
    reg [31:0] active_vl;  // Actual vector length for current operation
    
    integer i, j;  // Declare all integer variables here
    
    // State machine logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            op_done <= 1'b0;
            pipe_valid <= 1'b0;
            mem_read <= 1'b0;
            mem_write <= 1'b0;
            mem_counter <= 32'd0;
            
            // Clear registers
            for (i = 0; i < NUM_REGISTERS; i = i + 1) begin
                for (j = 0; j < VECTOR_LENGTH; j = j + 1) begin
                    vector_reg[i][j] <= {DATA_WIDTH{1'b0}};
                end
            end
        end else begin
            case (state)
                IDLE: begin
                    op_done <= 1'b0;
                    if (enable && start_op) begin
                        // Latch operation parameters
                        pipe_funct <= funct;
                        pipe_vs1 <= vs1;
                        pipe_vs2 <= vs2;
                        pipe_vr <= vr;
                        pipe_valid <= 1'b1;
                        
                        // Determine actual vector length (min of vl and VECTOR_LENGTH)
                        active_vl <= (vl < VECTOR_LENGTH) ? vl : VECTOR_LENGTH;
                        pipe_vl <= (vl < VECTOR_LENGTH) ? vl : VECTOR_LENGTH;
                        
                        // Load source operands
                        for (i = 0; i < VECTOR_LENGTH; i = i + 1) begin
                            if (i < vl) begin
                                src1_data[i] <= vector_reg[vs1][i];
                                src2_data[i] <= vector_reg[vs2][i];
                            end else begin
                                // Zero out elements beyond vector length
                                src1_data[i] <= {DATA_WIDTH{1'b0}};
                                src2_data[i] <= {DATA_WIDTH{1'b0}};
                            end
                        end
                        
                        // Determine next state
                        if (funct == VLE || funct == VSE) begin
                            state <= MEMORY_OP;
                            mem_counter <= 32'd0;
                            if (funct == VLE) begin
                                mem_read <= 1'b1;
                                mem_write <= 1'b0;
                            end else begin
                                mem_read <= 1'b0;
                                mem_write <= 1'b1;
                            end
                            // Initialize memory address
                            mem_addr <= 32'd0;
                        end else begin
                            state <= EXECUTE;
                        end
                    end
                end
                
                EXECUTE: begin
                    if (pipe_valid) begin
                        // Perform vector operation
                        for (i = 0; i < VECTOR_LENGTH; i = i + 1) begin
                            if (i < pipe_vl) begin
                                case (pipe_funct)
                                    VADD: result_data[i] <= src1_data[i] + src2_data[i];
                                    VSUB: result_data[i] <= src1_data[i] - src2_data[i];
                                    VMUL: result_data[i] <= src1_data[i] * src2_data[i];
                                    default: result_data[i] <= src1_data[i];
                                endcase
                            end else begin
                                result_data[i] <= {DATA_WIDTH{1'b0}};
                            end
                        end
                        
                        state <= WRITEBACK;
                    end
                end
                
                WRITEBACK: begin
                    // Write results to destination register
                    if (pipe_valid) begin
                        for (i = 0; i < VECTOR_LENGTH; i = i + 1) begin
                            if (i < pipe_vl) begin
                                vector_reg[pipe_vr][i] <= result_data[i];
                            end
                        end
                    end
                    
                    pipe_valid <= 1'b0;
                    op_done <= 1'b1;
                    state <= IDLE;
                end
                
                MEMORY_OP: begin
                    if (mem_counter < active_vl) begin
                        if (pipe_funct == VLE) begin
                            // Vector load operation
                            if (mem_read) begin
                                vector_reg[pipe_vr][mem_counter] <= mem_data_in;
                                mem_addr <= mem_addr + (DATA_WIDTH/8);  // Increment by element size
                                mem_counter <= mem_counter + 1;
                            end
                        end else if (pipe_funct == VSE) begin
                            // Vector store operation
                            if (mem_write) begin
                                mem_data_out <= vector_reg[pipe_vs2][mem_counter];
                                mem_addr <= mem_addr + (DATA_WIDTH/8);  // Increment by element size
                                mem_counter <= mem_counter + 1;
                            end
                        end
                    end else begin
                        // Memory operation complete
                        mem_read <= 1'b0;
                        mem_write <= 1'b0;
                        pipe_valid <= 1'b0;
                        op_done <= 1'b1;
                        state <= IDLE;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end
    
endmodule
