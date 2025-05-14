`timescale 1 ns / 1 ps

module picorv32_top2 #(
    parameter [0:0] ENABLE_COUNTERS = 1,
    parameter [0:0] ENABLE_COUNTERS64 = 1,
    parameter [0:0] ENABLE_REGS_16_31 = 1,
    parameter [0:0] ENABLE_REGS_DUALPORT = 1,
    parameter [0:0] LATCHED_MEM_RDATA = 0,
    parameter [0:0] TWO_STAGE_SHIFT = 1,
    parameter [0:0] BARREL_SHIFTER = 0,
    parameter [0:0] TWO_CYCLE_COMPARE = 0,
    parameter [0:0] TWO_CYCLE_ALU = 0,
    parameter [0:0] COMPRESSED_ISA = 0,
    parameter [0:0] CATCH_MISALIGN = 1,
    parameter [0:0] CATCH_ILLINSN = 1,
    parameter [0:0] ENABLE_PCPI = 0,
    parameter [0:0] ENABLE_MUL = 0,
    parameter [0:0] ENABLE_FAST_MUL = 0,
    parameter [0:0] ENABLE_DIV = 0,
    parameter [0:0] ENABLE_IRQ = 0,
    parameter [0:0] ENABLE_IRQ_QREGS = 1,
    parameter [0:0] ENABLE_IRQ_TIMER = 1,
    parameter [0:0] ENABLE_TRACE = 0,
    parameter [0:0] REGS_INIT_ZERO = 0,
    parameter [31:0] MASKED_IRQ = 32'h0000_0000,
    parameter [31:0] LATCHED_IRQ = 32'hffff_ffff,
    parameter [31:0] PROGADDR_RESET = 32'h0000_0000,
    parameter [31:0] PROGADDR_IRQ = 32'h0000_0010,
    parameter [31:0] STACKADDR = 32'hffff_ffff,
    
    // Vector unit parameters
    parameter VECTOR_LENGTH = 4,
    parameter DATA_WIDTH = 32,
    parameter VECTOR_REGISTERS = 32
) (
    input clk,
    resetn,
    output reg trap,

    // Look-Ahead Interface
    output            mem_la_read,
    output            mem_la_write,
    output     [31:0] mem_la_addr,
    output reg [31:0] mem_la_wdata,
    output reg [ 3:0] mem_la_wstrb,

    // Pico Co-Processor Interface (PCPI)
    output reg        pcpi_valid,
    output reg [31:0] pcpi_insn,
    output     [31:0] pcpi_rs1,
    output     [31:0] pcpi_rs2,
    input             pcpi_wr,
    input      [31:0] pcpi_rd,
    input             pcpi_wait,
    input             pcpi_ready,

    // IRQ Interface
    input      [31:0] irq,
    output reg [31:0] eoi,

`ifdef RISCV_FORMAL
    output reg        rvfi_valid,
    output reg [63:0] rvfi_order,
    output reg [31:0] rvfi_insn,
    output reg        rvfi_trap,
    output reg        rvfi_halt,
    output reg        rvfi_intr,
    output reg [ 1:0] rvfi_mode,
    output reg [ 1:0] rvfi_ixl,
    output reg [ 4:0] rvfi_rs1_addr,
    output reg [ 4:0] rvfi_rs2_addr,
    output reg [31:0] rvfi_rs1_rdata,
    output reg [31:0] rvfi_rs2_rdata,
    output reg [ 4:0] rvfi_rd_addr,
    output reg [31:0] rvfi_rd_wdata,
    output reg [31:0] rvfi_pc_rdata,
    output reg [31:0] rvfi_pc_wdata,
    output reg [31:0] rvfi_mem_addr,
    output reg [ 3:0] rvfi_mem_rmask,
    output reg [ 3:0] rvfi_mem_wmask,
    output reg [31:0] rvfi_mem_rdata,
    output reg [31:0] rvfi_mem_wdata,

    output reg [63:0] rvfi_csr_mcycle_rmask,
    output reg [63:0] rvfi_csr_mcycle_wmask,
    output reg [63:0] rvfi_csr_mcycle_rdata,
    output reg [63:0] rvfi_csr_mcycle_wdata,

    output reg [63:0] rvfi_csr_minstret_rmask,
    output reg [63:0] rvfi_csr_minstret_wmask,
    output reg [63:0] rvfi_csr_minstret_rdata,
    output reg [63:0] rvfi_csr_minstret_wdata,
`endif

    // Trace Interface
    output reg        trace_valid,
    output reg [35:0] trace_data
);

    // Memory signals.
    reg mem_valid, mem_instr, mem_ready;
    wire [31:0] mem_addr; // Changed from reg to wire
    wire [31:0] mem_wdata; // Wire for memory write data
    reg [ 3:0] mem_wstrb;
    reg [31:0] mem_rdata;
    
    // Add new wires for CPU and vector unit signals
    wire [31:0] cpu_mem_addr;
    wire [31:0] vector_mem_addr;
    wire [31:0] vector_mem_wdata;
    reg use_vector_data;
    
    // Add intermediate signals for mem_read and mem_write
    wire mem_read_signal;
    wire mem_write_signal;
    
    // Assign the intermediate signals
    assign mem_read_signal = mem_valid & ~|mem_wstrb;
    assign mem_write_signal = mem_valid & |mem_wstrb;

    // No 'ready' signal in sky130 SRAM macro; presumably it is single-cycle?
    always @(posedge clk) mem_ready <= mem_valid;
    
    // Mux to select between CPU and vector unit data
    assign mem_wdata = use_vector_data ? vector_mem_wdata : pcpi_rs2;
    
    // Mux to select between CPU and vector unit address
    assign mem_addr = use_vector_data ? vector_mem_addr : cpu_mem_addr;
    
    // Logic to determine when to use vector data
    always @(*) begin
        use_vector_data = pcpi_valid && (pcpi_insn[6:0] == 7'b0100000); // VSE instruction
    end

    // PicoRV32 CPU instantiation
    picorv32 rv32_soc (
        .clk(clk),
        .resetn(resetn),
        .trap(trap),
        .mem_valid(mem_valid),
        .mem_instr(mem_instr),
        .mem_ready(mem_ready),
        .mem_addr(cpu_mem_addr), // Connect to CPU-specific address
        .mem_wdata(pcpi_rs2),    // Connect to rs2 directly
        .mem_wstrb(mem_wstrb),
        .mem_rdata(mem_rdata),
        .mem_la_read(mem_la_read),
        .mem_la_write(mem_la_write),
        .mem_la_addr(mem_la_addr),
        .mem_la_wdata(mem_la_wdata),
        .mem_la_wstrb(mem_la_wstrb),
        .pcpi_valid(pcpi_valid),
        .pcpi_insn(pcpi_insn),
        .pcpi_rs1(pcpi_rs1),
        .pcpi_rs2(pcpi_rs2),
        .pcpi_wr(pcpi_wr),
        .pcpi_rd(pcpi_rd),
        .pcpi_wait(pcpi_wait),
        .pcpi_ready(pcpi_ready),
        .irq(irq),
        .eoi(eoi),
        `ifdef RISCV_FORMAL
        .rvfi_valid(rvfi_valid),
        .rvfi_order(rvfi_order),
        .rvfi_insn(rvfi_insn),
        .rvfi_trap(rvfi_trap),
        .rvfi_halt(rvfi_halt),
        .rvfi_intr(rvfi_intr),
        .rvfi_mode(rvfi_mode),
        .rvfi_ixl(rvfi_ixl),
        .rvfi_rs1_addr(rvfi_rs1_addr),
        .rvfi_rs2_addr(rvfi_rs2_addr),
        .rvfi_rs1_rdata(rvfi_rs1_rdata),
        .rvfi_rs2_rdata(rvfi_rs2_rdata),
        .rvfi_rd_addr(rvfi_rd_addr),
        .rvfi_rd_wdata(rvfi_rd_wdata),
        .rvfi_pc_rdata(rvfi_pc_rdata),
        .rvfi_pc_wdata(rvfi_pc_wdata),
        .rvfi_mem_addr(rvfi_mem_addr),
        .rvfi_mem_rmask(rvfi_mem_rmask),
        .rvfi_mem_wmask(rvfi_mem_wmask),
        .rvfi_mem_rdata(rvfi_mem_rdata),
        .rvfi_mem_wdata(rvfi_mem_wdata),
        .rvfi_csr_mcycle_rmask(rvfi_csr_mcycle_rmask),
        .rvfi_csr_mcycle_wmask(rvfi_csr_mcycle_wmask),
        .rvfi_csr_mcycle_rdata(rvfi_csr_mcycle_rdata),
        .rvfi_csr_mcycle_wdata(rvfi_csr_mcycle_wdata),
        .rvfi_csr_minstret_rmask(rvfi_csr_minstret_rmask),
        .rvfi_csr_minstret_wmask(rvfi_csr_minstret_wmask),
        .rvfi_csr_minstret_rdata(rvfi_csr_minstret_rdata),
        .rvfi_csr_minstret_wdata(rvfi_csr_minstret_wdata),
        `endif
        .trace_valid(trace_valid),
        .trace_data(trace_data)
    );
    
    // Vector Processing Unit instantiation
    VectorProcessingV3 #(
        .VECTOR_LENGTH(VECTOR_LENGTH),
        .DATA_WIDTH(DATA_WIDTH),
        .NUM_REGISTERS(VECTOR_REGISTERS)
    ) vector_unit (
        .clk         (clk),
        .rst_n       (resetn),
        
        .enable      (1'b1),
        .funct       (pcpi_insn[6:0]),
        .vs1         (pcpi_rs1),
        .vs2         (pcpi_rs2),
        .vr          (pcpi_rd),
        .start_op    (pcpi_valid),
        .op_done     (pcpi_ready),
        
        .mem_data_in (mem_rdata),
        .mem_data_out(vector_mem_wdata), // Connect to the vector-specific data wire
        .mem_addr    (vector_mem_addr),  // Connect to the vector-specific address wire
        .mem_read    (mem_read_signal),  // Connect to the intermediate signal
        .mem_write   (mem_write_signal), // Connect to the intermediate signal
        
        .vl          (pcpi_insn[31:7])
    );
endmodule

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
