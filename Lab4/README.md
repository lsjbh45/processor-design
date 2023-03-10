# Lab 4: 5-stage Pipelined Processor Design

## Design

### Goal

Complete the top design of the 5-stage pipelined processor

- Pipeline register design
- Internal forwarding in the register file
- Forwarding unit design
- Hazard detection unit

### Implementation

- [pipeline_cpu.sv](pipeline_cpu.sv) (5-stage pipelined processor)

## Testbench

### Goal

Verify the design with the testbench

- Set the clock frequency as 100 MHz
- Instantiate the top design file as "dut" (design under test)

### Testcase

- Given initial memory data for the instruction memory, the data memory, and the register file
  - [imem.mem](imem.mem) (instruction memory)
  - [dmem.mem](dmem.mem) (data memory)
  - [regfile.mem](regfile.mem) (register file)

### Implementation

- [tb_pipeline_cpu.sv](tb_pipeline_cpu.sv)

### Result

- [report.txt](report.txt)
