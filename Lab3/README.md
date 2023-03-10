# Lab 3: Single-cycle Processor Design

## Design

### Goal

Complete the top design of the single-cycle processor using the previously designed datapath elements

### Implementation

- [single_cycle_cpu.sv](single_cycle_cpu.sv) (Single-cycle Processor)

## Testbench

### Goal

Verify the design with the testbench

- Set the clock frequency as 100 MHz
- Instantiate the top design file as "dut" (design under test)

### Testcase

- Given initial memory data for the instruction memory and the data memory
  - [imem.mem](imem.mem) (instruction memory)
  - [dmem.mem](dmem.mem) (data memory)

### Implementation

- [tb_single_cycle_cpu.sv](tb_single_cycle_cpu.sv)

### Result

- [report.txt](report.txt)
