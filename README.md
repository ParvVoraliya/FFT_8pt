# 8-Point FFT — Pipelined Radix-2 DIF in Verilog

![Language](https://img.shields.io/badge/HDL-Verilog-blue?style=flat-square)
![Architecture](https://img.shields.io/badge/Architecture-Radix--2%20DIF-teal?style=flat-square)
![Points](https://img.shields.io/badge/FFT%20Size-8--pt-orange?style=flat-square)
![Simulator](https://img.shields.io/badge/Simulator-ModelSim%20%2F%20QuestaSim-purple?style=flat-square)
![License](https://img.shields.io/badge/License-MIT-green?style=flat-square)

A fully pipelined, hardware-efficient **8-point Fast Fourier Transform** implemented in Verilog using a **radix-2 Decimation-In-Frequency (DIF)** butterfly architecture. This design is derived from a proven 32-pt FFT pipeline, retaining all three output stages and their pre-computed twiddle ROMs unchanged.

---

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Signal Flow](#signal-flow)
- [Module Hierarchy](#module-hierarchy)
- [Port Description](#port-description)
- [Twiddle Factors](#twiddle-factors)
- [Output Bit-Reversal Map](#output-bit-reversal-map)
- [Simulation Results](#simulation-results)
- [Performance & Resource Usage](#performance--resource-usage)
- [Getting Started](#getting-started)
- [File Structure](#file-structure)
- [Author](#author)

---

## Overview

The 8-pt FFT decomposes an 8-sample complex input sequence into 8 complex frequency bins using the standard DIF butterfly decomposition:

```
X[k] = Σ x[n] · W_N^(nk),   N = 8,  k = 0..7
```

Key design decisions:

| Property | Value |
|---|---|
| Transform size | 8-point |
| Algorithm | Radix-2 DIF (Cooley-Tukey) |
| Pipeline stages | 3 |
| Butterfly style | Single `radix2` module, reused × 3 |
| Twiddle storage | ROM (look-up table), pre-computed Q8 fixed-point |
| Input word width | 12-bit signed (real + imag) |
| Internal precision | 24-bit Q8 fixed-point |
| Output word width | 16-bit signed (real + imag) |
| Output ordering | Natural (bit-reversal applied in hardware) |

---

## Architecture

The design uses a **3-stage linear feedback shift-register pipeline**. Each stage pairs one `radix2` butterfly module with a shift-register delay line and a ROM holding the stage's twiddle factors.

```
             ┌────────────┐     ┌────────────┐     ┌────────────┐
din_r/i ───► │  Stage 1   │────►│  Stage 2   │────►│  Stage 3   │────► dout_r/i
             │ shift_4    │     │ shift_2    │     │ shift_1    │
             │ ROM_4      │     │ ROM_2      │     │ W = 1      │
             │ radix_no1  │     │ radix_no2  │     │ radix_no3  │
             └────────────┘     └────────────┘     └────────────┘
               W_8^0..W_8^3       W_4^0, W_4^1        W_2^0 = 1
```

Each `radix2` instance operates as a **DIF butterfly**: given inputs `a` and `b` with twiddle `W`:

```
   state=01 (first half) :  out = a + b,   feedback = a − b
   state=10 (second half):  out = W·(a−b), feedback = a        (complex multiply)
   state=00              :  pass-through (waiting for valid data)
```

The three stages implement the complete 3-level DIF signal flow graph for N=8.

---

## Signal Flow

```
Time →   clk 0   clk 1   clk 2   clk 3   clk 4   ...

Inputs:  x[0]   x[1]   x[2]   x[3]   x[4]   x[5]   x[6]   x[7]

Stage 1 (shift_4 depth):
         fill   fill   fill   fill   B1_0   B1_1   B1_2   B1_3
         ──────────────────────── 4-sample accumulation ──────────

Stage 2 (shift_2 depth):
                              fill   fill   B2_0   B2_1   B2_2  ...
                              ──────────── 2-sample accumulation ──

Stage 3 (shift_1 depth):
                                           fill   B3_0   B3_1  ...
                                           ────── 1-sample ───────

Output reorder + capture:
                                                  out[0]  out[1] ...
```

The total **pipeline latency** from the first valid input to the first valid output is determined by the shift-register fill times: 4 + 2 + 1 = **7 clock cycles** plus internal control delays.

---

## Module Hierarchy

```
FFT_8pt  (top)
├── radix2        radix_no1   — stage-1 butterfly
├── shift_4       shift4_inst — stage-1 delay line  (4 × 24-bit cells)
├── ROM_4         rom4_inst   — stage-1 twiddle ROM  (W_8^0..W_8^3)
│
├── radix2        radix_no2   — stage-2 butterfly
├── shift_2       shift2_inst — stage-2 delay line  (2 × 24-bit cells)
├── ROM_2         rom2_inst   — stage-2 twiddle ROM  (W_4^0, W_4^1)
│
├── radix2        radix_no3   — stage-3 butterfly  (trivial W=1)
└── shift_1       shift1_inst — stage-3 delay line  (1 × 24-bit cell)
```

---

## Port Description

### `FFT_8pt` top-level

| Port | Dir | Width | Description |
|---|---|---|---|
| `clk` | in | 1 | System clock (rising-edge triggered) |
| `reset` | in | 1 | Synchronous active-high reset |
| `in_valid` | in | 1 | Assert high for each valid input sample |
| `din_r` | in | 12 | Real part of input sample, signed Q0 |
| `din_i` | in | 12 | Imaginary part of input sample, signed Q0 |
| `out_valid` | out | 1 | High when `dout_r / dout_i` are valid |
| `dout_r` | out | 16 | Real part of output bin, signed Q8 |
| `dout_i` | out | 16 | Imaginary part of output bin, signed Q8 |

> **Note:** Outputs are presented in natural (bit-reversed) order — `dout` at clock cycle `k` corresponds to frequency bin `k`.

### `radix2` butterfly

| Port | Dir | Width | Description |
|---|---|---|---|
| `state` | in | 2 | `00` idle, `01` first half, `10` second half |
| `din_a_r/i` | in | 24 | Feedback path input (from shift register) |
| `din_b_r/i` | in | 24 | Feed-forward path input |
| `w_r / w_i` | in | 24 | Twiddle factor (Q8 fixed-point) |
| `op_r / op_i` | out | 24 | Butterfly output (to next stage) |
| `delay_r / delay_i` | out | 24 | Data to be written into shift register |
| `outvalid` | out | 1 | Output valid strobe |

---

## Twiddle Factors

All twiddle factors are stored in Q8 fixed-point (1 bit sign, 7 bits integer, 8 bits fraction), giving a full-scale value of `1.0 = 0x0100`.

### Stage 1 — ROM_4  (W₈ twiddles)

| `s_count` | W | w_r (hex) | w_i (hex) |
|---|---|---|---|
| 4 | W₈⁰ = 1 | `0x000100` | `0x000000` |
| 5 | W₈¹ = cos45°−j·sin45° | `0x0000B5` | `0xFFFF4B` |
| 6 | W₈² = −j | `0x000000` | `0xFFFF00` |
| 7 | W₈³ = −cos45°−j·sin45° | `0xFFFF4B` | `0xFFFF4B` |

### Stage 2 — ROM_2  (W₄ twiddles)

| `s_count` | W | w_r (hex) | w_i (hex) |
|---|---|---|---|
| 2 | W₄⁰ = 1 | `0x000100` | `0x000000` |
| 3 | W₄¹ = −j | `0x000000` | `0xFFFF00` |

### Stage 3 — trivial (W₂⁰ = 1, hardwired)

`w_r = 24'd256`  (`0x000100`),  `w_i = 24'd0`

---

## Output Bit-Reversal Map

The radix-2 DIF pipeline naturally produces outputs in bit-reversed order. The hardware corrects this in the result capture logic:

| Pipeline slot `y_1` | Natural FFT bin `k` |
|:---:|:---:|
| 0 | 7 |
| 1 | 3 |
| 2 | 1 |
| 3 | 5 |
| 4 | 0 |
| 5 | 4 |
| 6 | 2 |
| 7 | 6 |

> ⚠️ **Verify this mapping** against a software reference FFT in your testbench before using in production — swap indices in the `case` statement if bins appear reordered.

---

## Simulation Results

Simulated using **ModelSim / QuestaSim**. The waveforms below show the end-to-end pipeline: eight 12-bit complex inputs asserted with `in_valid`, the three-stage pipeline filling, and eight 16-bit frequency bins appearing on `dout_r / dout_i` when `out_valid` goes high.

### Waveform — full pipeline

<!-- Replace with your ModelSim screenshot -->
> 📷 _Upload your ModelSim waveform screenshot here_
```
![Simulation Waveform](./sim/waveform_full.png)
```

### Waveform — output bins detail

<!-- Replace with your ModelSim screenshot -->
> 📷 _Upload your output bins detail screenshot here_
```
![Output Bins](./sim/waveform_output.png)
```

### FFT output spectrum (magnitude)

<!-- Replace with your ModelSim screenshot -->
> 📷 _Upload your FFT magnitude plot here_
```
![FFT Spectrum](./sim/fft_spectrum.png)
```

---

## Performance & Resource Usage

> Values below are estimates for a typical 7-series / Artix-7 FPGA. Run synthesis for exact figures.

### Timing

| Parameter | Value |
|---|---|
| Target clock | 100 MHz |
| Pipeline latency | ~7 clock cycles (fill) + control overhead |
| Throughput | 1 output frame per 8 input samples |
| Samples per second | Up to 100 MS/s (clock-rate dependent) |

### Resource Utilization (estimated, Artix-7 XC7A35T)

| Resource | Used | Notes |
|---|---|---|
| LUTs | ~350–450 | Butterfly arithmetic + control logic |
| FFs | ~200–280 | Shift registers + pipeline registers |
| DSP48s | 0–6 | Depends on synthesis tool mapping of complex multiply |
| BRAMs | 0 | ROMs small enough to fit in LUTs (distributed RAM) |
| IOBs | ~52 | 12-bit × 2 in + 16-bit × 2 out + control |

> Run `report_utilization` in Vivado or the equivalent in Quartus for accurate figures after synthesis.

### Compared to 32-pt parent design

| Metric | 32-pt | 8-pt | Reduction |
|---|---|---|---|
| Pipeline stages | 5 | 3 | −40% |
| Shift-register depth | 16+8+4+2+1=31 | 4+2+1=7 | −77% |
| Twiddle ROM entries | 16+8+4+2 = 30 | 4+2 = 6 | −80% |
| Output bins | 32 | 8 | −75% |
| LUT estimate | ~900 | ~400 | ~−55% |

---

## Getting Started

### Prerequisites

- ModelSim / QuestaSim (any recent version)
- All source files listed under [File Structure](#file-structure)

### Compile & simulate

```tcl
# Start ModelSim, then in the transcript:

# 1. Create a working library
vlib work
vmap work work

# 2. Compile all source files (order matters for `include resolution)
vlog -sv radix2.v
vlog -sv ROM_2.v ROM_4.v
vlog -sv shift_1.v shift_2.v shift_4.v
vlog -sv FFT_8pt.v
vlog -sv tb_FFT_8pt.v   # your testbench

# 3. Start simulation
vsim -t 1ns work.tb_FFT_8pt

# 4. Add waves and run
add wave -r /*
run -all
```

### Testbench stimulus example

```verilog
// Apply 8 complex samples, then check dout against reference
initial begin
    reset = 1; in_valid = 0;
    #20 reset = 0;

    // Feed x[0..7]
    repeat (8) begin
        @(posedge clk);
        in_valid = 1;
        din_r = $signed(sample_r[sample_idx]);
        din_i = $signed(sample_i[sample_idx]);
        sample_idx = sample_idx + 1;
    end
    @(posedge clk); in_valid = 0;

    // Wait for out_valid then check bins
    @(posedge out_valid);
    // ... compare dout_r / dout_i against reference
end
```

---

## File Structure

```
.
├── RTL/
│   ├── FFT_8pt.v          # Top-level 8-pt FFT module
│   ├── radix2.v           # Radix-2 DIF butterfly (shared by all stages)
│   ├── ROM_2.v            # Stage-2 twiddle ROM  (W_4 factors)
│   ├── ROM_4.v            # Stage-1 twiddle ROM  (W_8 factors)
│   ├── shift_1.v          # 1-sample complex delay line
│   ├── shift_2.v          # 2-sample complex delay line
│   └── shift_4.v          # 4-sample complex delay line
│
├── sim/
│   ├── tb_FFT_8pt.v       # ModelSim testbench
│   ├── waveform_full.png  # ← add your simulation screenshots here
│   ├── waveform_output.png
│   └── fft_spectrum.png
│
└── README.md
```

---

## Author

**Ahmed Abdelazeem**
- GitHub: [@abdelazeem201](https://github.com/abdelazeem201)
- Email: ahmed_abdelazeem@outlook.com

---

*Derived from a 32-pt pipelined FFT architecture. The 8-pt variant retains stages 3–5 of the original pipeline (shift_4 → shift_2 → shift_1) and reuses the ROM_4 / ROM_2 twiddle tables without modification.*
