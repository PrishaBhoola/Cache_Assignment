# Cache Simulator (with MESI Protocol)

This project implements a 4-core L1 cache simulator using the MESI cache coherence protocol. It processes trace files for each core and simulates inter-core communication and memory coherence via a shared bus.

---

## Building the Project

You can compile the project using the provided `Makefile`.

### To build:
```bash
make
```

## Running the Simulator
The Makefile provides a run target that accepts arguments via the ARGS variable.
```bash
make run ARGS="<s> <E> <b> <trace_prefix> <output_file>"
```

For example, running:
```bash
make run ARGS="5 4 6 app1 stats.txt"
```
is equivalent to running:
```bash
./simulation -s 5 -E 4 -b 6 -t app1 -o stats.txt
```

which will simulate 4 cores using the following trace files:
app1_proc0.trace
app1_proc1.trace
app1_proc2.trace
app1_proc3.trace

## Trace File Format
Each line in a trace file represents a memory operation:
```bash
R 0x1A2B3C4D
W 0x0A0B0C0D
```

- R = Read
- W = Write

Each core has its own file, named using the trace prefix:
```bash
<trace_prefix>_proc0.trace
<trace_prefix>_proc1.trace
<trace_prefix>_proc2.trace
<trace_prefix>_proc3.trace
```

## Output
Simulation results include:
- Cache statistics per core (hits, misses, writebacks, etc.)
- Bus traffic in bytes
- Total cycle count
These are printed to the specified output file (e.g., stats.txt).

## Cleaning Up
To remove the compiled executable and temporary files:
```bash
make clean
```
