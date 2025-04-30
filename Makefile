# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

# Files
SRC = cache_simulation.cpp
OUT = simulation

# Default: just build
all: $(OUT)

$(OUT): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(OUT)

# Run with 5 positional arguments: s E b trace_prefix output_file
run: $(OUT)
	@if [ "$(ARGS)" = "" ]; then \
		echo "Usage: make run ARGS=\"<s> <E> <b> <trace_prefix> <output_file>\""; \
		exit 1; \
	fi; \
	set -- $(ARGS); \
	s=$$1; E=$$2; b=$$3; trace=$$4; out=$$5; \
	echo "Running: ./$(OUT) -s $$s -E $$E -b $$b -t $$trace -o $$out"; \
	./$(OUT) -s $$s -E $$E -b $$b -t $$trace -o $$out

# Clean
clean:
	rm -f $(OUT) *.o *.txt  tmp_out.txt

.PHONY: all run clean
