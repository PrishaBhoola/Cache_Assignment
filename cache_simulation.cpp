#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include <unordered_map>
#include <list>
#include <getopt.h>
#include <cstdint>
using namespace std;

// Cache line states for MESI protocol
enum class CacheLineState {
    M, // Modified
    E, // Exclusive
    S, // Shared
    I  // Invalid
};

// Structure to represent a cache line
struct CacheLine {
    bool valid;                   // Is the line valid?
    bool dirty;                   // Is the line dirty (M)?
    uint32_t tag;                 // Tag bits
    CacheLineState state;         // MESI state
    uint32_t lru_counter;         // LRU counter (higher = more recently used)
    vector<uint8_t> data;    // Actual data stored in cache line

    // Constructor
    CacheLine(int blockSize) : valid(false), dirty(false), tag(0), state(CacheLineState::I), lru_counter(0) {
        data.resize(blockSize);
    }
};

// Structure to represent a cache set
struct CacheSet {
    vector<CacheLine> lines; // Cache lines in this set
    uint32_t lru_timestamp;       // Global timestamp for LRU

    // Constructor
    CacheSet(int associativity, int blockSize) : lru_timestamp(0) {
        lines.reserve(associativity);
        for (int i = 0; i < associativity; i++) {
            lines.emplace_back(blockSize);
        }
    }

    // Update LRU counters when a line is accessed
    void updateLRU(int lineIndex) {
        lru_timestamp++;
        lines[lineIndex].lru_counter = lru_timestamp;
    }

    // Find the LRU line in the set
    int findLRULine() {
        int lru_index = 0;
        uint32_t min_counter = lines[0].lru_counter;
        
        for (size_t i = 1; i < lines.size(); i++) {
            if (lines[i].lru_counter < min_counter) {
                min_counter = lines[i].lru_counter;
                lru_index = i;
            }
        }
        return lru_index;
    }
};

// L1 Cache implementation
class L1Cache {
private:
    int s;              // Number of set index bits
    int E;              // Associativity (lines per set)
    int b;              // Number of block bits
    int S;              // Number of sets (2^s)
    int B;              // Block size in bytes (2^b)
    
    vector<CacheSet> sets;

    // Statistics
    uint64_t accesses;
    uint64_t hits;
    uint64_t misses;
    uint64_t evictions;
    uint64_t reads;
    uint64_t writes;
    uint64_t writebacks;
    uint64_t total_cycles;
    uint64_t idle_cycles;

public:
    // Constructor
    L1Cache(int set_bits, int associativity, int block_bits) 
        : s(set_bits), E(associativity), b(block_bits),
          accesses(0), hits(0), misses(0), evictions(0), reads(0), writes(0),
          writebacks(0), total_cycles(0), idle_cycles(0) {
        
        S = 1 << s;  // 2^s
        B = 1 << b;  // 2^b
        
        // Initialize cache sets
        sets.reserve(S);
        for (int i = 0; i < S; i++) {
            sets.emplace_back(E, B);
        }
        
        cout << "Created L1 Cache with:" << endl;
        cout << "  Sets: " << S << " (2^" << s << ")" << endl;
        cout << "  Associativity: " << E << endl;
        cout << "  Block size: " << B << " bytes (2^" << b << ")" << endl;
        cout << "  Cache size: " << (S * E * B) / 1024.0 << " KB" << endl;
    }

    // Helper functions to extract address components
    uint32_t getTag(uint32_t address) const {
        return address >> (s + b);
    }

    uint32_t getSetIndex(uint32_t address) const {
        return (address >> b) & ((1 << s) - 1);
    }

    uint32_t getBlockOffset(uint32_t address) const {
        return address & ((1 << b) - 1);
    }

    // Find a line in a set by tag
    int findLine(uint32_t setIndex, uint32_t tag) {
        for (int i = 0; i < E; i++) {
            if (sets[setIndex].lines[i].valid && sets[setIndex].lines[i].tag == tag) {
                return i;
            }
        }
        return -1;  // Not found
    }

    // Process a memory access (read or write)
    void accessMemory(bool isWrite, uint32_t address) {
        accesses++;
        if (isWrite) {
            writes++;
        } else {
            reads++;
        }

        uint32_t tag = getTag(address);
        uint32_t setIndex = getSetIndex(address);
        // uint32_t blockOffset = getBlockOffset(address);  // Will be used later for actual data access

        // Find if the block is in cache
        int lineIndex = findLine(setIndex, tag);

        if (lineIndex != -1) {
            // Cache hit
            hits++;
            total_cycles += 1;  // L1 cache hit takes 1 cycle
            
            // Update LRU status for this line
            sets[setIndex].updateLRU(lineIndex);
            
            // Update dirty bit for writes
            if (isWrite) {
                sets[setIndex].lines[lineIndex].dirty = true;
                sets[setIndex].lines[lineIndex].state = CacheLineState::M;
            }
        } else {
            // Cache miss
            misses++;
            
            // Find a place to put this block
            bool foundEmpty = false;
            
            // First, look for an I (empty) line
            for (int i = 0; i < E; i++) {
                if (!sets[setIndex].lines[i].valid) {
                    lineIndex = i;
                    foundEmpty = true;
                    break;
                }
            }
            
            // If no empty line, use LRU replacement policy
            if (!foundEmpty) {
                lineIndex = sets[setIndex].findLRULine();
                
                // Handle eviction of the LRU line
                if (sets[setIndex].lines[lineIndex].valid) {
                    evictions++;
                    
                    // If dirty, need to write back to memory
                    if (sets[setIndex].lines[lineIndex].dirty) {
                        writebacks++;
                        idle_cycles += 100;  // Memory write takes 100 cycles
                    }
                }
            }
            
            // Load the new line
            sets[setIndex].lines[lineIndex].valid = true;
            sets[setIndex].lines[lineIndex].tag = tag;
            sets[setIndex].lines[lineIndex].dirty = isWrite;  // Set dirty if write
            sets[setIndex].lines[lineIndex].state = isWrite ? CacheLineState::M : CacheLineState::E;
            
            // Update LRU status
            sets[setIndex].updateLRU(lineIndex);
            
            // Fetch from memory
            idle_cycles += 100;  // Memory read takes 100 cycles
            total_cycles += 100 + 1;  // Memory access + cache access
        }
    }

    // Print statistics
    void printStats() {
        cout << "Cache Statistics:" << endl;
        cout << "  Total Accesses: " << accesses << endl;
        cout << "  Reads: " << reads << endl;
        cout << "  Writes: " << writes << endl;
        cout << "  Hits: " << hits << endl;
        cout << "  Misses: " << misses << endl;
        cout << "  Evictions: " << evictions << endl;
        cout << "  Writebacks: " << writebacks << endl;
        cout << "  Miss Rate: " << (accesses > 0 ? (100.0 * misses / accesses) : 0) << "%" << endl;
        cout << "  Total Cycles: " << total_cycles << endl;
        cout << "  Idle Cycles: " << idle_cycles << endl;
    }

    // Get statistics values
    uint64_t getAccesses() const { return accesses; }
    uint64_t getHits() const { return hits; }
    uint64_t getMisses() const { return misses; }
    uint64_t getEvictions() const { return evictions; }
    uint64_t getReads() const { return reads; }
    uint64_t getWrites() const { return writes; }
    uint64_t getWritebacks() const { return writebacks; }
    uint64_t getTotalCycles() const { return total_cycles; }
    uint64_t getIdleCycles() const { return idle_cycles; }
    double getMissRate() const { return (accesses > 0) ? (100.0 * misses / accesses) : 0; }
};

// Parse command line arguments
bool parseArgs(int argc, char* argv[], string& tracePrefix, int& s, int& E, int& b, string& outFilename) {
    int opt;
    while ((opt = getopt(argc, argv, "t:s:E:b:o:h")) != -1) {
        switch (opt) {
            case 't':
                tracePrefix = optarg;
                break;
            case 's':
                s = stoi(optarg);
                break;
            case 'E':
                E = stoi(optarg);
                break;
            case 'b':
                b = stoi(optarg);
                break;
            case 'o':
                outFilename = optarg;
                break;
            case 'h':
                cout << "Usage: " << argv[0] << " -t <tracefile> -s <s> -E <E> -b <b> [-o <outfilename>] [-h]" << endl;
                cout << "-t <tracefile>: name of parallel application (e.g. app1) whose 4 traces are to be used" << endl;
                cout << "-s <s>: number of set index bits (number of sets in the cache = S = 2^s)" << endl;
                cout << "-E <E>: associativity (number of cache lines per set)" << endl;
                cout << "-b <b>: number of block bits (block size = B = 2^b)" << endl;
                cout << "-o: <outfilename> logs output in file for plotting etc." << endl;
                cout << "-h: prints this help" << endl;
                return false;
            default:
                cerr << "Invalid argument" << endl;
                return false;
        }
    }
    
    // Check if required arguments are provided
    if (tracePrefix.empty() || s <= 0 || E <= 0 || b <= 0) {
        cerr << "Missing or Invalid required arguments. Use -h for help." << endl;
        return false;
    }
    
    return true;
}

// Process a single trace file
void processTraceFile(const string& filename, L1Cache& cache) {
    ifstream traceFile(filename);
    if (!traceFile.is_open()) {
        cerr << "Failed to open trace file: " << filename << endl;
        return;
    }

    string operation;
    string addressStr;
    
    while (traceFile >> operation >> addressStr) {
        // Convert hex string to uint32_t
        uint32_t address = stoul(addressStr, nullptr, 16);
        
        // Process the memory operation
        bool isWrite = (operation == "W");
        cache.accessMemory(isWrite, address);
    }
    
    traceFile.close();
}

int main(int argc, char* argv[]) {
    string tracePrefix;
    int s = 0;
    int E = 0;
    int b = 0;
    string outFilename;
    
    // Parse command line arguments
    if (!parseArgs(argc, argv, tracePrefix, s, E, b, outFilename)) {
        return 1;
    }
    
    // Create the L1 cache
    L1Cache cache(s, E, b);
    
    // Process the trace file (for single core)
    string traceFile = tracePrefix + "_proc0.trace";
    processTraceFile(traceFile, cache);
    
    // Print statistics
    cache.printStats();
    
    // If an output file is specified, write the statistics to it
    if (!outFilename.empty()) {
        ofstream outFile(outFilename);
        if (outFile.is_open()) {
            outFile << "Accesses: " << cache.getAccesses() << endl;
            outFile << "Reads: " << cache.getReads() << endl;
            outFile << "Writes: " << cache.getWrites() << endl;
            outFile << "Hits: " << cache.getHits() << endl;
            outFile << "Misses: " << cache.getMisses() << endl;
            outFile << "Evictions: " << cache.getEvictions() << endl;
            outFile << "Writebacks: " << cache.getWritebacks() << endl;
            outFile << "Miss Rate: " << cache.getMissRate() << "%" << endl;
            outFile << "Total Cycles: " << cache.getTotalCycles() << endl;
            outFile << "Idle Cycles: " << cache.getIdleCycles() << endl;
            outFile.close();
        } else {
            cerr << "Failed to open output file: " << outFilename << endl;
        }
    }
    
    return 0;
}