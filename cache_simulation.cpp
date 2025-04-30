#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include <unordered_map>
#include <list>
#include <getopt.h>
#include <cstdint>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
using namespace std;

// Number of cores
const int NUM_CORES = 4;

// Bus operation types
enum class BusOp {
    BUS_RD,    // Bus Read (for read miss)
    BUS_RDX,   // Bus Read Exclusive (for write miss)
    BUS_UPD,   // Bus Update (for cache-to-cache transfer)
    INVALIDATE, // Invalidate request
    WRITEBACK   // Writeback to memory
};

// Cache line states for MESI protocol
enum class CacheLineState {
    M, // Modified
    E, // Exclusive
    S, // Shared
    I  // Invalid
};

// String representation of cache line states
string mesiStateToString(CacheLineState state) {
    switch (state) {
        case CacheLineState::M: return "M";
        case CacheLineState::E: return "E";
        case CacheLineState::S: return "S";
        case CacheLineState::I: return "I";
        default: return "?";
    }
}

// Bus transaction request
struct BusTransaction {
    BusOp operation;
    uint32_t address;
    int sourceCore;
    bool serviced; // Whether someone has responded to this transaction
    
    BusTransaction(BusOp op, uint32_t addr, int core) 
        : operation(op), address(addr), sourceCore(core), serviced(false) {}
    
};

// Structure to represent a cache line
struct CacheLine {
    bool valid;                   // Is the line valid?
    bool dirty;                   // Is the line dirty (Modified)?
    uint32_t tag;                 // Tag bits
    CacheLineState state;         // MESI state
    uint32_t lru_counter;         // LRU counter (higher = more recently used)
    vector<uint8_t> data;         // Actual data stored in cache line

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

// Forward declaration of Bus class
class CacheSimulator; 
class Bus;

// L1 Cache implementation
class L1Cache {
private:
    int coreId;          // Core ID (0-3)
    int s;              // Number of set index bits
    int E;              // Associativity (lines per set)
    int b;              // Number of block bits
    int S;              // Number of sets (2^s)
    int B;              // Block size in bytes (2^b)
    int W;              // Words per block (B/4)
    shared_ptr<Bus> bus; // Pointer to the shared bus
    
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
    uint64_t execution_cycles;
    uint64_t invalidations;
    uint64_t bus_transfers;
    uint64_t data_traffic_bytes;
    
    // Current pending request
    bool has_pending_request;
    bool is_waiting_for_bus;
    uint32_t pending_address;
    bool pending_is_write;
    int pending_response;
    int cache_bus_request; 

public:
    // Constructor
    
    L1Cache(int core_id, int set_bits, int associativity, int block_bits) 
        : coreId(core_id), s(set_bits), E(associativity), b(block_bits),
          accesses(0), hits(0), misses(0), evictions(0), reads(0), writes(0),
          writebacks(0), total_cycles(0), idle_cycles(0), invalidations(0),
          bus_transfers(0), data_traffic_bytes(0),
          has_pending_request(false), is_waiting_for_bus(false), pending_response(0), cache_bus_request(0) {
        
        S = 1 << s;  // 2^s
        B = 1 << b;  // 2^b
        W = B / 4;   // Words per block (assuming 4-byte words)
        
        // Initialize cache sets
        sets.reserve(S);
        for (int i = 0; i < S; i++) {
            sets.emplace_back(E, B);
        }
    }

    // Set bus reference
    void setBus(shared_ptr<Bus> b) {
        bus = b;
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
    int cacheBusrequest(){
        return cache_bus_request;
    }
    
    // Get block address (remove offset bits)
    uint32_t getBlockAddress(uint32_t address) const {
        return address & ~((1 << b) - 1);
    }
    void modifytotalcycles(uint64_t cycles) {
        total_cycles = cycles;
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

    // Snoop the bus for transactions from other cores
    int snoopBus(const BusTransaction& trans, uint64_t current_cycle);

    // Process a memory access (read or write)
    bool accessMemory(bool isWrite, uint32_t address, uint64_t current_cycle);

    // Continue processing a pending memory access
    bool continuePendingAccess(uint64_t current_cycle);

    // Calculate transfer cycles
    int calculateTransferCycles(int numWords) const {
        return 2 * numWords;  // 2 cycles per word
    }

    // Print statistics
    void printStats(ostream& os = cout) {
        os << "\nCache Statistics for Core " << coreId << ":" << endl;
        os << "  Total Instructions: " << accesses << endl;
        os << "  Total Reads: " << reads << endl;
        os << "  Total Writes: " << writes << endl;
        os << "  Total Execution Cycles: " << accesses << endl;
        os << "  Idle Cycles: " << idle_cycles << endl;
        os << "  Cache Misses: " << misses << endl;
        os << "  Cache Miss Rate: " << (accesses > 0 ? (100.0 * misses / accesses) : 0) << "%" << endl;
        os << "  Cache Evictions: " << evictions << endl;
        os << "  Writebacks: " << writebacks << endl;
        os << "  Bus Invalidations: " << invalidations << endl;
        os << "  Data Traffic (Bytes): " << data_traffic_bytes << endl;    
    }

    // Print current cache state (for debugging)
    void printCacheState() {
        cout << "\nCurrent Cache State for Core " << coreId << ":\n";
        cout << "==========================================\n";
        for (size_t i = 0; i < sets.size(); i++) {
            bool setUsed = false;
            for (int j = 0; j < E; j++) {
                if (sets[i].lines[j].valid) {
                    setUsed = true;
                    break;
                }
            }
            if (!setUsed) continue;
            
            cout << "Set " << i << ":\n";
            for (int j = 0; j < E; j++) {
                const auto& line = sets[i].lines[j];
                cout << "  Line " << j << ": ";
                if (line.valid) {
                    cout << "Valid, Tag=0x" << hex << line.tag << dec;
                    cout << ", State=" << mesiStateToString(line.state);
                    cout << ", Dirty=" << line.dirty;
                    cout << ", LRU=" << line.lru_counter;
                } else {
                    cout << "Invalid";
                }
                cout << "\n";
            }
        }
        cout << "==========================================\n";
    }

    // Getter methods
    uint64_t getAccesses() const { return accesses; }
    uint64_t getHits() const { return hits; }
    uint64_t getMisses() const { return misses; }
    uint64_t getEvictions() const { return evictions; }
    uint64_t getReads() const { return reads; }
    uint64_t getWrites() const { return writes; }
    uint64_t getWritebacks() const { return writebacks; }
    uint64_t getTotalCycles() const { return total_cycles; }
    uint64_t getIdleCycles() const { return idle_cycles; }
    uint64_t getExecutionCycles() const { return execution_cycles; }
    uint64_t getInvalidations() const { return invalidations; }
    uint64_t getDataTrafficBytes() const { return data_traffic_bytes; }
    double getMissRate() const { return (accesses > 0) ? (100.0 * misses / accesses) : 0; }
    bool hasPendingRequest() const { return has_pending_request; }
    bool isBlocked() const { return has_pending_request; }
    int getBlockBits() const { return b; }
    int getCoreId() const { return coreId; }
    
    void setPendingResponse(int resp) { 
        pending_response = resp; 
    }
    
    int getPendingResponse() const { 
        return pending_response; 
    }
    
    void markPendingRequestDone() {
        has_pending_request = false;
        is_waiting_for_bus = false;
    }
};

// Bus implementation for inter-cache communication
class Bus {
private:
    vector<L1Cache*> caches;
    bool simulation_running;
    uint64_t bus_busy_until; 
    queue<BusTransaction> pending_requests;
    int last_serviced_core;

public:
    CacheSimulator* simulator;
    
    Bus() : simulation_running(true), bus_busy_until(0), last_serviced_core(-1) {}

    // Register a cache with the bus
    void registerCache(L1Cache* cache) {
        caches.push_back(cache);
    }
    
    void addPendingRequest(BusTransaction trans) {
        pending_requests.push(trans);
    }
    
    L1Cache* getCache(int coreId) {
        if (coreId >= 0 && coreId < (int)caches.size()) {
            return caches[coreId];
        }
        return nullptr;
    }
    
    bool hasPendingRequests() {
        return !pending_requests.empty();
    }
    int getNextPendingRequestCore() {
        if (pending_requests.empty()) {
            return -1;
        }
        return pending_requests.front().sourceCore;
    }
    
    BusTransaction popPendingRequest() {
        BusTransaction trans = pending_requests.front();
        pending_requests.pop();
        last_serviced_core = trans.sourceCore;
        return trans;
    }
    
    // Process the next pending request if available and bus is free
    bool processNextRequest(uint64_t current_cycle) {
        if (pending_requests.empty() || current_cycle < bus_busy_until) {
            return false;
        }
        
        BusTransaction trans = popPendingRequest();
        int sourceCore = trans.sourceCore;
        L1Cache* requester = getCache(sourceCore);
        
        if (!requester) {
            return false;
        }
        
        // Broadcast to all caches and check if any can service the request
        int response = 0;
        for (auto cache : caches) {
            if (cache != requester) {
                int result = cache->snoopBus(trans, current_cycle);
                if (result > 0) {
                    response = result;
                    trans.serviced = true;
                }
            }
        }
        
        // Store the response for the requesting cache
        requester->setPendingResponse(response);
        
        // Calculate how long the bus will be busy
        int block_size = 1 << requester->getBlockBits();
        int words_per_block = block_size / 4;
        
        if (trans.operation == BusOp::BUS_RD || trans.operation == BusOp::BUS_RDX) {
            if (response == 1) {
                // Cache-to-cache transfer: 2N cycles (where N is words per block)
                bus_busy_until = current_cycle + (2 * words_per_block);
            } else if (response == 3 || response == 0) {
                // Memory access: 100 cycles
                bus_busy_until = current_cycle + 100;
            } else if (response == 2) {
                // Special cache-to-cache transfer
                bus_busy_until = current_cycle + 200;
            }
        } else if (trans.operation == BusOp::WRITEBACK) {
            // Writeback to memory takes 100 cycles
            bus_busy_until = current_cycle + 100;
        } else if (trans.operation == BusOp::INVALIDATE) {
            // Invalidations are quick
            bus_busy_until = current_cycle;
        }
        
        return true;
    }
    
    // Check if there are pending requests from a specific core
    bool hasTransactionFromCore(int coreId) const {
        queue<BusTransaction> q = pending_requests;  // Copy to preserve original
        
        while (!q.empty()) {
            const BusTransaction& trans = q.front();
            if (trans.sourceCore == coreId) {
                return true;
            }
            q.pop();
        }
        return false;
    }
    
    // Broadcast invalidation to all caches except requester
    void invalidateAll(uint32_t address, int requesterCore, uint64_t current_cycle) {
        for (auto cache : caches) {
            if (cache->getCoreId() != requesterCore) {
                BusTransaction invalidateTrans(BusOp::INVALIDATE, address, requesterCore);
                cache->snoopBus(invalidateTrans, current_cycle);
            }
        }
    }
    
    // Get when bus will be free
    uint64_t getBusAvailableCycle() const {
        return bus_busy_until;
    }

    // Check if bus is busy
    bool isBusy(uint64_t current_cycle) const {
        return current_cycle < bus_busy_until;
    }
    
    // Stop the simulation
    void stopSimulation() {
        simulation_running = false;
    }
};

// Implementation of L1Cache::snoopBus
int L1Cache::snoopBus(const BusTransaction& trans, uint64_t current_cycle) {
    // We only care about transactions from other cores
    if (trans.sourceCore == coreId) {
        return 0;
    }

    uint32_t address = trans.address;
    uint32_t tag = getTag(address);
    uint32_t setIndex = getSetIndex(address);
    int lineIndex = findLine(setIndex, tag);

    // If we don't have the block, nothing to do
    if (lineIndex == -1) {
        return 0;
    }

    CacheLine& line = sets[setIndex].lines[lineIndex];
    // CacheLineState oldState = line.state;
    int respondToTransaction = 0;

    switch (trans.operation) {
        case BusOp::BUS_RD:
            // Another core is reading
            if (line.state == CacheLineState::M) {
                // Need to provide data and change to Shared
                line.state = CacheLineState::S;
                line.dirty = false;
                writebacks++;
                data_traffic_bytes += 2*B;
                bus_transfers++;
                
                // Signal that we're responding with data
                respondToTransaction = 1;
            } else if (line.state == CacheLineState::E) {
                // Change Exclusive to Shared
                line.state = CacheLineState::S;
                data_traffic_bytes += B;
                respondToTransaction = 1;
            } else if (line.state == CacheLineState::S) {
                // Already shared, just indicate we have it
                data_traffic_bytes += B;
                respondToTransaction = 1;
            }
            break;

        case BusOp::BUS_RDX:
            // Another core wants exclusive access
            if (line.state == CacheLineState::M) {
                // Need to writeback and invalidate
                line.state = CacheLineState::I;
                line.valid = false;
                line.dirty = false;
                writebacks++;
                data_traffic_bytes += B;
                respondToTransaction = 2;
            } else if (line.state == CacheLineState::E || line.state == CacheLineState::S) {
                // Just invalidate
                line.state = CacheLineState::I;
                line.valid = false;
                respondToTransaction = 3;
            }
            break;

        case BusOp::INVALIDATE:
            // Another core requests invalidation
            if (line.state != CacheLineState::I) {
                line.state = CacheLineState::I;
                line.valid = false;
            }
            break;
            
        case BusOp::BUS_UPD:
        case BusOp::WRITEBACK:
            // Nothing to do for these operations when snooping
            break;
    }
    
    return respondToTransaction;
}

// Implementation of L1Cache::accessMemory
bool L1Cache::accessMemory(bool isWrite, uint32_t address, uint64_t current_cycle) {
    // If we're already processing a request, can't take a new one
    if (has_pending_request) {
        return false;
    }
    
    accesses++;
    
    uint32_t tag = getTag(address);
    uint32_t setIndex = getSetIndex(address);

    // Find if the block is in cache
    int lineIndex = findLine(setIndex, tag);

    if (lineIndex != -1) {
        // Cache hit
        sets[setIndex].updateLRU(lineIndex);
        CacheLine& line = sets[setIndex].lines[lineIndex];

        // Handle MESI state transitions for hit
        if (isWrite) {
            writes++;
            switch (line.state) {
                case CacheLineState::M:
                    // Already modified, nothing to do
                    hits++;
                    break;
                    
                case CacheLineState::E:
                    // Exclusive to Modified
                    line.state = CacheLineState::M;
                    line.dirty = true;
                    hits++;
                    break;
                    
                case CacheLineState::S:{
                    // Need to invalidate other copies
                    has_pending_request = true;
                    pending_address = address;
                    pending_is_write = isWrite;
                    is_waiting_for_bus = true;
                    
                    // Add invalidation request to bus
                    BusTransaction trans(BusOp::INVALIDATE, getBlockAddress(address), coreId);
                    bus->addPendingRequest(trans);
                    cache_bus_request++;
                    invalidations++;
                    
                    // Will be processed later - not a complete hit yet
                    return false;
                }
                    
                case CacheLineState::I:
                    // Should not happen
                    cerr << "Error: Invalid state on hit!" << endl;
                    break;
            }
        } else {
            // Read hit
            reads++;
            hits++;
        }
        
        execution_cycles++;
        return true;
    } else {
        // Cache miss
        idle_cycles++;
        misses++;
        data_traffic_bytes += B;
        
        if (isWrite) {
            invalidations++;
            writes++;
        } else {
            reads++;
        }
        
        // Set up for miss handling
        has_pending_request = true;
        pending_address = address;
        pending_is_write = isWrite;
        is_waiting_for_bus = true;
        
        // Check for potential eviction
        int evictionLineIndex = -1;
        bool needEviction = true;
        
        // First, look for an invalid line
        for (int i = 0; i < E; i++) {
            if (!sets[setIndex].lines[i].valid) {
                evictionLineIndex = i;
                needEviction = false;
                break;
            }
        }
        
        // If no empty line, use LRU replacement policy
        if (needEviction) {
            evictionLineIndex = sets[setIndex].findLRULine();
            CacheLine& line = sets[setIndex].lines[evictionLineIndex];
            
            if (line.valid) {
                evictions++;
                // If dirty, need to write back to memory
                if (line.dirty) {
                    writebacks++;
                    data_traffic_bytes += B;
                    uint32_t evictionAddress = (line.tag << (s + b)) | (setIndex << b);
                    
                    // Queue the writeback request first
                    BusTransaction wbTrans(BusOp::WRITEBACK, evictionAddress, coreId);
                    bus->addPendingRequest(wbTrans);
                    cache_bus_request++;
                }
            }
        }
        
        // Queue the read request
        BusOp busOp = isWrite ? BusOp::BUS_RDX : BusOp::BUS_RD;
        BusTransaction readTrans(busOp, getBlockAddress(address), coreId);
        bus->addPendingRequest(readTrans);
        cache_bus_request++;
        
        return false;  // Request not completed yet
    }
}

// Implementation of L1Cache::continuePendingAccess
bool L1Cache::continuePendingAccess(uint64_t current_cycle) {
    if (!has_pending_request) {
        return true;  // No pending request
    }
    
    // Wait for bus to be available
    if (bus->isBusy(current_cycle)) {
        idle_cycles++;
        return false;
    }
    
    // Process request if we're next in line
    if (bus->hasPendingRequests() && bus->getNextPendingRequestCore() == coreId) {
        if (!bus->processNextRequest(current_cycle)) {
            idle_cycles++;
            return false;
        }
    }
    
    // Check if we still need to wait (might need multiple bus transactions)
    if (bus->isBusy(current_cycle) || (bus->hasPendingRequests() && bus->getNextPendingRequestCore() == coreId)) {
        idle_cycles++;
        return false;
    }
    
    uint32_t address = pending_address;
    bool isWrite = pending_is_write;
    uint32_t tag = getTag(address);
    uint32_t setIndex = getSetIndex(address);
    
    // For a shared-to-modified transition (write hit to shared line)
    int lineIndex = findLine(setIndex, tag);
    if (lineIndex != -1 && isWrite && sets[setIndex].lines[lineIndex].state == CacheLineState::S) {
        // This was a write hit to a shared line, now we've invalidated others
        CacheLine& line = sets[setIndex].lines[lineIndex];
        line.state = CacheLineState::M;
        line.dirty = true;
        sets[setIndex].updateLRU(lineIndex);
        has_pending_request = false;
        is_waiting_for_bus = false;
        hits++;
        return true;
    }
    
    // For misses, handle cache line allocation
    int newLineIndex = -1;
    bool foundEmpty = false;
    
    // First, look for an invalid line
    for (int i = 0; i < E; i++) {
        if (!sets[setIndex].lines[i].valid) {
            newLineIndex = i;
            foundEmpty = true;
            break;
        }
    }
    
    // If no empty line, use LRU replacement
    if (!foundEmpty) {
        newLineIndex = sets[setIndex].findLRULine();
    }
    
    // Load the new line
    CacheLine& newLine = sets[setIndex].lines[newLineIndex];
    newLine.valid = true;
    newLine.tag = tag;
    newLine.dirty = isWrite;
    
    // Set appropriate MESI state based on the response from other caches
    int response = pending_response;
    if (isWrite) {
        newLine.state = CacheLineState::M;
    } else {
        if (response > 0) {  // Someone else has this line
            newLine.state = CacheLineState::S;
            bus_transfers++;
        } else {  // No one else has this line
            newLine.state = CacheLineState::E;
        }
    }
    
    // Update LRU status
    sets[setIndex].updateLRU(newLineIndex);
    
    // Request complete
    execution_cycles++;
    has_pending_request = false;
    is_waiting_for_bus = false;
    pending_response = 0;
    
    return true;
}

bool parseArgs(int argc, char* argv[], string& tracePrefix, int& s, int& E, int& b, string& outFilename) {
    int opt;
    while ((opt = getopt(argc, argv, "t:s:E:b:o:h")) != -1) {
        switch (opt) {
            case 't': tracePrefix = optarg; break;
            case 's': s = stoi(optarg); break;
            case 'E': E = stoi(optarg); break;
            case 'b': b = stoi(optarg); break;
            case 'o': outFilename = optarg; break;
            case 'h':
                cout << "Usage: " << argv[0] << " -t <tracefile> -s <s> -E <E> -b <b> [-o <outfilename>] [-h]\n";
                cout << "-t <tracefile>: prefix of trace files (e.g., app1)\n";
                cout << "-s <s>: number of set index bits (S = 2^s)\n";
                cout << "-E <E>: associativity (lines per set)\n";
                cout << "-b <b>: number of block bits (B = 2^b)\n";
                cout << "-o <file>: output stats to file\n";
                cout << "-h: show this help\n";
                return false;
            default:
                cerr << "Unknown argument. Use -h for help.\n";
                return false;
        }
    }

    if (tracePrefix.empty() || s <= 0 || E <= 0 || b <= 0) {
        cerr << "Missing required arguments. Use -h for help.\n";
        return false;
    }

    return true;
}

class CacheSimulator {
private:
    vector<unique_ptr<L1Cache>> caches;
    shared_ptr<Bus> bus;
    vector<ifstream> traceFiles;
    vector<bool> coreFinished;
    vector<string> currentOps;
    vector<uint32_t> currentAddrs;

    string tracePrefix;
    int s, E, b;
    uint64_t cycle_count = 0;

public:
    CacheSimulator(const string& prefix, int setBits, int assoc, int blockBits)
        : coreFinished(NUM_CORES, false), currentOps(NUM_CORES), currentAddrs(NUM_CORES, 0),tracePrefix(prefix), s(setBits), E(assoc), b(blockBits) {

        bus = make_shared<Bus>();
        bus->simulator = this;

        for (int i = 0; i < NUM_CORES; i++) {
            caches.push_back(make_unique<L1Cache>(i, s, E, b));
            caches[i]->setBus(bus);
            bus->registerCache(caches[i].get());

            string filename = tracePrefix + "_proc" + to_string(i) + ".trace";
            traceFiles.emplace_back(filename);
            if (!traceFiles[i]) {
                cerr << "Failed to open trace file: " << filename << endl;
                coreFinished[i] = true;
            }
        }
    }

    void readNextInstruction(int coreId) {
        string op, addrStr;
        if (traceFiles[coreId] >> op >> addrStr) {
            currentOps[coreId] = (op == "W" || op == "w") ? "W" : "R";
            currentAddrs[coreId] = stoul(addrStr, nullptr, 16);
        } else {
            coreFinished[coreId] = true;
            caches[coreId]->modifytotalcycles(cycle_count);
        }
    }

    void runSimulation() {
        for (int i = 0; i < NUM_CORES; i++) {
            if (!coreFinished[i]) readNextInstruction(i);
        }

        while (true) {
            bool allDone = true;
            cycle_count++;

            for (int i = 0; i < NUM_CORES; i++) {
                if (coreFinished[i]) continue;
                allDone = false;

                if (caches[i]->isBlocked()) {
                    if (caches[i]->continuePendingAccess(cycle_count)) {
                        readNextInstruction(i);
                    }
                } else {
                    bool isWrite = (currentOps[i] == "W");
                    if (caches[i]->accessMemory(isWrite, currentAddrs[i], cycle_count)) {
                        readNextInstruction(i);
                    }
                }
            }

            bus->processNextRequest(cycle_count);
            if (allDone) break;
        }

        cout << "Simulation completed in " << cycle_count << " cycles\n";
    }

    void printFinalStats(const string& outFilename) {
        uint64_t total_invalidations = 0, total_traffic = 0, total_bus_requests = 0;
        uint64_t maxExecTime = 0;
            for (int i = 0; i < NUM_CORES; i++) {
                maxExecTime = max(maxExecTime, caches[i]->getTotalCycles());
            }
            
        if (!outFilename.empty()) {
            ofstream out(outFilename);
            out<< "Simulation Parameters:"
            << "\nTrace Prefix: " << tracePrefix
            << "\nSet Index Bits: " << s
            << "\nAssociativity: " << E
            << "\nBlock Bits: " << b
            << "\nBlock Size (Bytes): " << (1 << b)
            << "\nNumber of Sets: " << (1 << s)
            << "\nCache Size (KB per core): " << ((1 << s) * E * (1 << b))/1024 
            << "\nMESI Protocol: Enabled"
            << "\nWrite Policy: Write-back, Write-allocate"
            << "\nReplacement Policy: LRU"
            << "\nBus: Central snooping bus\n";
            for (auto& cache : caches) {
                cache->printStats(out);
                total_invalidations += cache->getInvalidations();
                total_traffic += cache->getDataTrafficBytes();
                total_bus_requests += cache->cacheBusrequest();
            }
            out << "\nOverall Bus Summary:"
                << "\nTotal Bus Transactions: " << total_bus_requests
                << "\nTotal Bus Traffic (Bytes): " << total_traffic
                << "\nMaximum Execution Time:" << maxExecTime << endl;
            out.close();
        }
    }
};

int main(int argc, char* argv[]) {
    string tracePrefix, outFile;
    int s = 0, E = 0, b = 0;

    if (!parseArgs(argc, argv, tracePrefix, s, E, b, outFile)) {
        return 1;
    }

    CacheSimulator sim(tracePrefix, s, E, b);
    sim.runSimulation();
    sim.printFinalStats(outFile);

    return 0;
}
