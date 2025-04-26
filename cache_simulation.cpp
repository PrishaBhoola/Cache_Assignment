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
    uint64_t invalidations;
    uint64_t bus_transfers;
    uint64_t data_traffic_bytes;
    
    // Current pending request
    bool has_pending_request;
    bool is_waiting_for_bus;
    uint32_t pending_address;
    bool pending_is_write;
    bool pending_was_shared; // For write hits to shared lines

public:
    // Constructor
    L1Cache(int core_id, int set_bits, int associativity, int block_bits) 
        : coreId(core_id), s(set_bits), E(associativity), b(block_bits),
          accesses(0), hits(0), misses(0), evictions(0), reads(0), writes(0),
          writebacks(0), total_cycles(0), idle_cycles(0), invalidations(0),
          bus_transfers(0), data_traffic_bytes(0),
          has_pending_request(false), is_waiting_for_bus(false) {
        
        S = 1 << s;  // 2^s
        B = 1 << b;  // 2^b
        W = B / 4;   // Words per block (assuming 4-byte words)
        
        // Initialize cache sets
        sets.reserve(S);
        for (int i = 0; i < S; i++) {
            sets.emplace_back(E, B);
        }
        
        cout << "Created L1 Cache for Core " << coreId << " with:" << endl;
        cout << "  Sets: " << S << " (2^" << s << ")" << endl;
        cout << "  Associativity: " << E << endl;
        cout << "  Block size: " << B << " bytes (2^" << b << ")" << endl;
        cout << "  Cache size: " << (S * E * B) / 1024.0 << " KB" << endl;
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
    
    // Get block address (remove offset bits)
    uint32_t getBlockAddress(uint32_t address) const {
        return address & ~((1 << b) - 1);
    }

    void incrementIdleCycles() {
        idle_cycles++;
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
    bool snoopBus(const BusTransaction& trans, uint64_t current_cycle);

    // Process a memory access (read or write)
    bool accessMemory(bool isWrite, uint32_t address, uint64_t current_cycle);

    // Continue processing a pending memory access
    bool continuePendingAccess(uint64_t current_cycle);

    // Get cycle count for cache to cache transfers
    int calculateTransferCycles(int numWords) const {
        return 2 * numWords;  // 2 cycles per word
    }

    // Print statistics
    void printStats() {
        cout << "\nCache Statistics for Core " << coreId << ":" << endl;
        cout << "  Total Accesses: " << accesses << endl;
        cout << "  Reads: " << reads << endl;
        cout << "  Writes: " << writes << endl;
        cout << "  Hits: " << hits << endl;
        cout << "  Misses: " << misses << endl;
        cout << "  Evictions: " << evictions << endl;
        cout << "  Writebacks: " << writebacks << endl;
        cout << "  Invalidations: " << invalidations << endl;
        cout << "  Data Traffic (Bytes): " << data_traffic_bytes << endl;
        cout << "  Miss Rate: " << (accesses > 0 ? (100.0 * misses / accesses) : 0) << "%" << endl;
        cout << "  Total Cycles: " << total_cycles << endl;
        cout << "  Idle Cycles: " << idle_cycles << endl;
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
    uint64_t getInvalidations() const { return invalidations; }
    uint64_t getDataTrafficBytes() const { return data_traffic_bytes; }
    double getMissRate() const { return (accesses > 0) ? (100.0 * misses / accesses) : 0; }
    bool hasPendingRequest() const { return has_pending_request; }
    bool isBlocked() const { return has_pending_request; }
    int getBlockBits() const { return b; }
};

// Bus implementation for inter-cache communication
class Bus {
private:
    vector<L1Cache*> caches;
    mutex bus_mutex;
    condition_variable cv;
    queue<BusTransaction> transactions;
    bool simulation_running;
    uint64_t bus_busy_until; // Track when the bus becomes free
    bool bus_in_use;         // Flag to indicate bus is in use

public:
    Bus() : simulation_running(true), bus_busy_until(0), bus_in_use(false) {}

    // Register a cache with the bus
    void registerCache(L1Cache* cache) {
        caches.push_back(cache);
    }

    // Request bus access for a transaction
    // Returns true if the transaction was serviced by another cache
    bool requestBus(BusTransaction trans, L1Cache* requester, uint64_t current_cycle) {
        unique_lock<mutex> lock(bus_mutex);
        
        // Check if bus is busy
        if (bus_in_use || current_cycle < bus_busy_until) {
            return false; // Bus is busy, request not accepted
        }
        
        // Bus is now in use
        bus_in_use = true;
        
        // Add transaction to queue
        transactions.push(trans);
        
        // Broadcast to all caches
        bool serviced = false;
        for (auto cache : caches) {
            if (cache != requester) {
                if (cache->snoopBus(trans, current_cycle)) {
                    trans.serviced = true;
                    serviced = true;
                }
            }
        }
        
        // Calculate how long the bus will be busy
        if (trans.operation == BusOp::BUS_RD || trans.operation == BusOp::BUS_RDX) {
            int block_size = 1 << requester->getBlockBits();
            int words_per_block = block_size / 4;
            
            if (serviced) {
                // Cache-to-cache transfer: 2N cycles (where N is words per block)
                bus_busy_until = current_cycle + (2 * words_per_block);
            } else {
                // Memory access: 100 cycles
                bus_busy_until = current_cycle + 100;
            }
        } else if (trans.operation == BusOp::INVALIDATE) {
            // Invalidation is fast, only 1 cycle
            bus_busy_until = current_cycle + 1;
        } else if (trans.operation == BusOp::WRITEBACK) {
            // Writeback to memory takes 100 cycles
            bus_busy_until = current_cycle + 100;
        }
        
        // Notify all waiting threads
        cv.notify_all();
        
        return serviced;
    }

    // Mark the bus as free
    void releaseBus(uint64_t current_cycle) {
        unique_lock<mutex> lock(bus_mutex);
        // Simply mark the bus as free regardless of timing
        // Since we check isBusy() elsewhere using bus_busy_until
        bus_in_use = false;
        
        // If there are transactions in queue, pop the completed one
        if (!transactions.empty()) {
            transactions.pop();
        }
        cv.notify_all();
    }

    // Check if bus is busy
    bool isBusy(uint64_t current_cycle) const {
        return bus_in_use || current_cycle < bus_busy_until;
    }

    // Get when bus will be free
    uint64_t getBusAvailableCycle() const {
        return bus_busy_until;
    }

    // Stop the simulation
    void stopSimulation() {
        unique_lock<mutex> lock(bus_mutex);
        simulation_running = false;
        cv.notify_all();
    }
    };

// Implementation of L1Cache::snoopBus
bool L1Cache::snoopBus(const BusTransaction& trans, uint64_t current_cycle) {

    cout << "[SNOOP] Core " << coreId 
    << " sees bus operation from Core " << trans.sourceCore 
    << " for address 0x" << hex << trans.address << dec << endl;

    // We only care about transactions from other cores
    if (trans.sourceCore == coreId) {
        return false;
    }

    uint32_t address = trans.address;
    uint32_t tag = getTag(address);
    uint32_t setIndex = getSetIndex(address);
    int lineIndex = findLine(setIndex, tag);

    // If we don't have the block, nothing to do
    if (lineIndex == -1) {
        cout << "[SNOOP] Core " << coreId 
            << " has no line for set " << setIndex 
            << " tag 0x" << hex << getTag(address) << dec << endl;
        return false;
    }

    CacheLine& line = sets[setIndex].lines[lineIndex];
    CacheLineState oldState = line.state;

    cout << "[SNOOP] Core " << coreId << " snooping " 
    << static_cast<int>(trans.operation) << " for address 0x" 
    << hex << address << dec 
    << " — Line State: " << mesiStateToString(line.state) << endl;

    bool respondToTransaction = false;

    switch (trans.operation) {
        case BusOp::BUS_RD:
            // Another core is reading
            if (line.state == CacheLineState::M) {
                // Need to provide data and change to Shared
                line.state = CacheLineState::S;
                line.dirty = false;
                
                // Cache-to-cache transfer
                data_traffic_bytes += B;
                total_cycles += calculateTransferCycles(W);
                
                // Record bus transfer
                bus_transfers++;
                respondToTransaction = true;
            } else if (line.state == CacheLineState::E) {
                // Change Exclusive to Shared
                line.state = CacheLineState::S;
                respondToTransaction = true;
            }
            break;

        case BusOp::BUS_RDX:
            if (line.state != CacheLineState::I) {
                CacheLineState prevState = line.state;  // Store state before invalidating
                line.state = CacheLineState::I;
                line.valid = false;
                invalidations++;
        
                if (prevState == CacheLineState::M) {
                    data_traffic_bytes += B;
                    total_cycles += calculateTransferCycles(W);
                    bus_transfers++;
                }
                respondToTransaction = true;
            }
            break;

        case BusOp::INVALIDATE:
            // Another core requests invalidation
            if (line.state != CacheLineState::I) {
                line.state = CacheLineState::I;
                line.valid = false;
                invalidations++;
                respondToTransaction = true;
            }
            break;
            
        case BusOp::BUS_UPD:
        case BusOp::WRITEBACK:
            // Nothing to do for these operations when snooping
            break;
    }
    if (oldState != line.state) {
        cout << "[SNOOP] Core " << coreId 
            << " changed state from " << mesiStateToString(oldState)
            << " to " << mesiStateToString(line.state) 
            << " on " << static_cast<int>(trans.operation) << endl;
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
    total_cycles++;  // Always takes at least 1 cycle
    
    if (isWrite) {
        writes++;
    } else {
        reads++;
    }

    uint32_t tag = getTag(address);
    uint32_t setIndex = getSetIndex(address);
    // uint32_t blockOffset = getBlockOffset(address);  // Will be needed for actual data access

    // Find if the block is in cache
    int lineIndex = findLine(setIndex, tag);

    if (lineIndex != -1) {
        // Cache hit
        hits++;
        
        // Update LRU status for this line
        sets[setIndex].updateLRU(lineIndex);
        
        CacheLine& line = sets[setIndex].lines[lineIndex];
        CacheLineState oldState = line.state;

        cout << "[ACCESS] Core " << coreId 
        << " HIT on 0x" << hex << address << dec 
        << " — state = " << mesiStateToString(line.state) << endl;
        
        // Handle MESI state transitions for hit
        if (isWrite) {
            cout << "[WRITE] Core " << coreId 
                    << " writing to 0x" << hex << address << dec 
                    << " — line state = " 
                    << mesiStateToString(line.state) << endl;
            switch (line.state) {
                case CacheLineState::M:
                    // Already modified, nothing to do
                    break;
                    
                case CacheLineState::E:
                    // Exclusive to Modified
                    line.state = CacheLineState::M;
                    line.dirty = true;
                    break;
                    
                case CacheLineState::S:
                    {// Shared to Modified, need to invalidate others
                    if (bus->isBusy(current_cycle)) {
                        // Bus is busy, can't proceed
                        return false;
                    }
                    has_pending_request = true;
                    pending_address = address;
                    pending_is_write = isWrite;
                    is_waiting_for_bus = true;
                    
                    // Send invalidate message on bus
                    BusTransaction trans(BusOp::INVALIDATE, getBlockAddress(address), coreId);
                    pending_was_shared = bus->requestBus(trans, this, current_cycle);
                    }
                    // Will complete in continuePendingAccess
                    return false;
                    
                case CacheLineState::I:
                    // Should not happen
                    cerr << "Error: Invalid state on hit!" << endl;
                    break;
            }
            if (oldState != line.state) {
                cout << "[STATE] Core " << coreId 
                    << " changed state from " << mesiStateToString(oldState)
                    << " to " << mesiStateToString(line.state) << endl;
            }            
        }
        
        // Hit was handled completely
        return true;
        
    } else {
        // Cache miss
        misses++;
        
        // BusTransaction trans(isWrite ? BusOp::BUS_RDX : BusOp::BUS_RD, getBlockAddress(address), coreId);
        // bus->requestBus(trans, this);
        if (bus->isBusy(current_cycle)) {
            // Bus is busy, can't proceed with miss handling
            idle_cycles++;
            return false;
        }

        // Set up pending request
        has_pending_request = true;
        pending_address = address;
        pending_is_write = isWrite;
        is_waiting_for_bus = true;
        // pending_was_shared = trans.serviced;
        BusOp busOp = isWrite ? BusOp::BUS_RDX : BusOp::BUS_RD;
        BusTransaction trans(busOp, getBlockAddress(address), coreId);
        pending_was_shared = bus->requestBus(trans, this, current_cycle);       
        // Issue appropriate bus request
        // if (isWrite) {
        //     // Write miss: BusRdX
        //     bus->requestBus(BusTransaction(BusOp::BUS_RDX, getBlockAddress(address), coreId), this);
        // } else {
        //     // Read miss: BusRd
        //     bus->requestBus(BusTransaction(BusOp::BUS_RD, getBlockAddress(address), coreId), this);
        // }
        cout << "[MISS] Core " << coreId 
            << (isWrite ? " write" : " read") << " miss on 0x" 
            << hex << address << dec 
            << " — issuing " << (isWrite ? "BUS_RDX" : "BUS_RD") << endl;        
        // Wait for bus transaction to complete
        return false;  // Request not completed yet
    }
}

// Implementation of L1Cache::continuePendingAccess
bool L1Cache::continuePendingAccess(uint64_t current_cycle) {
        if (!has_pending_request) {
            return true;  // No pending request
        }
        
        if (current_cycle < bus->getBusAvailableCycle()) {
            // Still waiting for bus/transfer to complete
            idle_cycles++;
            return false;
        }
        
        uint32_t address = pending_address;
        bool isWrite = pending_is_write;
        uint32_t tag = getTag(address);
        uint32_t setIndex = getSetIndex(address);
        bus->releaseBus(current_cycle);
        // For a shared-to-modified transition (from a write hit)
        int lineIndex = findLine(setIndex, tag);
        if (lineIndex != -1 && !pending_was_shared) {
            // This was a write hit to a shared line
            CacheLine& line = sets[setIndex].lines[lineIndex];
            CacheLineState oldState = line.state;
            
            line.state = CacheLineState::M;
            line.dirty = true;
            
            cout << "[STATE] Core " << coreId 
                << " changed state from " << mesiStateToString(oldState)
                << " to " << mesiStateToString(line.state) 
                << " after invalidation" << endl;
            
            has_pending_request = false;
            return true;
        }
        
        // For actual misses, handle cache line allocation
        
        // Find a place to put this block
        bool foundEmpty = false;
        lineIndex = -1;
        
        // First, look for an invalid line
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
            CacheLine& line = sets[setIndex].lines[lineIndex];
            if (line.valid) {
                evictions++;
                
                // If dirty, need to write back to memory
                if (line.dirty) {
                    writebacks++;
                    data_traffic_bytes += B;
                    
                    uint32_t evictionAddress = (line.tag << (s + b)) | (setIndex << b);
                    
                    cout << "[EVICT] Core " << coreId 
                        << " evicting 0x" << hex << evictionAddress << dec 
                        << " — dirty=" << line.dirty << endl;
                    
                    // Broadcast writeback on bus
                    BusTransaction wbTrans(BusOp::WRITEBACK, evictionAddress, coreId);
                    if (bus->isBusy(current_cycle)) {
                        // Need to try again later
                        return false;
                    }
                    bus->requestBus(wbTrans, this, current_cycle);
                }
            }
        }
        
        // Load the new line
        CacheLine& newLine = sets[setIndex].lines[lineIndex];
        newLine.valid = true;
        newLine.tag = tag;
        newLine.dirty = isWrite;  // Set dirty if write
        
        // Set appropriate MESI state
        if (isWrite) {
            newLine.state = CacheLineState::M;
        } else {
            if (pending_was_shared) {
                newLine.state = CacheLineState::S;
                cout << "[TRANSFER] Core " << coreId 
                << " received data from another cache for address 0x" << hex << address << dec << endl;
            } else {
                newLine.state = CacheLineState::E;
                cout << "[MEMORY] Core " << coreId 
                << " loaded data from memory for address 0x" << hex << address << dec << endl;
            }
        }

        cout << "[LOAD] Core " << coreId 
            << " loaded address 0x" << hex << address << dec 
            << " — state=" << mesiStateToString(newLine.state) << endl;
        
        // Update LRU status
        sets[setIndex].updateLRU(lineIndex);
        
        // Account for memory access time
        if (!pending_was_shared) {
            // If from memory
            data_traffic_bytes += B;  // Data from memory to cache
        } else {
            // If from another cache
            bus_transfers++;  // Count as a cache-to-cache transfer
        }
        
        // Request complete
        has_pending_request = false;
        return true;
    }

// Main simulator class to manage all caches and execution
class CacheSimulator {
private:
    vector<unique_ptr<L1Cache>> caches;
    shared_ptr<Bus> bus;
    vector<ifstream> traceFiles;
    vector<bool> coreFinished;
    vector<string> currentOps;
    vector<uint32_t> currentAddrs;
    
    // Parameters
    string tracePrefix;
    int s;
    int E;
    int b;
    
    uint64_t cycle_count;
    
public:
    CacheSimulator(const string& prefix, int setIndexBits, int associativity, int blockBits)
        : tracePrefix(prefix), s(setIndexBits), E(associativity), b(blockBits), cycle_count(0) {
        
        // Create shared bus
        bus = make_shared<Bus>();
        
        // Initialize flags
        coreFinished.resize(NUM_CORES, false);
        currentOps.resize(NUM_CORES);
        currentAddrs.resize(NUM_CORES, 0);
        
        // Create caches for each core
        for (int i = 0; i < NUM_CORES; i++) {
            caches.push_back(make_unique<L1Cache>(i, s, E, b));
            caches[i]->setBus(bus);
            bus->registerCache(caches[i].get());
        }
        
        // Open trace files
        for (int i = 0; i < NUM_CORES; i++) {
            string filename = tracePrefix + "_proc" + to_string(i) + ".trace";
            traceFiles.emplace_back(filename);
            
            if (!traceFiles[i].is_open()) {
                cerr << "Failed to open trace file: " << filename << endl;
                coreFinished[i] = true;
            }
        }
    }
    
    // Run the simulation
    void runSimulation() {
        cout << "\nStarting simulation with trace: " << tracePrefix << endl;
        
        bool allFinished;
        uint64_t current_cycle = 0;
        
        // Initialize: read first instruction for each core
        for (int i = 0; i < NUM_CORES; i++) {
            if (!coreFinished[i]) {
                readNextInstruction(i);
            }
        }
        
        // Main simulation loop
        do {
            cycle_count++;  // Increment global cycle counter
            current_cycle = cycle_count;  // Update current cycle
            
            // Each cycle, try to execute one instruction per core
            for (int i = 0; i < NUM_CORES; i++) {
                // Skip if core is finished
                if (coreFinished[i]) {
                    caches[i]->incrementIdleCycles();
                    continue;
                }
                
                if (caches[i]->isBlocked()) {
                    // Try to continue pending request
                    if (caches[i]->continuePendingAccess(current_cycle)) {
                        // Request completed, read next instruction
                        readNextInstruction(i);
                    }
                } else {
                    // Process the current instruction
                    bool isWrite = (currentOps[i] == "W");
                    if (caches[i]->accessMemory(isWrite, currentAddrs[i], current_cycle)) {
                        // Memory access completed in this cycle, read next instruction
                        readNextInstruction(i);
                    }
                }
            }
            
            // Check if all cores are finished
            allFinished = true;
            for (int i = 0; i < NUM_CORES; i++) {
                if (!coreFinished[i]) {
                    allFinished = false;
                    break;
                }
            }
        } while (!allFinished);
        
        cout << "Simulation completed in " << cycle_count << " cycles" << endl;
    }
    
    // Read the next instruction for a core
    void readNextInstruction(int coreId) {
        if (coreFinished[coreId]) {
            return;
        }
        
        string operation;
        string addressStr;
        
        if (traceFiles[coreId] >> operation >> addressStr) {
            // Convert hex string to uint32_t
            uint32_t address = stoul(addressStr, nullptr, 16);
            
            // Store the instruction for later execution
            if (operation == "R" || operation == "r") {
                currentOps[coreId] = "R";
            } else if (operation == "W" || operation == "w") {
                currentOps[coreId] = "W";
            } else {
                cerr << "Unknown operation type: " << operation << endl;
                currentOps[coreId] = "R";  // Default to read
            }
            currentAddrs[coreId] = address;
            cout << "[TRACE] Core " << coreId 
            << " next operation: " << currentOps[coreId] 
            << " address: 0x" << hex << address << dec << endl;
        } else {
            // No more instructions for this core
            coreFinished[coreId] = true;
        }
    }
    
    // Print final statistics
    void printFinalStats(const string& outFilename = "") {
        cout << "\n===== Final Statistics =====\n";
        cout << "Simulation Parameters:" << endl;
        cout << "Trace Prefix: " << tracePrefix << endl;
        cout << "Set Index Bits: " << s << endl;
        cout << "Associativity: " << E << endl;
        cout << "Block Bits: " << b << endl;
        cout << "Block Size (Bytes): " << (1 << b) << endl;
        cout << "Number of Sets: " << (1 << s) << endl;
        cout << "Cache Size (KB per core): " << ((1 << s) * E * (1 << b)) / 1024.0 << endl;
        cout << "MESI Protocol: Enabled" << endl;
        cout << "Write Policy: Write-back, Write-allocate" << endl;
        cout << "Replacement Policy: LRU" << endl;
        cout << "Bus: Central snooping bus" << endl;
        
        // Calculate total bus statistics
        uint64_t total_invalidations = 0;
        uint64_t total_bus_traffic = 0;
        
        for (int i = 0; i < NUM_CORES; i++) {
            caches[i]->printStats();
            total_invalidations += caches[i]->getInvalidations();
            total_bus_traffic += caches[i]->getDataTrafficBytes();
        }
        
        cout << "\nOverall Bus Summary:" << endl;
        cout << "  Total Bus Invalidations: " << total_invalidations << endl;
        cout << "  Total Bus Traffic (Bytes): " << total_bus_traffic << endl;
        
        // Write to output file if specified
        if (!outFilename.empty()) {
            ofstream outFile(outFilename);
            if (outFile.is_open()) {
                outFile << "Simulation Parameters:" << endl;
                outFile << "Trace Prefix: " << tracePrefix << endl;
                outFile << "Set Index Bits: " << s << endl;
                outFile << "Associativity: " << E << endl;
                outFile << "Block Bits: " << b << endl;
                outFile << "Block Size (Bytes): " << (1 << b) << endl;
                outFile << "Number of Sets: " << (1 << s) << endl;
                outFile << "Cache Size (KB per core): " << ((1 << s) * E * (1 << b)) / 1024.0 << endl;
                outFile << "MESI Protocol: Enabled" << endl;
                outFile << "Write Policy: Write-back, Write-allocate" << endl;
                outFile << "Replacement Policy: LRU" << endl;
                outFile << "Bus: Central snooping bus" << endl;
                
                for (int i = 0; i < NUM_CORES; i++) {
                    outFile << "\nCore " << i << " Statistics:" << endl;
                    outFile << "  Total Instructions: " << caches[i]->getAccesses() << endl;
                    outFile << "  Total Reads: " << caches[i]->getReads() << endl;
                    outFile << "  Total Writes: " << caches[i]->getWrites() << endl;
                    outFile << "  Total Execution Cycles: " << caches[i]->getTotalCycles() << endl;
                    outFile << "  Idle Cycles: " << caches[i]->getIdleCycles() << endl;
                    outFile << "  Cache Misses: " << caches[i]->getMisses() << endl;
                    outFile << "  Cache Miss Rate: " << caches[i]->getMissRate() << "%" << endl;
                    outFile << "  Cache Evictions: " << caches[i]->getEvictions() << endl;
                    outFile << "  Writebacks: " << caches[i]->getWritebacks() << endl;
                    outFile << "  Bus Invalidations: " << caches[i]->getInvalidations() << endl;
                    outFile << "  Data Traffic (Bytes): " << caches[i]->getDataTrafficBytes() << endl;
                }
                
                outFile << "\nOverall Bus Summary:" << endl;
                outFile << "  Total Bus Transactions: " << total_invalidations << endl;
                outFile << "  Total Bus Traffic (Bytes): " << total_bus_traffic << endl;
                
                outFile.close();
            } else {
                cerr << "Failed to open output file: " << outFilename << endl;
            }
        }
    }
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
// void processTraceFile(const string& filename, L1Cache& cache) {
//     ifstream traceFile(filename);
//     if (!traceFile.is_open()) {
//         cerr << "Failed to open trace file: " << filename << endl;
//         return;
//     }

//     string operation;
//     string addressStr;
    
//     while (traceFile >> operation >> addressStr) {
//         // Convert hex string to uint32_t
//         uint32_t address = stoul(addressStr, nullptr, 16);
        
//         // Process the memory operation
//         bool isWrite = (operation == "W");
//         cache.accessMemory(isWrite, address);
//     }
    
//     traceFile.close();
// }

// int main(int argc, char* argv[]) {
//     string tracePrefix;
//     int s = 0;
//     int E = 0;
//     int b = 0;
//     string outFilename;
    
//     // Parse command line arguments
//     if (!parseArgs(argc, argv, tracePrefix, s, E, b, outFilename)) {
//         return 1;
//     }
    
//     // Create the L1 cache
//     L1Cache cache(s, E, b);
    
//     // Process the trace file (for single core)
//     string traceFile = tracePrefix + "_proc0.trace";
//     processTraceFile(traceFile, cache);
    
//     // Print statistics
//     cache.printStats();
    
//     // If an output file is specified, write the statistics to it
//     if (!outFilename.empty()) {
//         ofstream outFile(outFilename);
//         if (outFile.is_open()) {
//             outFile << "Accesses: " << cache.getAccesses() << endl;
//             outFile << "Reads: " << cache.getReads() << endl;
//             outFile << "Writes: " << cache.getWrites() << endl;
//             outFile << "Hits: " << cache.getHits() << endl;
//             outFile << "Misses: " << cache.getMisses() << endl;
//             outFile << "Evictions: " << cache.getEvictions() << endl;
//             outFile << "Writebacks: " << cache.getWritebacks() << endl;
//             outFile << "Miss Rate: " << cache.getMissRate() << "%" << endl;
//             outFile << "Total Cycles: " << cache.getTotalCycles() << endl;
//             outFile << "Idle Cycles: " << cache.getIdleCycles() << endl;
//             outFile.close();
//         } else {
//             cerr << "Failed to open output file: " << outFilename << endl;
//         }
//     }
    
//     return 0;
// }

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

    // Run the multi-core simulation
    CacheSimulator simulator(tracePrefix, s, E, b);
    simulator.runSimulation();
    simulator.printFinalStats(outFilename);

    return 0;
}