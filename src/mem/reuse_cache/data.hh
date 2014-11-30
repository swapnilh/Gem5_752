/*
 * Copyright (c) 2012-2014 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Erik Hallnor
 *          Andreas Sandberg
 */

/** @file
 * Definitions of a simple cache block class.
 */

#ifndef __CACHE_DATA_HH__
#define __CACHE_DATA_HH__

#include <list>

#include "base/printable.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "sim/core.hh"          // for Tick
#include "mem/reuse_cache/base.hh"          // RUC
#include "mem/reuse_cache/blk.hh"          // RUC
#define mahBlkSize 64 //


/**
 * A Basic Cache data block.
 * Contains the data, and later a pointer to the tag entry.
 */
class DataBlock
{
  public:
    /**
     * Contains a copy of the data in this block for easy access. This is used
     * for efficient execution when the data could be actually stored in
     * another format (COW, compressed, sub-blocked, etc). In all cases the
     * data stored here should be kept consistant with the actual data
     * referenced by this block.
     */
    uint8_t data[mahBlkSize];
    /** the number of bytes stored in this block. */
    int size;

    int data_valid;
//    ReuseCacheBlk * back_ptr;
    int nru_bit;
    
    // Back pointer
    int bp_set, bp_way;	
    
    Tick tickInserted;

  protected:
    /**
     * Represents that the indicated thread context has a "lock" on
     * the block, in the LL/SC sense.
     */
    class Lock {
      public:
        int contextId;     // locking context
        Addr lowAddr;      // low address of lock range
        Addr highAddr;     // high address of lock range

        // check for matching execution context
        bool matchesContext(Request *req)
        {
            Addr req_low = req->getPaddr();
            Addr req_high = req_low + req->getSize() -1;
            return (contextId == req->contextId()) &&
                   (req_low >= lowAddr) && (req_high <= highAddr);
        }

        bool overlapping(Request *req)
        {
            Addr req_low = req->getPaddr();
            Addr req_high = req_low + req->getSize() - 1;

            return (req_low <= highAddr) && (req_high >= lowAddr);
        }

        Lock(Request *req)
            : contextId(req->contextId()),
              lowAddr(req->getPaddr()),
              highAddr(lowAddr + req->getSize() - 1)
        {
        }
    };

    /** List of thread contexts that have performed a load-locked (LL)
     * on the block since the last store. */
    std::list<Lock> lockList;

  public:

    DataBlock()
        //: data(NULL), size(0), data_valid(0), back_ptr(NULL), nru_bit(0), tickInserted(0)
        : size(0), data_valid(0), nru_bit(0), bp_set(0), bp_way(0), tickInserted(0)
    {
	    int i =0;
	    while (i < mahBlkSize)
	    {
	    	data[i] = 0;
	        i++;
	    }

    }

    /**
     * Copy the state of the given block into this one.
     * @param rhs The block to copy.
     * @return a const reference to this block.
     */
    const DataBlock& operator=(const DataBlock& rhs)//RUC
    {
        int i =0;
	while (i < mahBlkSize)
	{
		data[i] = rhs.data[i];
		i++;
	}
	//data = rhs.data;
        size = rhs.size;
        data_valid = rhs.data_valid;
      //  back_ptr = rhs.back_ptr;
        nru_bit = rhs.nru_bit;
	bp_set = rhs.bp_set;
	bp_way = rhs.bp_way;
        return *this;
    }

    /**
     * Invalidate the block and clear all state.
     */
    void invalidate() //FIXME RUC Need to call this at tag level
    {
//        status = 0;
 //       isTouched = false;
//        clearLoadLocks();
//	back_ptr = NULL;
	data_valid = 0;
	bp_set = -1;
	bp_way = -1;
    }

 
    bool isValid() const
    {
        return data_valid;
    }
    /**
     * Pretty-print a tag, and interpret state bits to readable form
     * including mapping to a MOESI stat.
     *
     * @return string with basic state information
     */
    std::string print() const
    {
        /**
         *  state       M   O   E   S   I
         *  writable    1   0   1   0   0
         *  dirty       1   1   0   0   0
         *  valid       1   1   1   1   0
         *
         *  state   writable    dirty   valid
         *  M       1           1       1
         *  O       0           1       1
         *  E       1           0       1
         *  S       0           0       1
         *  I       0           0       0
         **/
     /*   unsigned state = isWritable() << 2 | isDirty() << 1 | isValid();
        char s = '?';
        switch (state) {
          case 0b111: s = 'M'; break;
          case 0b011: s = 'O'; break;
          case 0b101: s = 'E'; break;
          case 0b001: s = 'S'; break;
          case 0b000: s = 'I'; break;
          default:    s = 'T'; break; // @TODO add other types
        }
        return csprintf("state: %x (%c) valid: %d writable: %d readable: %d "
                        "dirty: %d tag: %x", status, s, isValid(),
                        isWritable(), isReadable(), isDirty(), tag);*/ //FIXME
	return csprintf("No return --- FIXME");
    }

 };

/**
 * Simple class to provide virtual print() method on cache blocks
 * without allocating a vtable pointer for every single cache block.
 * Just wrap the CacheBlk object in an instance of this before passing
 * to a function that requires a Printable object.
 */
class DataBlockPrintWrapper : public Printable
{
    DataBlock *blk;
  public:
    DataBlockPrintWrapper(DataBlock *_blk) : blk(_blk) {}
    virtual ~DataBlockPrintWrapper() {}
    void print(std::ostream &o, int verbosity = 0,
               const std::string &prefix = "") const;
};

/**
 * Wrap a method and present it as a cache block visitor.
 *
 * For example the forEachBlk method in the tag arrays expects a
 * callable object/function as their parameter. This class wraps a
 * method in an object and presents  callable object that adheres to
 * the cache block visitor protocol.
 */
template <typename T, typename BlkType>
class DataBlockVisitorWrapper
{
  public:
    typedef bool (T::*visitorPtr)(BlkType &blk);

    DataBlockVisitorWrapper(T &_obj, visitorPtr _visitor)
        : obj(_obj), visitor(_visitor) {}

    bool operator()(BlkType &blk) {
        return (obj.*visitor)(blk);
    }

  private:
    T &obj;
    visitorPtr visitor;
};

/**
 * Cache block visitor that determines if there are dirty blocks in a
 * cache.
 *
 * Use with the forEachBlk method in the tag array to determine if the
 * array contains dirty blocks.
 */
/*template <typename BlkType>
class DataBlockIsDirtyVisitor
{
	Do i need this? FIXME
 };
*/
#endif //__CACHE_DATA_HH__
