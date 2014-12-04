/*
 * Copyright (c) 2014 The Regents of The University of Michigan
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
 * Authors: Anthony Gutierrez
 */

/**
 * @file
 * Definitions of a random replacement tag store.
 */

#include "base/random.hh"
#include "debug/CacheRepl.hh"
#include "mem/reuse_cache/tags/random_repl.hh"
#include "mem/reuse_cache/base.hh"

RandomRepl2::RandomRepl2(const Params *p)
    : BaseSetAssoc2(p)

{
}

BaseSetAssoc2::BlkType*
RandomRepl2::accessBlock(Addr addr, bool is_secure, Cycles &lat, int master_id)
{
    return BaseSetAssoc2::accessBlock(addr, is_secure, lat, master_id);
}

BaseSetAssoc2::BlkType*
RandomRepl2::findVictim(Addr addr) const
{
    BlkType *blk = BaseSetAssoc2::findVictim(addr);
    int set = extractSet(addr);

    // if all blocks are valid, pick a replacement at random
 if (blk->isValid()) {
        for (int i = 0; i < assoc; ++i) {
            blk = sets[set].blks[i];
            if (!blk->isFilled()) {
		    DPRINTF(Cache, "CS752:: Found Tag only victim in way i=%d for assoc %d \n", i, assoc);
		    break;
            }
        }
	if (blk->isFilled()) {

		// find a random index within the bounds of the set
		int idx = random_mt.random<int>(0, assoc - 1);
		assert(idx < assoc);
		assert(idx >= 0);
		blk = sets[extractSet(addr)].blks[idx];

		DPRINTF(CacheRepl, "CS752:: set %x: selecting blk %x in way %d with assoc %d for replacement\n",
                blk->set, idx, assoc, regenerateBlkAddr(blk->tag, blk->set));
	}
    }

    return blk;
}

void
RandomRepl2::insertBlock(PacketPtr pkt, BlkType *blk)
{
    BaseSetAssoc2::insertBlock(pkt, blk);
}

void
RandomRepl2::invalidate(BlkType *blk)
{
    BaseSetAssoc2::invalidate(blk);
}

RandomRepl2*
RandomRepl2Params::create()
{
    return new RandomRepl2(this);
}
