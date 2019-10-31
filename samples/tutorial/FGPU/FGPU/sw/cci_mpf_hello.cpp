//
// Copyright (c) 2017, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>

#include <iostream>
#include <string>
#include <fstream>
#include <time.h>

using namespace std;

#include "opae_svc_wrapper.h"
#include "csr_mgr.h"

// State from the AFU's JSON file, extracted using OPAE's afu_json_mgr script
#include "afu_json_info.h"

#define MEM_ADDR 0
#define MEM_RW 1
#define MEM_WRDATA 2
#define MEM_RDDATA 3
#define READY_FOR_SW_CMD 4
#define MEM_BASE_ADDR 5
#define NUM_WRITE 6
#define KRNL_RAM 0x0
#define C_RAM 0x1000
#define RSTAT 0x2000
#define RSTART 0x2001
#define RCLEANCACHE 0x2002
#define RINITIATE 0x2003

#define BILLION 1000000000.0

void write_cmd(CSR_MGR csrs, uint32_t idx, uint32_t data)
{
    csrs.writeCSR(MEM_ADDR, uint64_t(idx));
    csrs.writeCSR(MEM_WRDATA, uint64_t(data));
    csrs.writeCSR(MEM_RW, 1);
    uint64_t res = 0;
    do {
        res = csrs.readCSR(READY_FOR_SW_CMD);
    } while (res != 1);
}

uint32_t read_cmd(CSR_MGR csrs, uint32_t idx)
{
    csrs.writeCSR(MEM_ADDR, uint64_t(idx));
    csrs.writeCSR(MEM_RW, 3);
    uint64_t res = 0;
    do {
        res = csrs.readCSR(READY_FOR_SW_CMD);
    } while (res != 1);
    res = csrs.readCSR(MEM_RDDATA);
    return uint32_t(res);
}

int load_kernel(CSR_MGR csrs, char *filename)
{
    ifstream kernel(filename);
    string line;
    int addr = C_RAM;
    while (getline(kernel, line))
    {
        size_t pos = line.find(" ");
        if (pos != string::npos) line = line.substr(0, pos);
        write_cmd(csrs, addr++, strtoul(line.c_str(), 0, 16));
    }
    return 0;
}

int verify_mem(CSR_MGR csrs, char *filename, int base)
{
    ifstream kernel(filename);
    string line;
    int addr = base;
    int err = 0;
    while (getline(kernel, line))
    {
        size_t pos = line.find(" ");
        if (pos != string::npos) line = line.substr(0, pos);
        uint32_t ref = strtoul(line.c_str(), 0, 16);
        uint32_t res = read_cmd(csrs, addr++);
        if (ref != res) err++;
    }
    return err;
}

int load_kernel_param(CSR_MGR csrs, char *filename)
{
    ifstream kernel(filename);
    string line;
    int addr = KRNL_RAM;
    while (getline(kernel, line))
    {
        size_t pos = line.find(" ");
        if (pos != string::npos) line = line.substr(0, pos);
        write_cmd(csrs, addr++, strtoul(line.c_str(), 0, 16));
    }
    return 0;
}

int start_kernel(CSR_MGR csrs)
{
    write_cmd(csrs, RINITIATE, 1);
    write_cmd(csrs, RCLEANCACHE, 1);
    write_cmd(csrs, RSTART, 1);
}

int wait_kernel_finish(CSR_MGR csrs)
{
    int done = 0;
    uint64_t write_count = 0;
    int res;
    do {
        done = read_cmd(csrs, RSTAT);
        sleep(1);
        res = read_cmd(csrs, RCLEANCACHE);
        cout << "clean cache: " << res << endl;
        res = read_cmd(csrs, 0x2004);
        cout << "start_kernel: " << res << endl;
        res = read_cmd(csrs, 0x2005);
        cout << "start_CUs: " << res << endl;
        res = read_cmd(csrs, 0x2006);
        cout << "WGsdispatched: " << res << endl;
        res = read_cmd(csrs, 0x2007);
        cout << "wf_active: " << res << endl;
        res = read_cmd(csrs, 0x2008);
        cout << "wf_all_active: " << res << endl;
        //res = read_cmd(csrs, 0x2009);
        //cout << "st_smem_rd6: " << hex << res << endl;
        //res = read_cmd(csrs, 0x200a);
        //cout << "st_smem_rd7: " << hex << res << endl;
        res = read_cmd(csrs, 0x200b);
        cout << "st_gmem: " << hex << res << endl;
        //res = read_cmd(csrs, 0x200c);
        //cout << "st_smem_wr6: " << hex << res << endl;
        //res = read_cmd(csrs, 0x200d);
        //cout << "st_smem_wr7: " << hex << res << endl;
        write_count = csrs.readCSR(NUM_WRITE);
        cout << "num write: " << hex << write_count << endl;
        write_count = csrs.readCSR(7);
        cout << "st_write: " << hex << write_count << endl;
    } while (done != 1);
    return 0;
}

int main(int argc, char *argv[])
{
    struct timespec wall_start, wall_end;
    double total_time_used;
    ifstream infile("init_mem_lstm.mif");
    ofstream outfile("output.txt");

    if (argc != 3) {
        cout << "./cci_mpf_hello cram krnl_ram" << endl;
    }
    // Find and connect to the accelerator
    OPAE_SVC_WRAPPER fpga(AFU_ACCEL_UUID);
    assert(fpga.isOk());

    // Connect the CSR manager
    CSR_MGR csrs(fpga);

    // Allocate a single page memory buffer
    cout << "page size:" << getpagesize() << endl;
    auto buf_handle = fpga.allocBuffer(getpagesize());
    auto buf = reinterpret_cast<volatile uint32_t*>(buf_handle->c_type());
    uint64_t buf_pa = buf_handle->io_address();
    //int *base = (int *)((intptr_t(buf) + 0x40) & 0xffffffffffffff80);
    //uint64_t base_pa = (buf_pa + 0x40) & 0xffffffffffffff80;
    //int *base = (int *)buf;
    //uint64_t base_pa = buf_pa;
    assert(NULL != buf);

    // Set the low byte of the shared buffer to 0.  The FPGA will write
    // a non-zero value to it.
    //float tmp = -7.98;
    //float tmp = -1.0;
    for (int i = 0; i < 1024; i++) {
        //buf[i] = *(int *)(&tmp);
        //tmp += 0.03125;
        //buf[i] = 0x3f000000;
        //buf[i] = i;
        unsigned int tmp;
        infile >> hex >> tmp;
        buf[i] = tmp;
    }

    // Tell the accelerator the address of the buffer using cache line
    // addresses by writing to application CSR 0.  The CSR manager maps
    // its registers to MMIO space.  The accelerator will respond by
    // writing to the buffer.
    int res = read_cmd(csrs, RCLEANCACHE);
    int err = 0;
    cout << "clean cache: " << res << endl;

    cout << "write base address:" << hex << buf_pa << endl;
    clock_gettime(CLOCK_REALTIME, &wall_start);
    csrs.writeCSR(MEM_BASE_ADDR, buf_pa);

    cout << "load kernel" << endl;
    load_kernel(csrs, argv[1]);
    cout << "load kernel params" << endl;
    load_kernel_param(csrs, argv[2]);

    cout << "read kernel" << endl;

    cout << "start kernel" << endl;
    start_kernel(csrs);
    wait_kernel_finish(csrs);
    clock_gettime(CLOCK_REALTIME, &wall_end);

    total_time_used = (wall_end.tv_sec - wall_start.tv_sec) + (wall_end.tv_nsec - wall_start.tv_nsec)/BILLION;
    // Spin, waiting for the value in memory to change to something non-zero.
    //while (100 == buf[0])
    //{
    //  // A well-behaved program would use _mm_pause(), nanosleep() or
    //  // equivalent to save power here.
    //};

    // Print the string written by the FPGA
    err = 0;
    for (int i = 0; i < 1024; i++) {
        //if (buf[i] != 2*i+512) err++;
        //cout << buf[i] << "\t";
        outfile << hex << buf[i] << "\n";
    }
    outfile << endl;
    outfile.close();

    if (err > 0)
        cout << "Failed. Total error: " << err << endl;
    else
        cout << "Passed!" << endl;

    cout << "total time:" << total_time_used << "s" << endl;
    // Ask the FPGA-side CSR manager the AFU's frequency
    cout << endl
         << "# AFU frequency: " << csrs.getAFUMHz() << " MHz"
         << (fpga.hwIsSimulated() ? " [simulated]" : "")
         << endl;

    // All shared buffers are automatically released and the FPGA connection
    // is closed when their destructors are invoked here.
    return 0;
}
