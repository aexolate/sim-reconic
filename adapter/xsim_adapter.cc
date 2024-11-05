#include <atomic>
#include <cstdint>
#include <cstring>
#include <exception>
#define AXI_R_DEBUG 1
#define AXI_W_DEBUG 1
#define JPGD_DEBUG 1
#define AXIL_R_DEBUG 1
#define AXIL_W_DEBUG 1

#define NUM_ADAPTERS 2

#include "xsim_adapter.hh"

#include <signal.h>
#include <simbricks/base/cxxatomicfix.h>
extern "C"
{
#include <simbricks/pcie/if.h>
}
// #include <svdpi.h>
#include <fcntl.h>    // For O_* constants
#include <sys/mman.h> // For shm_open(), mmap(), etc.
#include <sys/stat.h> // For mode constants
#include <unistd.h>   // For ftruncate()

// AXI DMA read signals
uint64_t s_axi_araddr;
uint8_t s_axi_arid;
uint8_t s_axi_arready;
uint8_t s_axi_arvalid;
uint8_t s_axi_arlen;
uint8_t s_axi_arsize;
uint8_t s_axi_arburst;
uint8_t s_axi_rdata[BYTES_DATA];
uint8_t s_axi_rid;
uint8_t s_axi_rready;
uint8_t s_axi_rvalid;
uint8_t s_axi_rlast;

// AXI DMA write signals
uint64_t s_axi_awaddr;
uint8_t s_axi_awid;
uint8_t s_axi_awready;
uint8_t s_axi_awvalid;
uint8_t s_axi_awlen;
uint8_t s_axi_awsize;
uint8_t s_axi_awburst;
uint8_t s_axi_wdata[BYTES_DATA];
uint8_t s_axi_wready;
uint8_t s_axi_wvalid;
uint8_t s_axi_wstrb;
uint8_t s_axi_wlast;
uint8_t s_axi_bid;
uint8_t s_axi_bready;
uint8_t s_axi_bvalid;
uint8_t s_axi_bresp;

// AXI Lite signals
uint32_t m_axil_araddr;
uint8_t m_axil_arready;
uint8_t m_axil_arvalid;
uint32_t m_axil_rdata;
uint8_t m_axil_rready;
uint8_t m_axil_rvalid;
uint8_t m_axil_rresp;
uint32_t m_axil_awaddr;
uint8_t m_axil_awready;
uint8_t m_axil_awvalid;
uint32_t m_axil_wdata;
uint8_t m_axil_wready;
uint8_t m_axil_wvalid;
uint8_t m_axil_wstrb;
uint8_t m_axil_bready;
uint8_t m_axil_bvalid;
uint8_t m_axil_bresp;

RecoNicAXISubordinateRead dma_read{};
RecoNicAXISubordinateWrite dma_write{};

RecoNicAXILManager reg_read_write{};
uint64_t clock_period = 1'000'000 / 150ULL; // 150 MHz
uint64_t main_time = 0;
volatile bool exiting = 0;
struct SimbricksPcieIf pcieif;
bool synchronized = false;
uint8_t num_adapters_ticked = 0;

volatile union SimbricksProtoPcieD2H *d2h_alloc(uint64_t cur_ts)
{
  volatile union SimbricksProtoPcieD2H *msg;
  while (!(msg = SimbricksPcieIfD2HOutAlloc(&pcieif, cur_ts)))
  {
  }
  return msg;
}

void RecoNicAXISubordinateRead::do_read(const simbricks::AXIOperation &axi_op)
{
#if JPGD_DEBUG
  std::cout << "JpegDecoderMemReader::doRead() ts=" << main_time
            << " id=" << axi_op.id << " addr=" << axi_op.addr
            << " len=" << axi_op.len << "\n";
#endif

  volatile union SimbricksProtoPcieD2H *msg = d2h_alloc(main_time);
  if (!msg)
  {
    throw "RecoNicAXISubordinateRead::doRead() dma read alloc failed";
  }

  unsigned int max_size = SimbricksPcieIfH2DOutMsgLen(&pcieif) -
                          sizeof(SimbricksProtoPcieH2DReadcomp);
  if (axi_op.len > max_size)
  {
    std::cerr << "error: read data of length " << axi_op.len
              << " doesn't fit into a SimBricks message\n";
    std::terminate();
  }

  volatile struct SimbricksProtoPcieD2HRead *read = &msg->read;
  read->req_id = axi_op.id;
  read->offset = axi_op.addr;
  read->len = axi_op.len;
  SimbricksPcieIfD2HOutSend(&pcieif, msg, SIMBRICKS_PROTO_PCIE_D2H_MSG_READ);
}

void RecoNicAXISubordinateWrite::do_write(
    const simbricks::AXIOperation &axi_op)
{
#if JPGD_DEBUG
  std::cout << "JpegDecoderMemWriter::doWrite() ts=" << main_time
            << " id=" << axi_op.id << " addr=" << axi_op.addr
            << " len=" << axi_op.len << "\n";
#endif

  volatile union SimbricksProtoPcieD2H *msg = d2h_alloc(main_time);
  if (!msg)
  {
    throw "JpegDecoderMemWriter::doWrite() dma read alloc failed";
  }

  volatile struct SimbricksProtoPcieD2HWrite *write = &msg->write;
  unsigned int max_size = SimbricksPcieIfH2DOutMsgLen(&pcieif) - sizeof(*write);
  if (axi_op.len > max_size)
  {
    std::cerr << "error: write data of length " << axi_op.len
              << " doesn't fit into a SimBricks message\n";
    std::terminate();
  }

  write->req_id = axi_op.id;
  write->offset = axi_op.addr;
  write->len = axi_op.len;
  std::memcpy(const_cast<uint8_t *>(write->data), axi_op.buf.get(), axi_op.len);
  SimbricksPcieIfD2HOutSend(&pcieif, msg, SIMBRICKS_PROTO_PCIE_D2H_MSG_WRITE);
}

void RecoNicAXILManager::read_done(simbricks::AXILOperationR &axi_op)
{

  std::cout << "RecoNicAXILManager::read_done() ts=" << main_time
            << " id=" << axi_op.req_id << " addr=" << axi_op.addr << "\n";

  volatile union SimbricksProtoPcieD2H *msg = d2h_alloc(main_time);
  if (!msg)
  {
    throw "RecoNicAXILManager::read_done() completion alloc failed";
  }

  volatile struct SimbricksProtoPcieD2HReadcomp *readcomp = &msg->readcomp;
  std::memcpy(const_cast<uint8_t *>(readcomp->data), &axi_op.data, 4);
  readcomp->req_id = axi_op.req_id;
  SimbricksPcieIfD2HOutSend(&pcieif, msg,
                            SIMBRICKS_PROTO_PCIE_D2H_MSG_READCOMP);
}

void RecoNicAXILManager::write_done(simbricks::AXILOperationW &axi_op)
{
#if JPGD_DEBUG
  std::cout << "RecoNicAXILManager::write_done ts=" << main_time
            << " id=" << axi_op.req_id << " addr=" << axi_op.addr << "\n";
#endif

  if (axi_op.posted)
  {
    return;
  }

  volatile union SimbricksProtoPcieD2H *msg = d2h_alloc(main_time);
  if (!msg)
  {
    throw "RecoNicAXILManager::write_done completion alloc failed";
  }

  volatile struct SimbricksProtoPcieD2HWritecomp *writecomp = &msg->writecomp;
  writecomp->req_id = axi_op.req_id;
  SimbricksPcieIfD2HOutSend(&pcieif, msg,
                            SIMBRICKS_PROTO_PCIE_D2H_MSG_WRITECOMP);
}

bool PciIfInit(const char *shm_path,
               struct SimbricksBaseIfParams &baseif_params)
{
  struct SimbricksBaseIfSHMPool pool;
  struct SimBricksBaseIfEstablishData ests;
  struct SimbricksProtoPcieDevIntro d_intro;
  struct SimbricksProtoPcieHostIntro h_intro;

  std::memset(&pool, 0, sizeof(pool));
  std::memset(&ests, 0, sizeof(ests));
  std::memset(&d_intro, 0, sizeof(d_intro));

  // RecoNIC PCI Vendor/Device ID
  d_intro.pci_vendor_id = 0x10ee;
  d_intro.pci_device_id = 0x903f;

  d_intro.pci_class = 0x40;
  d_intro.pci_subclass = 0x00;
  d_intro.pci_revision = 0x00;

  d_intro.bars[0].len = 4 * 1024 * 1024;
  d_intro.bars[0].flags = 0;

  ests.base_if = &pcieif.base;
  ests.tx_intro = &d_intro;
  ests.tx_intro_len = sizeof(d_intro);
  ests.rx_intro = &h_intro;
  ests.rx_intro_len = sizeof(h_intro);

  if (SimbricksBaseIfInit(&pcieif.base, &baseif_params))
  {
    std::cerr << "PciIfInit: SimbricksBaseIfInit failed\n";
    return false;
  }

  if (SimbricksBaseIfSHMPoolCreate(
          &pool, shm_path, SimbricksBaseIfSHMSize(&pcieif.base.params)) != 0)
  {
    std::cerr << "PciIfInit: SimbricksBaseIfSHMPoolCreate failed\n";
    return false;
  }

  if (SimbricksBaseIfListen(&pcieif.base, &pool) != 0)
  {
    std::cerr << "PciIfInit: SimbricksBaseIfListen failed\n";
    return false;
  }

  if (SimBricksBaseIfEstablish(&ests, 1))
  {
    std::cerr << "PciIfInit: SimBricksBaseIfEstablish failed\n";
    return false;
  }

  return true;
}

bool h2d_read(volatile struct SimbricksProtoPcieH2DRead &read,
              uint64_t main_time)
{

  std::cout << "h2d_read ts=" << main_time
            << " bar=" << static_cast<int>(read.bar)
            << " offset=" << read.offset << " len=" << read.len << std::endl;

  switch (read.bar)
  {
  case 0:
  {
    reg_read_write.issue_read(read.req_id, read.offset);
    break;
  }
  default:
  {
    std::cerr << "error: read from unexpected bar " << read.bar << "\n";
    return false;
  }
  }
  return true;
}

bool h2d_write(volatile struct SimbricksProtoPcieH2DWrite &write,
               uint64_t main_time, bool posted)
{
#if JPGD_DEBUG
  std::cout << "h2d_write ts=" << main_time
            << " bar=" << static_cast<int>(write.bar)
            << " offset=" << write.offset << " len=" << write.len << "\n";
#endif

  switch (write.bar)
  {
  case 0:
  {
    uint32_t data;
    if (write.len != 4)
    {
      throw "h2d_write() JPEG decoder register write needs to be 32 uint8_ts";
    }
    std::memcpy(&data, const_cast<uint8_t *>(write.data), write.len);
    reg_read_write.issue_write(write.req_id, write.offset, data, posted);
    break;
  }
  default:
  {
    std::cerr << "error: write to unexpected bar " << write.bar << "\n";
    return false;
  }
  }

  if (!posted)
  {
    volatile union SimbricksProtoPcieD2H *msg = d2h_alloc(main_time);
    volatile struct SimbricksProtoPcieD2HWritecomp &writecomp = msg->writecomp;
    writecomp.req_id = write.req_id;

    SimbricksPcieIfD2HOutSend(&pcieif, msg,
                              SIMBRICKS_PROTO_PCIE_D2H_MSG_WRITECOMP);
  }
  return true;
}

bool h2d_readcomp(volatile struct SimbricksProtoPcieH2DReadcomp &readcomp,
                  uint64_t main_time)
{
  dma_read.read_done(readcomp.req_id, const_cast<uint8_t *>(readcomp.data));
  return true;
}

bool h2d_writecomp(volatile struct SimbricksProtoPcieH2DWritecomp &writecomp,
                   uint64_t main_time)
{
  dma_write.write_done(writecomp.req_id);
  return true;
}

bool poll_h2d(uint64_t main_time)
{
  volatile union SimbricksProtoPcieH2D *msg =
      SimbricksPcieIfH2DInPoll(&pcieif, main_time);

  // no msg available
  if (msg == nullptr)
    return true;

  uint8_t type = SimbricksPcieIfH2DInType(&pcieif, msg);

  switch (type)
  {
  case SIMBRICKS_PROTO_PCIE_H2D_MSG_READ:
    if (!h2d_read(msg->read, main_time))
    {
      return false;
    }
    break;
  case SIMBRICKS_PROTO_PCIE_H2D_MSG_WRITE:
    if (!h2d_write(msg->write, main_time, false))
    {
      return false;
    }
    break;
  case SIMBRICKS_PROTO_PCIE_H2D_MSG_WRITE_POSTED:
    if (!h2d_write(msg->write, main_time, true))
    {
      return false;
    }
    break;
  case SIMBRICKS_PROTO_PCIE_H2D_MSG_READCOMP:
    if (!h2d_readcomp(msg->readcomp, main_time))
    {
      return false;
    }
    break;
  case SIMBRICKS_PROTO_PCIE_H2D_MSG_WRITECOMP:
    if (!h2d_writecomp(msg->writecomp, main_time))
    {
      return false;
    }
    break;
  case SIMBRICKS_PROTO_PCIE_H2D_MSG_DEVCTRL:
  case SIMBRICKS_PROTO_MSG_TYPE_SYNC:
    break; /* noop */
  case SIMBRICKS_PROTO_MSG_TYPE_TERMINATE:
    std::cerr << "poll_h2d: peer terminated\n";
    exiting = true;
    break;
  default:
    std::cerr << "warn: poll_h2d: unsupported type=" << type << "\n";
  }

  SimbricksPcieIfH2DInDone(&pcieif, msg);
  return true;
}

extern "C" void sigint_handler(int dummy) { exiting = 1; }

extern "C" void sigusr1_handler(int dummy)
{
  std::cerr << "main_time=" << main_time << "\n";
}

extern "C" void simbricks_init(const char *pci_socket, const char *shm_path,
                               uint64_t sync_period, uint64_t pci_latency)
{
  std::cout << "simbricks_init(): pci_socket=" << pci_socket
            << " shm_path=" << shm_path << " sync_period=" << sync_period
            << " pci_latency=" << pci_latency << std::endl;
  struct SimbricksBaseIfParams if_params;
  std::memset(&if_params, 0, sizeof(if_params));
  SimbricksPcieIfDefaultParams(&if_params);

  main_time = 0;
  if_params.sync_interval = sync_period * 1000ULL;
  if_params.link_latency = pci_latency * 1000ULL;
  clock_period = 1000000ULL / 500; // placeholder value

  if_params.sock_path = pci_socket;
  if (!PciIfInit(shm_path, if_params))
  {
    throw "PciIfInit failed";
  }

  synchronized = SimbricksBaseIfSyncEnabled(&pcieif.base);
  signal(SIGINT, sigint_handler);
  signal(SIGUSR1, sigusr1_handler);
}

void simbricks_sync_poll()
{
  if (num_adapters_ticked > 0)
  {
    return;
  }

  // send required sync messages
  while (SimbricksPcieIfD2HOutSync(&pcieif, main_time) < 0)
  {
    std::cerr << "warn: SimbricksPcieIfD2HOutSync failed main_time="
              << main_time << "\n";
  }

  // process available incoming messages for current timestamp
  do
  {
    poll_h2d(main_time);
  } while (!exiting && ((synchronized &&
                         SimbricksPcieIfH2DInTimestamp(&pcieif) <= main_time)));
}

void simbricks_tick()
{
  num_adapters_ticked++;
  if (num_adapters_ticked == NUM_ADAPTERS)
  {
    num_adapters_ticked = 0;
    main_time += clock_period;
  }
}

extern "C" unsigned char simbricks_is_exit() { return exiting ? 1 : 0; }

extern "C" void s_axi_adapter_step(
    const uint8_t dpi_awid, const uint64_t dpi_awaddr, const uint8_t dpi_awlen,
    const uint8_t dpi_awsize, const uint8_t dpi_awburst,
    const uint8_t dpi_awvalid, uint8_t *const dpi_awready,
    const uint8_t *dpi_wdata, const uint16_t dpi_wstrb, const uint8_t dpi_wlast,
    const uint8_t dpi_wvalid, uint8_t *const dpi_wready, uint8_t *const dpi_bid,
    uint8_t *const dpi_bresp, uint8_t *const dpi_bvalid,
    const uint8_t dpi_bready, const uint8_t dpi_arid, const uint64_t dpi_araddr,
    const uint8_t dpi_arlen, const uint8_t dpi_arsize,
    const uint8_t dpi_arburst, const uint8_t dpi_arvalid,
    uint8_t *const dpi_arready, uint8_t *const dpi_rid,
    uint8_t *const dpi_rdata, uint8_t *const dpi_rresp,
    uint8_t *const dpi_rlast, uint8_t *const dpi_rvalid,
    const uint8_t dpi_rready)
{
  simbricks_sync_poll();

  // copy over input signals
  s_axi_awid = dpi_awid;
  s_axi_awaddr = dpi_awaddr;
  s_axi_awlen = dpi_awlen;
  s_axi_awsize = dpi_awsize;
  s_axi_awburst = dpi_awburst;
  s_axi_awvalid = dpi_awvalid;
  std::memcpy(s_axi_wdata, dpi_wdata, BYTES_DATA);
  s_axi_wstrb = dpi_wstrb;
  s_axi_wlast = dpi_wlast;
  s_axi_wvalid = dpi_wvalid;
  s_axi_bready = dpi_bready;
  s_axi_arid = dpi_arid;
  s_axi_araddr = dpi_araddr;
  s_axi_arlen = dpi_arlen;
  s_axi_arsize = dpi_arsize;
  s_axi_arburst = dpi_arburst;
  s_axi_arvalid = dpi_arvalid;
  s_axi_rready = dpi_rready;

  dma_read.step(main_time);
  dma_write.step(main_time);
  dma_read.step_apply();
  dma_write.step_apply();

  // write output signals
  *dpi_awready = s_axi_awready;
  *dpi_wready = s_axi_wready;
  *dpi_bid = s_axi_bid;
  *dpi_bresp = s_axi_bresp;
  *dpi_bvalid = s_axi_bvalid;
  *dpi_arready = s_axi_arready;
  *dpi_rid = s_axi_rid;
  *dpi_rresp = 0;
  *dpi_rlast = s_axi_rlast;
  *dpi_rvalid = s_axi_rvalid;
  std::memcpy(dpi_rdata, s_axi_rdata, BYTES_DATA);

  simbricks_tick();
}

extern "C" void
m_axil_adapter_step(uint32_t *const dpi_awaddr, uint8_t *const dpi_awprot,
                    uint8_t *const dpi_awvalid, const uint8_t dpi_awready,
                    uint32_t *const dpi_wdata, uint8_t *const dpi_wstrb,
                    uint8_t *const dpi_wvalid, const uint8_t dpi_wready,
                    const uint8_t dpi_bresp, const uint8_t dpi_bvalid,
                    uint8_t *const dpi_bready, uint32_t *const dpi_araddr,
                    uint8_t *const dpi_arprot, uint8_t *const dpi_arvalid,
                    const uint8_t dpi_arready, const int dpi_rdata,
                    const uint8_t dpi_rresp, const uint8_t dpi_rvalid,
                    uint8_t *const dpi_rready)
{
  simbricks_sync_poll();

  // copy over input signals
  m_axil_awready = dpi_awready;
  m_axil_wready = dpi_wready;
  m_axil_bresp = dpi_bresp;
  m_axil_bvalid = dpi_bvalid;
  m_axil_arready = dpi_arready;
  m_axil_rdata = dpi_rdata;
  m_axil_rresp = dpi_rresp;
  m_axil_rvalid = dpi_rvalid;

  reg_read_write.step(main_time);
  reg_read_write.step_apply();

  // write output signals
  *dpi_awaddr = m_axil_awaddr;
  *dpi_awprot = 0;
  *dpi_awvalid = m_axil_awvalid;
  *dpi_wdata = m_axil_wdata;
  *dpi_wstrb = m_axil_wstrb;
  *dpi_wvalid = m_axil_wvalid;
  *dpi_bready = m_axil_bready;
  *dpi_araddr = m_axil_araddr;
  *dpi_arprot = 0;
  *dpi_arvalid = m_axil_arvalid;
  *dpi_rready = m_axil_rready;

  simbricks_tick();
}

// testbench signal exchange code

// Define the size of the shared memory segment
#define SHM_SIZE sizeof(struct axi_signals)

struct axi
{
  uint8_t awid;      // 1 bit (can be stored in uint8_t)
  uint8_t awaddr[8]; // 64 bits (8 bytes)
  uint8_t awuser[4]; // 32 bits (4 bytes)
  uint8_t awqos;     // 4 bits (can fit in uint8_t)
  uint8_t awlen;     // 8 bits (can fit in uint8_t)
  uint8_t awsize;    // 3 bits (can fit in uint8_t)
  uint8_t awburst;   // 2 bits (can fit in uint8_t)
  uint8_t awcache;   // 4 bits (can fit in uint8_t)
  uint8_t awprot;    // 3 bits (can fit in uint8_t)
  int awvalid;       // 1 bit
  int awready;       // 1 bit
  uint8_t wdata[64]; // 512 bits (64 bytes)
  uint8_t wstrb[8];  // 64 bits (8 bytes)
  int wlast;         // 1 bit
  int wvalid;        // 1 bit
  int wready;        // 1 bit
  int awlock;        // 1 bit
  int bid;           // 1 bit
  uint8_t bresp;     // 2 bits (can fit in uint8_t)
  int bvalid;        // 1 bit
  int bready;        // 1 bit
  int arid;          // 1 bit
  uint8_t araddr[8]; // 64 bits (8 bytes)
  uint8_t arlen;     // 8 bits (can fit in uint8_t)
  uint8_t arsize;    // 3 bits (can fit in uint8_t)
  uint8_t arburst;   // 2 bits (can fit in uint8_t)
  uint8_t arcache;   // 4 bits (can fit in uint8_t)
  uint8_t arprot;    // 3 bits (can fit in uint8_t)
  int arvalid;       // 1 bit
  int arready;       // 1 bit
  int rid;           // 1 bit
  uint8_t rdata[64]; // 512 bits (64 bytes)
  uint8_t rresp;     // 2 bits (can fit in uint8_t)
  int rlast;         // 1 bit
  int rvalid;        // 1 bit
  int rready;        // 1 bit
  int arlock;        // 1 bit
  uint8_t arqos;     // 4 bits (can fit in uint8_t)
};

struct rp_signals
{
  uint8_t tdata[64]; // 512 bits (64 bytes)
  uint8_t tkeep[8];  // 64 bits (8 bytes)
  int tvalid;        // 1 bit
  int tlast;         // 1 bit
  int tready;        // 1 bit
};

struct cmac_tx_signals
{
  uint8_t tdata[64]; // 512 bits (64 bytes)
  uint8_t tkeep[8];  // 64 bits (8 bytes)
  int tvalid;        // 1 bit
  int tlast;         // 1 bit
  int tuser_size;    // 16 bits
  int tready;
};

struct axi_signals
{
  int init_sys_mem_done;
  int init_dev_mem_done;

  int finish_config_rdma2;
  int start_rdma2_stat;

  struct rp_signals rp;
  struct cmac_tx_signals cmac_tx;
  struct axi axi_rdma2_send_write_payload;
  struct axi axi_rdma_rsp_payload;
  struct axi axi_rdma_get_wqe;
  struct axi axi_rdma2_get_payload;
  struct axi axi_rdma_completion;
};

// Create a pointer to the shared memory
struct axi_signals *axi_signals_ptr;

// Initialize shared memory
extern "C" void init_shared_memory()
{
  int shm_fd = shm_open("/signals", O_CREAT | O_RDWR, 0666);
  if (shm_fd < 0)
  {
    perror("shm_open");
    exit(1);
  }

  // Resize the shared memory segment if it's the first process
  if (ftruncate(shm_fd, SHM_SIZE) == -1)
  {
    perror("ftruncate");
    exit(1);
  }

  // Map the shared memory segment
  axi_signals_ptr = (struct axi_signals *)mmap(0, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
  if (axi_signals_ptr == MAP_FAILED)
  {
    perror("mmap");
    exit(1);
  }
}

extern "C" void set_finish_config_rdma2(int finish_config_rdma2)
{
  axi_signals_ptr->finish_config_rdma2 = finish_config_rdma2;
}

extern "C" void get_finish_config_rdma2(int *finish_config_rdma2)
{
  *finish_config_rdma2 = axi_signals_ptr->finish_config_rdma2;
}

extern "C" void set_start_rdma2_stat(int start_rdma2_stat)
{
  axi_signals_ptr->start_rdma2_stat = start_rdma2_stat;
}

extern "C" void get_start_rdma2_stat(int *start_rdma2_stat)
{
  *start_rdma2_stat = axi_signals_ptr->start_rdma2_stat;
}



extern "C" void set_init_mem_signals(int sys_done, int dev_done)
{
  axi_signals_ptr->init_dev_mem_done = dev_done;
  axi_signals_ptr->init_sys_mem_done = sys_done;
}

extern "C" void get_init_mem_signals(int *sys_done, int *dev_done)
{
  if (axi_signals_ptr == NULL)
  {
    fprintf(stderr, "Error: Shared memory not initialized.\n");
  }

  *dev_done = axi_signals_ptr->init_dev_mem_done;
  *sys_done = axi_signals_ptr->init_sys_mem_done;
}

extern "C" void set_rp_signals(uint8_t *tdata, uint8_t *tkeep, int tvalid, int tlast)
{
  memcpy(axi_signals_ptr->rp.tdata, tdata, sizeof(axi_signals_ptr->rp.tdata));
  memcpy(axi_signals_ptr->rp.tkeep, tkeep, sizeof(axi_signals_ptr->rp.tkeep));

  axi_signals_ptr->rp.tvalid = tvalid;
  axi_signals_ptr->rp.tlast = tlast;
}

extern "C" void set_rp_signal_tready(int tready)
{
  axi_signals_ptr->rp.tready = tready;
}

extern "C" void get_rp_signals(uint8_t *tdata, uint8_t *tkeep, int *tvalid, int *tlast)
{
  memcpy(tdata, axi_signals_ptr->rp.tdata, sizeof(axi_signals_ptr->rp.tdata));
  memcpy(tkeep, axi_signals_ptr->rp.tkeep, sizeof(axi_signals_ptr->rp.tkeep));

  *tvalid = axi_signals_ptr->rp.tvalid;
  *tlast = axi_signals_ptr->rp.tlast;
}

extern "C" void get_rp_signal_tready(int *tready)
{
  *tready = axi_signals_ptr->rp.tready;
}

extern "C" void set_cmac_tx_signals(uint8_t *tdata, uint8_t *tkeep, int tvalid, int tlast, int tuser_size)
{
  memcpy(axi_signals_ptr->cmac_tx.tdata, tdata, sizeof(axi_signals_ptr->cmac_tx.tdata));
  memcpy(axi_signals_ptr->cmac_tx.tkeep, tkeep, sizeof(axi_signals_ptr->cmac_tx.tkeep));

  axi_signals_ptr->cmac_tx.tvalid = tvalid;
  axi_signals_ptr->cmac_tx.tlast = tlast;
  axi_signals_ptr->cmac_tx.tuser_size = tuser_size;
}

extern "C" void set_cmac_tx_signal_tready(int tready)
{
  axi_signals_ptr->cmac_tx.tready = tready;
}

extern "C" void get_cmac_tx_signals(uint8_t *tdata, uint8_t *tkeep, int *tvalid, int *tlast, int *tuser_size)
{
  memcpy(tdata, axi_signals_ptr->cmac_tx.tdata, sizeof(axi_signals_ptr->cmac_tx.tdata));
  memcpy(tkeep, axi_signals_ptr->cmac_tx.tkeep, sizeof(axi_signals_ptr->cmac_tx.tkeep));

  *tvalid = axi_signals_ptr->cmac_tx.tvalid;
  *tlast = axi_signals_ptr->cmac_tx.tlast;
  *tuser_size = axi_signals_ptr->cmac_tx.tuser_size;
}

extern "C" void get_cmac_tx_signal_tready(int *tready)
{
  *tready = axi_signals_ptr->cmac_tx.tready;
}



extern "C" void set_axi_input_signals(
    uint8_t awid, uint8_t *awaddr, uint8_t awqos, uint8_t awlen, uint8_t awsize,
    uint8_t awburst, uint8_t awcache, uint8_t awprot, int awvalid, uint8_t *wdata,
    uint8_t *wstrb, int wlast, int wvalid, int awlock, int arid, uint8_t *araddr,
    uint8_t arlen, uint8_t arsize, uint8_t arburst, uint8_t arcache, uint8_t arprot,
    int arvalid, int arlock, uint8_t arqos, axi *axi_ptr)
{
  axi_ptr->awid = awid;
  memcpy(axi_ptr->awaddr, awaddr, sizeof(axi_ptr->awaddr));
  axi_ptr->awqos = awqos;
  axi_ptr->awlen = awlen;
  axi_ptr->awsize = awsize;
  axi_ptr->awburst = awburst;
  axi_ptr->awcache = awcache;
  axi_ptr->awprot = awprot;
  axi_ptr->awvalid = awvalid;
  memcpy(axi_ptr->wdata, wdata, sizeof(axi_ptr->wdata));
  memcpy(axi_ptr->wstrb, wstrb, sizeof(axi_ptr->wstrb));
  axi_ptr->wlast = wlast;
  axi_ptr->wvalid = wvalid;
  axi_ptr->awlock = awlock;
  axi_ptr->arid = arid;
  memcpy(axi_ptr->araddr, araddr, sizeof(axi_ptr->araddr));
  axi_ptr->arlen = arlen;
  axi_ptr->arsize = arsize;
  axi_ptr->arburst = arburst;
  axi_ptr->arcache = arcache;
  axi_ptr->arprot = arprot;
  axi_ptr->arvalid = arvalid;
  axi_ptr->arlock = arlock;
  axi_ptr->arqos = arqos;
}

extern "C" void set_axi_output_signals(
    int awready,
    int wready,
    int bid,
    uint8_t bresp,
    int bvalid,
    int arready,
    int rid,
    uint8_t *rdata,
    uint8_t rresp,
    int rlast,
    int rvalid,
    axi *axi_ptr)
{
  axi_ptr->awready = awready;
  axi_ptr->wready = wready;
  axi_ptr->bid = bid;
  axi_ptr->bresp = bresp;
  axi_ptr->bvalid = bvalid;
  axi_ptr->arready = arready;
  axi_ptr->rid = rid;
  memcpy(axi_ptr->rdata, rdata, sizeof(axi_ptr->rdata));
  axi_ptr->rresp = rresp;
  axi_ptr->rlast = rlast;
  axi_ptr->rvalid = rvalid;
}

void get_axi_input_signals(
    uint8_t *awid,
    uint8_t *awaddr,
    uint8_t *awqos,
    uint8_t *awlen,
    uint8_t *awsize,
    uint8_t *awburst,
    uint8_t *awcache,
    uint8_t *awprot,
    int *awvalid,
    uint8_t *wdata,
    uint8_t *wstrb,
    int *wlast,
    int *wvalid,
    int *awlock,
    int *arid,
    uint8_t *araddr,
    uint8_t *arlen,
    uint8_t *arsize,
    uint8_t *arburst,
    uint8_t *arcache,
    uint8_t *arprot,
    int *arvalid,
    int *arlock,
    uint8_t *arqos,
    axi *axi_ptr)
{
 
  *awid = axi_ptr->awid;
  memcpy(awaddr, axi_ptr->awaddr, sizeof(axi_ptr->awaddr));
  *awqos = axi_ptr->awqos;
  *awlen = axi_ptr->awlen;
  *awsize = axi_ptr->awsize;
  *awburst = axi_ptr->awburst;
  *awcache = axi_ptr->awcache;
  *awprot = axi_ptr->awprot;
  *awvalid = axi_ptr->awvalid;
  memcpy(wdata, axi_ptr->wdata, sizeof(axi_ptr->wdata));
  memcpy(wstrb, axi_ptr->wstrb, sizeof(axi_ptr->wstrb));
  *wlast = axi_ptr->wlast;
  *wvalid = axi_ptr->wvalid;
  *awlock = axi_ptr->awlock;
  *arid = axi_ptr->arid;
  memcpy(araddr, axi_ptr->araddr, sizeof(axi_ptr->araddr));
  *arlen = axi_ptr->arlen;
  *arsize = axi_ptr->arsize;
  *arburst = axi_ptr->arburst;
  *arcache = axi_ptr->arcache;
  *arprot = axi_ptr->arprot;
  *arvalid = axi_ptr->arvalid;
  *arlock = axi_ptr->arlock;
  *arqos = axi_ptr->arqos;
}

void get_axi_output_signals(
    int *awready,
    int *wready,
    int *bid,
    uint8_t *bresp,
    int *bvalid,
    int *arready,
    int *rid,
    uint8_t *rdata,
    uint8_t *rresp,
    int *rlast,
    int *rvalid,
    axi *axi_ptr)
{
  *awready = axi_ptr->awready;
  *wready = axi_ptr->wready;
  *bid = axi_ptr->bid;
  *bresp = axi_ptr->bresp;
  *bvalid = axi_ptr->bvalid;
  *arready = axi_ptr->arready;
  *rid = axi_ptr->rid;
  memcpy(rdata, axi_ptr->rdata, sizeof(axi_ptr->rdata));
  *rresp = axi_ptr->rresp;
  *rlast = axi_ptr->rlast;
  *rvalid = axi_ptr->rvalid;
}



#define DEFINE_AXI_FUNCTIONS(name) \
extern "C" void set_##name##_1( \
    uint8_t awid, uint8_t *awaddr, uint8_t awqos, uint8_t awlen, uint8_t awsize, \
    uint8_t awburst, uint8_t awcache, uint8_t awprot, int awvalid, uint8_t *wdata, \
    uint8_t *wstrb, int wlast, int wvalid, int awlock, int arid, uint8_t *araddr, \
    uint8_t arlen, uint8_t arsize, uint8_t arburst, uint8_t arcache, uint8_t arprot, \
    int arvalid, int arlock, uint8_t arqos) { \
      set_axi_input_signals(awid, awaddr, awqos, awlen, awsize, awburst, awcache, \
        awprot, awvalid, wdata, wstrb, wlast, wvalid, awlock, arid, araddr, arlen, \
         arsize, arburst, arcache, arprot, arvalid, arlock, arqos, &axi_signals_ptr->name); \
} \
extern "C" void set_##name##_2( \
    int awready, int wready, int bid, uint8_t bresp, int bvalid, \
    int arready, int rid, uint8_t *rdata, uint8_t rresp, \
    int rlast, int rvalid, axi *axi_ptr) { \
    set_axi_output_signals(awready, wready, bid, bresp, bvalid, \
        arready, rid, rdata, rresp, rlast, rvalid, &axi_signals_ptr->name); \
} \
extern "C" void get_##name##_1( \
    uint8_t *awid, uint8_t *awaddr, uint8_t *awqos, uint8_t *awlen, uint8_t *awsize, \
    uint8_t *awburst, uint8_t *awcache, uint8_t *awprot, int *awvalid, uint8_t *wdata, \
    uint8_t *wstrb, int *wlast, int *wvalid, int *awlock, int *arid, uint8_t *araddr, \
    uint8_t *arlen, uint8_t *arsize, uint8_t *arburst, uint8_t *arcache, uint8_t *arprot, \
    int *arvalid, int *arlock, uint8_t *arqos) { \
    get_axi_input_signals(awid, awaddr, awqos, awlen, awsize, awburst, awcache, \
        awprot, awvalid, wdata, wstrb, wlast, wvalid, awlock, arid, araddr, arlen, \
        arsize, arburst, arcache, arprot, arvalid, arlock, arqos, &axi_signals_ptr->name); \
} \
extern "C" void get_##name##_2( \
    int *awready, int *wready, int *bid, uint8_t *bresp, int *bvalid, \
    int *arready, int *rid, uint8_t *rdata, uint8_t *rresp, \
    int *rlast, int *rvalid, axi *axi_ptr) { \
    get_axi_output_signals(awready, wready, bid, bresp, bvalid, \
        arready, rid, rdata, rresp, rlast, rvalid, \
        &axi_signals_ptr->name); \
}

// Define AXI functions for each signal
DEFINE_AXI_FUNCTIONS(axi_rdma2_send_write_payload)
DEFINE_AXI_FUNCTIONS(axi_rdma_rsp_payload)
DEFINE_AXI_FUNCTIONS(axi_rdma_get_wqe)
DEFINE_AXI_FUNCTIONS(axi_rdma2_get_payload)
DEFINE_AXI_FUNCTIONS(axi_rdma_completion)