`timescale 1ns / 1ps

module m_axil_adapter_wrapper(
    input clk,
    output reg [31:0] m_axil_awaddr,
    output reg [2:0] m_axil_awprot,
    output reg m_axil_awvalid,
    input m_axil_awready,
    output reg [31:0] m_axil_wdata,
    output reg [3:0] m_axil_wstrb,
    output reg m_axil_wvalid,
    input m_axil_wready,
    input [1:0] m_axil_bresp,
    input m_axil_bvalid,
    output reg m_axil_bready,
    output reg [31:0] m_axil_araddr,
    output reg [2:0] m_axil_arprot,
    output reg m_axil_arvalid,
    input m_axil_arready,
    input [31:0] m_axil_rdata,
    input [1:0] m_axil_rresp,
    input m_axil_rvalid,
    output reg m_axil_rready
);

    m_axil_adapter m_axil_adapter_inst(
        .clk(clk),
        .m_axil_awaddr(m_axil_awaddr),
        .m_axil_awprot(m_axil_awprot),
        .m_axil_awvalid(m_axil_awvalid),
        .m_axil_awready(m_axil_awready),
        .m_axil_wdata(m_axil_wdata),
        .m_axil_wstrb(m_axil_wstrb),
        .m_axil_wvalid(m_axil_wvalid),
        .m_axil_wready(m_axil_wready),
        .m_axil_bresp(m_axil_bresp),
        .m_axil_bvalid(m_axil_bvalid),
        .m_axil_bready(m_axil_bready),
        .m_axil_araddr(m_axil_araddr),
        .m_axil_arprot(m_axil_arprot),
        .m_axil_arvalid(m_axil_arvalid),
        .m_axil_arready(m_axil_arready),
        .m_axil_rdata(m_axil_rdata),
        .m_axil_rresp(m_axil_rresp),
        .m_axil_rvalid(m_axil_rvalid),
        .m_axil_rready(m_axil_rready)
    );
endmodule