module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    //axilite interface==============================
    //write(input)--
    output  wire                     awready,
    output  wire                     wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,

    //read(output)--
    output  wire                     arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,    

    //axis
    //stream slave (input data)=========================
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  wire                     ss_tready, 

    //stream master (output data)=======================
    input   wire                     sm_tready, 
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

    //write
    reg                              awready_reg;
    reg                              wready_reg;

    //read       
    reg                              arready_reg;
    reg [(pDATA_WIDTH-1):0]          rdata_reg;
    reg                              rvalid_reg;
    //tap bram       
    reg [(pADDR_WIDTH-1):0]          tap_A_reg;
    reg [3:0]                        tap_WE_reg; 
    reg [(pDATA_WIDTH-1):0]          tap_Di_reg; //wrtie  into bram
    reg tap_EN_reg;
    // ap_ctrl       
    reg                              ap_start;
    reg                              ap_idle;
    reg                              ap_done;
    //data bram      
    reg [3:0]                        data_WE_reg;
    reg [(pADDR_WIDTH-1):0]          data_A_reg;
    reg [(pDATA_WIDTH-1):0]          data_Di_reg;
    reg data_EN_reg;

    reg                              ss_tready_reg;

    reg                              sm_tvalid_reg;
    reg                              sm_tlast_reg;
    reg [(pDATA_WIDTH-1):0]          acc_reg;

    reg [(pDATA_WIDTH-1):0]            fir_multi;

    reg                              data_bram;
    reg                              fir_cal; //fir calculate state
    reg                              last_flag;

    assign awready = awready_reg;
    assign wready  = wready_reg; 

    assign arready  = arready_reg;
    assign rvalid   = rvalid_reg;
    assign rdata    = rdata_reg;

    assign tap_EN   = 1'b1;
    assign tap_WE   = tap_WE_reg;
    assign tap_Di   = tap_Di_reg;
    assign tap_A    = tap_A_reg;

    assign data_WE  = data_WE_reg;
    assign data_EN  = 1'b1;
    assign data_A   = data_A_reg;
    assign data_Di  = data_Di_reg;


    assign ss_tready = ss_tready_reg;

    assign sm_tvalid = sm_tvalid_reg;
    assign sm_tlast  = sm_tlast_reg;
    assign sm_tdata  = acc_reg;



    //** Axi lite write **// 
    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(~axis_rst_n)                      awready_reg <= 1'b0;
        else if(awvalid && ~awready_reg)     awready_reg <= 1'b1;
        else                                 awready_reg <= 1'b0;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(~axis_rst_n)                  wready_reg <= 1'b0;
        else if(wvalid && ~wready_reg)   wready_reg <= 1'b1;
        else                             wready_reg <= 1'b0;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(~axis_rst_n) tap_A_reg <= 12'h00;
        else if(awvalid && ~awready_reg) 
            if(awaddr == 12'h00) tap_A_reg <= 12'h00;
            else                tap_A_reg <= awaddr - 12'h20;
        else if(arvalid && ~arready_reg && ~rvalid_reg && araddr != 12'h00) 
            tap_A_reg <= araddr - 12'h20;
        else if(fir_cal)
            if(tap_A_reg == 12'h2c) tap_A_reg <= 12'h00;
            else                    tap_A_reg <= tap_A_reg + 12'h04;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(~axis_rst_n)                                     tap_Di_reg <= 0;
        else if(wvalid && ~wready_reg && awaddr != 12'h00)  tap_Di_reg <= wdata;
        else                                                tap_Di_reg <= 0;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if (~axis_rst_n)                                    tap_WE_reg <= 4'b0; 
        else if ((awaddr == 12'h0) || (awaddr == 12'h10))   tap_WE_reg <= 4'b0; 
        else if (wvalid && ~wready_reg)                     tap_WE_reg <= 4'b1111;
        else                                                tap_WE_reg <= 4'b0;
    end


    //** Axi lite read **//
    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(~axis_rst_n)                             arready_reg <= 1'b0;
        else if(arvalid) begin
            if(~arready_reg && rvalid_reg) arready_reg <= 1'b0;
            else if(arready_reg) arready_reg <= 1'b0;
            else if(~arready_reg) arready_reg <= 1'b1;
        end
        else arready_reg <= 1'b0;
    end 
    
    //rvalid 
    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(~axis_rst_n)                                rvalid_reg <= 1'b0;
        else if(rready && arvalid && arready_reg)      rvalid_reg <= 1'b1;
        else if(rvalid_reg && rready)                  rvalid_reg <= 1'b0;
        else                                           rvalid_reg <= 1'b0;
    end

    //read data
    always @(*) begin
        if(rvalid_reg && rready)  begin
            if(araddr != 12'h00)    rdata_reg <= tap_Do;
            else                    rdata_reg <= {29'b0, ap_idle, ap_done, ap_start};
        end
        else                        rdata_reg <= 0;
    end

    //** ap config (can also store in bram) **// 
    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(~axis_rst_n)                 ap_start <= 1'b0;
        else if(wvalid && ~wready_reg && awaddr == 12'h0 && wdata[0])  
                                        ap_start <= 1'b1;
        else if(ss_tvalid && ap_start)  ap_start <= 1'b0;
        else                            ap_start <= 1'b0;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(~axis_rst_n)                          ap_done <= 1'b0;
        else if(last_flag && sm_tlast_reg)       ap_done <= 1'b1;
        else if(ap_done && rvalid_reg && rready) ap_done <= 1'b0;
        else ap_done <= ap_done;    

    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(~axis_rst_n)                                 ap_idle <= 1'b1;
        else if(wvalid && ~wready_reg && awaddr == 12'h0 && wdata[0]) 
                                                        ap_idle <= 1'b0;
        else if(last_flag && rvalid_reg && rready)      ap_idle <= 1'b1;
    end

    //** fir engine **//
    //reset databram state
    always @(posedge axis_clk, negedge axis_rst_n) begin
        if (~axis_rst_n) data_bram <= 0;
        else if (~data_bram && data_A_reg == 12'h28) data_bram <= 1'b1; 
    end
    

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if (~axis_rst_n)                    last_flag <= 1'b0;
        else if (ss_tlast && ss_tready_reg) last_flag <= 1'b1;  
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(~axis_rst_n)                     data_WE_reg <= 4'b0000;
        else if(ss_tready_reg && ss_tvalid) data_WE_reg <= 4'b1111;
        else if(~fir_cal)                   data_WE_reg <= 4'b1111;
        else                                data_WE_reg <= 4'b0000;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(~axis_rst_n)                     data_Di_reg <= 0;
        else if(ss_tready_reg && ss_tvalid) data_Di_reg <= ss_tdata;
        // else                                data_Di_reg <= data_Di_reg;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(~axis_rst_n)                     ss_tready_reg <= 1'b0;
        else if (ss_tvalid && ap_start)     ss_tready_reg <= 1'b1;
        else if(fir_cal && tap_A_reg == 12'h2c && ~last_flag) ss_tready_reg <= 1'b1;
        else if(ss_tready_reg && ss_tvalid) ss_tready_reg <= 1'b0;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(~axis_rst_n)                             sm_tvalid_reg <= 1'b0;
        else if(fir_cal && tap_A_reg == 12'h2c)     sm_tvalid_reg <= 1'b1; 
        else if(ss_tready_reg && ss_tvalid)         sm_tvalid_reg <= 1'b0;
        else if(last_flag && sm_tvalid_reg)         sm_tvalid_reg <= 1'b0;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(~axis_rst_n) sm_tlast_reg <= 1'b0;
        else if(fir_cal && tap_A_reg == 12'h2c && last_flag)    sm_tlast_reg <= 1'b1; 
        else sm_tlast_reg <= 1'b0;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(~axis_rst_n) fir_multi <= 32'd0;
        else if(fir_cal) 
            if(tap_A_reg == 12'h2c) fir_multi <= 32'd0;
            else fir_multi <= tap_Do * data_Do;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(~axis_rst_n) acc_reg <= 32'd0;
        else if(fir_cal)
            if((tap_A_reg != 12'h00)) acc_reg <= fir_multi + acc_reg; // send data out
            else acc_reg <= acc_reg;
        else acc_reg <= 0;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(~axis_rst_n)                     data_A_reg <= 12'h00;
        else if(fir_cal) begin
            if(tap_A_reg == 12'h28 || tap_A_reg == 12'h2c) 
                data_A_reg <= data_A_reg;
            else data_A_reg <= (data_A_reg == 12'h28) ? 12'h00 : data_A_reg + 12'h04;
        end
        else if(~data_bram) //設databram裡面的值為0，address : 00~28
            if(data_A_reg == 12'h28) data_A_reg <= 12'h00;
            else data_A_reg <= data_A_reg + 12'h04;
    end

    always @(posedge axis_clk, negedge axis_rst_n) begin
        if(~axis_rst_n)                                fir_cal <= 1'b0;
        else if(ss_tready_reg && ss_tvalid)            fir_cal <= 1'b1; 
        else if(fir_cal && tap_A_reg == 12'h2c)        fir_cal <= 1'b0;
    end

endmodule
