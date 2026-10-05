`timescale 1ns / 1ps

// 第4章：RGB888液晶时序、几何图形与中英文字符叠加。
// 工程：CHAPT3_200T；Design Top：top_lcd_chapter3。
// 文件名：tb_chapter4_all.v；Simulation Top：tb_chapter4_all。
// 添加到 Simulation Sources，右键本模块 Set as Top，运行行为仿真。
// 保留工程原有 clk_wiz_lcd IP、ODDR 仿真库和 chinese_font_16x16.mem。
//
// 一个仿真顶层覆盖图4-1至图4-4：
// 图4-1 行场扫描与有效区（完整DUT）：
//   pix_clk, pix_rst_n, h_count, v_count, de_raw, lcd_de,
//   hsync_raw, vsync_raw, lcd_hsync, lcd_vsync。
//   先 run 200 us 看连续行；累积约26 ms看帧末、场同步和帧切换。
//   原始计数 h_count=0..1343、v_count=0..634；有效区1024x600。
//   HS低20个像素周期、VS低3行。输出同步信号存在流水线延迟。
//
// 图4-2 像素输出与启动顺序（完整DUT）：
//   像素局部：pix_clk, lcd_pclk, de_raw, de_pipe, lcd_de,
//              mixed_rgb, lcd_rgb。
//   启动全程：clk_locked, pix_rst_n, lcd_rst_n, lcd_bl。
//   将总仿真时长运行到80 ms，查看复位释放和背光开启。
//   按实际RTL的posedge输出寄存关系检查，不按源码中的旧边沿注释判断。
//   启动延迟按实际像素周期及500000/2500000个周期观察。
//
// 图4-3 同步字模ROM与像素对齐（原工程模块的像素点测试）：
//   text_clk, text_x, text_y, ascii_font_addr, ascii_row_data,
//   ascii_column, ascii_area, ascii_pixel, text_rgb。
//   截图时间约90..660 ns；text_stage=1。
//   此处看text_*信号；全屏DUT的pix_clk/pixel_x/mixed_rgb用于前两图。
//
// 图4-4 文本边界与显示模式（原工程模块的像素点测试）：
//   text_clk, text_x, text_y, text_mode, cn2_column, cn2_area,
//   cn2_pixel, cn2_layer_on, geometry_rgb, text_rgb。
//   截图时间约650..1780 ns；text_stage=2，text_y=354。
//   展示中文2倍列号成对重复、x=703/704边界和模式100/101/110/111。
//   当前源码101与110均为透明显示，111为反显。
//
// 显示格式：坐标/计数/列号=Unsigned Decimal；
// 字模地址/行数据/RGB=Hexadecimal；mode/单比特标志=Binary。
// 可先 run 5 us 截图4-3/4-4，再继续运行长时间查看4-1/4-2。
// 所有信号均在tb_chapter4_all顶层，可直接Add to Wave。
// 图4-3/4-4是模块级定点测试，不是全屏扫描，不用于测量板级PCLK。
// 测试使用工程原有四组文字模块、几何模块及混合器，参数与DUT一致。
// 完整DUT使用工程真实时钟IP；未强制locked、计数器或修改工程时序参数。
// 到80 ms时：checks_done=1且checks_pass=1表示本文件自动检查全部通过。
// 模块级结果在约3.6 us完成；error_count为两部分累计错误数。
// 无$finish/$stop，不主动关闭仿真。text_clk完成测试后停止，DUT持续运行。

module tb_chapter4_all;

    // SECTION A: full project, scan / pipeline / startup.

reg       sys_clk   = 1'b0;
    reg       sys_rst_n = 1'b0;
    reg [2:0] mode_sw   = 3'b000;

    wire       lcd_pclk;
    wire       lcd_hsync;
    wire       lcd_vsync;
    wire       lcd_de;
    wire       lcd_bl;
    wire       lcd_rst_n;
    wire [7:0] lcd_r;
    wire [7:0] lcd_g;
    wire [7:0] lcd_b;

    // Read-only aliases at testbench scope.
    wire        pix_clk;
    wire        clk_locked;
    wire        pix_rst_n;
    wire [2:0]  mode;
    wire [10:0] h_count;
    wire [10:0] v_count;
    wire [10:0] pixel_x;
    wire [10:0] pixel_y;
    wire        de_raw;
    wire        hsync_raw;
    wire        vsync_raw;
    wire        de_pipe;
    wire        hsync_pipe;
    wire        vsync_pipe;
    wire [23:0] mixed_rgb;
    wire [23:0] lcd_rgb;
    wire        line_start;
    wire        frame_start;

    integer scan_error_count        = 0;
    integer frames_checked     = 0;
    integer lines_checked      = 0;
    integer line_de_cycles     = 0;
    integer line_hs_low_cycles = 0;
    integer frame_active_lines = 0;
    integer frame_vs_low_lines = 0;
    integer startup_cycles     = 0;
    reg panel_reset_seen       = 1'b0;
    reg backlight_seen         = 1'b0;
    reg scan_checks_done            = 1'b0;
    reg scan_checks_pass            = 1'b0;
    reg expected_de            = 1'b0;
    reg expected_hs            = 1'b1;
    reg expected_vs            = 1'b1;
    reg [23:0] expected_rgb     = 24'h000000;
    realtime pixel_reset_time_ns = 0.0;

    top_lcd_chapter3 dut (
        .sys_clk   (sys_clk),
        .sys_rst_n (sys_rst_n),
        .mode_sw   (mode_sw),
        .lcd_pclk  (lcd_pclk),
        .lcd_hsync (lcd_hsync),
        .lcd_vsync (lcd_vsync),
        .lcd_de    (lcd_de),
        .lcd_bl    (lcd_bl),
        .lcd_rst_n (lcd_rst_n),
        .lcd_r     (lcd_r),
        .lcd_g     (lcd_g),
        .lcd_b     (lcd_b)
    );

    assign pix_clk    = dut.pix_clk;
    assign clk_locked = dut.clk_locked;
    assign pix_rst_n  = dut.pix_rst_n;
    assign mode       = dut.mode;
    assign h_count    = dut.u_lcd_timing.h_count;
    assign v_count    = dut.u_lcd_timing.v_count;
    assign pixel_x    = dut.pixel_x;
    assign pixel_y    = dut.pixel_y;
    assign de_raw    = dut.de_raw;
    assign hsync_raw = dut.hsync_raw;
    assign vsync_raw = dut.vsync_raw;
    assign de_pipe    = dut.de_pipe;
    assign hsync_pipe = dut.hsync_pipe;
    assign vsync_pipe = dut.vsync_pipe;
    assign mixed_rgb = dut.mixed_rgb;
    assign lcd_rgb   = {lcd_r, lcd_g, lcd_b};
    assign line_start  = dut.u_lcd_timing.line_start;
    assign frame_start = dut.u_lcd_timing.frame_start;

    // 20 ns period / 50 MHz, matching this project's clock IP.
    always #10 sys_clk = ~sys_clk;

    initial begin
        $timeformat(-6, 3, " us", 12);
        #200;
        // The actual DUT releases pixel reset through locked -> reset_sync.
        sys_rst_n = 1'b1;
    end

    task report_error;
        input [8*80-1:0] message;
        begin
            if (scan_error_count < 12)
                $display("CH4 ERROR [%0t]: %0s", $time, message);
            scan_error_count = scan_error_count + 1;
        end
    endtask

    initial begin
        wait (clk_locked === 1'b1);
        $display("CH4: clock locked at %0.3f us", $realtime / 1000.0);
        wait (pix_rst_n === 1'b1);
        pixel_reset_time_ns = $realtime;
        $display("CH4: pixel reset released at %0.3f us",
                 $realtime / 1000.0);
    end

    // Validate the RAW scan independently of the output pipeline latency.
    // All widths use the actual timing-module parameters.
    always @(posedge pix_clk) begin
        if (pix_rst_n !== 1'b1) begin
            line_de_cycles     = 0;
            line_hs_low_cycles = 0;
            frame_active_lines = 0;
            frame_vs_low_lines = 0;
            startup_cycles     = 0;
            frames_checked     = 0;
            lines_checked      = 0;
        end else begin
            startup_cycles = startup_cycles + 1;
            if (h_count == 0) begin
                line_de_cycles     = 0;
                line_hs_low_cycles = 0;
                if (v_count == 0) begin
                    frame_active_lines = 0;
                    frame_vs_low_lines = 0;
                end
            end
            if (de_raw === 1'b1)
                line_de_cycles = line_de_cycles + 1;
            if (hsync_raw === 1'b0)
                line_hs_low_cycles = line_hs_low_cycles + 1;

            if (h_count == dut.u_lcd_timing.H_TOTAL - 1) begin
                if (v_count < dut.u_lcd_timing.V_ACTIVE) begin
                    if (line_de_cycles != dut.u_lcd_timing.H_ACTIVE)
                        report_error("RAW DE active-line width is incorrect");
                end else if (line_de_cycles != 0)
                    report_error("RAW DE is high in vertical blanking");
                if (line_hs_low_cycles != dut.u_lcd_timing.H_SYNC)
                    report_error("RAW HS low pulse width is incorrect");
                lines_checked = lines_checked + 1;
                if (line_de_cycles == dut.u_lcd_timing.H_ACTIVE)
                    frame_active_lines = frame_active_lines + 1;
                if (vsync_raw === 1'b0)
                    frame_vs_low_lines = frame_vs_low_lines + 1;
                if (v_count == dut.u_lcd_timing.V_TOTAL - 1) begin
                    if (frame_active_lines != dut.u_lcd_timing.V_ACTIVE)
                        report_error("RAW frame active-line count is incorrect");
                    if (frame_vs_low_lines != dut.u_lcd_timing.V_SYNC)
                        report_error("RAW VS low-line count is incorrect");
                    frames_checked = frames_checked + 1;
                    $display("CH4: frame %0d checked at %0.3f ms; errors=%0d",
                             frames_checked, $realtime / 1000000.0,
                             scan_error_count);
                end
            end
        end
    end

    // Capture the pipeline values BEFORE the clock edge's NBA updates.
    // Wait one precision step, then check this DUT's actual rising-edge
    // output registers. Never compare these outputs directly with RAW DE.
    always @(posedge pix_clk) begin
        if (pix_rst_n === 1'b1) begin
            expected_de  = de_pipe;
            expected_hs  = hsync_pipe;
            expected_vs  = vsync_pipe;
            expected_rgb = de_pipe ? mixed_rgb : 24'h000000;
            #0.001;
            if (lcd_de !== expected_de)
                report_error("registered DE does not match the pipeline");
            if (lcd_hsync !== expected_hs)
                report_error("registered HS does not match the pipeline");
            if (lcd_vsync !== expected_vs)
                report_error("registered VS does not match the pipeline");
            if (lcd_rgb !== expected_rgb)
                report_error("registered RGB does not match the pipeline");
        end
    end

    always @(posedge lcd_rst_n) begin
        if (pix_rst_n === 1'b1) begin
            panel_reset_seen = 1'b1;
            if (startup_cycles != dut.u_lcd_power_seq.RESET_DELAY_CYCLES)
                report_error("panel reset delay cycle count is incorrect");
            $display("CH4: lcd_rst_n at %0.6f ms; delay=%0.6f ms",
                     $realtime / 1000000.0,
                     ($realtime - pixel_reset_time_ns) / 1000000.0);
        end
    end

    always @(posedge lcd_bl) begin
        if (pix_rst_n === 1'b1) begin
            backlight_seen = 1'b1;
            if (lcd_rst_n !== 1'b1)
                report_error("backlight enabled before panel reset release");
            if (startup_cycles != dut.u_lcd_power_seq.BL_DELAY_CYCLES)
                report_error("backlight delay cycle count is incorrect");
            $display("CH4: lcd_bl at %0.6f ms; delay=%0.6f ms",
                     $realtime / 1000000.0,
                     ($realtime - pixel_reset_time_ns) / 1000000.0);
        end
    end

    initial begin
        #80000000;
        if (frames_checked < 1)
            report_error("no full RAW frame observed by 80 ms");
        if (!panel_reset_seen)
            report_error("panel reset release not observed by 80 ms");
        if (!backlight_seen)
            report_error("backlight startup not observed by 80 ms");
        scan_checks_pass = (scan_error_count == 0);
        scan_checks_done = 1'b1;
        $display("CH4 SCAN RESULT: pass=%0b; errors=%0d; frames=%0d; lines=%0d",
                 scan_checks_pass, scan_error_count, frames_checked, lines_checked);
    end

    // SECTION B: original graphics/text modules, controlled pixel tests.

reg text_clk = 1'b0;
    reg text_rst_n = 1'b0;
    reg [10:0] text_x = 11'd0;
    reg [10:0] text_y = 11'd0;
    reg text_de_raw = 1'b0;
    reg [2:0] text_mode = 3'b000;

    // Current project: 101/110 are transparent, 111 is inverse.
    wire [1:0] text_display_mode = (text_mode == 3'b111) ? 2'd2 : 2'd0;
    wire geometry_on;
    wire [23:0] geometry_rgb;
    wire ascii1_layer_on, ascii2_layer_on, cn1_layer_on, cn2_layer_on;
    wire [23:0] ascii1_rgb, ascii2_rgb, cn1_rgb, cn2_rgb;
    wire text_de_pipe, text_hsync_pipe, text_vsync_pipe;
    wire [23:0] text_rgb;

    geometry_renderer u_geometry (
        .lcd_de(text_de_raw), .pixel_x(text_x), .pixel_y(text_y),
        .geometry_on(geometry_on), .geometry_rgb(geometry_rgb)
    );
    ascii_text_line #(
        .SCALE(1), .LINE_X(80), .LINE_Y(35),
        .FG_COLOR(24'hFFFFFF), .BG_COLOR(24'h000000)
    ) u_ascii_1x (
        .pix_clk(text_clk), .rst_n(text_rst_n),
        .pixel_x(text_x), .pixel_y(text_y),
        .display_mode(text_display_mode),
        .text_layer_on(ascii1_layer_on), .text_rgb(ascii1_rgb)
    );
    ascii_text_line #(
        .SCALE(2), .LINE_X(80), .LINE_Y(60),
        .FG_COLOR(24'hFFE060), .BG_COLOR(24'h402010)
    ) u_ascii_2x (
        .pix_clk(text_clk), .rst_n(text_rst_n),
        .pixel_x(text_x), .pixel_y(text_y),
        .display_mode(text_display_mode),
        .text_layer_on(ascii2_layer_on), .text_rgb(ascii2_rgb)
    );
    chinese_text_line #(
        .SCALE(1), .LINE_X(416), .LINE_Y(310),
        .FG_COLOR(24'h40FFFF), .BG_COLOR(24'h000000)
    ) u_cn_1x (
        .pix_clk(text_clk), .rst_n(text_rst_n),
        .pixel_x(text_x), .pixel_y(text_y),
        .display_mode(text_display_mode),
        .text_layer_on(cn1_layer_on), .text_rgb(cn1_rgb)
    );
    chinese_text_line #(
        .SCALE(2), .LINE_X(320), .LINE_Y(350),
        .FG_COLOR(24'hFF80C0), .BG_COLOR(24'h401028)
    ) u_cn_2x (
        .pix_clk(text_clk), .rst_n(text_rst_n),
        .pixel_x(text_x), .pixel_y(text_y),
        .display_mode(text_display_mode),
        .text_layer_on(cn2_layer_on), .text_rgb(cn2_rgb)
    );
    pixel_mixer u_mixer (
        .pix_clk(text_clk), .rst_n(text_rst_n), .mode(text_mode),
        .lcd_de_raw(text_de_raw), .lcd_hsync_raw(1'b1), .lcd_vsync_raw(1'b1),
        .geometry_on_raw(geometry_on), .geometry_rgb_raw(geometry_rgb),
        .ascii_1x_on(ascii1_layer_on), .ascii_1x_rgb(ascii1_rgb),
        .ascii_2x_on(ascii2_layer_on), .ascii_2x_rgb(ascii2_rgb),
        .chinese_1x_on(cn1_layer_on), .chinese_1x_rgb(cn1_rgb),
        .chinese_2x_on(cn2_layer_on), .chinese_2x_rgb(cn2_rgb),
        .lcd_de_pipe(text_de_pipe), .lcd_hsync_pipe(text_hsync_pipe),
        .lcd_vsync_pipe(text_vsync_pipe), .pixel_rgb(text_rgb)
    );

    // Read-only aliases of the original module signals.
    wire [11:0] ascii_font_addr = u_ascii_1x.font_addr;
    wire [7:0] ascii_row_data = u_ascii_1x.font_row_data;
    wire [2:0] ascii_column = u_ascii_1x.u_ascii_renderer.glyph_x_d;
    wire ascii_area = u_ascii_1x.character_area;
    wire ascii_pixel = u_ascii_1x.font_pixel;
    wire [11:0] cn2_font_addr = u_cn_2x.font_addr;
    wire [15:0] cn2_row_data = u_cn_2x.font_row_data;
    wire [3:0] cn2_column = u_cn_2x.u_chinese_renderer.glyph_x_d;
    wire cn2_area = u_cn_2x.line_area_d;
    wire cn2_pixel = u_cn_2x.font_pixel;

    reg [1:0] text_stage = 2'd0;
    integer text_error_count = 0;
    integer points_checked = 0;
    reg text_checks_done = 1'b0;
    reg text_checks_pass = 1'b0;
    // Stop only the module-test clock after its checks; the real DUT keeps running.
    initial begin
        while (!text_checks_done) begin
            #10;
            text_clk = ~text_clk;
        end
    end

    task report_text_error;
        input [8*80-1:0] message;
        begin
            if (text_error_count < 12)
                $display("CH4 TEXT ERROR [%0t]: %0s; x=%0d y=%0d text_mode=%b RGB=%h",
                    $time, message, text_x, text_y, text_mode, text_rgb);
            text_error_count = text_error_count + 1;
        end
    endtask

    // Each point lasts 80 ns; drive away from ROM sampling edges.
    task drive_point;
        input [10:0] x_value;
        input [10:0] y_value;
        input [2:0] mode_value;
        input de_value;
        input [23:0] expected_rgb;
        begin
            @(negedge text_clk);
            text_x = x_value;
            text_y = y_value;
            text_mode = mode_value;
            text_de_raw = de_value;
            repeat (4) @(posedge text_clk);
            #1;
            if (text_rgb !== expected_rgb)
                report_text_error("unexpected composited RGB at test point");
            if (text_de_pipe !== de_value)
                report_text_error("DE did not settle with the sampled pixel");
            points_checked = points_checked + 1;
        end
    endtask

    // Snapshot pre-edge inputs, inspect synchronous outputs after NBA updates.
    reg [7:0] expected_ascii_row;
    reg [15:0] expected_cn2_row;
    reg [2:0] expected_ascii_column;
    reg [3:0] expected_cn2_column;
    reg expected_ascii_area, expected_cn2_area;
    always @(posedge text_clk) begin
        if (text_rst_n === 1'b1) begin
            expected_ascii_row = u_ascii_1x.u_ascii_font_rom.bitmap[
                127 - (ascii_font_addr[3:0] * 8) -: 8];
            expected_cn2_row = u_cn_2x.u_chinese_font_rom.font_mem[
                cn2_font_addr[8:0]];
            expected_ascii_column = u_ascii_1x.u_ascii_renderer.glyph_x_now;
            expected_cn2_column = u_cn_2x.u_chinese_renderer.glyph_x_now;
            expected_ascii_area = u_ascii_1x.u_ascii_renderer.area_now;
            expected_cn2_area = u_cn_2x.line_area_now;
            #0.001;
            if (ascii_row_data !== expected_ascii_row ||
                ascii_column !== expected_ascii_column ||
                ascii_area !== expected_ascii_area)
                report_text_error("ASCII ROM, area and column are not edge-aligned");
            if (cn2_row_data !== expected_cn2_row ||
                cn2_column !== expected_cn2_column ||
                cn2_area !== expected_cn2_area)
                report_text_error("Chinese ROM, area and column are not edge-aligned");
        end
    end

    initial begin
        $timeformat(-9, 3, " ns", 12);
        repeat (4) @(negedge text_clk);
        text_rst_n = 1'b1;

        // Figure 4-3: ASCII X/C, ROM row changes and character boundary.
        text_stage = 2'd1;
        drive_point(79, 37, 3'b001, 1'b1, 24'h081828);
        if (ascii_row_data !== 8'hC3)
            report_text_error("unexpected X glyph row 2");
        drive_point(80, 37, 3'b001, 1'b1, 24'hFFFFFF);
        drive_point(87, 37, 3'b001, 1'b1, 24'hFFFFFF);
        drive_point(88, 37, 3'b001, 1'b1, 24'h081828);
        drive_point(88, 38, 3'b001, 1'b1, 24'h081828);
        drive_point(89, 38, 3'b001, 1'b1, 24'hFFFFFF);
        drive_point(89, 39, 3'b001, 1'b1, 24'hFFFFFF);

        // Figure 4-4: Chinese 2x, x=[320,704), y=[350,382).
        text_stage = 2'd2;
        drive_point(320, 354, 3'b100, 1'b1, 24'hFF80C0);
        if (cn2_row_data !== 16'hFC80)
            report_text_error("Chinese font data missing or unexpected glyph row");
        drive_point(321, 354, 3'b100, 1'b1, 24'hFF80C0);
        drive_point(322, 354, 3'b100, 1'b1, 24'hFF80C0);
        drive_point(323, 354, 3'b100, 1'b1, 24'hFF80C0);
        drive_point(350, 354, 3'b100, 1'b1, 24'h081828);
        drive_point(351, 354, 3'b100, 1'b1, 24'h081828);
        drive_point(352, 354, 3'b100, 1'b1, 24'h081828);
        drive_point(703, 354, 3'b100, 1'b1, 24'h102820);
        drive_point(704, 354, 3'b100, 1'b1, 24'h40FF80);
        drive_point(351, 354, 3'b101, 1'b1, 24'h081828);
        drive_point(351, 354, 3'b110, 1'b1, 24'h081828);
        drive_point(351, 354, 3'b111, 1'b1, 24'hFF80C0);
        drive_point(320, 354, 3'b111, 1'b1, 24'h401028);
        drive_point(704, 354, 3'b111, 1'b1, 24'h40FF80);

        // Other chapter-specific boundary checks; no extra screenshot required.
        text_stage = 2'd3;
        drive_point(79, 220, 3'b000, 1'b1, 24'h081828);
        drive_point(80, 220, 3'b000, 1'b1, 24'hFF8C20);
        drive_point(299, 220, 3'b000, 1'b1, 24'hFF8C20);
        drive_point(300, 220, 3'b000, 1'b1, 24'h081828);
        drive_point(359, 220, 3'b000, 1'b1, 24'h081828);
        drive_point(360, 220, 3'b000, 1'b1, 24'h20C8FF);
        drive_point(500, 220, 3'b000, 1'b1, 24'h20C8FF);
        drive_point(501, 220, 3'b000, 1'b1, 24'h081828);
        drive_point(575, 150, 3'b000, 1'b1, 24'h102820);
        drive_point(576, 150, 3'b000, 1'b1, 24'h40FF80);
        drive_point(944, 150, 3'b000, 1'b1, 24'h081828);
        drive_point(255, 37, 3'b001, 1'b1, 24'hFFFFFF);
        drive_point(256, 37, 3'b001, 1'b1, 24'h081828);
        drive_point(80, 64, 3'b010, 1'b1, 24'hFFE060);
        drive_point(81, 64, 3'b010, 1'b1, 24'hFFE060);
        drive_point(431, 64, 3'b010, 1'b1, 24'hFFE060);
        drive_point(432, 64, 3'b010, 1'b1, 24'h081828);
        // Existing SCALE=1 Chinese renderer applies its one-bit bolding.
        drive_point(425, 312, 3'b011, 1'b1, 24'h40FFFF);
        drive_point(431, 312, 3'b011, 1'b1, 24'h081828);
        drive_point(607, 312, 3'b011, 1'b1, 24'h102820);
        drive_point(608, 312, 3'b011, 1'b1, 24'h40FF80);
        drive_point(80, 220, 3'b000, 1'b0, 24'h000000);

        text_checks_pass = (text_error_count == 0);
        text_checks_done = 1'b1;
        $display("CH4 TEXT RESULT: pass=%0d text_error_count=%0d points=%0d",
            text_checks_pass, text_error_count, points_checked);
    end

    // Combined result for all four figures.
    wire [31:0] error_count = scan_error_count + text_error_count;
    wire checks_done = scan_checks_done && text_checks_done;
    wire checks_pass = checks_done && scan_checks_pass && text_checks_pass;

    always @(posedge checks_done) begin
        #0.001;
        $display("CH4 ALL RESULT: pass=%0b errors=%0d scan_errors=%0d text_errors=%0d frames=%0d points=%0d",
            checks_pass, error_count, scan_error_count, text_error_count,
            frames_checked, points_checked);
    end

endmodule

