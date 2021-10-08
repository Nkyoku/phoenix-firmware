/**
 * @file vector_controller_master.sv
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************

# Register Map

### STATUS (0x0) Read Only
|      15     |      14     |      13     |      12     |       11      |       10      |       9       |       8       |
|-------------|-------------|-------------|-------------|---------------|---------------|---------------|---------------|
| DRV_OTW_4_N | DRV_OTW_3_N | DRV_OTW_2_N | DRV_OTW_1_N | DRV_FAULT_4_N | DRV_FAULT_3_N | DRV_FAULT_2_N | DRV_FAULT_1_N |

|        7       |        6       |        5       |        4       |       3       |       2       |       1       |       0       |
|----------------|----------------|----------------|----------------|---------------|---------------|---------------|---------------|
| HALL_FAULT_4_N | HALL_FAULT_3_N | HALL_FAULT_2_N | HALL_FAULT_1_N | ENC_FAULT_4_N | ENC_FAULT_3_N | ENC_FAULT_2_N | ENC_FAULT_1_N |

それぞれstatus_driver_otw_n[4:1], status_driver_fault_n[4:1], status_hall_fault_n[4:1], status_encoder_fault_n[4:1]の現在値。


### INTFLAG (0x2) Read Only
|      15     |      14     |      13     |      12     |       11      |       10      |       9       |       8       |
|-------------|-------------|-------------|-------------|---------------|---------------|---------------|---------------|
| DRV_OTW_4_N | DRV_OTW_3_N | DRV_OTW_2_N | DRV_OTW_1_N | DRV_FAULT_4_N | DRV_FAULT_3_N | DRV_FAULT_2_N | DRV_FAULT_1_N |

|        7       |        6       |        5       |        4       |          3 ~ 0           |
|----------------|----------------|----------------|----------------|--------------------------|
| HALL_FAULT_4_N | HALL_FAULT_3_N | HALL_FAULT_2_N | HALL_FAULT_1_N | reserved (Read as all 1) |

平常時は全てのビットが1だがSTATUSの各ビットが0になる瞬間があったら対応するビットが0にクリアされる。
DRV_OTW_[n]_N, DRV_FAULT_[n]_N, HALL_FAULT_[n]_N (n=1...4)のいずれかのビットがクリアされた瞬間にフォルト出力と割り込み出力がアサートされる。
読むと全てのビットが1にセットされる。


### FAULT (0x4) Read/Write
|  15 ~ 10 |      9      |      8      |      7      |      6      |      5      |      4      |      3      |      2      |     1     |     0     |
|----------|-------------|-------------|-------------|-------------|-------------|-------------|-------------|-------------|-----------|-----------|
| reserved | BRAKE_4_CLR | BRAKE_4_SET | BRAKE_3_CLR | BRAKE_3_SET | BRAKE_2_CLR | BRAKE_2_SET | BRAKE_1_CLR | BRAKE_1_SET | FAULT_CLR | FAULT_SET |

FAULT_CLRに1を書き込むとフォルト出力がデアサートされる。
FAULT_SETに1を書き込むとフォルト出力がアサートされる。
FAULT_CLRよりFAULT_SETのほうが優先される。
現在のフォルト出力の状態がFAULT_SETの値として読める。

BRAKE_[n]_CLRに1を書き込むとショートブレーキが解除される。
BRAKE_[n]_SETに1を書き込むとショートブレーキになる。
BRAKE_[n]_CLRよりBRAKE_[n]_SETのほうが優先される。
現在のショートブレーキの状態はBRAKE_[n]_SETの値として読める。


### POSITION (0x6) Read Only
|  15 ~ 8  |      7      |      6      |      5      |      4      |        3        |        2        |        1        |        0        |
|----------|-------------|-------------|-------------|-------------|-----------------|-----------------|-----------------|-----------------|
| reserved | POS_ERROR_4 | POS_ERROR_3 | POS_ERROR_2 | POS_ERROR_1 | POS_UNCERTAIN_4 | POS_UNCERTAIN_3 | POS_UNCERTAIN_2 | POS_UNCERTAIN_1 |

POS_ERROR_[n]が1のとき回転子の角度の取得にエラーが起きていることを示す。
ホールセンサーの出力が不正なときや不正な遷移をしたときに有効になる。
POS_UNCERTAIN_[n]が1のとき回転子の角度の推定が不確定なことを示す。
ホールセンサーによって大まかな角度が得られているがエンコーダの値が推定に用いられていないときに有効になる。


### ENCODER[n] (0x8, 0xA, 0xC, 0xE) Read Only
|        15 ~ 0        |
|----------------------|
| ENCODER_[n] (signed) |

速度制御周期(1ms)における各エンコーダのパルス検知回数。


### IMEASD[n] (0x10, 0x14, 0x18, 0x1C) Read Only
|               15 ~ 0               |
|------------------------------------|
| CURRENT_MEASUREMENT_D_[n] (signed) |

d相電流の測定値。


### IMEASQ[n] (0x12, 0x16, 0x1A, 0x1E) Read Only
|               15 ~ 0               |
|------------------------------------|
| CURRENT_MEASUREMENT_Q_[n] (signed) |

q相電流の測定値。


### IREFD[n] (0x20, 0x24, 0x28, 0x2C) Read/Write
|              15 ~ 0              |
|----------------------------------|
| CURRENT_REFERENCE_D_[n] (signed) |

d相電流の指令値。


### IREFQ[n] (0x22, 0x26, 0x2A, 0x2E) Read/Write
|              15 ~ 0              |
|----------------------------------|
| CURRENT_REFERENCE_Q_[n] (signed) |

q相電流の指令値。


### KP (0x30) Read/Write
|        15 ~ 0       |
|---------------------|
| PARAM_KP (unsigned) |

電流制御の比例ゲイン。
vector_controllerのGAIN_SCALEパラメータの分の桁数の下駄を履いた固定小数点数となる。
すなわち Kp = PARAM_KP / 2**GAIN_SCALE


### KI (0x32) Read/Write
|        15 ~ 0       |
|---------------------|
| PARAM_KI (unsigned) |

電流制御の積分ゲイン。
KPレジスタと同様に固定小数点数である。

*******************************************************************************/

module vector_controller_master (
		input  wire        clk,
		input  wire        reset,
		output reg         fault,
		output reg [3:0]   brake,
		input  wire [3:0]  status_driver_otw_n,
		input  wire [3:0]  status_driver_fault_n,
		input  wire [3:0]  status_hall_fault_n,
		input  wire [3:0]  status_encoder_fault_n,
        input  wire [3:0]  status_pos_error,
        input  wire [3:0]  status_pos_uncertain,
        output reg  [15:0] param_kp,
        output reg  [15:0] param_ki,
		input  wire [15:0] encoder_1_data,
		input  wire [15:0] encoder_2_data,
		input  wire [15:0] encoder_3_data,
		input  wire [15:0] encoder_4_data,
		input  wire [31:0] current_measurement_1_data,
		input  wire        current_measurement_1_valid,
		input  wire [31:0] current_measurement_2_data,
		input  wire        current_measurement_2_valid,
		input  wire [31:0] current_measurement_3_data,
		input  wire        current_measurement_3_valid,
		input  wire [31:0] current_measurement_4_data,
		input  wire        current_measurement_4_valid,
		output wire [31:0] current_reference_1_data,
		output reg         current_reference_1_valid,
		output wire [31:0] current_reference_2_data,
		output reg         current_reference_2_valid,
		output wire [31:0] current_reference_3_data,
		output reg         current_reference_3_valid,
		output wire [31:0] current_reference_4_data,
		output reg         current_reference_4_valid,
		input  wire [4:0]  slave_address,
		input  wire        slave_read,
		output reg  [15:0] slave_readdata,
		input  wire        slave_write,
		input  wire [15:0] slave_writedata,
        output reg         irq
	);

    // Register Address Map
    typedef enum {
        REGISTER_STATUS   = 'h00,
        REGISTER_INTFLAG  = 'h01, 
        REGISTER_FAULT    = 'h02,
        REGISTER_POSITION = 'h03,
        REGISTER_ENCODER1 = 'h04,
        REGISTER_ENCODER2 = 'h05,
        REGISTER_ENCODER3 = 'h06,
        REGISTER_ENCODER4 = 'h07,
        REGISTER_IMEASD1  = 'h08,
        REGISTER_IMEASQ1  = 'h09,
        REGISTER_IMEASD2  = 'h0A,
        REGISTER_IMEASQ2  = 'h0B,
        REGISTER_IMEASD3  = 'h0C,
        REGISTER_IMEASQ3  = 'h0D,
        REGISTER_IMEASD4  = 'h0E,
        REGISTER_IMEASQ4  = 'h0F,
        REGISTER_IREFD1   = 'h10,
        REGISTER_IREFQ1   = 'h11,
        REGISTER_IREFD2   = 'h12,
        REGISTER_IREFQ2   = 'h13,
        REGISTER_IREFD3   = 'h14,
        REGISTER_IREFQ3   = 'h15,
        REGISTER_IREFD4   = 'h16,
        REGISTER_IREFQ4   = 'h17,
        REGISTER_KP       = 'h18,
        REGISTER_KI       = 'h19
    } REGISTER_MAP;

    // Fault
    logic fault_set = 1'b0;
    logic fault_clear = 1'b0;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            fault <= 1'b0;
        end
        else begin
            if ((status_driver_otw_n != '1) | (status_driver_fault_n != '1) | (status_hall_fault_n != '1)) begin
                fault <= 1'b1;
            end
            else begin
                fault <= (fault & ~fault_clear) | fault_set;
            end
        end
    end
    
    // Brake
    logic [3:0] brake_set = '0;
    logic [3:0] brake_clear = '0;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            brake <= '0;
        end
        else begin
            brake <= (brake & ~brake_clear) | brake_set;
        end
    end

    // Interrupt Flag
    logic [3:0] status_driver_otw_n_ff = '1;
    logic [3:0] status_driver_fault_n_ff = '1;
    logic [3:0] status_hall_fault_n_ff = '1;
    logic [3:0] intflag_driver_otw_n = '1;
    logic [3:0] intflag_driver_fault_n = '1;
    logic [3:0] intflag_hall_fault_n = '1;
    wire intflag_clear = slave_read & (slave_address == REGISTER_INTFLAG);
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            status_driver_otw_n_ff <= '1;
            status_driver_fault_n_ff <= '1;
            status_hall_fault_n_ff <= '1;
            intflag_driver_otw_n <= '1;
            intflag_driver_fault_n <= '1;
            intflag_hall_fault_n <= '1;
            irq <= 1'b0;
        end
        else begin
            status_driver_otw_n_ff    <= {4{fault_clear}} | status_driver_otw_n;
            status_driver_fault_n_ff  <= {4{fault_clear}} | status_driver_fault_n;
            status_hall_fault_n_ff    <= {4{fault_clear}} | status_hall_fault_n;
            intflag_driver_otw_n   <= ({4{intflag_clear}} | intflag_driver_otw_n  ) & (status_driver_otw_n   | ~status_driver_otw_n_ff  );
            intflag_driver_fault_n <= ({4{intflag_clear}} | intflag_driver_fault_n) & (status_driver_fault_n | ~status_driver_fault_n_ff);
            intflag_hall_fault_n   <= ({4{intflag_clear}} | intflag_hall_fault_n  ) & (status_hall_fault_n   | ~status_hall_fault_n_ff  );
            irq <= (intflag_driver_otw_n != '1) | (intflag_driver_fault_n != '1) | (intflag_hall_fault_n != '1);
        end
    end

    // Current measurement
    logic signed [15:0] motor_current_meas_d [1:4];
    logic signed [15:0] motor_current_meas_q [1:4];
    always @(posedge clk) begin
        if (current_measurement_1_valid == 1'b1) begin
            motor_current_meas_d[1] <= current_measurement_1_data[31:16];
            motor_current_meas_q[1] <= current_measurement_1_data[15:0];
        end
        if (current_measurement_2_valid == 1'b1) begin
            motor_current_meas_d[2] <= current_measurement_2_data[31:16];
            motor_current_meas_q[2] <= current_measurement_2_data[15:0];
        end
        if (current_measurement_3_valid == 1'b1) begin
            motor_current_meas_d[3] <= current_measurement_3_data[31:16];
            motor_current_meas_q[3] <= current_measurement_3_data[15:0];
        end
        if (current_measurement_4_valid == 1'b1) begin
            motor_current_meas_d[4] <= current_measurement_4_data[31:16];
            motor_current_meas_q[4] <= current_measurement_4_data[15:0];
        end
    end
    
    // Avalon-MM
    logic signed [15:0] motor_current_ref_d [1:4] = '{16'h0000, 16'h0000, 16'h0000, 16'h0000};
    logic signed [15:0] motor_current_ref_q [1:4] = '{16'h0000, 16'h0000, 16'h0000, 16'h0000};
    assign current_reference_1_data = {motor_current_ref_d[1], motor_current_ref_q[1]};
    assign current_reference_2_data = {motor_current_ref_d[2], motor_current_ref_q[2]};
    assign current_reference_3_data = {motor_current_ref_d[3], motor_current_ref_q[3]};
    assign current_reference_4_data = {motor_current_ref_d[4], motor_current_ref_q[4]};
    always @(posedge clk, posedge reset) begin
        int i;
        if (reset == 1'b1) begin
            fault_set <= 1'b0;
            fault_clear <= 1'b0;
            brake_set <= '0;
            brake_clear <= '0;
            param_kp <= '0;
            param_ki <= '0;
            current_reference_1_valid <= 1'b0;
            current_reference_2_valid <= 1'b0;
            current_reference_3_valid <= 1'b0;
            current_reference_4_valid <= 1'b0;
            for (i = 1; i <= 4; i = i + 1) begin
                motor_current_ref_d[i] <= '0;
                motor_current_ref_q[i] <= '0;
            end
        end
        else begin
            fault_set <= 1'b0;
            fault_clear <= 1'b0;
            brake_set <= '0;
            brake_clear <= '0;
            current_reference_1_valid <= 1'b0;
            current_reference_2_valid <= 1'b0;
            current_reference_3_valid <= 1'b0;
            current_reference_4_valid <= 1'b0;
            if (slave_read == 1'b1) begin
                case (slave_address)
                    REGISTER_STATUS   : slave_readdata <= {status_driver_otw_n, status_driver_fault_n, status_hall_fault_n, status_encoder_fault_n};
                    REGISTER_INTFLAG  : slave_readdata <= {intflag_driver_otw_n, intflag_driver_fault_n, intflag_hall_fault_n, 4'hF};
                    REGISTER_FAULT    : slave_readdata <= {7'h00, brake[3], 1'b0, brake[2], 1'b0, brake[1], 1'b0, brake[0], 1'b0, fault};
                    REGISTER_POSITION : slave_readdata <= {8'h00, status_pos_error, status_pos_uncertain};
                    REGISTER_ENCODER1 : slave_readdata <= encoder_1_data;
                    REGISTER_ENCODER2 : slave_readdata <= encoder_2_data;
                    REGISTER_ENCODER3 : slave_readdata <= encoder_3_data;
                    REGISTER_ENCODER4 : slave_readdata <= encoder_4_data;
                    REGISTER_IMEASD1  : slave_readdata <= motor_current_meas_d[1];
                    REGISTER_IMEASQ1  : slave_readdata <= motor_current_meas_q[1];
                    REGISTER_IMEASD2  : slave_readdata <= motor_current_meas_d[2];
                    REGISTER_IMEASQ2  : slave_readdata <= motor_current_meas_q[2];
                    REGISTER_IMEASD3  : slave_readdata <= motor_current_meas_d[3];
                    REGISTER_IMEASQ3  : slave_readdata <= motor_current_meas_q[3];
                    REGISTER_IMEASD4  : slave_readdata <= motor_current_meas_d[4];
                    REGISTER_IMEASQ4  : slave_readdata <= motor_current_meas_q[4];
                    REGISTER_IREFD1   : slave_readdata <= motor_current_ref_d[1];
                    REGISTER_IREFQ1   : slave_readdata <= motor_current_ref_q[1];
                    REGISTER_IREFD2   : slave_readdata <= motor_current_ref_d[2];
                    REGISTER_IREFQ2   : slave_readdata <= motor_current_ref_q[2];
                    REGISTER_IREFD3   : slave_readdata <= motor_current_ref_d[3];
                    REGISTER_IREFQ3   : slave_readdata <= motor_current_ref_q[3];
                    REGISTER_IREFD4   : slave_readdata <= motor_current_ref_d[4];
                    REGISTER_IREFQ4   : slave_readdata <= motor_current_ref_q[4];
                    REGISTER_KP       : slave_readdata <= param_kp;
                    REGISTER_KI       : slave_readdata <= param_ki;
                    default           : slave_readdata <= 16'h0000;
                endcase
            end
            if (slave_write == 1'b1) begin
                if (slave_address == REGISTER_FAULT) begin
                    fault_set <= slave_writedata[0];
                    fault_clear <= slave_writedata[1];
                    brake_set[0] <= slave_writedata[2];
                    brake_clear[0] <= slave_writedata[3];
                    brake_set[1] <= slave_writedata[4];
                    brake_clear[1] <= slave_writedata[5];
                    brake_set[2] <= slave_writedata[6];
                    brake_clear[2] <= slave_writedata[7];
                    brake_set[3] <= slave_writedata[8];
                    brake_clear[3] <= slave_writedata[9];
                end
                if (slave_address == REGISTER_IREFD1) begin
                    motor_current_ref_d[1] <= slave_writedata;
                end
                if (slave_address == REGISTER_IREFQ1) begin
                    motor_current_ref_q[1] <= slave_writedata;
                    current_reference_1_valid <= 1'b1; // qの値を書き込んだ時に転送する (dは通常0のため)
                end
                if (slave_address == REGISTER_IREFD2) begin
                    motor_current_ref_d[2] <= slave_writedata;
                end
                if (slave_address == REGISTER_IREFQ2) begin
                    motor_current_ref_q[2] <= slave_writedata;
                    current_reference_2_valid <= 1'b1;
                end
                if (slave_address == REGISTER_IREFD3) begin
                    motor_current_ref_d[3] <= slave_writedata;
                end
                if (slave_address == REGISTER_IREFQ3) begin
                    motor_current_ref_q[3] <= slave_writedata;
                    current_reference_3_valid <= 1'b1;
                end
                if (slave_address == REGISTER_IREFD4) begin
                    motor_current_ref_d[4] <= slave_writedata;
                end
                if (slave_address == REGISTER_IREFQ4) begin
                    motor_current_ref_q[4] <= slave_writedata;
                    current_reference_4_valid <= 1'b1;
                end
                if (slave_address == REGISTER_KP) begin
                    param_kp = slave_writedata;
                end
                if (slave_address == REGISTER_KI) begin
                    param_ki = slave_writedata;
                end
            end
        end
    end
endmodule
