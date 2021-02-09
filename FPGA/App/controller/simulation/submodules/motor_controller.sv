/*******************************************************************************

# Register Map

### STATUS (0x0) Read Only
|  15 ~ 3  |     2     |      1      |      0       |
|----------|-----------|-------------|--------------|
| reserved | DRV_OTW_N | DRV_FAULT_N | HALL_FAULT_N |

それぞれstatus_driver_otw_n, status_driver_fault_n, status_hall_fault_nの現在値。


### INTFLAG (0x2) Read Only
|  15 ~ 3  |     2     |      1      |      0       |
|----------|-----------|-------------|--------------|
| reserved | DRV_OTW_N | DRV_FAULT_N | HALL_FAULT_N |

平常時は1でSTATUSの各ビットが0になる瞬間があったら対応するビットが0にクリアされる。
DRV_OTW_NかDRV_FAULT_Nがクリアされているとフォルト出力と割り込み出力がアサートになる。
FAULT.FAULT_CLRに1を書き込むと全てのビットが1に戻る。


### FAULT (0x4) Write Only
|  15 ~ 2  |     1     |     0     |
|----------|-----------|-----------|
| reserved | FAULT_SET | FAULT_CLR |

FAULT_CLRに1を書き込むとINTFLAGが1にセットされ、フォルト出力と割り込み出力がデアサートされる。
FAULT_SETに1を書き込むとフォルト出力と割り込み出力がアサートされる。


### POWER (0x6) Read/Write
|      15 ~ 0     |
|-----------------|
|  PWM ON cycles  |
| (-2985 ~ +2985) |

PWMドライバに指令値を送る。
PWMデューティ比はPOWERを3000で割った値になる。


*******************************************************************************/

module motor_controller (
        input  wire        clk,
        input  wire        reset,
		output reg         fault,
        input  wire        status_driver_otw_n,
		input  wire        status_driver_fault_n,
		input  wire        status_hall_fault_n,
        output reg  [15:0] pwm_source_data,
        output reg         pwm_source_valid,
        input  wire        pwm_source_ready,
        input  wire [1:0]  slave_address,
        output reg  [15:0] slave_readdata,
        input  wire [15:0] slave_writedata,
        input  wire        slave_read,
        input  wire        slave_write,
		output wire        irq
    );
    
    // Fault output
    assign irq = fault;
    logic fault_set, fault_clear;
    logic status_driver_otw_n_latch = 1'b1;
    logic status_driver_fault_n_latch = 1'b1;
    logic status_hall_fault_n_latch = 1'b1;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            fault <= 1'b0;
            status_driver_otw_n_latch <= 1'b1;
            status_driver_fault_n_latch <= 1'b1;
            status_hall_fault_n_latch <= 1'b1;
        end
        else begin
            if (~status_driver_otw_n_latch | ~status_driver_fault_n_latch) begin
                // モータードライバの異常検知に対しフォルト出力をアサートする
                fault <= 1'b1;
            end
            else if (fault_clear == 1'b1) begin
                fault <= 1'b0;
            end
            else if (fault_set == 1'b1) begin
                fault <= 1'b1;
            end
            status_driver_otw_n_latch   <= (fault_clear ? 1'b1 : status_driver_otw_n_latch  ) & status_driver_otw_n;
            status_driver_fault_n_latch <= (fault_clear ? 1'b1 : status_driver_fault_n_latch) & status_driver_fault_n;
            status_hall_fault_n_latch   <= (fault_clear ? 1'b1 : status_hall_fault_n_latch  ) & status_hall_fault_n;
        end
    end
    
    // Avalon-MM
    typedef enum {
        REGISTER_STATUS   = 'h0,
        REGISTER_INTFLAG  = 'h1, 
        REGISTER_FAULT    = 'h2,
        REGISTER_POWER    = 'h3
    } REGISTER_MAP;
    always @(posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            pwm_source_data <= '0;
            pwm_source_valid <= 1'b0;
        end
        else begin
            fault_set <= 1'b0;
            fault_clear <= 1'b0;
            if (pwm_source_valid & pwm_source_ready) begin
                pwm_source_valid <= 1'b0;
            end
            if (slave_read == 1'b1) begin
                case (slave_address)
                    REGISTER_STATUS  : slave_readdata <= {13'h0000, status_driver_otw_n, status_driver_fault_n, status_hall_fault_n};
                    REGISTER_INTFLAG : slave_readdata <= {13'h0000, status_driver_otw_n_latch, status_driver_fault_n_latch, status_hall_fault_n_latch};
                    REGISTER_FAULT   : slave_readdata <= 16'h0000;
                    REGISTER_POWER   : slave_readdata <= pwm_source_data;
                    default          : slave_readdata <= 16'h0000;
                endcase
            end
            if (slave_write == 1'b1) begin
                if (slave_address == REGISTER_FAULT) begin
                    fault_set <= slave_writedata[1];
                    fault_clear <= slave_writedata[0];
                end
                if (slave_address == REGISTER_POWER) begin
                    pwm_source_data <= slave_writedata[15:0];
                    pwm_source_valid <= 1'b1;
                end
            end
            
        end
    end
endmodule
