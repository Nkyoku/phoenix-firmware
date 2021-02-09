`timescale 1 ns / 1 ps
module test ();
    logic reset = 1'b1;
    logic clk = 1'b0;
    
    real theta_deg = 0.0; // degree
    int theta = 0;
    
    logic [2:0] hall_uvw;
    logic enc_a, enc_b;
    logic qdec_inc;
    logic qdec_dec;
    
    logic error_test = 1'b0;
    
    always @(theta_deg) begin
        theta = $rtoi(8000 * theta_deg) % 2880000;
        if (theta < 0) theta = theta + 2880000;
        hall_uvw[2] <= ~error_test ? ((theta < 720000) | (2160000 <= theta)) : 1'b0;
        hall_uvw[1] <= ~error_test ? ((240000 < theta) & (theta <= 1680000)) : 1'b0;
        hall_uvw[0] <= ~error_test ? ((1200000 < theta) & (theta <= 2640000)) : 1'b0;
        enc_a <= (theta / 11250) % 2;
        enc_b <= ((theta + 5625) / 11250) % 2;
    end
    
    quadrature_decoder #(
        .INVERSE(1),
        .DATA_WIDTH(16)
    ) qdec (
        .clk(clk),
        .reset(reset),
        .latch(1'b0),
        .enc_a(enc_a),
        .enc_b(enc_b),
        .inc(qdec_inc),
        .dec(qdec_dec),
        .counter()
    );
    
    position_estimator #(
        .THETA_WIDTH(9)
    ) est (
        .clk(clk),
        .reset(reset),
        .hall_uvw(hall_uvw),
        .qdec_inc(qdec_inc),
        .qdec_dec(qdec_dec),
        .theta_data(),
        .theta_error(),
        .theta_uncertain()
    );
    
    /*initial begin
        @(negedge reset);
        repeat(1) @(posedge clk);
        in_ref_valid <= 1'b1;
        in_proc_valid <= 1'b1;
        in_ref_data <= 1000;
        in_proc_data <= 0;
        @(posedge clk);
        in_ref_valid <= 1'b0;
        in_proc_valid <= 1'b0;
        in_ref_data <= 'X;
        in_proc_data <= 'X;
        repeat(2) @(posedge clk);
        
        fork
            repeat(30) begin
                trigger <= 1'b1;
                @(posedge clk);
                trigger <= 1'b0;
                repeat(6) @(posedge clk);
            end
            begin
                repeat(20) @(posedge trigger);
                repeat(3) @(posedge clk);
                reset <= 1'b1;
                @(posedge clk);
                reset <= 1'b0;
                repeat(2) @(posedge trigger);
                repeat(7) @(posedge clk);
                in_ref_valid <= 1'b1;
                in_proc_valid <= 1'b1;
                in_ref_data <= 1000;
                @(posedge clk);
                in_ref_valid <= 1'b0;
                in_proc_valid <= 1'b0;
                in_ref_data <= 'X;
            end
        join
        
        $stop;
    end*/
    
    logic dir = 1'b0;
    always @(posedge clk) begin
        if (dir == 1'b0) begin
            theta_deg <= theta_deg + (1.0 / 8.0);
            if (360.0 <= theta_deg) begin
                dir <= 1'b1;
            end
        end
        else begin
            theta_deg <= theta_deg - (1.0 / 8.0);
            if (theta_deg <= 0.0) begin
                dir <= 1'b0;
            end
        end
    end
    
    initial begin
        repeat(4000) @(negedge clk);
        repeat(10) begin
            error_test <= 1'b1;
            repeat(100) @(negedge clk);
            error_test <= 1'b0;
            repeat(400) @(negedge clk);
        end
    end
    
    // Clock generation
    always #5ns begin
        clk <= ~clk;
    end
    
    // Reset generaton
    initial begin
        reset <= 1'b1;
        repeat(5) @(negedge clk);
        reset <= 1'b0;
        repeat(10000) @(negedge clk);
        $stop;
    end
endmodule
