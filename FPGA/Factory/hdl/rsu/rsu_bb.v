
module rsu (
	busy,
	data_out,
	param,
	read_param,
	reconfig,
	reset_timer,
	read_source,
	clock,
	reset,
	write_param,
	data_in);	

	output		busy;
	output	[28:0]	data_out;
	input	[2:0]	param;
	input		read_param;
	input		reconfig;
	input		reset_timer;
	input	[1:0]	read_source;
	input		clock;
	input		reset;
	input		write_param;
	input	[23:0]	data_in;
endmodule
