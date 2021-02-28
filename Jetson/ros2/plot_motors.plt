set datafile separator ","
set xzeroaxis
set yzeroaxis
set xlabel "time [ms]"
set multiplot layout 3,1
set ylabel "Velocity [m/s]"
plot "test.csv" using 1:2 with lines title "Velocity 1 Meas",\
     ""         using 1:3 with lines title "Velocity 2 Meas",\
     ""         using 1:4 with lines title "Velocity 3 Meas",\
     ""         using 1:5 with lines title "Velocity 4 Meas",\
     ""         using 1:10 with lines title "Velocity 1 Ref",\
     ""         using 1:11 with lines title "Velocity 2 Ref",\
     ""         using 1:12 with lines title "Velocity 3 Ref",\
     ""         using 1:13 with lines title "Velocity 4 Ref"
set ylabel "Current [A]"
plot "test.csv" using 1:6 with lines title "Current 1 Meas",\
     ""         using 1:7 with lines title "Current 2 Meas",\
     ""         using 1:8 with lines title "Current 3 Meas",\
     ""         using 1:9 with lines title "Current 4 Meas",\
     ""         using 1:14 with lines title "Current 1 Ref",\
     ""         using 1:15 with lines title "Current 2 Ref",\
     ""         using 1:16 with lines title "Current 3 Ref",\
     ""         using 1:17 with lines title "Current 4 Ref"
#set ylabel "Acceleration [m/s2]"
#plot "test.csv" using 1:18 with lines title "Current Limit 1",\
#     ""         using 1:19 with lines title "Current Limit 2",\
#     ""         using 1:20 with lines title "Current Limit 3",\
#     ""         using 1:21 with lines title "Current Limit 4"
set ylabel "Velocity [m/s]"
set y2label "Rotation [rad/s]"
set y2tics
set yrange [-5:5]
set y2range [-20:20]
set ytics nomirror
plot "test.csv" using 1:18 with lines title "Vx",\
     ""         using 1:19 with lines title "Vy",\
     ""         using 1:20 with lines title "Omega" axis x1y2
