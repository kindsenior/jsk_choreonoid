load "../file-name-parse.plt"

set xtics 0.1
set ytics 0.1

set title strsubst(motion._plugin._params._dt._fps,"_","\\_")

b=0.5
c=5
plot contact u 1:3, contact u 1:12, wrench u 1:($4/1000), wrench u 1:($10/1000),input u 1:($13/1000),  contact u 1:($2+0.5),contact u 1:($5+0.4),contact u 1:($10/10),  contact u 1:($11-0.5),contact u 1:($14-0.4),contact u 1:($19/10),  wrench u 1:(-$6/$4) t "lCOPx", wrench u 1:(-$12/$10) t "rCOPx", input u 1:($2*10),ref u 1:($2*10),input u 1:5,ref u 1:5, ref u 1:3,ref u 1:6, ref u 1:8,ref u 1:9,ref u 1:($10/100),   ref u 1:4,ref u 1:($7/100), wrench u 1:($2/1000+0.2), wrench u 1:($3/1000+0.2), wrench u 1:($8/1000-0.2), wrench u 1:($9/1000-0.2), wrenches u 1:($4/1000) t "rfz", init u 1:4,pre u 1:4,init u 1:($7/100),pre u 1:($7/42.7*b), iee u 1:($7*b) t "init ldz", iee u 1:($4*c) t "init lz",ee u 1:($4*c) t "ref lz", opt u 1:6 t "rleg time"

a=0.1;x=0;t=0;
hip_p_dq=udiff(4)
replot pos @hip_p_dq t "r-hip-p dq*".sprintf("%.2f",a)
knee_dq=udiff(5)
replot pos @knee_dq t "r-knee dq*".sprintf("%.2f",a)
