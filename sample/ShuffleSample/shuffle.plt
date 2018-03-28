load "../file-name-parse.plt"

set title strsubst(motion._plugin._params._dt._fps,"_","\\_")

set xtics 0.1
set ytics 0.2

M=37.721
b=0.5
c=5
cmxscale=1
Lscale=10
plot contact u 1:3, contact u 1:12,input u 1:($13/1000)
replot contact u 1:2,contact u 1:($5+0.4),contact u 1:($10/10),  contact u 1:11,contact u 1:($14-0.4),contact u 1:($19/10)
replot wrench u 1:(-$6/$4) t "lCOPx", wrench u 1:(-$12/$10) t "rCOPx"
replot input u 1:($2*cmxscale),ref u 1:($2*cmxscale), input u 1:($5/M) t "input dCMx",ref u 1:($5/M) t "ref dCMx", ref u 1:3,ref u 1:($6/M) t "ref dCMy", ref u 1:($8/Lscale),ref u 1:($9/Lscale),ref u 1:($10/100)#, ref u 1:4,ref u 1:($7/100)
replot wrenches u 1:($8/100+1.2) t "lfx",wrenches u 1:($2/100-1.2) t "rfx",wrenches u 1:($4/1000) t "rfz"
replot wrench u 1:($2/100+1.2),wrench u 1:($3/100+1.2),wrench u 1:($4/1000), wrench u 1:($8/100-1.2),wrench u 1:($9/100-1.2),wrench u 1:($10/1000)
replot opt u 1:2 t "rleg contact", opt u 1:3 t "lleg contact", opt u 1:6 t "rleg time", opt u 1:7 t "lleg time"
# replot init u 1:4,pre u 1:4,init u 1:($7/100),pre u 1:($7/42.7*b), iee u 1:($7*b) t "init ldz", iee u 1:($4*c) t "init lz",ee u 1:($4*c) t "ref lz", opt u 1:6 t "rleg time"
replot wrenches u 1:($2+$6)/M t "total fx/M"

# ddcmx=udiff(5)
# # a=1/M;
# a=0.0265
# x=0;t=0;
# replot ref @ddcmx t "ddcmx"

# a=0.1;x=0;t=0;
# hip_p_dq=udiff(4)
# replot pos @hip_p_dq t "r-hip-p dq*".sprintf("%.2f",a)
# knee_dq=udiff(5)
# replot pos @knee_dq t "r-knee dq*".sprintf("%.2f",a)
