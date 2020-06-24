dirname(path)=strstrt(path, "/")!= 0 ? substr(path, 1, strstrlt(path,"/")-1)."/" : "./"
load sprintf("%s../../file-name-parse.plt", dirname(ARG0))

set xtics 0.1
set ytics 0.1

set title strsubst(motion._plugin._params._dt._fps,"_","\\_")

c=1
M=37.721
cmxscale=2
dcmxscale=2
copscale=1
copoffset=-1
Lscale=0.1
dxscale=0.1*dcmxscale
dzscale=0.5
plot contact u 1:3, contact u 1:12, contact u 1:($2+0.5),contact u 1:($5+0.4),contact u 1:($10/10),  contact u 1:($11-0.5),contact u 1:($14-0.4),contact u 1:($19/10)
replot input u 1:($2*cmxscale),ref u 1:($2*cmxscale), input u 1:($5/M)*dcmxscale t "input dCMx",ref u 1:($5/M)*dcmxscale t "ref dCMx", ref u 1:3, input u 1:($6/M) t "input dCMy",ref u 1:($6/M) t "ref dCMy"
replot input u 1:($8*Lscale), ref u 1:($8*Lscale), input u 1:($9*Lscale), ref u 1:($9*Lscale), ref u 1:($10/100), ref u 1:4,ref u 1:($7/100), input u 1:($13/1000)
# replot pre u 1:($8*Lscale), ref u 1:($8*Lscale), pre u 1:($9*Lscale), ref u 1:($9*Lscale), ref u 1:($10/100), ref u 1:4,ref u 1:($7/100), input u 1:($13/1000)
replot wrench u 1:(-$6/$4)*copscale+copoffset t "lCOPx",wrench u 1:(-$12/$10)*copscale+copoffset t "rCOPx", wrench u 1:($5/$4)*copscale+copoffset t "lCOPy",wrench u 1:($11/$10)*copscale+copoffset t "rCOPy"
# replot wrenches u 1:($4/1000) t "rfz"
replot wrench u 1:($2/1000+0.2), wrench u 1:($3/1000+0.2), wrench u 1:($4/1000), wrench u 1:($8/1000-0.2), wrench u 1:($9/1000-0.2), wrench u 1:($10/1000)
replot init u 1:4,pre u 1:4,init u 1:($7/100),pre u 1:($7/M)*dzscale
replot iee u 1:($2*dxscale),iee u 1:($5*dxscale), iee u 1:($11*dxscale), iee u 1:($14*dxscale)
# replot iee u 1:($4*c),ee u 1:($4*c) t "ref lz",iee u 1:($7*dzscale),iee u 1:($16*dzscale), opt u 1:6 t "rleg time"
replot opt u 1:2 t "rleg contact", opt u 1:3 t "lleg contact", opt u 1:6 t "rleg time", opt u 1:7 t "lleg time"

a=0.1;x=0;t=0;
hip_p_dq=udiff(4)
replot pos @hip_p_dq t "r-hip-p dq*".sprintf("%.2f",a)
knee_dq=udiff(5)
replot pos @knee_dq t "r-knee dq*".sprintf("%.2f",a)

