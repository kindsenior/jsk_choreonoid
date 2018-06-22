load "../file-name-parse.plt"

set xtics 0.1
set ytics 0.1

set title strsubst(motion._plugin._params._dt._fps,"_","\\_")

c=1
M=37.721
cmxscale=1
dcmxscale=1
dcmzscale=0.5
copscale=10
copoffset=-1
Lscale=0.1
dxscale=0.5
dzscale=1
plot contact u 1:3, contact u 1:12, contact u 1:($2+0.5),contact u 1:($5+0.4),contact u 1:($10/10),  contact u 1:($11-0.5),contact u 1:($14-0.4),contact u 1:($19/10)
replot input u 1:($2*cmxscale),ref u 1:($2*cmxscale), input u 1:($5/M)*dcmxscale t "input dCMx",ref u 1:($5/M)*dcmxscale t "ref dCMx", ref u 1:3, input u 1:($6/M) t "input dCMy",ref u 1:($6/M) t "ref dCMy"
replot input u 1:($8*Lscale), ref u 1:($8*Lscale), input u 1:($9*Lscale), ref u 1:($9*Lscale), ref u 1:($10/100), ref u 1:4,ref u 1:($7/100), input u 1:($13/1000)
# replot pre u 1:($8*Lscale), ref u 1:($8*Lscale), pre u 1:($9*Lscale), ref u 1:($9*Lscale), ref u 1:($10/100), ref u 1:4,ref u 1:($7/100), input u 1:($13/1000)
replot wrench u 1:(-$6/$4)*copscale+copoffset t "lCOPx",wrench u 1:(-$12/$10)*copscale+copoffset t "rCOPx", wrench u 1:($5/$4)*copscale+copoffset t "lCOPy",wrench u 1:($11/$10)*copscale+copoffset t "rCOPy"
# replot wrenches u 1:($4/1000) t "rfz"
replot wrench u 1:($2/1000+0.2), wrench u 1:($3/1000+0.2), wrench u 1:($4/1000), wrench u 1:($8/1000-0.2), wrench u 1:($9/1000-0.2), wrench u 1:($10/1000)
replot init u 1:4,pre u 1:4, init u 1:($7/M)*dcmzscale,pre u 1:($7/M)*dcmzscale # cmz dcmz
replot init u 1:($2*cmxscale),pre u 1:($2*cmxscale) # cmx
replot init u 1:($5/M)*dcmxscale,pre u 1:($5/M)*dcmxscale # dcmx
replot init u 1:($9*Lscale),pre u 1:($9*Lscale) # Ly
replot iee u 1:($2*dxscale),iee u 1:($5*dxscale)# , iee u 1:($11*dxscale), iee u 1:($14*dxscale) # px vx
replot iee u 1:($4*dzscale),iee u 1:($7*dzscale)# , iee u 1:($13*dzscale), iee u 1:($16*dzscale) # pz vz
# replot iee u 1:($4*c),ee u 1:($4*c) t "ref lz",iee u 1:($7*dzscale),iee u 1:($16*dzscale)

replot opt u 1:2, opt u 1:3

a=0.1;x=0;t=0;
hip_p_dq=udiff(4)
replot pos @hip_p_dq t "r-hip-p dq*".sprintf("%.2f",a)
knee_dq=udiff(5)
replot pos @knee_dq t "r-knee dq*".sprintf("%.2f",a)

