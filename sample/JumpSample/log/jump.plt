dirname(path)=strstrt(path, "/")!= 0 ? substr(path, 1, strstrlt(path,"/")-1)."/" : "./"
load sprintf("%s../../file-name-parse.plt", dirname(ARG0))

set xtics 0.1
set ytics 0.1

set title strsubst(motion._plugin._params._dt._fps,"_","\\_")

c=1
M=39.721
cmxscale=1
dcmxscale=1
dcmzscale=0.5
copscale=10
copoffset=-1
contactoffset=0.5
Lscale=0.1
xscale=cmxscale
dxscale=0.5
zscale=1
dxscale=dcmxscale
dzscale=dcmzscale
plot init u 1:($2*cmxscale),    pre u 1:($2*cmxscale),    input u 1:($2*cmxscale), ref u 1:($2*cmxscale) # CMx
replot init u 1:($5/M)*dcmxscale, pre u 1:($5/M)*dcmxscale, input u 1:($5/M)*dcmxscale t "input dCMx", ref u 1:($5/M)*dcmxscale t "ref dCMx" # dCMx
replot input u 1:3, ref u 1:3 # CMy
replot input u 1:($6/M) t "input dCMy",ref u 1:($6/M) t "ref dCMy" # dCMy
replot init u 1:4, pre u 1:4, ref u 1:4 # CMz
replot init u 1:($7/M)*dcmzscale, pre u 1:($7/M)*dcmzscale, ref u 1:($7/M)*dcmzscale t "ref dCMz" # dCMz

replot input u 1:($8*Lscale), ref u 1:($8*Lscale) # Lx
replot init u 1:($9*Lscale), pre u 1:($9*Lscale), input u 1:($9*Lscale), ref u 1:($9*Lscale) # Ly
replot ref u 1:($10/100)

replot input u 1:($13/1000) # Fz

replot wrenches u 1:(-$6/$4)*copscale+copoffset   t "rCOPx", wrench u 1:(-$12/$10)*copscale+copoffset t "rCOPx"
replot wrenches u 1:(-$12/$10)*copscale+copoffset t "lCOPx", wrench u 1:(-$6/$4)*copscale+copoffset   t "lCOPx"
replot wrench u 1:($5/$4)*copscale+copoffset t "lCOPy",wrench u 1:($11/$10)*copscale+copoffset t "rCOPy"
# replot wrenches u 1:($4/1000) t "rfz"
replot wrench u 1:($2/1000+0.2), wrench u 1:($8/1000-0.2) # Fx
replot wrench u 1:($3/1000+0.2), wrench u 1:($9/1000-0.2) # Fy
replot wrench u 1:($4/1000), wrench u 1:($10/1000) # Fz

replot iee u 1:($2*xscale+contactoffset),  contact u 1:($2+contactoffset), contact u 1:($11-contactoffset) # px
replot iee u 1:($5*dxscale+contactoffset), contact u 1:($5*dxscale+contactoffset), contact u 1:($14*dxscale-contactoffset) # vx
replot iee u 1:($4*zscale),iee u 1:($7*dzscale) # pz vz
replot contact u 1:3, contact u 1:12 # py
replot iee u 1:($9/10),  contact u 1:($9/10) # wy
replot contact u 1:($10/10), contact u 1:($19/10) # wz

replot opt u 1:2, opt u 1:3

a=0.1;x=0;t=0;
hip_p_dq=udiff(4)
replot pos @hip_p_dq t "r-hip-p dq*".sprintf("%.2f",a)
knee_dq=udiff(5)
replot pos @knee_dq t "r-knee dq*".sprintf("%.2f",a)
ankle_dq=udiff(6)
replot pos @ankle_dq t "r-ankle dq*".sprintf("%.2f",a)

root_dp=udiff(3)
replot hip @root_dp t "root pitch vel*".sprintf("%.2f",a)