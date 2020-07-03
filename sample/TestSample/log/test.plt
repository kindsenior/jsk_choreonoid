dirname(path)=strstrt(path, "/")!= 0 ? substr(path, 1, strstrlt(path,"/")-1)."/" : "./"
load sprintf("%s../../file-name-parse.plt", dirname(ARG0))

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
xscale=cmxscale
dxscale=0.5
yscale=xscale
dyscale=dxscale
zscale=1
dxscale=dcmxscale
dzscale=1
wscale=0.1

a=0.1;x=0;t=0;

# # rleg
# plot   iee u 1:($2*xscale),iee u 1:($5*dxscale)# px vx
# replot iee u 1:($3*yscale),iee u 1:($4*dyscale)# py vy
# replot iee u 1:($4*zscale),iee u 1:($7*dzscale)# pz vz
# replot iee u 1:($9*wscale),iee u 1:($10*wscale)# wy wz

# lleg
plot   iee u 1:($11*xscale),iee u 1:($14*dxscale)# px vx
lf_dvx=udiff(14)
replot iee @lf_dvx t "LLEG_JOINT5 dvx*".sprintf("%.2f",a)
# replot iee u 1:($12*yscale),iee u 1:($15*dyscale)# py vy
replot iee u 1:($13*zscale),iee u 1:($16*dzscale)# pz vz
lf_dvz=udiff(16)
replot iee @lf_dvz t "LLEG_JOINT5 dvz*".sprintf("%.2f",a)
replot iee u 1:($18*wscale),iee u 1:($19*wscale)# wy wz

