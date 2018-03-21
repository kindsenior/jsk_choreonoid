set xtics 0.1
set ytics 0.1

wrench_tmp=wrench

# after cut
ext=substr_after(wrench_tmp,strstrlt(wrench_tmp,"."))
wrench_tmp=substr_str_before(wrench_tmp,ext)

_fps=substr_after(wrench_tmp,strstrlt(wrench_tmp,"_"))
wrench_tmp=substr_str_before(wrench_tmp,_fps)

_dt=substr_after(wrench_tmp,strstrlt(wrench_tmp,"_"))
wrench_tmp=substr_str_before(wrench_tmp,_dt)

# before cut
if (!exist("robot")) robot=system("echo $ROBOT")
wrench_tmp=substr_str_after(wrench_tmp,robot."_")

_motion="_".substr_str_before(wrench_tmp,"_")
wrench_tmp=substr_str_after(wrench_tmp,_motion."_")

_plugin="_".substr_str_before(wrench_tmp,"_")
_params="_".substr_str_after(wrench_tmp,_plugin."_")

# SFC
ref=strsubst(wrench,"wrench","refPL")
input=robot._motion._plugin."_inputPL"._dt._fps.ext
contact=robot._motion._plugin."_contact"._dt._fps.ext
pre=robot._motion._plugin."_prePL"._fps.ext

# init
init=robot._motion."_initCM"._fps.ext
iee=robot._motion."_initEE"._fps.ext

# RMC
ee=robot._motion."_RMC"."_EE"._fps."_0".ext

# seq
motion=substr_str_after(_motion,"_")
wrenches=motion.".wrenches"
pos=motion.".pos"
opt=motion.".optionaldata"


set title strsubst(motion._plugin._params._dt._fps,"_","\\_")

b=0.5
c=5
plot contact u 1:3, contact u 1:12, wrench u 1:($4/1000), wrench u 1:($10/1000),input u 1:($13/1000),  contact u 1:($2+0.5),contact u 1:($5+0.4),contact u 1:($10/10),  contact u 1:($11-0.5),contact u 1:($14-0.4),contact u 1:($19/10),  wrench u 1:(-$6/$4) t "lCOPx", wrench u 1:(-$12/$10) t "rCOPx", input u 1:($2*10),ref u 1:($2*10),input u 1:5,ref u 1:5, ref u 1:3,ref u 1:6, ref u 1:8,ref u 1:9,ref u 1:($10/100),   ref u 1:4,ref u 1:($7/100), wrench u 1:($2/1000+0.2), wrench u 1:($3/1000+0.2), wrench u 1:($8/1000-0.2), wrench u 1:($9/1000-0.2), wrenches u 1:($4/1000) t "rfz", init u 1:4,pre u 1:4,init u 1:($7/100),pre u 1:($7/42.7*b), iee u 1:($7*b) t "init ldz", iee u 1:($4*c) t "init lz",ee u 1:($4*c) t "ref lz", opt u 1:6 t "rleg time"

a=0.1;x=0;t=0;
hip_p_dq=udiff(4)
replot pos @hip_p_dq t "r-hip-p dq*".sprintf("%.2f",a)
knee_dq=udiff(5)
replot pos @knee_dq t "r-knee dq*".sprintf("%.2f",a)
