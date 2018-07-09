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
hip=motion.".hip"
opt=motion.".optionaldata"
