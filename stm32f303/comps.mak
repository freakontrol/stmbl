HV_SRC_COMPS = stm32f303/src/comps/hv.c
HV_SRC_COMPS += stm32f303/src/comps/io.c
HV_SRC_COMPS += stm32f303/src/comps/ls.c
#HV_SRC_COMPS += stm32f303/src/comps/enc.c

HV_SHARED_COMPS = shared/comps/sim.c
HV_SHARED_COMPS += shared/comps/term.c
HV_SHARED_COMPS += shared/comps/curpid.c
HV_SHARED_COMPS += shared/comps/svm.c
HV_SHARED_COMPS += shared/comps/dq.c
HV_SHARED_COMPS += shared/comps/idq.c
HV_SHARED_COMPS += shared/comps/sensorless.c
# HV_SHARED_COMPS += shared/comps/vel.c
# HV_SHARED_COMPS += shared/comps/hal_test.c
# HV_SHARED_COMPS += shared/comps/dc.c
# HV_SHARED_COMPS += shared/comps/ypid.c

F3COMPS = $(HV_SRC_COMPS) $(HV_SHARED_COMPS)