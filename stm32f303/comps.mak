SRC_COMPS += stm32f303/src/comps/hv.c
SRC_COMPS += stm32f303/src/comps/io.c
SRC_COMPS += stm32f303/src/comps/ls.c
#SRC_COMPS += stm32f303/src/comps/enc.c

SHARED_COMPS += shared/comps/sim.c
SHARED_COMPS += shared/comps/term.c
SHARED_COMPS += shared/comps/curpid.c
SHARED_COMPS += shared/comps/svm.c
SHARED_COMPS += shared/comps/dq.c
SHARED_COMPS += shared/comps/idq.c
SHARED_COMPS += shared/comps/sensorless.c
# SHARED_COMPS += shared/comps/vel.c
# SHARED_COMPS += shared/comps/hal_test.c
# SHARED_COMPS += shared/comps/dc.c
# SHARED_COMPS += shared/comps/ypid.c

F3COMPS = $(SRC_COMPS) $(SHARED_COMPS)