LV_SRC_COMPS += src/comps/hw/io4.c
LV_SRC_COMPS += src/comps/hv.c
LV_SRC_COMPS += src/comps/enc_cmd.c
LV_SRC_COMPS += src/comps/o_fb.c
LV_SRC_COMPS += src/comps/sserial.c
LV_SRC_COMPS += src/comps/yaskawa.c
LV_SRC_COMPS += src/comps/encs.c
LV_SRC_COMPS += src/comps/encf.c
LV_SRC_COMPS += src/comps/endat.c

LV_SRC_COMPS += src/comps/usart.c
LV_SRC_COMPS += src/comps/encm.c
LV_SRC_COMPS += src/comps/dmm.c
LV_SRC_COMPS += src/comps/smartabs.c
LV_SRC_COMPS += src/comps/adc.c
LV_SRC_COMPS += src/comps/enc_fb.c
LV_SRC_COMPS += src/comps/conf.c
LV_SRC_COMPS += src/comps/res.c
LV_SRC_COMPS += src/comps/hx711.c
LV_SRC_COMPS += src/comps/siserial.c
LV_SRC_COMPS += src/comps/smart_torque.c

LV_SHARED_COMPS += shared/comps/sim.c
LV_SHARED_COMPS += shared/comps/term.c
LV_SHARED_COMPS += shared/comps/svm.c

LV_SHARED_COMPS += shared/comps/vel.c
LV_SHARED_COMPS += shared/comps/rev.c
LV_SHARED_COMPS += shared/comps/hal_test.c
# LV_SHARED_COMPS += shared/comps/dc.c
LV_SHARED_COMPS += shared/comps/ypid.c
LV_SHARED_COMPS += shared/comps/fault.c
LV_SHARED_COMPS += shared/comps/pid.c
LV_SHARED_COMPS += shared/comps/spid.c
LV_SHARED_COMPS += shared/comps/pe.c
LV_SHARED_COMPS += shared/comps/pmsm_limits.c
LV_SHARED_COMPS += shared/comps/pmsm_ttc.c
LV_SHARED_COMPS += shared/comps/dc_limits.c
LV_SHARED_COMPS += shared/comps/dc_ttc.c
LV_SHARED_COMPS += shared/comps/acim_ttc.c
LV_SHARED_COMPS += shared/comps/uvw.c
LV_SHARED_COMPS += shared/comps/fanuc.c
LV_SHARED_COMPS += shared/comps/fb_switch.c
LV_SHARED_COMPS += shared/comps/reslimit.c
LV_SHARED_COMPS += shared/comps/iit.c
LV_SHARED_COMPS += shared/comps/vel_int.c
LV_SHARED_COMPS += shared/comps/linrev.c
LV_SHARED_COMPS += shared/comps/psi.c
LV_SHARED_COMPS += shared/comps/stp.c
#LV_SHARED_COMPS += shared/comps/uf.c
LV_SHARED_COMPS += shared/comps/uf2.c
LV_SHARED_COMPS += shared/comps/ramp.c
LV_SHARED_COMPS += shared/comps/scale.c
LV_SHARED_COMPS += shared/comps/idx_home.c
LV_SHARED_COMPS += shared/comps/move.c
# LV_SHARED_COMPS += shared/comps/ac.c
LV_SHARED_COMPS += shared/comps/not.c
LV_SHARED_COMPS += shared/comps/and.c
LV_SHARED_COMPS += shared/comps/or.c
LV_SHARED_COMPS += shared/comps/jog.c
LV_SHARED_COMPS += shared/comps/velbuf.c
LV_SHARED_COMPS += shared/comps/avg.c
LV_SHARED_COMPS += shared/comps/mux.c
LV_SHARED_COMPS += shared/comps/veltopos.c
# LV_SHARED_COMPS += shared/comps/wobl.c
LV_SHARED_COMPS += shared/comps/debounce.c
LV_SHARED_COMPS += shared/comps/pos_filter.c
LV_SHARED_COMPS += shared/comps/rl.c
LV_SHARED_COMPS += shared/comps/mad.c
LV_SHARED_COMPS += shared/comps/sensorless.c
LV_SHARED_COMPS += shared/comps/field.c
LV_SHARED_COMPS += shared/comps/gain.c
LV_SHARED_COMPS += shared/comps/rlpsij.c
LV_SHARED_COMPS += shared/comps/veltime.c
LV_SHARED_COMPS += shared/comps/mpid.c
LV_SHARED_COMPS += shared/comps/fmove.c
LV_SHARED_COMPS += shared/comps/home.c
LV_SHARED_COMPS += shared/comps/en.c
LV_SHARED_COMPS += shared/comps/th.c
#LV_SHARED_COMPS += shared/comps/asm.c
LV_SHARED_COMPS += shared/comps/idpmsm.c
LV_SHARED_COMPS += shared/comps/iddc.c
LV_SHARED_COMPS += shared/comps/idm.c
LV_SHARED_COMPS += shared/comps/ids.c
LV_SHARED_COMPS += shared/comps/motsim.c

LV_SHARED_COMPS += shared/comps/zv_ip.c

F4COMPS = $(LV_SRC_COMPS) $(LV_SHARED_COMPS)

CONFIG_TEMPLATES = $(wildcard conf/template/*.txt)