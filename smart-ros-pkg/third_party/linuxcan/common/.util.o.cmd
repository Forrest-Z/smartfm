cmd_/home/sxt/ros_workspace/linuxcan/pcican/../common/util.o := gcc -Wp,-MD,/home/sxt/ros_workspace/linuxcan/pcican/../common/.util.o.d  -nostdinc -isystem /usr/lib/gcc/x86_64-linux-gnu/4.6/include  -I/usr/src/linux-headers-3.5.0-34-generic/arch/x86/include -Iarch/x86/include/generated -Iinclude  -include /usr/src/linux-headers-3.5.0-34-generic/include/linux/kconfig.h -Iubuntu/include  -D__KERNEL__ -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -Wno-format-security -fno-delete-null-pointer-checks -O2 -m64 -mtune=generic -mno-red-zone -mcmodel=kernel -funit-at-a-time -maccumulate-outgoing-args -fstack-protector -DCONFIG_AS_CFI=1 -DCONFIG_AS_CFI_SIGNAL_FRAME=1 -DCONFIG_AS_CFI_SECTIONS=1 -DCONFIG_AS_FXSAVEQ=1 -DCONFIG_AS_AVX=1 -pipe -Wno-sign-compare -fno-asynchronous-unwind-tables -mno-sse -mno-mmx -mno-sse2 -mno-3dnow -mno-avx -Wframe-larger-than=1024 -Wno-unused-but-set-variable -fno-omit-frame-pointer -fno-optimize-sibling-calls -pg -Wdeclaration-after-statement -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -DCC_HAVE_ASM_GOTO -DLINUX=1 -D_LINUX=1 -I/home/sxt/ros_workspace/linuxcan/pcican/../include/ -D_DEBUG=0 -DDEBUG=0 -DWIN32=0  -DMODULE  -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(util)"  -D"KBUILD_MODNAME=KBUILD_STR(kvpcican)" -c -o /home/sxt/ros_workspace/linuxcan/pcican/../common/.tmp_util.o /home/sxt/ros_workspace/linuxcan/pcican/../common/util.c

source_/home/sxt/ros_workspace/linuxcan/pcican/../common/util.o := /home/sxt/ros_workspace/linuxcan/pcican/../common/util.c

deps_/home/sxt/ros_workspace/linuxcan/pcican/../common/util.o := \
  /usr/src/linux-headers-3.5.0-34-generic/arch/x86/include/asm/div64.h \
    $(wildcard include/config/x86/32.h) \
  include/asm-generic/div64.h \
  include/linux/types.h \
    $(wildcard include/config/uid16.h) \
    $(wildcard include/config/lbdaf.h) \
    $(wildcard include/config/arch/dma/addr/t/64bit.h) \
    $(wildcard include/config/phys/addr/t/64bit.h) \
    $(wildcard include/config/64bit.h) \
  /usr/src/linux-headers-3.5.0-34-generic/arch/x86/include/asm/types.h \
  include/asm-generic/types.h \
  include/asm-generic/int-ll64.h \
  /usr/src/linux-headers-3.5.0-34-generic/arch/x86/include/asm/bitsperlong.h \
  include/asm-generic/bitsperlong.h \
  include/linux/posix_types.h \
  include/linux/stddef.h \
  include/linux/compiler.h \
    $(wildcard include/config/sparse/rcu/pointer.h) \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  include/linux/compiler-gcc.h \
    $(wildcard include/config/arch/supports/optimized/inlining.h) \
    $(wildcard include/config/optimize/inlining.h) \
  include/linux/compiler-gcc4.h \
  /usr/src/linux-headers-3.5.0-34-generic/arch/x86/include/asm/posix_types.h \
  /usr/src/linux-headers-3.5.0-34-generic/arch/x86/include/asm/posix_types_64.h \
  include/asm-generic/posix_types.h \
  /home/sxt/ros_workspace/linuxcan/pcican/../include/util.h \

/home/sxt/ros_workspace/linuxcan/pcican/../common/util.o: $(deps_/home/sxt/ros_workspace/linuxcan/pcican/../common/util.o)

$(deps_/home/sxt/ros_workspace/linuxcan/pcican/../common/util.o):
