SIMBRICKS_LIB_DIR := /home/gexl/Desktop/simbricks/lib
LIBSIMBRICKS := $(SIMBRICKS_LIB_DIR)/libsimbricks.a

SV_DIR := /tools/Xilinx/Vivado/2021.2/data/xsim/include

CPPFLAGS := -I$(SV_DIR) -I$(SIMBRICKS_LIB_DIR) -fPIC -std=gnu++17 -shared -O3 \
	-static-libgcc -static-libstdc++

all: xsim_adapter.so

xsim_adapter.so: xsim_adapter.cc
	g++ $(CPPFLAGS) -o xsim_adapter.so $< $(LIBSIMBRICKS)

.PHONY: clean

clean:
	rm -rf xsim.dir $(XSIM_ADAPTER_OBJ) xsim_adapter.so
