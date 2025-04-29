#
# SPDX-License-Identifier: Apache-2.0

# filepath: CMakeLists.txt
#cmake_minimum_required(VERSION 3.21)
#cmake_policy(SET CMP0057 NEW)
#find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
#project(LPS22FRESH424)

#target_sources(app PRIVATE src/main.c)
#zephyr_include_directories(.) # (inc)

# NORDIC SDK APP START
#target_sources(app PRIVATE
#    src/main.c
#    src/lps22.c
#    src/services/ble_shs.c
#)

# NORDIC SDK APP END
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(LPS22FRESH424)


zephyr_include_directories(.) # (inc)

# NORDIC SDK APP START
target_sources(app PRIVATE
	src/main.c
	src/lps22.c
	src/services/ble_shs.c
)

# NORDIC SDK APP END
