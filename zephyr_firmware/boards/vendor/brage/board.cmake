# Copyright (c) 2024
# SPDX-License-Identifier: Apache-2.0

board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-mode=hw")
board_runner_args(openocd --target-handle=_CHIPNAME.cpu0)
board_runner_args(openocd "--transport=swd")
board_runner_args(openocd "--speed=2000")

board_runner_args(jlink "--device=STM32U575CG" "--speed=4000")
board_runner_args(pyocd "--target=stm32u575cgtx")

include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)