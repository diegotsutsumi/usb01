modifications:

- sys_tmr.c <-- entirely new
- usb_host.c <-- Tsutsumi`s Patch + GetVendorId
- sys_clk_pic32mx.c <-- divcount to divisor in SYS_CLK_ClosestFactorsGet
- peripheral/peripheral_common.h <-- commented out the warning message of _PLIB_UNSUPPORTED
