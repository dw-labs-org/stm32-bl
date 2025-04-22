MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  /* Write size is 16 bytes */
  /* Page size is 8K */
  /* DFU must be one page larger than active */
  /* 2 memory banks each 512K */
  /* See embedded_stm32::out::_generated.rs */

  FLASH                             : ORIGIN = 0x08000000, LENGTH = 56K
  BOOTLOADER_STATE                  : ORIGIN = 0x0800e000, LENGTH = 8K
  ACTIVE                            : ORIGIN = 0x08010000, LENGTH = 448K
  DFU                               : ORIGIN = 0x08080000, LENGTH = 456K
  RAM                         (rwx) : ORIGIN = 0x20000000, LENGTH = 768K
}

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE) - ORIGIN(FLASH);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE) - ORIGIN(FLASH);

__bootloader_active_start = ORIGIN(ACTIVE) - ORIGIN(FLASH);
__bootloader_active_end = ORIGIN(ACTIVE) + LENGTH(ACTIVE) - ORIGIN(FLASH);

__bootloader_dfu_start = ORIGIN(DFU) - ORIGIN(DFU);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU) - ORIGIN(DFU);