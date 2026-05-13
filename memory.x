/* STM32F446RE memory map
 *
 * Flash sectors:
 *   0–3  : 4 × 16 KB  = 0x08000000–0x0800FFFF
 *   4    : 1 × 64 KB  = 0x08010000–0x0801FFFF
 *   5–6  : 2 × 128 KB = 0x08020000–0x0805FFFF
 *   ---- : available for firmware code above
 *   7    : 1 × 128 KB = 0x08060000–0x0807FFFF  ← reserved for flash_log
 */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 384K  /* sectors 0–6 only */
    LOG   : ORIGIN = 0x08060000, LENGTH = 128K  /* sector 7 — flash_log, not used by the linker */
    RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}
