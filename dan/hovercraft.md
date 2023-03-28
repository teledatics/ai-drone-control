Linux:

Pulled branch 5.15-2.2.x-imx from Freescale/linux-fslc
Applied 0001-imx8mm-ares-m-board-with-meta-teledatics-patches-app.patch to the build tree.
copied arch/arm64/boot/dts/freescale/imx8mm-ares-m.dts to 
arch/arm64/boot/dts/freescale/imx8mm-kimchi.dts and added a new imx8mm-kimchi.dtb target.

 To enable using the Linux remote processor (rproc) framework, had to edit imx8mm-kimchi.dts and add
 <&m4_reserved> to the memory-region list for an imx8mm-cm4. This allows the rproc driver to map a virtual address 
for the m4 reserved DDR4 memory, allowing it to load the M4 OS from an ELF file.

Pushed branch dan/5.15-2.2.x-kimchi to my [linux repo:](git@github.com:slaterd314/linux.git)


PX4-Autopilot:

Forked [PX4-Autopilot](git@github.com:slaterd314/PX4-Autopilot.git)  from PX4/PX4-Autopilot. Checked out nxp-dev branch, and created branch nxp-dev-dan from there.
Also Forked [PX4-Nuttx](git@github.com:slaterd314/PX4-NuttX.git) from [PX4/PX4-Nuttx](git@github.com:PX4/NuttX.git) and created branch nxp-dev-dan from px4_firmware_nuttx-10.1.0+ 
Started building and modifying the fmuk66-v3 configuration for the kimchi board. The original fmuk66-v3 has an M4 with a modified system memory map. It maps 2MB of flash memory starting at addess 0, and
loads PX4-Autopilot (about 1.9MB) into low memory.
  The imx8mm chip has a more conventional system memory map for the M4. In this case, 16 MB of DDR4 RAM is reserved for the M4 starting at address 0x80000000. 
  As a result, the first change necessary was to modify the linker script to load PX4-Autopilot into RAM starting at 0x80006000. There is also a small (256K) datasram segment that 
  on the original fmuk66-v3 board was loaded at 0x1fff0000. This address is an alias for the TCM memory. On the imx8mm chip, this same alias starts at 0x1ffE0000, so updated this
  in the linker script as well.
   With these changes, we can now load the M4 processor with Px4-Autopilot and start it running. However, we can't see any output from nsh because the kimchi board 
  uses lpuart4 for the M4 console, while the fmuk66-v3 board uses lpuart0. Latest code attempts to change console to lpuart4, but still needs work.
	  