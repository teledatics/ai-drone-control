# Loading PX4 Autopilot Firmware From Linux

#### Linux
   The NXP i.MX 8M Mini SoC has 4 A53 cores running Linux and an M4 core running PX4 Autopilot. We used the [Linux Remote Processor Framework (rproc)][1] to load PX4 Autopilot. The M4 processor in an i.MX 8MM system memory map maps DDR memory starting at address 0x40000000. 
   In our dts [file][2], we reserved 16MB of DDR memory for the m4 starting at address 0x80000000. This was based on the m4_reserved node from the ddr4 flavors of the nxp imx8 evk boards dts files under boards in the PX4-Autopilot repository. This node reserves physical RAM for the M4 processor, telling Linux not to use it. The existing nxp evk examples rely on u-boot to load Linux on the A cores, and PX4 on the M core. 
  However, even though we need to reserve RAM for the M4, in order to load a binary for the M4 to run, the Linux rproc driver needs to map a virtual address for the M4 memory. To enable this, the m4_reserved node needs to be listed in the memory-region list that’s part of the imx8mm-cm4 node in the dts file.


#### PX4-Autopilot
   For PX4-Autopilot, we based our code on the existing nxp fmuk66-v3 board using the px4_firmware_nuttx-10.1.0+ branch of [PX4/PX4-Autopilot][3] as our starting point. We modified the existing linker script to place the binary into DDR memory at 0x80000000. 


#### Loading through rproc
  The Linux Remote Processor Framework interface works through the sysfs file system, and is very simple. The file `/sys/module/firmware_class/parameters/path` holds the path to a directory containing firmware images. The default path if not specified is `/lib/firmware`. The file `/sys/class/remoteproc/remoteproc0/firmware` holds the filename of an ELF image, which defaults to `rproc-0-fw` if not specified. Finally, the file `/sys/class/remoteproc/remoteproc0/state` controls the M4 processor. To load and run the firmware:
  
  - `echo -n /root/firmware > /sys/module/firmware_class/parameters/path`
  - `echo -n hoverboard.elf > /sys/class/remoteproc/remoteproc0/firmware`
  - `echo -n start > /sys/class/remoteproc/remoteproc0/state`
  - `cat /sys/class/remoteproc/remoteproc0/state` Should display `running`
   

#### Future
   Currently, PX4-Autopilot is about 1.9MB, too large to fit in the TCM memory of an M4 processor. Zephyr, on the other hand, compiles to a 13K binary. Moving forward, we’d make even more use of Asymmetric Multi-Processing on i.MX chips by running just the control portions of Autopilot on the M4, while leaving higher level functionality running under Linux on the A cores. Communication between the A and M cores will be accomplished using the [Remote Processor Messaging (rpmg) Framework][4].


[1]: https://www.kernel.org/doc/Documentation/remoteproc.txt "Linux  Remote Processor Framework Document"
[2]: https://github.com/slaterd314/linux/blob/dan/5.15-2.2.x-kimchi/arch/arm64/boot/dts/freescale/imx8mm-kimchi.dts "Kimchi board dts file."
[3]: https://github.com/slaterd314/PX4-Autopilot/tree/nxp-dev-dan "Hovergames entry fork of PX4-Autopilot"
[4]: https://docs.kernel.org/staging/rpmsg.html "Remote Processor Messaging (rpmsg) Framework"
