#dump
dd of=arch/mips/boot/uImageBackup if=/dev/sdd skip=4194304 count=2097152 bs=1 conv=notrunc
#write
dd if=arch/mips/boot/uImage of=/dev/sdd seek=4194304 bs=1 conv=notrunc
