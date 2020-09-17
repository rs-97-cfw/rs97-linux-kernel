/****************************************************************************
*
*    Copyright (C) 2005 - 2010 by Vivante Corp.
*
*    This program is free software; you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation; either version 2 of the license, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not write to the Free Software
*    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*
*****************************************************************************/




#include <linux/device.h>
#include <linux/slab.h>

#include "gc_hal_kernel_linux.h"
#include "gc_hal_driver.h"
#include "gc_hal_user_context.h"

#if USE_PLATFORM_DRIVER
#include <linux/platform_device.h>
#endif

MODULE_DESCRIPTION("Vivante Graphics Driver");
MODULE_LICENSE("GPL");

#if gcdNULL_DRIVER_TEST == 2
int gcdnull_driver_test=0;
#endif

struct class *gpuClass;

static gckGALDEVICE galDevice;

static int major = 199;
module_param(major, int, 0644);

#if defined(CONFIG_SOC_JZ4770)
#include "asm/jzsoc.h"

//#define JZ_GPU_MEM_BASE      0xf0000000 

#ifndef IRQ_GPU
#define IRQ_GPU 6
#endif
#ifndef GPU_BASE
#define GPU_BASE 0x13040000
#endif
#ifndef JZ_GPU_MEM_BASE
#define JZ_GPU_MEM_BASE 0		/* if GPU_MEM_BASE = 0, alloc gpu memory dynamicly on bootup */
#endif
#ifndef JZ_GPU_MEM_SIZE
#define JZ_GPU_MEM_SIZE 0x1000000	/* set default reserved memory 16M Bytes. */
#endif

int irqLine = IRQ_GPU;
module_param(irqLine, int, 0644);

long registerMemBase = GPU_BASE;
module_param(registerMemBase, long, 0644);

ulong registerMemSize = 256 << 10;
module_param(registerMemSize, ulong, 0644);

long contiguousSize = JZ_GPU_MEM_SIZE;
module_param(contiguousSize, long, 0644);

ulong contiguousBase = JZ_GPU_MEM_BASE;
module_param(contiguousBase, ulong, 0644);

#else /* CONFIG_SOC_JZ4770 */

int irqLine = -1;
module_param(irqLine, int, 0644);

long registerMemBase = 0x80000000;
module_param(registerMemBase, long, 0644);

ulong registerMemSize = 256 << 10;
module_param(registerMemSize, ulong, 0644);

long contiguousSize = 4 << 20;
module_param(contiguousSize, long, 0644);

ulong contiguousBase = 0;
module_param(contiguousBase, ulong, 0644);
#endif	/* CONFIG_SOC_JZ4770 */


long bankSize = 32 << 20;
module_param(bankSize, long, 0644);

int fastClear = -1;
module_param(fastClear, int, 0644);

int compression = -1;
module_param(compression, int, 0644);

int signal = 48;
module_param(signal, int, 0644);

ulong baseAddress = 0;
module_param(baseAddress, ulong, 0644);

int showArgs = 0;
module_param(showArgs, int, 0644);

#if ENABLE_GPU_CLOCK_BY_DRIVER
unsigned long coreClock = 156000000;
module_param(coreClock, ulong, 0644);
#endif

static int drv_open(struct inode *inode, struct file *filp);
static int drv_release(struct inode *inode, struct file *filp);
static int drv_ioctl(struct inode *inode, struct file *filp,
                     unsigned int ioctlCode, unsigned long arg);
static int drv_mmap(struct file * filp, struct vm_area_struct * vma);

struct file_operations driver_fops =
{
    .open   	= drv_open,
    .release	= drv_release,
    .ioctl  	= drv_ioctl,
    .mmap   	= drv_mmap,
};

int drv_open(struct inode *inode, struct file* filp)
{
    gcsHAL_PRIVATE_DATA_PTR	private;

    gcmkTRACE_ZONE(gcvLEVEL_VERBOSE, gcvZONE_DRIVER,
    	    	  "Entering drv_open\n");

    private = kmalloc(sizeof(gcsHAL_PRIVATE_DATA), GFP_KERNEL);

    if (private == gcvNULL)
    {
    	return -ENOTTY;
    }

    private->device				= galDevice;
    private->mappedMemory		= gcvNULL;
	private->contiguousLogical	= gcvNULL;

#if gcdkUSE_MEMORY_RECORD
	private->memoryRecordList.prev = &private->memoryRecordList;
	private->memoryRecordList.next = &private->memoryRecordList;
#endif

	/* A process gets attached. */
	gcmkVERIFY_OK(
		gckKERNEL_AttachProcess(galDevice->kernel, gcvTRUE));

    if (!galDevice->contiguousMapped)
    {
    	gcmkVERIFY_OK(gckOS_MapMemory(galDevice->os,
									galDevice->contiguousPhysical,
									galDevice->contiguousSize,
									&private->contiguousLogical));
    }

    filp->private_data = private;

    return 0;
}

extern void
OnProcessExit(
	IN gckOS Os,
	IN gckKERNEL Kernel
	);

int drv_release(struct inode* inode, struct file* filp)
{
    gcsHAL_PRIVATE_DATA_PTR	private;
    gckGALDEVICE			device;

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_DRIVER,
    	    	  "Entering drv_close\n");

    private = filp->private_data;
    gcmkASSERT(private != gcvNULL);

    device = private->device;

#if gcdkUSE_MEMORY_RECORD
	FreeAllMemoryRecord(galDevice->os, &private->memoryRecordList);

#ifdef ANDROID
	gcmkVERIFY_OK(gckOS_Delay(galDevice->os, 1000));
#else
	gcmkVERIFY_OK(gckCOMMAND_Stall(device->kernel->command));
#endif
#endif

    if (!device->contiguousMapped)
    {
		if (private->contiguousLogical != gcvNULL)
		{
			gcmkVERIFY_OK(gckOS_UnmapMemory(galDevice->os,
											galDevice->contiguousPhysical,
											galDevice->contiguousSize,
											private->contiguousLogical));
		}
    }

	/* A process gets detached. */
	gcmkVERIFY_OK(
		gckKERNEL_AttachProcess(galDevice->kernel, gcvFALSE));

    kfree(private);
    filp->private_data = NULL;

    return 0;
}

int drv_ioctl(struct inode *inode,
    	      struct file *filp,
    	      unsigned int ioctlCode,
	      unsigned long arg)
{
    gcsHAL_INTERFACE iface;
    gctUINT32 copyLen;
    DRIVER_ARGS drvArgs;
    gckGALDEVICE device;
    gceSTATUS status;
    gcsHAL_PRIVATE_DATA_PTR private;

    private = filp->private_data;

#if gcdNULL_DRIVER_TEST == 2
#define IOCTL_GCHAL_KERNEL_ENABLE_NULL_DRIVER_TEST           0x8282
#define IOCTL_GCHAL_KERNEL_DISABLE_NULL_DRIVER_TEST          0x6661

    if (ioctlCode == IOCTL_GCHAL_KERNEL_ENABLE_NULL_DRIVER_TEST) {
	    printk("galcore ioctl IOCTL_GCHAL_KERNEL_ENABLE_NULL_DRIVER_TEST\n");
	    gcdnull_driver_test = 1;
	    return 0;
    }
    else if (ioctlCode == IOCTL_GCHAL_KERNEL_DISABLE_NULL_DRIVER_TEST) {
	    printk("galcore ioctl IOCTL_GCHAL_KERNEL_DISABLE_NULL_DRIVER_TEST\n");
	    gcdnull_driver_test = 0;
	    return 0;
    }
#endif

    if (private == gcvNULL)
    {
    	gcmkTRACE_ZONE(gcvLEVEL_ERROR, gcvZONE_DRIVER,
    	    	      "[galcore] drv_ioctl: private_data is NULL\n");

    	return -ENOTTY;
    }

    device = private->device;

    if (device == gcvNULL)
    {
    	gcmkTRACE_ZONE(gcvLEVEL_ERROR, gcvZONE_DRIVER,
    	    	      "[galcore] drv_ioctl: device is NULL\n");

    	return -ENOTTY;
    }

    if (ioctlCode != IOCTL_GCHAL_INTERFACE
		&& ioctlCode != IOCTL_GCHAL_KERNEL_INTERFACE)
    {
        /* Unknown command. Fail the I/O. */
        return -ENOTTY;
    }

    /* Get the drvArgs to begin with. */
    copyLen = copy_from_user(&drvArgs,
    	    	    	     (void *) arg,
			     sizeof(DRIVER_ARGS));

    if (copyLen != 0)
    {
    	/* The input buffer is not big enough. So fail the I/O. */
        return -ENOTTY;
    }

    /* Now bring in the gcsHAL_INTERFACE structure. */
    if ((drvArgs.InputBufferSize  != sizeof(gcsHAL_INTERFACE))
    ||  (drvArgs.OutputBufferSize != sizeof(gcsHAL_INTERFACE))
    )
    {
    	return -ENOTTY;
    }

    copyLen = copy_from_user(&iface,
    	    	    	     drvArgs.InputBuffer,
			     sizeof(gcsHAL_INTERFACE));

    if (copyLen != 0)
    {
        /* The input buffer is not big enough. So fail the I/O. */
        return -ENOTTY;
    }

#if gcdkUSE_MEMORY_RECORD
	if (iface.command == gcvHAL_EVENT_COMMIT)
	{
		MEMORY_RECORD_PTR mr;
		gcsQUEUE_PTR queue = iface.u.Event.queue;

		while (queue != gcvNULL)
		{
			gcsQUEUE_PTR record, next;

			/* Map record into kernel memory. */
			gcmkERR_BREAK(gckOS_MapUserPointer(device->os,
											  queue,
											  gcmSIZEOF(gcsQUEUE),
											  (gctPOINTER *) &record));

			switch (record->iface.command)
			{
			case gcvHAL_FREE_VIDEO_MEMORY:
				mr = FindMemoryRecord(device->os,
									&private->memoryRecordList,
									record->iface.u.FreeVideoMemory.node);

				if (mr != gcvNULL)
				{
					DestoryMemoryRecord(device->os, mr);
				}
				else
				{
					printk("*ERROR* Invalid video memory (%p) for free\n",
						record->iface.u.FreeVideoMemory.node);
				}
                break;

			default:
				break;
			}

			/* Next record in the queue. */
			next = record->next;

			/* Unmap record from kernel memory. */
			gcmkERR_BREAK(gckOS_UnmapUserPointer(device->os,
												queue,
												gcmSIZEOF(gcsQUEUE),
												(gctPOINTER *) record));
			queue = next;
		}
	}
#endif

    status = gckKERNEL_Dispatch(device->kernel,
		(ioctlCode == IOCTL_GCHAL_INTERFACE) , &iface);

    if (gcmIS_ERROR(status))
    {
    	gcmkTRACE_ZONE(gcvLEVEL_WARNING, gcvZONE_DRIVER,
	    	      "[galcore] gckKERNEL_Dispatch returned %d.\n",
		      status);
    }

    else if (gcmIS_ERROR(iface.status))
    {
    	gcmkTRACE_ZONE(gcvLEVEL_WARNING, gcvZONE_DRIVER,
	    	      "[galcore] IOCTL %d returned %d.\n",
		      iface.command,
		      iface.status);
    }

    /* See if this was a LOCK_VIDEO_MEMORY command. */
    else if (iface.command == gcvHAL_LOCK_VIDEO_MEMORY)
    {
    	/* Special case for mapped memory. */
    	if (private->mappedMemory != gcvNULL
			&& iface.u.LockVideoMemory.node->VidMem.memory->object.type
				== gcvOBJ_VIDMEM)
		{
	   		/* Compute offset into mapped memory. */
	    	gctUINT32 offset = (gctUINT8 *) iface.u.LockVideoMemory.memory
	    	    	     	- (gctUINT8 *) device->contiguousBase;

    	    /* Compute offset into user-mapped region. */
    	    iface.u.LockVideoMemory.memory =
	    	(gctUINT8 *)  private->mappedMemory + offset;
		}
    }
#if gcdkUSE_MEMORY_RECORD
	else if (iface.command == gcvHAL_ALLOCATE_VIDEO_MEMORY)
	{
		CreateMemoryRecord(device->os,
							&private->memoryRecordList,
							iface.u.AllocateVideoMemory.node);
	}
	else if (iface.command == gcvHAL_ALLOCATE_LINEAR_VIDEO_MEMORY)
	{
		CreateMemoryRecord(device->os,
							&private->memoryRecordList,
							iface.u.AllocateLinearVideoMemory.node);
	}
	else if (iface.command == gcvHAL_FREE_VIDEO_MEMORY)
	{
		MEMORY_RECORD_PTR mr;

		mr = FindMemoryRecord(device->os,
							&private->memoryRecordList,
							iface.u.FreeVideoMemory.node);

		if (mr != gcvNULL)
		{
			DestoryMemoryRecord(device->os, mr);
		}
		else
		{
			printk("*ERROR* Invalid video memory for free\n");
		}
	}
#endif

    /* Copy data back to the user. */
    copyLen = copy_to_user(drvArgs.OutputBuffer,
    	    	    	   &iface,
			   sizeof(gcsHAL_INTERFACE));

    if (copyLen != 0)
    {
    	/* The output buffer is not big enough. So fail the I/O. */
        return -ENOTTY;
    }

    return 0;
}

static int drv_mmap(struct file * filp, struct vm_area_struct * vma)
{
    gcsHAL_PRIVATE_DATA_PTR private = filp->private_data;
    gckGALDEVICE device;
    int ret;
    unsigned long size = vma->vm_end - vma->vm_start;

    if (private == gcvNULL)
    {
    	return -ENOTTY;
    }

    device = private->device;

    if (device == gcvNULL)
    {
        return -ENOTTY;
    }

    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
    vma->vm_flags    |= VM_IO | VM_DONTCOPY | VM_DONTEXPAND;
    vma->vm_pgoff     = 0;

#if FIXED_MMAP_AS_CACHEABLE
 	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
// 	pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;		/* Uncacheable */
	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_NONCOHERENT;	/* Write-Back */
#endif

    if (device->contiguousMapped)
    {
    	ret = io_remap_pfn_range(vma,
	    	    	    	 vma->vm_start,
    	    	    	    	 (gctUINT32) device->contiguousPhysical >> PAGE_SHIFT,
				 size,
				 vma->vm_page_prot);

    	private->mappedMemory = (ret == 0) ? (gctPOINTER) vma->vm_start : gcvNULL;

    	return ret;
    }
    else
    {
    	return -ENOTTY;
    }
}

#if defined(CONFIG_JZSOC)
static void enable_jzsoc_gpu_clock(void)
{
#if defined(CONFIG_SOC_JZ4770)
	{
		/* JZ4770 GPU CLK2x 100MHz -- 500MHz */
#define GPU_CLK_MAX 500000000
		extern unsigned int cpm_get_pllout1(void);
		unsigned int GPUCDR_VAL=0;
		int div;
		int gpu_use_pll1 = 1;
		unsigned int pll_clk;
		unsigned int gpu_clk;
		
		pll_clk = cpm_get_pllout1();
//		pll_clk = 0 ;

		if ( pll_clk == 0 ) {
			gpu_use_pll1 = 0;	/* use pll0 */
			pll_clk = cpm_get_pllout();
#if defined(CONFIG_SOC_JZ4770)
			if ((INREG32(CPM_CPCCR) & CPCCR_PCS) != 0 )
#else
			if ((INREG32(CPM_CPCCR) & CPCCR_PCS) == 0 ) /* JZ4760 */
#endif
				pll_clk /= 2;
		}

		for ( div=1; div <= ((GPUCDR_GPUDIV_MASK>>GPUCDR_GPUDIV_LSB)+1); div++ ) {
			gpu_clk = pll_clk/div;
			if ( gpu_clk < GPU_CLK_MAX )
				break;
		}

		cpm_stop_clock(CGM_GPU);
		GPUCDR_VAL = (div-1);
		if (gpu_use_pll1)
			GPUCDR_VAL |= 1<<31;
		REG_CPM_GPUCDR = GPUCDR_VAL;
		cpm_start_clock(CGM_GPU);

		printk("REG_CPM_GPUCDR= 0x%08x\n", GPUCDR_VAL);
		printk("GPU CLOCK USE PLL%d\n", gpu_use_pll1);
		printk("GPU GPU_CLK2x= %d MHz\n", gpu_clk/1000000);
//		udelay(1000);
	}
#else
#error "Not defined SOC_JZ4770"
#endif
}
#endif

#if !USE_PLATFORM_DRIVER
static int __init drv_init(void)
#else
static int drv_init(void)
#endif
{
    int ret;
    gckGALDEVICE device;

#if ENABLE_GPU_CLOCK_BY_DRIVER && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
    struct clk * clk = NULL;
#endif

    gcmkTRACE_ZONE(gcvLEVEL_VERBOSE, gcvZONE_DRIVER,
    	    	  "Entering drv_init\n");

#if defined(CONFIG_JZSOC)
    enable_jzsoc_gpu_clock(); // remove kernel/arch/mips/mach-jz4770/common/setup.c
#endif

#if ENABLE_GPU_CLOCK_BY_DRIVER && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
    clk = clk_get(NULL, "GCCLK");
    if (IS_ERR(clk))
    {
        int retval = PTR_ERR(clk);
        printk("clk get error: %d\n", retval);
        return -ENODEV;
    }

    /* APMU_GC_156M, APMU_GC_624M, APMU_GC_PLL2, APMU_GC_PLL2_DIV2 currently */
    if (clk_set_rate(clk, 624000000))
    {
       	gcmkTRACE_ZONE(gcvLEVEL_ERROR, gcvZONE_DRIVER,
    	    	      "[galcore] Can't set core clock.");
        return -EAGAIN;
    }
    clk_enable(clk);
#endif

	if (showArgs)
	{
		printk("galcore options:\n");
		printk("  irqLine         = %d\n",      irqLine);
		printk("  registerMemBase = 0x%08lX\n", registerMemBase);
		printk("  contiguousSize  = %ld\n",     contiguousSize);
		printk("  contiguousBase  = 0x%08lX\n", contiguousBase);
		printk("  bankSize        = 0x%08lX\n", bankSize);
		printk("  fastClear       = %d\n",      fastClear);
		printk("  compression     = %d\n",      compression);
		printk("  signal          = %d\n",      signal);
		printk("  baseAddress     = 0x%08lX\n", baseAddress);
	}

    /* Create the GAL device. */
    gcmkVERIFY_OK(gckGALDEVICE_Construct(irqLine,
    	    	    	    	    	registerMemBase,
					registerMemSize,
					contiguousBase,
					contiguousSize,
					bankSize,
					fastClear,
					compression,
					baseAddress,
					signal,
					&device));

    /* Start the GAL device. */
    if (gcmIS_ERROR(gckGALDEVICE_Start(device)))
    {
    	gcmkTRACE_ZONE(gcvLEVEL_ERROR, gcvZONE_DRIVER,
    	    	      "[galcore] Can't start the gal device.\n");

    	/* Roll back. */
    	gckGALDEVICE_Stop(device);
    	gckGALDEVICE_Destroy(device);

    	return -1;
    }

    /* Register the character device. */
    ret = register_chrdev(major, DRV_NAME, &driver_fops);
    if (ret < 0)
    {
    	gcmkTRACE_ZONE(gcvLEVEL_ERROR, gcvZONE_DRIVER,
    	    	      "[galcore] Could not allocate major number for mmap.\n");

    	/* Roll back. */
    	gckGALDEVICE_Stop(device);
    	gckGALDEVICE_Destroy(device);

    	return -1;
    }
    else
    {
    	if (major == 0)
    	{
    	    major = ret;
    	}
    }

    galDevice = device;

	gpuClass = class_create(THIS_MODULE, "graphics_class");
	if (IS_ERR(gpuClass)) {
    	gcmkTRACE_ZONE(gcvLEVEL_ERROR, gcvZONE_DRIVER,
					  "Failed to create the class.\n");
		return -1;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
	device_create(gpuClass, NULL, MKDEV(major, 0), NULL, "galcore");
#else
	device_create(gpuClass, NULL, MKDEV(major, 0), "galcore");
#endif

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_DRIVER,
    	    	  "[galcore] irqLine->%ld, contiguousSize->%lu, memBase->0x%lX\n",
		  irqLine,
		  contiguousSize,
		  registerMemBase);

    gcmkTRACE_ZONE(gcvLEVEL_VERBOSE, gcvZONE_DRIVER,
    	    	  "[galcore] driver registered successfully.\n");

    return 0;
}

#if !USE_PLATFORM_DRIVER
static void __exit drv_exit(void)
#else
static void drv_exit(void)
#endif
{
#if ENABLE_GPU_CLOCK_BY_DRIVER && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
    struct clk * clk = NULL;
#endif
    gcmkTRACE_ZONE(gcvLEVEL_VERBOSE, gcvZONE_DRIVER,
    	    	  "[galcore] Entering drv_exit\n");

	device_destroy(gpuClass, MKDEV(major, 0));
	class_destroy(gpuClass);

    unregister_chrdev(major, DRV_NAME);

    gckGALDEVICE_Stop(galDevice);
    gckGALDEVICE_Destroy(galDevice);

#if ENABLE_GPU_CLOCK_BY_DRIVER && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
    clk = clk_get(NULL, "GCCLK");
    clk_disable(clk);
#endif
}

#if !USE_PLATFORM_DRIVER
module_init(drv_init);
module_exit(drv_exit);
#else

#ifdef CONFIG_DOVE_GPU
#define DEVICE_NAME "dove_gpu"
#else
#define DEVICE_NAME "galcore"
#endif


static int __devinit gpu_probe(struct platform_device *pdev)
{
	int ret = -ENODEV;
	struct resource *res;
	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,"gpu_irq");
	if (!res) {
		printk(KERN_ERR "%s: No irq line supplied.\n",__FUNCTION__);
		goto gpu_probe_fail;
	}
	irqLine = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,"gpu_base");
	if (!res) {
		printk(KERN_ERR "%s: No register base supplied.\n",__FUNCTION__);
		goto gpu_probe_fail;
	}
	registerMemBase = res->start;
	registerMemSize = res->end - res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,"gpu_mem");
	if (!res) {
		printk(KERN_ERR "%s: No memory base supplied.\n",__FUNCTION__);
		goto gpu_probe_fail;
	}
	contiguousBase  = res->start;
	contiguousSize  = res->end-res->start;

	ret = drv_init();
	if(!ret) {
		platform_set_drvdata(pdev,galDevice);
		return ret;
	}

gpu_probe_fail:
	printk(KERN_INFO "Failed to register gpu driver.\n");
	return ret;
}

static int __devinit gpu_remove(struct platform_device *pdev)
{
	drv_exit();

	return 0;
}

static int __devinit gpu_suspend(struct platform_device *dev, pm_message_t state)
{
	gceSTATUS status;
	gckGALDEVICE device;

	device = platform_get_drvdata(dev);

	status = gckHARDWARE_SetPowerManagementState(device->kernel->hardware, gcvPOWER_OFF);

#if defined(CONFIG_JZSOC)
	cpm_stop_clock(CGM_GPU);
#endif

	if (gcmIS_ERROR(status))
	{
		return -1;
	}

	return 0;
}

static int __devinit gpu_resume(struct platform_device *dev)
{
	gceSTATUS status;
	gckGALDEVICE device;

#if defined(CONFIG_JZSOC)
	cpm_start_clock(CGM_GPU);
#endif

	device = platform_get_drvdata(dev);

	status = gckHARDWARE_SetPowerManagementState(device->kernel->hardware, gcvPOWER_ON);

	if (gcmIS_ERROR(status))
	{
		return -1;
	}

	return 0;
}

static struct platform_driver gpu_driver = {
	.probe		= gpu_probe,
	.remove		= gpu_remove,

	.suspend	= gpu_suspend,
	.resume		= gpu_resume,

	.driver		= {
		.name	= DEVICE_NAME,
	}
};

#ifndef CONFIG_DOVE_GPU
static struct resource gpu_resources[] = {
    {
        .name   = "gpu_irq",
        .flags  = IORESOURCE_IRQ,
    },
    {
        .name   = "gpu_base",
        .flags  = IORESOURCE_MEM,
    },
    {
        .name   = "gpu_mem",
        .flags  = IORESOURCE_MEM,
    },
};

static struct platform_device * gpu_device;
#endif

static int __init gpu_init(void)
{
	int ret = 0;

#ifndef CONFIG_DOVE_GPU
	gpu_resources[0].start = gpu_resources[0].end = irqLine;

	gpu_resources[1].start = registerMemBase;
	gpu_resources[1].end   = registerMemBase + registerMemSize;

	gpu_resources[2].start = contiguousBase;
	gpu_resources[2].end   = contiguousBase + contiguousSize;

	/* Allocate device */
	gpu_device = platform_device_alloc(DEVICE_NAME, -1);
	if (!gpu_device)
	{
		printk(KERN_ERR "galcore: platform_device_alloc failed.\n");
		ret = -ENOMEM;
		goto out;
	}

	/* Insert resource */
	ret = platform_device_add_resources(gpu_device, gpu_resources, 3);
	if (ret)
	{
		printk(KERN_ERR "galcore: platform_device_add_resources failed.\n");
		goto put_dev;
	}

	/* Add device */
	ret = platform_device_add(gpu_device);
	if (ret)
	{
		printk(KERN_ERR "galcore: platform_device_add failed.\n");
		goto del_dev;
	}
#endif

	ret = platform_driver_register(&gpu_driver);
	if (!ret)
	{
		goto out;
	}

#ifndef CONFIG_DOVE_GPU
del_dev:
	platform_device_del(gpu_device);
put_dev:
	platform_device_put(gpu_device);
#endif

out:
	return ret;

}

static void __exit gpu_exit(void)
{
	platform_driver_unregister(&gpu_driver);
#ifndef CONFIG_DOVE_GPU
	platform_device_unregister(gpu_device);
#endif
}

module_init(gpu_init);
module_exit(gpu_exit);

#endif

