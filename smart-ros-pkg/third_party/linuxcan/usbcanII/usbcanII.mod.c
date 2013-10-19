#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x942dc435, "module_layout" },
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0xd2b09ce5, "__kmalloc" },
	{ 0xadaabe1b, "pv_lock_ops" },
	{ 0x86d5255f, "_raw_write_lock_irqsave" },
	{ 0x6a1ceaf2, "dev_set_drvdata" },
	{ 0x43a53735, "__alloc_workqueue_key" },
	{ 0xc8b57c27, "autoremove_wake_function" },
	{ 0xb5dcab5b, "remove_wait_queue" },
	{ 0x1637ff0f, "_raw_spin_lock_bh" },
	{ 0xf89843f9, "schedule_work" },
	{ 0xe0130dce, "usb_kill_urb" },
	{ 0xf2eff307, "remove_proc_entry" },
	{ 0x6729d3df, "__get_user_4" },
	{ 0x26576139, "queue_work" },
	{ 0xf287ea9c, "__register_chrdev" },
	{ 0xfb0e29f, "init_timer_key" },
	{ 0x91715312, "sprintf" },
	{ 0x3bcbbb33, "kthread_create_on_node" },
	{ 0x7d11c268, "jiffies" },
	{ 0x168f1082, "_raw_write_unlock_irqrestore" },
	{ 0xf432dd3d, "__init_waitqueue_head" },
	{ 0x4f8b5ddb, "_copy_to_user" },
	{ 0xffd5a395, "default_wake_function" },
	{ 0x6d0aba34, "wait_for_completion" },
	{ 0xd5f2172f, "del_timer_sync" },
	{ 0xfb578fc5, "memset" },
	{ 0x8f64aa4, "_raw_spin_unlock_irqrestore" },
	{ 0x5310fe6d, "current_task" },
	{ 0x562b9321, "usb_deregister" },
	{ 0x27e1a049, "printk" },
	{ 0xa1c76e0a, "_cond_resched" },
	{ 0xb4390f9a, "mcount" },
	{ 0x8c03d20c, "destroy_workqueue" },
	{ 0xbe2c0274, "add_timer" },
	{ 0x698398fb, "usb_free_coherent" },
	{ 0x9642e329, "usb_submit_urb" },
	{ 0x78764f4e, "pv_irq_ops" },
	{ 0x1790d76b, "_raw_read_lock_irqsave" },
	{ 0xb2fd5ceb, "__put_user_4" },
	{ 0xba63339c, "_raw_spin_unlock_bh" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0x3bd1b1f6, "msecs_to_jiffies" },
	{ 0xcbdb9f8, "usb_bulk_msg" },
	{ 0xd62c833f, "schedule_timeout" },
	{ 0x1000e51, "schedule" },
	{ 0x6d334118, "__get_user_8" },
	{ 0x43261dca, "_raw_spin_lock_irq" },
	{ 0x703b4352, "_raw_read_unlock_irqrestore" },
	{ 0x9276b9c9, "create_proc_entry" },
	{ 0xdfdb413f, "__module_put_and_exit" },
	{ 0xa9f30641, "wake_up_process" },
	{ 0xd52bf1ce, "_raw_spin_lock" },
	{ 0x9327f5ce, "_raw_spin_lock_irqsave" },
	{ 0xcf21d241, "__wake_up" },
	{ 0x4f68e5c9, "do_gettimeofday" },
	{ 0x5860aad4, "add_wait_queue" },
	{ 0x37a0cba, "kfree" },
	{ 0x69acdf38, "memcpy" },
	{ 0x801678, "flush_scheduled_work" },
	{ 0x5c8b5ce8, "prepare_to_wait" },
	{ 0xdeb87602, "usb_register_driver" },
	{ 0xfa66f77c, "finish_wait" },
	{ 0x4b06d2e7, "complete" },
	{ 0xcb122738, "usb_alloc_coherent" },
	{ 0x4f6b400b, "_copy_from_user" },
	{ 0x466649c3, "dev_get_drvdata" },
	{ 0x56973dc, "usb_free_urb" },
	{ 0x91f04916, "try_module_get" },
	{ 0x17bc9672, "usb_alloc_urb" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("usb:v0BFDp0004d*dc*dsc*dp*ic*isc*ip*");
MODULE_ALIAS("usb:v0BFDp0002d*dc*dsc*dp*ic*isc*ip*");
MODULE_ALIAS("usb:v0BFDp0005d*dc*dsc*dp*ic*isc*ip*");
MODULE_ALIAS("usb:v0BFDp0003d*dc*dsc*dp*ic*isc*ip*");

MODULE_INFO(srcversion, "3DA05C32CB95D8485910721");
