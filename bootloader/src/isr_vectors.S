.syntax unified
.thumb

.section .text.SysTick_Handler,"ax",%progbits
.global SysTick_Handler
.type SysTick_Handler,%function
.thumb_func
SysTick_Handler:
    b boot_systick_handler_impl
.size SysTick_Handler, .-SysTick_Handler

.section .text.OTG_FS_IRQHandler,"ax",%progbits
.global OTG_FS_IRQHandler
.type OTG_FS_IRQHandler,%function
.thumb_func
OTG_FS_IRQHandler:
    b boot_otg_fs_irq_impl
.size OTG_FS_IRQHandler, .-OTG_FS_IRQHandler
