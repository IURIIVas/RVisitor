/// \file core_riscv.h
/// \brief RISC-V core peripheral access layer header file for ch32v307
/// \author 1jura.vas@gmail.com
///
/// \details
///
#ifndef __CORE_RISCV_H__
#define __CORE_RISCV_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* IO definitions */
#ifdef __cplusplus
  #define     __RO      volatile                     /* defines 'read only' permissions */
#else
  #define     __RO      volatile const          /* defines 'read only' permissions */
#endif
#define __WO            volatile                /* defines 'write only' permissions */
#define __RW            volatile                /* defines 'read / write' permissions */

#define SYSRST_BIT       (31)
#define SETEVENT_BIT     (5)
#define SEVOPEND_BIT     (4)
#define WFITOWFE_BIT     (3)
#define SLEEPDEEP_BIT    (2)
#define SLEEPONEXIT_BIT  (1)

typedef enum {NO_READY = 0, READY = 1} error_status_e;

typedef enum {DISABLE = 0, ENABLE = 1} functional_state_e;

typedef enum {RESET = 0, SET = 1} flag_status_e, it_status_e;

typedef enum {FAIL = 0, OK = 1} status_e;

typedef struct{
  __RO uint32_t isr[8];
  __RO uint32_t ipr[8];
  __RW uint32_t ithresdr;
       uint32_t reserved;
  __RW uint32_t cfgr;
  __RO uint32_t gisr;
  __RW uint8_t vtfidr[4];
       uint8_t reserved0[12];
  __RW uint32_t vtfaddr[4];
       uint8_t reserved1[0x90];
  __WO uint32_t ienr[8];
       uint8_t reserved2[0x60];
  __WO uint32_t irer[8];
       uint8_t reserved3[0x60];
  __WO uint32_t ipsr[8];
       uint8_t reserved4[0x60];
  __WO uint32_t iprr[8];
       uint8_t reserved5[0x60];
  __RW uint32_t iactr[8];
       uint8_t reserved6[0xE0];
  __RW uint8_t iprior[256];
       uint8_t reserved7[0x810];
  __RW uint32_t sctlr;
} pfic_s;

/* memory mapped structure for SysTick */
typedef struct
{
    __RW uint32_t ctlr;
    __RW uint32_t sr;
    __RW uint64_t cnt;
    __RW uint64_t cmp;
} systick_s;


#define PFIC            ((pfic_s *) 0xE000E000)

#define CFG_RSTSYS_BIT  (7)
#define PFIC_KEY1       ((uint32_t) 0xFA050000)
#define	PFIC_KEY2		((uint32_t) 0xBCAF0000)
#define	PFIC_KEY3		((uint32_t) 0xBEEF0000)

#define SYSTICK         ((systick_s *) 0xE000F000)

/// \brief Enable global interrupt
/// \param None
/// \return None
static inline void __enable_irq()
{
  __asm volatile ("csrw 0x800, %0" : : "r" (0x6088) );
}

/// \brief disable global interrupt
/// \param None
/// \return None
static inline void __disable_irq()
{
  __asm volatile ("csrw 0x800, %0" : : "r" (0x6000) );
}

/// \brief nop
/// \param None
/// \return None
static inline void __NOP()
{
  __asm volatile ("nop");
}

/// \brief enable interrupt
/// \param irqn: interrupt number
/// \return None
static inline void __pfic_enable_irq(irqn_e irqn)
{
    PFIC->ienr[((uint32_t)(irqn) >> 5)] = (1 << ((uint32_t)(irqn) & 0x1F));
}

/// \brief disable interrupt
/// \param irqn: interrupt number
/// \return None
static inline void __pfic_disable_irq(irqn_e irqn)
{
    PFIC->irer[((uint32_t)(irqn) >> 5)] = (1 << ((uint32_t)(irqn) & 0x1F));
}

/// \brief get interrupt enable state
/// \param irqn: interrupt number
/// \return 1 - interrupt enable
///         0 - interrupt disable
static inline uint32_t __pfic_get_status_irq(irqn_e irqn)
{
    return((uint32_t) ((PFIC->isr[(uint32_t)(irqn) >> 5] & (1 << ((uint32_t)(irqn) & 0x1F))) ? 1 : 0));
}

/// \brief get interrupt pending state
/// \param irqn: interrupt number
/// \return 1 - interrupt pending enable
///         0 - interrupt pending disable
static inline uint32_t __pfic_get_pending_irq(irqn_e irqn)
{
    return((uint32_t) ((PFIC->ipr[(uint32_t)(irqn) >> 5] & (1 << ((uint32_t)(irqn) & 0x1F))) ? 1 : 0));
}

/// \brief set interrupt pending state
/// \param irqn: interrupt number
/// \return None
static inline void __pfic_set_pending_irq(irqn_e irqn)
{
    PFIC->ipsr[((uint32_t)(irqn) >> 5)] = (1 << ((uint32_t)(irqn) & 0x1F));
}

/// \brief clear interrupt pending state
/// \param irqn: interrupt number
/// \return None
static inline void __pfic_clear_pending_irq(irqn_e irqn)
{
    PFIC->iprr[((uint32_t)(irqn) >> 5)] = (1 << ((uint32_t)(irqn) & 0x1F));
}

/// \brief get interrupt active state
/// \param irqn: interrupt number
/// \return 1 - interrupt active
///         0 - interrupt no active
static inline uint32_t __pfic_get_active(irqn_e irqn)
{
    return((uint32_t)((PFIC->iactr[(uint32_t)(irqn) >> 5] & (1 << ((uint32_t)(irqn) & 0x1F))) ? 1 : 0));
}

/// \brief set interrupt priority
/// \param irqn: interrupt number
///        priority: bit7 - pre-emption priority
///                  bit6~4 - subpriority
/// \return None
static inline void __pfic_set_priority(irqn_e irqn, uint8_t priority)
{
    PFIC->iprior[(uint32_t)(irqn)] = priority;
}

/// \brief Wait for interrupt
/// \param None
/// \return None
__attribute__( ( always_inline ) ) static inline void __WFI(void)
{
    PFIC->sctlr &= ~(1 << WFITOWFE_BIT);	// wfi
    asm volatile ("wfi");
}

/// \brief Wait for events
/// \param None
/// \return None
__attribute__( ( always_inline ) ) static inline void __WFE(void)
{
    uint32_t t;

    t = PFIC->sctlr;
    PFIC->sctlr |= (1 << WFITOWFE_BIT) | (1 << SETEVENT_BIT);		// (wfi->wfe) + (__sev)
    PFIC->sctlr = (PFIC->sctlr & ~(1 << SETEVENT_BIT)) | (t & (1 << SETEVENT_BIT));
    asm volatile ("wfi");
    asm volatile ("wfi");
}

/// \brief Set VTF interrupt
/// \param addr - VTF interrupt service function base address
///        irqn - interrupt number
///        num - VTF interrupt number
///        new_state - DISABLE or ENABLE
/// \return None
static inline void SetVTFIRQ(uint32_t addr, irqn_e irqn, uint8_t num, functional_state_e new_state)
{
    if (num > 3)
    {
        return;
    }

    if (new_state != DISABLE)
    {
        PFIC->vtfidr[num] = irqn;
        PFIC->vtfaddr[num] = ((addr & 0xFFFFFFFE) | 0x1);
    }
    else{
        PFIC->vtfidr[num] = irqn;
        PFIC->vtfaddr[num] = ((addr & 0xFFFFFFFE) & (~0x1));
    }
}

/// \brief Initiate a system reset request
/// \param None
/// \return None
static inline void __pfic_system_reset(void)
{
    PFIC->cfgr = PFIC_KEY3 | (1 << CFG_RSTSYS_BIT);
}


/* Core_Exported_Functions */  
extern uint32_t __get_FFLAGS(void);
extern void __set_FFLAGS(uint32_t value);
extern uint32_t __get_FRM(void);
extern void __set_FRM(uint32_t value);
extern uint32_t __get_FCSR(void);
extern void __set_FCSR(uint32_t value);
extern uint32_t __get_MSTATUS(void);
extern void __set_MSTATUS(uint32_t value);
extern uint32_t __get_MISA(void);
extern void __set_MISA(uint32_t value);
extern uint32_t __get_MTVEC(void);
extern void __set_MTVEC(uint32_t value);
extern uint32_t __get_MSCRATCH(void);
extern void __set_MSCRATCH(uint32_t value);
extern uint32_t __get_MEPC(void);
extern void __set_MEPC(uint32_t value);
extern uint32_t __get_MCAUSE(void);
extern void __set_MCAUSE(uint32_t value);
extern uint32_t __get_MTVAL(void);
extern void __set_MTVAL(uint32_t value);
extern uint32_t __get_MVENDORID(void);
extern uint32_t __get_MARCHID(void);
extern uint32_t __get_MIMPID(void);
extern uint32_t __get_MHARTID(void);
extern uint32_t __get_SP(void);

#ifdef __cplusplus
}
#endif

#endif





