#if defined(F103_BUILD_BOOTLOADER) && !defined(F103_BOOT_USB_LEGACY)

#include "boot_usb.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_pcd.h"
#include "stm32f1xx_ll_usb.h"
#include <string.h>

/*
 * Minimal STM32F103 USB CDC ACM transport for the resident bootloader.
 * No USBD Core/class middleware is used. Only EP0 control, EP1 bulk IN/OUT,
 * and the CDC notification descriptor on EP2 are retained.
 */

#define RX_RING_SIZE 512U
#define EP0_MPS      64U
#define DATA_MPS     64U

#define EP0_RX_PMA   0x18U
#define EP0_TX_PMA   0x58U
#define EP1_TX_PMA   0xC0U
#define EP2_TX_PMA   0x100U
#define EP1_RX_PMA   0x110U

#define REQ_GET_STATUS        0x00U
#define REQ_SET_ADDRESS       0x05U
#define REQ_GET_DESCRIPTOR    0x06U
#define REQ_GET_CONFIGURATION 0x08U
#define REQ_SET_CONFIGURATION 0x09U
#define REQ_GET_INTERFACE     0x0AU
#define REQ_SET_INTERFACE     0x0BU

#define CDC_SET_LINE_CODING        0x20U
#define CDC_GET_LINE_CODING        0x21U
#define CDC_SET_CONTROL_LINE_STATE 0x22U
#define CDC_SEND_BREAK             0x23U

#define DESC_DEVICE        0x01U
#define DESC_CONFIGURATION 0x02U
#define DESC_STRING        0x03U

static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t rx_dropped;
static uint8_t rx_ring[RX_RING_SIZE];
static uint8_t ep_rx[DATA_MPS];

static volatile bool configured;
static volatile bool tx_busy;

static const uint8_t *ctrl_ptr;
static uint16_t ctrl_remaining;
static bool ctrl_send_zlp;
static uint8_t pending_address;
static uint8_t ctrl_out_kind;
static uint8_t line_coding[7] = {0x40U,0x42U,0x0FU,0x00U,0x00U,0x00U,0x08U};

static const uint8_t dev_desc[] = {
  18, DESC_DEVICE, 0x00,0x02, 0x02,0x02,0x00, EP0_MPS,
  0x83,0x04, 0x41,0x57, 0x00,0x02, 1,2,3,1
};

/* Standard CDC ACM config: EP2 interrupt IN, EP1 bulk OUT/IN. */
static const uint8_t cfg_desc[] = {
  9,DESC_CONFIGURATION,67,0, 2,1,0,0x80,50,
  9,4,0,0,1, 0x02,0x02,0x01,0,
  5,0x24,0x00,0x10,0x01,
  5,0x24,0x01,0x00,0x01,
  4,0x24,0x02,0x02,
  5,0x24,0x06,0x00,0x01,
  7,5,0x82,0x03,8,0,16,
  9,4,1,0,2, 0x0A,0x00,0x00,0,
  7,5,0x01,0x02,64,0,0,
  7,5,0x81,0x02,64,0,0
};
static const uint8_t lang_desc[] = {4,DESC_STRING,0x09,0x04};
static const uint8_t mfg_desc[] = {
  38,DESC_STRING,
  'S',0,'T',0,'M',0,'i',0,'c',0,'r',0,'o',0,'e',0,'l',0,'e',0,'c',0,
  't',0,'r',0,'o',0,'n',0,'i',0,'c',0,'s',0
};
static const uint8_t product_desc[] = {
  46,DESC_STRING,
  'B',0,'L',0,'U',0,'E',0,'P',0,'I',0,'L',0,'L',0,'_',0,'F',0,'1',0,'0',0,
  '3',0,' ',0,'B',0,'O',0,'O',0,'T',0,' ',0,'C',0,'D',0,'C',0
};
static uint8_t serial_desc[26] = {26,DESC_STRING};

static void build_serial_desc(void) {
  uint32_t value[2];
  value[0] = *(const uint32_t *)0x1FFFF7E8UL + *(const uint32_t *)0x1FFFF7F0UL;
  value[1] = *(const uint32_t *)0x1FFFF7ECUL;
  uint8_t *dst = &serial_desc[2];
  for (uint8_t word=0U; word<2U; ++word) {
    const uint8_t digits = word==0U ? 8U : 4U;
    for (uint8_t i=0U; i<digits; ++i) {
      const uint8_t nibble = (uint8_t)(value[word] >> 28U);
      const uint8_t base = nibble < 10U ? (uint8_t)'0' : (uint8_t)('A' - 10);
      *dst++ = (uint8_t)(base + nibble);
      *dst++ = 0U;
      value[word] <<= 4U;
    }
  }
}
_Static_assert(sizeof(dev_desc) == 18U, "USB device descriptor size mismatch");
_Static_assert(sizeof(cfg_desc) == 67U, "USB CDC config descriptor size mismatch");
_Static_assert(sizeof(mfg_desc) == 38U, "USB manufacturer descriptor size mismatch");
_Static_assert(sizeof(product_desc) == 46U, "USB product descriptor size mismatch");

static void usb_ep0_init(void) {
  PCD_SET_ENDPOINT(USB,0U,(uint16_t)(USB_EP_CONTROL|USB_EP_CTR_RX|USB_EP_CTR_TX));
  PCD_SET_EP_ADDRESS(USB,0U,0U);
  PCD_SET_EP_TX_ADDRESS(USB,0U,EP0_TX_PMA);
  PCD_SET_EP_RX_ADDRESS(USB,0U,EP0_RX_PMA);
  PCD_CLEAR_TX_DTOG(USB,0U);
  PCD_CLEAR_RX_DTOG(USB,0U);
  PCD_SET_EP_TX_CNT(USB,0U,0U);
  PCD_SET_EP_RX_CNT(USB,0U,EP0_MPS);
  PCD_SET_EP_TX_STATUS(USB,0U,USB_EP_TX_NAK);
  PCD_SET_EP_RX_STATUS(USB,0U,USB_EP_RX_VALID);
}

static void usb_data_eps_init(void) {
  PCD_SET_ENDPOINT(USB,1U,(uint16_t)(USB_EP_BULK|1U|USB_EP_CTR_RX|USB_EP_CTR_TX));
  PCD_SET_EP_TX_ADDRESS(USB,1U,EP1_TX_PMA);
  PCD_SET_EP_RX_ADDRESS(USB,1U,EP1_RX_PMA);
  PCD_CLEAR_TX_DTOG(USB,1U);
  PCD_CLEAR_RX_DTOG(USB,1U);
  PCD_SET_EP_TX_CNT(USB,1U,0U);
  PCD_SET_EP_RX_CNT(USB,1U,DATA_MPS);
  PCD_SET_EP_TX_STATUS(USB,1U,USB_EP_TX_NAK);
  PCD_SET_EP_RX_STATUS(USB,1U,USB_EP_RX_VALID);

  PCD_SET_ENDPOINT(USB,2U,(uint16_t)(USB_EP_INTERRUPT|2U|USB_EP_CTR_RX|USB_EP_CTR_TX));
  PCD_SET_EP_TX_ADDRESS(USB,2U,EP2_TX_PMA);
  PCD_CLEAR_TX_DTOG(USB,2U);
  PCD_SET_EP_TX_CNT(USB,2U,0U);
  PCD_SET_EP_TX_STATUS(USB,2U,USB_EP_TX_NAK);
  PCD_SET_EP_RX_STATUS(USB,2U,USB_EP_RX_DIS);
}

static void ep0_tx_start(const uint8_t *data,uint16_t len) {
  USB_WritePMA(USB,(uint8_t *)data,EP0_TX_PMA,len);
  PCD_SET_EP_TX_CNT(USB,0U,len);
  PCD_SET_EP_TX_STATUS(USB,0U,USB_EP_TX_VALID);
}

static void ep0_send_next(void) {
  uint16_t n = ctrl_remaining > EP0_MPS ? EP0_MPS : ctrl_remaining;
  if (n != 0U) {
    ep0_tx_start(ctrl_ptr,n);
    ctrl_ptr += n;
    ctrl_remaining = (uint16_t)(ctrl_remaining - n);
  } else if (ctrl_send_zlp) {
    ctrl_send_zlp = false;
    ep0_tx_start(NULL,0U);
  } else {
    PCD_SET_EP_TX_STATUS(USB,0U,USB_EP_TX_NAK);
  }
}

static void ep0_send(const uint8_t *data,uint16_t have,uint16_t want) {
  const uint16_t n = have < want ? have : want;
  ctrl_ptr = data;
  ctrl_remaining = n;
  ctrl_send_zlp = (n < want) && ((n & (EP0_MPS-1U)) == 0U);
  ep0_send_next();
}

static void ep0_status(void) {
  ctrl_ptr = NULL;
  ctrl_remaining = 0U;
  ctrl_send_zlp = false;
  ep0_tx_start(NULL,0U);
}

static void ep0_stall(void) {
  PCD_SET_EP_TX_STATUS(USB,0U,USB_EP_TX_STALL);
  PCD_SET_EP_RX_STATUS(USB,0U,USB_EP_RX_STALL);
}

static void setup_request(const uint8_t s[8]) {
  const uint8_t bm=s[0], req=s[1];
  const uint16_t value=(uint16_t)s[2]|((uint16_t)s[3]<<8U);
  const uint16_t index=(uint16_t)s[4]|((uint16_t)s[5]<<8U);
  const uint16_t length=(uint16_t)s[6]|((uint16_t)s[7]<<8U);
  (void)index;
  ctrl_ptr=NULL; ctrl_remaining=0U; ctrl_send_zlp=false; ctrl_out_kind=0U;
  PCD_SET_EP_TX_STATUS(USB,0U,USB_EP_TX_NAK);
  PCD_SET_EP_RX_CNT(USB,0U,EP0_MPS);
  PCD_SET_EP_RX_STATUS(USB,0U,USB_EP_RX_VALID);

  if ((bm & 0x60U) == 0U) {
    switch (req) {
      case REQ_GET_DESCRIPTOR: {
        const uint8_t type=(uint8_t)(value>>8U), idx=(uint8_t)value;
        if (type==DESC_DEVICE) { ep0_send(dev_desc,sizeof(dev_desc),length); return; }
        if (type==DESC_CONFIGURATION) { ep0_send(cfg_desc,sizeof(cfg_desc),length); return; }
        if (type==DESC_STRING) {
          if (idx==0U) { ep0_send(lang_desc,sizeof(lang_desc),length); return; }
          if (idx==1U) { ep0_send(mfg_desc,sizeof(mfg_desc),length); return; }
          if (idx==2U) { ep0_send(product_desc,sizeof(product_desc),length); return; }
          if (idx==3U) { build_serial_desc(); ep0_send(serial_desc,sizeof(serial_desc),length); return; }
        }
        break;
      }
      case REQ_SET_ADDRESS:
        pending_address=(uint8_t)(value&0x7FU); ep0_status(); return;
      case REQ_SET_CONFIGURATION:
        configured=((uint8_t)value)==1U;
        if (configured) usb_data_eps_init();
        ep0_status(); return;
      case REQ_GET_CONFIGURATION: {
        static uint8_t c; c=configured?1U:0U; ep0_send(&c,1U,length); return;
      }
      case REQ_GET_STATUS: {
        static const uint8_t z[2]={0,0}; ep0_send(z,2U,length); return;
      }
      case REQ_GET_INTERFACE: {
        static const uint8_t z=0; ep0_send(&z,1U,length); return;
      }
      case REQ_SET_INTERFACE:
        ep0_status(); return;
      default: break;
    }
  } else if ((bm & 0x60U) == 0x20U) {
    switch (req) {
      case CDC_SET_LINE_CODING:
        if (length==7U) {
          ctrl_out_kind=1U;
          PCD_SET_EP_RX_CNT(USB,0U,EP0_MPS);
          PCD_SET_EP_RX_STATUS(USB,0U,USB_EP_RX_VALID);
          return;
        }
        break;
      case CDC_GET_LINE_CODING:
        ep0_send(line_coding,sizeof(line_coding),length); return;
      case CDC_SET_CONTROL_LINE_STATE:
      case CDC_SEND_BREAK:
        ep0_status(); return;
      default: break;
    }
  }
  ep0_stall();
}

static void usb_bus_reset(void) {
  configured=false; tx_busy=false; pending_address=0U;
  ctrl_ptr=NULL; ctrl_remaining=0U; ctrl_send_zlp=false; ctrl_out_kind=0U;
  USB->DADDR=USB_DADDR_EF;
  usb_ep0_init();
}

void USB_LP_CAN1_RX0_IRQHandler(void) {
  uint16_t istr=USB->ISTR;
  if ((istr&USB_ISTR_RESET)!=0U) {
    USB->ISTR=(uint16_t)~USB_ISTR_RESET;
    usb_bus_reset();
  }

  while ((USB->ISTR&USB_ISTR_CTR)!=0U) {
    istr=USB->ISTR;
    const uint8_t ep=(uint8_t)(istr&USB_ISTR_EP_ID);
    uint16_t epr=PCD_GET_ENDPOINT(USB,ep);

    if (ep==0U) {
      if ((istr&USB_ISTR_DIR)!=0U && (epr&USB_EP_CTR_RX)!=0U) {
        const uint16_t count=(uint16_t)PCD_GET_EP_RX_CNT(USB,0U);
        if ((epr&USB_EP_SETUP)!=0U) {
          uint8_t setup[8];
          USB_ReadPMA(USB,setup,EP0_RX_PMA,8U);
          PCD_CLEAR_RX_EP_CTR(USB,0U);
          setup_request(setup);
        } else {
          if (count!=0U && ctrl_out_kind==1U) {
            const uint16_t n=count<sizeof(line_coding)?count:(uint16_t)sizeof(line_coding);
            USB_ReadPMA(USB,line_coding,EP0_RX_PMA,n);
          }
          PCD_CLEAR_RX_EP_CTR(USB,0U);
          if (ctrl_out_kind!=0U) { ctrl_out_kind=0U; ep0_status(); }
          PCD_SET_EP_RX_CNT(USB,0U,EP0_MPS);
          PCD_SET_EP_RX_STATUS(USB,0U,USB_EP_RX_VALID);
        }
      } else if ((epr&USB_EP_CTR_TX)!=0U) {
        PCD_CLEAR_TX_EP_CTR(USB,0U);
        if (pending_address!=0U) {
          USB->DADDR=(uint16_t)(USB_DADDR_EF|pending_address);
          pending_address=0U;
        }
        ep0_send_next();
      }
    } else {
      if ((epr&USB_EP_CTR_RX)!=0U) {
        const uint16_t count=(uint16_t)PCD_GET_EP_RX_CNT(USB,ep);
        if (ep==1U && count!=0U) {
          const uint16_t n=count>sizeof(ep_rx)?sizeof(ep_rx):count;
          USB_ReadPMA(USB,ep_rx,EP1_RX_PMA,n);
          boot_usb_on_receive(ep_rx,n);
        }
        PCD_CLEAR_RX_EP_CTR(USB,ep);
        if (ep==1U) {
          PCD_SET_EP_RX_CNT(USB,1U,DATA_MPS);
          PCD_SET_EP_RX_STATUS(USB,1U,USB_EP_RX_VALID);
        }
      }
      epr=PCD_GET_ENDPOINT(USB,ep);
      if ((epr&USB_EP_CTR_TX)!=0U) {
        PCD_CLEAR_TX_EP_CTR(USB,ep);
        if (ep==1U) tx_busy=false;
      }
    }
  }

  if ((USB->ISTR&USB_ISTR_ERR)!=0U) USB->ISTR=(uint16_t)~USB_ISTR_ERR;
  if ((USB->ISTR&USB_ISTR_PMAOVR)!=0U) USB->ISTR=(uint16_t)~USB_ISTR_PMAOVR;
}

static void pa12_low(void) {
  RCC->APB2ENR|=RCC_APB2ENR_IOPAEN; __DSB();
  uint32_t v=GPIOA->CRH;
  v=(v&~(0xFUL<<16U))|(0x2UL<<16U);
  GPIOA->CRH=v; GPIOA->BRR=GPIO_PIN_12; __DSB();
}
static void pa12_release(void) {
  RCC->APB2ENR|=RCC_APB2ENR_IOPAEN; __DSB();
  uint32_t v=GPIOA->CRH;
  v=(v&~(0xFUL<<16U))|(0x4UL<<16U);
  GPIOA->CRH=v; __DSB();
}

bool boot_usb_begin(void) {
  pa12_low();
  HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);

  __HAL_RCC_USB_CLK_ENABLE();
  USB->CNTR=(uint16_t)(USB_CNTR_FRES|USB_CNTR_PDWN);
  __DSB();
  HAL_Delay(20U);

  __HAL_RCC_USB_FORCE_RESET(); __DSB();
  __HAL_RCC_USB_RELEASE_RESET();
  __HAL_RCC_USB_CLK_ENABLE();
  HAL_Delay(350U);

  rx_head=rx_tail=0U; rx_dropped=0U; configured=false; tx_busy=false;
  USB->CNTR=USB_CNTR_FRES;
  USB->CNTR=0U;
  USB->ISTR=0U;
  USB->BTABLE=0U;
  usb_bus_reset();
  USB->CNTR=(uint16_t)(USB_CNTR_CTRM|USB_CNTR_RESETM|USB_CNTR_ERRM|USB_CNTR_PMAOVRM);
  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn,2U,0U);
  HAL_NVIC_ClearPendingIRQ(USB_LP_CAN1_RX0_IRQn);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  pa12_release();
  return true;
}

void boot_usb_end(void) {
  HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  USB->CNTR=USB_CNTR_FRES;
  USB->DADDR=0U;
  configured=false; tx_busy=false;
  __HAL_RCC_USB_CLK_DISABLE();
}

void boot_usb_disconnect_hold(void) { boot_usb_end(); pa12_low(); }
uint32_t boot_usb_rx_dropped(void) { return rx_dropped; }
bool boot_usb_connected(void) { return configured; }

int boot_usb_available(void) {
  const uint16_t h=rx_head,t=rx_tail;
  return h>=t?(int)(h-t):(int)(RX_RING_SIZE-t+h);
}
int boot_usb_read(void) {
  if (rx_head==rx_tail) return -1;
  const uint8_t v=rx_ring[rx_tail];
  rx_tail=(uint16_t)((rx_tail+1U)%RX_RING_SIZE);
  return v;
}
void boot_usb_on_receive(const uint8_t *data,uint32_t length) {
  for (uint32_t i=0U;i<length;++i) {
    const uint16_t next=(uint16_t)((rx_head+1U)%RX_RING_SIZE);
    if (next==rx_tail) { ++rx_dropped; break; }
    rx_ring[rx_head]=data[i]; rx_head=next;
  }
}

static bool wait_tx_idle(uint32_t timeout_ms) {
  const uint32_t start=HAL_GetTick();
  while (tx_busy) {
    if ((uint32_t)(HAL_GetTick()-start)>=timeout_ms) return false;
  }
  return true;
}
static bool write_bytes(const uint8_t *data,uint32_t len,uint32_t timeout_ms) {
  while (len!=0U) {
    if (!configured || !wait_tx_idle(timeout_ms)) return false;
    const uint16_t n=(uint16_t)(len>DATA_MPS?DATA_MPS:len);
    USB_WritePMA(USB,(uint8_t *)data,EP1_TX_PMA,n);
    PCD_SET_EP_TX_CNT(USB,1U,n);
    tx_busy=true;
    PCD_SET_EP_TX_STATUS(USB,1U,USB_EP_TX_VALID);
    if (!wait_tx_idle(timeout_ms)) return false;
    data+=n; len-=n;
  }
  return true;
}
bool boot_usb_write_line(const char *line,uint32_t timeout_ms) {
  if (line==NULL) return false;
  const uint32_t n=(uint32_t)strlen(line);
  if (!write_bytes((const uint8_t *)line,n,timeout_ms)) return false;
  static const uint8_t eol[2]={'\r','\n'};
  return write_bytes(eol,2U,timeout_ms);
}
void boot_usb_flush(uint32_t timeout_ms) { (void)wait_tx_idle(timeout_ms); }

#endif
