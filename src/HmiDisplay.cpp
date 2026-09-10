#include "HmiDisplay.h"
#include "BoardSupport.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "fonts/FreeSans9pt7b.h"
#include "fonts/FreeSansBold9pt7b.h"
#include "fonts/FreeSansBold12pt7b.h"
#include "fonts/FreeSansBold24pt7b.h"
#include "fonts/Classic5x7.inc"
#include "fonts/Font16.inc"

namespace {
constexpr uint16_t kTftCs = GPIO_PIN_0;
constexpr uint16_t kTftDc = GPIO_PIN_1;
constexpr uint16_t kTftRst = GPIO_PIN_2;
constexpr uint16_t kTouchCs = GPIO_PIN_4;
inline void TftCs(bool high) { HAL_GPIO_WritePin(GPIOB, kTftCs, high ? GPIO_PIN_SET : GPIO_PIN_RESET); }
inline void TftDc(bool high) { HAL_GPIO_WritePin(GPIOB, kTftDc, high ? GPIO_PIN_SET : GPIO_PIN_RESET); }
inline void TouchCs(bool high) { HAL_GPIO_WritePin(GPIOA, kTouchCs, high ? GPIO_PIN_SET : GPIO_PIN_RESET); }
int32_t ClampI32(int32_t value, int32_t low, int32_t high) { return std::max(low, std::min(value, high)); }
uint16_t Median3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) std::swap(a, b);
  if (b > c) std::swap(b, c);
  if (a > b) std::swap(a, b);
  return b;
}
}

bool HmiDisplay::waitSpiIdle(uint32_t timeoutUs) {
  const uint32_t cyclesPerUs = std::max<uint32_t>(1U, HAL_RCC_GetHCLKFreq() / 1000000U);
  const uint32_t budget = timeoutUs * cyclesPerUs;
  const uint32_t start = DWT->CYCCNT;
  while ((SPI1->SR & SPI_SR_BSY) != 0U) {
    if (static_cast<uint32_t>(DWT->CYCCNT - start) >= budget) {
      ++spi_timeout_count_;
      return false;
    }
  }
  return true;
}

void HmiDisplay::drainSpiRx() {
  while ((SPI1->SR & SPI_SR_RXNE) != 0U) {
    (void)*reinterpret_cast<volatile uint8_t *>(&SPI1->DR);
  }
  if ((SPI1->SR & SPI_SR_OVR) != 0U) {
    (void)SPI1->DR;
    (void)SPI1->SR;
  }
}

bool HmiDisplay::recoverSpi() {
  TftCs(true);
  TouchCs(true);
  spi_owner_ = SpiOwner::IDLE;
  ++spi_recovery_count_;
  const bool ok = Board_ReinitSpi1();
  if (!ok) {
    display_faulted_ = true;
    display_ready_ = false;
    return false;
  }
  drainSpiRx();
  return true;
}

bool HmiDisplay::beginTransaction(SpiOwner owner, uint32_t prescaler) {
  if (display_faulted_) return false;
  if (spi_owner_ != SpiOwner::IDLE) {
    ++spi_bus_conflict_count_;
    return false;
  }
  for (uint8_t attempt = 0U; attempt < 2U; ++attempt) {
    TftCs(true);
    TouchCs(true);
    if (!waitSpiIdle()) {
      if (!recoverSpi()) return false;
      continue;
    }
    drainSpiRx();
    CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE);
    MODIFY_REG(SPI1->CR1, SPI_CR1_BR, prescaler);
    SET_BIT(SPI1->CR1, SPI_CR1_SPE);
    spi_owner_ = owner;
    if (owner == SpiOwner::TOUCH) TouchCs(false);
    else TftCs(false);
    ++spi_transactions_;
    return true;
  }
  display_faulted_ = true;
  display_ready_ = false;
  return false;
}

bool HmiDisplay::endTransaction() {
  if (spi_owner_ == SpiOwner::IDLE) return true;
  const bool idle = waitSpiIdle();
  TftCs(true);
  TouchCs(true);
  drainSpiRx();
  spi_owner_ = SpiOwner::IDLE;
  if (!idle) return recoverSpi();
  return true;
}

bool HmiDisplay::tx(const uint8_t *bytes, uint16_t length) {
  if (bytes == nullptr || length == 0U) return true;
  const HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, const_cast<uint8_t *>(bytes), length, kSpiHalTimeoutMs);
  if (status != HAL_OK) {
    ++spi_hal_error_count_;
    return false;
  }
  spi_bytes_tx_ += length;
  return true;
}

bool HmiDisplay::txrx(const uint8_t *txBytes, uint8_t *rxBytes, uint16_t length) {
  if (txBytes == nullptr || rxBytes == nullptr || length == 0U) return true;
  const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
      &hspi1, const_cast<uint8_t *>(txBytes), rxBytes, length, kSpiHalTimeoutMs);
  if (status != HAL_OK) {
    ++spi_hal_error_count_;
    return false;
  }
  spi_bytes_tx_ += length;
  return true;
}

bool HmiDisplay::commandTx(uint8_t value) {
  TftDc(false);
  return tx(&value, 1U);
}

bool HmiDisplay::dataTx(const uint8_t *bytes, uint16_t length) {
  TftDc(true);
  return tx(bytes, length);
}

bool HmiDisplay::writeRegister(uint8_t command, const uint8_t *bytes, uint16_t length) {
  for (uint8_t attempt = 0U; attempt < 2U; ++attempt) {
    if (!beginTransaction(SpiOwner::TFT_WRITE, kTftPrescaler)) continue;
    const bool ok = commandTx(command) && dataTx(bytes, length);
    const bool ended = endTransaction();
    Board_RealtimeService();
    if (ok && ended) return true;
    if (!recoverSpi()) break;
  }
  display_faulted_ = true;
  display_ready_ = false;
  return false;
}

void HmiDisplay::init() {
  spi_owner_ = SpiOwner::IDLE;
  display_ready_ = false;
  display_faulted_ = false;
  TftCs(true); TouchCs(true); TftDc(true);
  (void)writeRegister(0x00U);
  HAL_GPIO_WritePin(GPIOB, kTftRst, GPIO_PIN_SET); HAL_Delay(5U);
  HAL_GPIO_WritePin(GPIOB, kTftRst, GPIO_PIN_RESET); HAL_Delay(20U);
  HAL_GPIO_WritePin(GPIOB, kTftRst, GPIO_PIN_SET); HAL_Delay(150U);

  const uint8_t rEF[]={0x03U,0x80U,0x02U}; (void)writeRegister(0xEFU,rEF,sizeof(rEF));
  const uint8_t rCF[]={0x00U,0xC1U,0x30U}; (void)writeRegister(0xCFU,rCF,sizeof(rCF));
  const uint8_t rED[]={0x64U,0x03U,0x12U,0x81U}; (void)writeRegister(0xEDU,rED,sizeof(rED));
  const uint8_t rE8[]={0x85U,0x00U,0x78U}; (void)writeRegister(0xE8U,rE8,sizeof(rE8));
  const uint8_t rCB[]={0x39U,0x2CU,0x00U,0x34U,0x02U}; (void)writeRegister(0xCBU,rCB,sizeof(rCB));
  const uint8_t rF7[]={0x20U}; (void)writeRegister(0xF7U,rF7,sizeof(rF7));
  const uint8_t rEA[]={0x00U,0x00U}; (void)writeRegister(0xEAU,rEA,sizeof(rEA));
  const uint8_t rC0[]={0x23U}; (void)writeRegister(0xC0U,rC0,sizeof(rC0));
  const uint8_t rC1[]={0x10U}; (void)writeRegister(0xC1U,rC1,sizeof(rC1));
  const uint8_t rC5[]={0x3EU,0x28U}; (void)writeRegister(0xC5U,rC5,sizeof(rC5));
  const uint8_t rC7[]={0x86U}; (void)writeRegister(0xC7U,rC7,sizeof(rC7));
  const uint8_t r36[]={0x48U}; (void)writeRegister(0x36U,r36,sizeof(r36));
  const uint8_t r3A[]={0x55U}; (void)writeRegister(0x3AU,r3A,sizeof(r3A));
  const uint8_t rB1[]={0x00U,0x13U}; (void)writeRegister(0xB1U,rB1,sizeof(rB1));
  const uint8_t rB6[]={0x08U,0x82U,0x27U}; (void)writeRegister(0xB6U,rB6,sizeof(rB6));
  const uint8_t rF2[]={0x00U}; (void)writeRegister(0xF2U,rF2,sizeof(rF2));
  const uint8_t r26[]={0x01U}; (void)writeRegister(0x26U,r26,sizeof(r26));
  const uint8_t gp[15]={0x0FU,0x31U,0x2BU,0x0CU,0x0EU,0x08U,0x4EU,0xF1U,0x37U,0x07U,0x10U,0x03U,0x0EU,0x09U,0x00U};
  const uint8_t gn[15]={0x00U,0x0EU,0x14U,0x03U,0x11U,0x07U,0x31U,0xC1U,0x48U,0x08U,0x0FU,0x0CU,0x31U,0x36U,0x0FU};
  (void)writeRegister(0xE0U,gp,sizeof(gp));
  (void)writeRegister(0xE1U,gn,sizeof(gn));
  (void)writeRegister(0x11U); HAL_Delay(120U);
  (void)writeRegister(0x29U); HAL_Delay(20U);
  setRotation(1U);
}

uint8_t HmiDisplay::readRegister8(uint8_t commandValue, uint8_t index) {
  for (uint8_t attempt = 0U; attempt < 2U; ++attempt) {
    if (!beginTransaction(SpiOwner::TFT_READ, kTftPrescaler)) continue;
    uint8_t select = static_cast<uint8_t>(0x10U + (index & 0x0FU));
    bool ok = commandTx(0xD9U) && dataTx(&select, 1U);
    TftCs(true); Board_DelayUs(2U); TftCs(false);
    ok = ok && commandTx(commandValue);
    uint8_t dummy = 0U, rx = 0U;
    TftDc(true);
    ok = ok && txrx(&dummy, &rx, 1U);
    const bool ended = endTransaction();
    if (ok && ended) return rx;
    if (!recoverSpi()) break;
  }
  return 0U;
}

uint32_t HmiDisplay::readId() {
  return (static_cast<uint32_t>(readRegister8(0xD3U, 0U)) << 24U) |
         (static_cast<uint32_t>(readRegister8(0xD3U, 1U)) << 16U) |
         (static_cast<uint32_t>(readRegister8(0xD3U, 2U)) << 8U) |
         static_cast<uint32_t>(readRegister8(0xD3U, 3U));
}

void HmiDisplay::setRotation(uint8_t rotation) {
  rotation_=static_cast<uint8_t>(rotation&3U); uint8_t madctl=0x48U;
  switch(rotation_){
    case 0U:width_=240;height_=320;madctl=0x48U;break;
    case 1U:width_=320;height_=240;madctl=0x28U;break;
    case 2U:width_=240;height_=320;madctl=0x88U;break;
    default:width_=320;height_=240;madctl=0xE8U;break;
  }
  (void)writeRegister(0x36U, &madctl, 1U);
}

bool HmiDisplay::setWindowTx(int32_t x,int32_t y,int32_t w,int32_t h) {
  const uint16_t x0=static_cast<uint16_t>(x),x1=static_cast<uint16_t>(x+w-1);
  const uint16_t y0=static_cast<uint16_t>(y),y1=static_cast<uint16_t>(y+h-1);
  uint8_t b[4];
  b[0]=static_cast<uint8_t>(x0>>8U);b[1]=static_cast<uint8_t>(x0);b[2]=static_cast<uint8_t>(x1>>8U);b[3]=static_cast<uint8_t>(x1);
  if (!commandTx(0x2AU) || !dataTx(b,4U)) return false;
  b[0]=static_cast<uint8_t>(y0>>8U);b[1]=static_cast<uint8_t>(y0);b[2]=static_cast<uint8_t>(y1>>8U);b[3]=static_cast<uint8_t>(y1);
  return commandTx(0x2BU) && dataTx(b,4U) && commandTx(0x2CU);
}

bool HmiDisplay::writeColorTx(uint16_t color,uint32_t count) {
  uint8_t block[256];
  for(std::size_t i=0;i<sizeof(block);i+=2U){block[i]=static_cast<uint8_t>(color>>8U);block[i+1U]=static_cast<uint8_t>(color);}
  TftDc(true);
  while(count>0U){
    const uint32_t n=std::min<uint32_t>(count,sizeof(block)/2U);
    if(!tx(block,static_cast<uint16_t>(n*2U))) return false;
    count-=n;
    Board_RealtimeService();
  }
  return true;
}

bool HmiDisplay::pushImageTx(const uint16_t *pixels,uint32_t count) {
  uint8_t block[256];
  uint32_t pos=0U;
  TftDc(true);
  while(pos<count){
    const uint32_t n=std::min<uint32_t>(count-pos,sizeof(block)/2U);
    for(uint32_t i=0U;i<n;++i){
      const uint16_t c=pixels[pos+i];
      if(swap_bytes_){block[2U*i]=static_cast<uint8_t>(c>>8U);block[2U*i+1U]=static_cast<uint8_t>(c);}
      else{block[2U*i]=static_cast<uint8_t>(c);block[2U*i+1U]=static_cast<uint8_t>(c>>8U);}
    }
    if(!tx(block,static_cast<uint16_t>(n*2U))) return false;
    pos+=n;
    Board_RealtimeService();
  }
  return true;
}

void HmiDisplay::fillRect(int32_t x,int32_t y,int32_t w,int32_t h,uint16_t color){
  if(w<=0||h<=0||x>=width_||y>=height_||x+w<=0||y+h<=0)return;
  const int32_t x0=std::max<int32_t>(0,x),y0=std::max<int32_t>(0,y);
  const int32_t x1=std::min<int32_t>(width_,x+w),y1=std::min<int32_t>(height_,y+h);
  const int32_t cw=x1-x0,ch=y1-y0;if(cw<=0||ch<=0)return;
  for(uint8_t attempt=0U;attempt<2U;++attempt){
    if(!beginTransaction(SpiOwner::TFT_WRITE,kTftPrescaler))continue;
    const bool ok=setWindowTx(x0,y0,cw,ch)&&writeColorTx(color,static_cast<uint32_t>(cw*ch));
    const bool ended=endTransaction();
    if(ok&&ended)return;
    if(!recoverSpi())break;
  }
  display_faulted_=true;display_ready_=false;
}

void HmiDisplay::fillScreen(uint16_t color){fillRect(0,0,width_,height_,color);}
void HmiDisplay::drawFastHLine(int32_t x,int32_t y,int32_t w,uint16_t color){fillRect(x,y,w,1,color);}
void HmiDisplay::drawFastVLine(int32_t x,int32_t y,int32_t h,uint16_t color){fillRect(x,y,1,h,color);}
void HmiDisplay::drawLine(int32_t x0,int32_t y0,int32_t x1,int32_t y1,uint16_t color){
  const int32_t dx=std::abs(x1-x0),sx=x0<x1?1:-1,dy=-std::abs(y1-y0),sy=y0<y1?1:-1;int32_t e=dx+dy;
  for(;;){fillRect(x0,y0,1,1,color);if(x0==x1&&y0==y1)break;const int32_t e2=2*e;if(e2>=dy){e+=dy;x0+=sx;}if(e2<=dx){e+=dx;y0+=sy;}}
}
void HmiDisplay::drawCircle(int32_t x0,int32_t y0,int32_t r,uint16_t color){
  if(r<0) return;
  int32_t f=1-r,ddx=1,ddy=-2*r,x=0,y=r;
  fillRect(x0,y0+r,1,1,color);fillRect(x0,y0-r,1,1,color);fillRect(x0+r,y0,1,1,color);fillRect(x0-r,y0,1,1,color);
  while(x<y){if(f>=0){--y;ddy+=2;f+=ddy;}++x;ddx+=2;f+=ddx;fillRect(x0+x,y0+y,1,1,color);fillRect(x0-x,y0+y,1,1,color);fillRect(x0+x,y0-y,1,1,color);fillRect(x0-x,y0-y,1,1,color);fillRect(x0+y,y0+x,1,1,color);fillRect(x0-y,y0+x,1,1,color);fillRect(x0+y,y0-x,1,1,color);fillRect(x0-y,y0-x,1,1,color);}
}
void HmiDisplay::fillCircle(int32_t x0,int32_t y0,int32_t r,uint16_t color){if(r<0)return;for(int32_t y=-r;y<=r;++y){const int32_t q=r*r-y*y;const int32_t span=static_cast<int32_t>(std::sqrt(static_cast<double>(q)));drawFastHLine(x0-span,y0+y,2*span+1,color);}}
void HmiDisplay::drawRoundRect(int32_t x,int32_t y,int32_t w,int32_t h,int32_t r,uint16_t color){
  if(w<=0||h<=0) return;
  r=std::max<int32_t>(0,std::min<int32_t>(r,std::min(w,h)/2));drawFastHLine(x+r,y,w-2*r,color);drawFastHLine(x+r,y+h-1,w-2*r,color);drawFastVLine(x,y+r,h-2*r,color);drawFastVLine(x+w-1,y+r,h-2*r,color);
  for(int32_t yy=0;yy<=r;++yy){const int32_t q=r*r-yy*yy;const int32_t xx=static_cast<int32_t>(std::sqrt(static_cast<double>(q)));fillRect(x+r-xx,y+r-yy,1,1,color);fillRect(x+w-1-r+xx,y+r-yy,1,1,color);fillRect(x+r-xx,y+h-1-r+yy,1,1,color);fillRect(x+w-1-r+xx,y+h-1-r+yy,1,1,color);}
}
void HmiDisplay::fillRoundRect(int32_t x,int32_t y,int32_t w,int32_t h,int32_t r,uint16_t color){
  if(w<=0||h<=0) return;
  r=std::max<int32_t>(0,std::min<int32_t>(r,std::min(w,h)/2));fillRect(x+r,y,w-2*r,h,color);
  for(int32_t yy=0;yy<r;++yy){const int32_t dy=r-yy;const int32_t q=r*r-dy*dy;const int32_t dx=static_cast<int32_t>(std::sqrt(static_cast<double>(q)));drawFastHLine(x+r-dx,y+yy,w-2*r+2*dx,color);drawFastHLine(x+r-dx,y+h-1-yy,w-2*r+2*dx,color);}
}
void HmiDisplay::fillTriangle(int32_t x0,int32_t y0,int32_t x1,int32_t y1,int32_t x2,int32_t y2,uint16_t color){
  if(y0>y1){std::swap(y0,y1);std::swap(x0,x1);}if(y1>y2){std::swap(y1,y2);std::swap(x1,x2);}if(y0>y1){std::swap(y0,y1);std::swap(x0,x1);}if(y0==y2){const int32_t lo=std::min(x0,std::min(x1,x2)),hi=std::max(x0,std::max(x1,x2));drawFastHLine(lo,y0,hi-lo+1,color);return;}
  const int32_t dx01=x1-x0,dy01=y1-y0,dx02=x2-x0,dy02=y2-y0,dx12=x2-x1,dy12=y2-y1;int64_t sa=0,sb=0;int32_t y=y0;const int32_t last=y1==y2?y1:y1-1;
  for(;y<=last;++y){const int32_t a=x0+(dy01==0?0:static_cast<int32_t>(sa/dy01)),b=x0+static_cast<int32_t>(sb/dy02);sa+=dx01;sb+=dx02;drawFastHLine(std::min(a,b),y,std::abs(a-b)+1,color);}sa=static_cast<int64_t>(dx12)*(y-y1);sb=static_cast<int64_t>(dx02)*(y-y0);
  for(;y<=y2;++y){const int32_t a=x1+(dy12==0?0:static_cast<int32_t>(sa/dy12)),b=x0+static_cast<int32_t>(sb/dy02);sa+=dx12;sb+=dx02;drawFastHLine(std::min(a,b),y,std::abs(a-b)+1,color);}
}

void HmiDisplay::pushImage(int32_t x,int32_t y,int32_t w,int32_t h,const uint16_t *pixels){
  if(pixels==nullptr||w<=0||h<=0||x<0||y<0||x+w>width_||y+h>height_)return;
  for(uint8_t attempt=0U;attempt<2U;++attempt){
    if(!beginTransaction(SpiOwner::TFT_WRITE,kTftPrescaler))continue;
    const bool ok=setWindowTx(x,y,w,h)&&pushImageTx(pixels,static_cast<uint32_t>(w*h));
    const bool ended=endTransaction();
    if(ok&&ended)return;
    if(!recoverSpi())break;
  }
  display_faulted_=true;display_ready_=false;
}

void HmiDisplay::textBounds(const char *text,int32_t &min_x,int32_t &min_y,int32_t &max_x,int32_t &max_y,int32_t &advance) const{
  min_x=min_y=max_x=max_y=advance=0;if(text==nullptr||*text=='\0')return;
  if(font_==nullptr){
    const int32_t scale=static_cast<int32_t>(text_size_);
    if(builtin_font_==2U){for(const char*q=text;*q!='\0';++q){const uint8_t c=static_cast<uint8_t>(*q);advance+=(c>=32U&&c<=127U?widtbl_f16[c-32U]:widtbl_f16[0])*scale;}max_x=advance>0?advance-1:0;max_y=16*scale-1;return;}
    advance=static_cast<int32_t>(std::strlen(text))*6*scale;max_x=advance-1;max_y=8*scale-1;return;
  }
  bool first=true;int32_t cursor=0;for(const char*q=text;*q!='\0';++q){const uint8_t c=static_cast<uint8_t>(*q);if(c<font_->first||c>font_->last)continue;const GFXglyph&g=font_->glyph[c-font_->first];const int32_t gx0=cursor+g.xOffset,gy0=g.yOffset,gx1=gx0+g.width-1,gy1=gy0+g.height-1;if(first){min_x=gx0;min_y=gy0;max_x=gx1;max_y=gy1;first=false;}else{min_x=std::min(min_x,gx0);min_y=std::min(min_y,gy0);max_x=std::max(max_x,gx1);max_y=std::max(max_y,gy1);}cursor+=g.xAdvance;}advance=cursor;if(first)min_x=min_y=max_x=max_y=0;
}

int16_t HmiDisplay::textWidth(const char *text) const{int32_t a,b,c,d,e;textBounds(text,a,b,c,d,e);(void)b;(void)d;const int32_t w=font_==nullptr?e:std::max(e,c-std::min<int32_t>(0,a)+1);return static_cast<int16_t>(std::min<int32_t>(32767,std::max<int32_t>(0,w)));}

int16_t HmiDisplay::drawString(const char *text,int32_t x,int32_t y){
  if(text==nullptr)return 0;
  int32_t minx,miny,maxx,maxy,advance;textBounds(text,minx,miny,maxx,maxy,advance);
  const int32_t cw=std::max<int32_t>(0,maxx-minx+1),ch=std::max<int32_t>(1,maxy-miny+1),lw=std::max<int32_t>(advance,cw);
  int32_t left=x,top=y;
  if(datum_==TC_DATUM||datum_==MC_DATUM||datum_==BC_DATUM)left-=lw/2;else if(datum_==TR_DATUM||datum_==MR_DATUM||datum_==BR_DATUM)left-=lw;
  if(datum_==ML_DATUM||datum_==MC_DATUM||datum_==MR_DATUM)top-=ch/2;else if(datum_==BL_DATUM||datum_==BC_DATUM||datum_==BR_DATUM)top-=ch;
  const int32_t clearw=std::max<int32_t>(lw,padding_);int32_t clearleft=left;
  if(padding_>lw){if(datum_==TC_DATUM||datum_==MC_DATUM||datum_==BC_DATUM)clearleft=x-clearw/2;else if(datum_==TR_DATUM||datum_==MR_DATUM||datum_==BR_DATUM)clearleft=x-clearw;}
  const int32_t ax0=std::max<int32_t>(0,clearleft),ay0=std::max<int32_t>(0,top);
  const int32_t ax1=std::min<int32_t>(width_,clearleft+clearw),ay1=std::min<int32_t>(height_,top+ch);
  if(ax1<=ax0||ay1<=ay0)return static_cast<int16_t>(lw);
  const int32_t aw=ax1-ax0;

  for(int32_t stripY=ay0;stripY<ay1;stripY+=kTextTileRows){
    const int32_t sh=std::min<int32_t>(kTextTileRows,ay1-stripY);
    std::fill_n(text_tile_,static_cast<std::size_t>(aw*sh),text_bg_);
    auto putPixel=[&](int32_t px,int32_t py){if(px>=ax0&&px<ax1&&py>=stripY&&py<stripY+sh)text_tile_[(py-stripY)*aw+(px-ax0)]=text_fg_;};
    auto putBlock=[&](int32_t px,int32_t py,uint8_t scale){for(uint8_t yy=0U;yy<scale;++yy)for(uint8_t xx=0U;xx<scale;++xx)putPixel(px+xx,py+yy);};

    if(font_==nullptr){
      const uint8_t scale=text_size_;int32_t cur=left;
      for(const char*q=text;*q!='\0';++q){const uint8_t c=static_cast<uint8_t>(*q);
        if(builtin_font_==2U){
          const uint8_t idx=(c>=32U&&c<=127U)?static_cast<uint8_t>(c-32U):0U;const uint8_t gw=widtbl_f16[idx];const uint8_t bpr=static_cast<uint8_t>((gw+6U)/8U);const unsigned char*glyph=chrtbl_f16[idx];
          for(uint8_t row=0U;row<16U;++row)for(uint8_t bi=0U;bi<bpr;++bi){const uint8_t bits=glyph[static_cast<uint16_t>(row)*bpr+bi];for(uint8_t bit=0U;bit<8U;++bit){const uint8_t px=static_cast<uint8_t>(bi*8U+bit);if(px>=gw)break;if((bits&static_cast<uint8_t>(0x80U>>bit))!=0U)putBlock(cur+px*scale,top+row*scale,scale);}}
          cur+=static_cast<int32_t>(gw)*scale;
        }else{
          for(uint8_t col=0U;col<5U;++col){uint8_t bits=kClassicFont[static_cast<uint16_t>(c)*5U+col];for(uint8_t row=0U;row<8U;++row){if((bits&1U)!=0U)putBlock(cur+col*scale,top+row*scale,scale);bits>>=1U;}}
          cur+=6*scale;
        }
      }
    }else{
      int32_t cur=left-minx;const int32_t baseline=top-miny;
      for(const char*q=text;*q!='\0';++q){const uint8_t c=static_cast<uint8_t>(*q);if(c<font_->first||c>font_->last)continue;const GFXglyph&g=font_->glyph[c-font_->first];
        for(uint8_t yy=0U;yy<g.height;++yy)for(uint8_t xx=0U;xx<g.width;++xx){const uint32_t bitIndex=static_cast<uint32_t>(yy)*g.width+xx;const uint8_t bits=font_->bitmap[g.bitmapOffset+(bitIndex>>3U)];if((bits&static_cast<uint8_t>(0x80U>>(bitIndex&7U)))!=0U)putPixel(cur+g.xOffset+xx,baseline+g.yOffset+yy);}
        cur+=g.xAdvance;
      }
    }
    pushImage(ax0,stripY,aw,sh,text_tile_);
  }
  return static_cast<int16_t>(lw);
}

void HmiDisplay::setTouch(const uint16_t *p){if(p==nullptr)return;touch_x0_=p[0]?p[0]:1U;touch_x1_=p[1]?p[1]:1U;touch_y0_=p[2]?p[2]:1U;touch_y1_=p[3]?p[3]:1U;touch_rotate_=(p[4]&1U)!=0U;touch_invert_x_=(p[4]&2U)!=0U;touch_invert_y_=(p[4]&4U)!=0U;}

uint8_t HmiDisplay::transferTx(uint8_t value){uint8_t rx=0U;(void)txrx(&value,&rx,1U);return rx;}
uint16_t HmiDisplay::transfer16Tx(uint16_t value){uint8_t t[2]={static_cast<uint8_t>(value>>8U),static_cast<uint8_t>(value)},r[2]{};(void)txrx(t,r,2U);return static_cast<uint16_t>((static_cast<uint16_t>(r[0])<<8U)|r[1]);}
uint16_t HmiDisplay::readTouchZTx(){int16_t z=0x0FFF;(void)transferTx(0xB0U);z=static_cast<int16_t>(z+static_cast<int16_t>(transfer16Tx(0x00C0U)>>3U));z=static_cast<int16_t>(z-static_cast<int16_t>(transfer16Tx(0x0000U)>>3U));return z==4095?0U:static_cast<uint16_t>(std::max<int16_t>(0,z));}
void HmiDisplay::readTouchRawTx(uint16_t*x,uint16_t*y){uint16_t t;(void)transferTx(0xD0U);(void)transferTx(0U);(void)transferTx(0xD0U);(void)transferTx(0U);(void)transferTx(0xD0U);(void)transferTx(0U);(void)transferTx(0xD0U);t=static_cast<uint16_t>(transferTx(0U))<<5U;t|=static_cast<uint16_t>((transferTx(0x90U)>>3U)&0x1FU);*x=t;(void)transferTx(0U);(void)transferTx(0x90U);(void)transferTx(0U);(void)transferTx(0x90U);(void)transferTx(0U);(void)transferTx(0x90U);(void)transferTx(0U);t=static_cast<uint16_t>(transferTx(0U))<<5U;t|=static_cast<uint16_t>((transferTx(0U)>>3U)&0x1FU);*y=t;}

bool HmiDisplay::getTouch(uint16_t*x,uint16_t*y,uint16_t threshold){
  if(x==nullptr||y==nullptr||spi_owner_!=SpiOwner::IDLE){if(spi_owner_!=SpiOwner::IDLE)++spi_bus_conflict_count_;return false;}
  ++touch_read_count_;threshold=std::max<uint16_t>(20U,threshold);if(static_cast<int32_t>(press_time_ms_-HAL_GetTick())>0)threshold=20U;
  if(!beginTransaction(SpiOwner::TOUCH,kTouchPrescaler))return false;
  const uint16_t z=readTouchZTx();
  if(z<=threshold){(void)endTransaction();++touch_reject_fast_count_;press_time_ms_=0U;return false;}
  uint16_t xs[3]{},ys[3]{};
  for(uint8_t i=0U;i<3U;++i){readTouchRawTx(&xs[i],&ys[i]);if(i<2U)Board_DelayUs(150U);}
  const uint16_t z2=readTouchZTx();
  const bool ended=endTransaction();
  if(!ended||z2<=threshold){press_time_ms_=0U;return false;}
  const uint16_t xmin=std::min(xs[0],std::min(xs[1],xs[2])),xmax=std::max(xs[0],std::max(xs[1],xs[2]));
  const uint16_t ymin=std::min(ys[0],std::min(ys[1],ys[2])),ymax=std::max(ys[0],std::max(ys[1],ys[2]));
  if(static_cast<uint16_t>(xmax-xmin)>80U||static_cast<uint16_t>(ymax-ymin)>80U)return false;
  const uint16_t rx=Median3(xs[0],xs[1],xs[2]),ry=Median3(ys[0],ys[1],ys[2]);
  press_time_ms_=HAL_GetTick()+50U;int32_t sx=0,sy=0;
  if(touch_rotate_){sx=(static_cast<int32_t>(ry)-touch_x0_)*width_/touch_x1_;sy=(static_cast<int32_t>(rx)-touch_y0_)*height_/touch_y1_;}
  else{sx=(static_cast<int32_t>(rx)-touch_x0_)*width_/touch_x1_;sy=(static_cast<int32_t>(ry)-touch_y0_)*height_/touch_y1_;}
  if(touch_invert_x_) sx=width_-sx;
  if(touch_invert_y_) sy=height_-sy;
  if(sx<0||sy<0||sx>=width_||sy>=height_) return false;
  *x=static_cast<uint16_t>(ClampI32(sx,0,width_-1));*y=static_cast<uint16_t>(ClampI32(sy,0,height_-1));return true;
}

void HmiDisplay::beginFrame(){if(frame_active_)return;frame_active_=true;frame_start_bytes_=spi_bytes_tx_;}
void HmiDisplay::endFrame(){if(!frame_active_)return;last_frame_bytes_=static_cast<uint32_t>(spi_bytes_tx_-frame_start_bytes_);if(last_frame_bytes_>max_frame_bytes_)max_frame_bytes_=last_frame_bytes_;frame_active_=false;}
