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
}

void HmiDisplay::setSpiPrescaler(uint32_t prescaler) {
  while ((SPI1->SR & SPI_SR_BSY) != 0U) { }
  CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE);
  MODIFY_REG(SPI1->CR1, SPI_CR1_BR, prescaler);
  SET_BIT(SPI1->CR1, SPI_CR1_SPE);
}

void HmiDisplay::command(uint8_t value) {
  setSpiPrescaler(SPI_BAUDRATEPRESCALER_16);
  TouchCs(true); TftCs(false); TftDc(false);
  (void)HAL_SPI_Transmit(&hspi1, &value, 1U, 20U);
  TftCs(true);
}

void HmiDisplay::data8(uint8_t value) {
  setSpiPrescaler(SPI_BAUDRATEPRESCALER_16);
  TouchCs(true); TftCs(false); TftDc(true);
  (void)HAL_SPI_Transmit(&hspi1, &value, 1U, 20U);
  TftCs(true);
}

void HmiDisplay::data(const uint8_t *bytes, uint16_t length) {
  if (bytes == nullptr || length == 0U) return;
  setSpiPrescaler(SPI_BAUDRATEPRESCALER_16);
  TouchCs(true); TftCs(false); TftDc(true);
  (void)HAL_SPI_Transmit(&hspi1, const_cast<uint8_t *>(bytes), length, 100U);
  TftCs(true);
}

void HmiDisplay::init() {
  TftCs(true); TouchCs(true); TftDc(true);
  // Mirror the known-good TFT_eSPI reset sequence exactly: put the SPI bus in
  // a known command state, then perform the ILI9341 hardware reset and allow
  // the full 150 ms recovery time before programming controller registers.
  command(0x00U);
  HAL_GPIO_WritePin(GPIOB, kTftRst, GPIO_PIN_SET); HAL_Delay(5U);
  HAL_GPIO_WritePin(GPIOB, kTftRst, GPIO_PIN_RESET); HAL_Delay(20U);
  HAL_GPIO_WritePin(GPIOB, kTftRst, GPIO_PIN_SET); HAL_Delay(150U);
  command(0xEFU); data8(0x03U); data8(0x80U); data8(0x02U);
  command(0xCFU); data8(0x00U); data8(0xC1U); data8(0x30U);
  command(0xEDU); data8(0x64U); data8(0x03U); data8(0x12U); data8(0x81U);
  command(0xE8U); data8(0x85U); data8(0x00U); data8(0x78U);
  command(0xCBU); data8(0x39U); data8(0x2CU); data8(0x00U); data8(0x34U); data8(0x02U);
  command(0xF7U); data8(0x20U); command(0xEAU); data8(0x00U); data8(0x00U);
  command(0xC0U); data8(0x23U); command(0xC1U); data8(0x10U);
  command(0xC5U); data8(0x3EU); data8(0x28U); command(0xC7U); data8(0x86U);
  command(0x36U); data8(0x48U); command(0x3AU); data8(0x55U);
  command(0xB1U); data8(0x00U); data8(0x13U);
  command(0xB6U); data8(0x08U); data8(0x82U); data8(0x27U);
  command(0xF2U); data8(0x00U); command(0x26U); data8(0x01U);
  const uint8_t gp[15]={0x0FU,0x31U,0x2BU,0x0CU,0x0EU,0x08U,0x4EU,0xF1U,0x37U,0x07U,0x10U,0x03U,0x0EU,0x09U,0x00U};
  const uint8_t gn[15]={0x00U,0x0EU,0x14U,0x03U,0x11U,0x07U,0x31U,0xC1U,0x48U,0x08U,0x0FU,0x0CU,0x31U,0x36U,0x0FU};
  command(0xE0U); data(gp,sizeof(gp)); command(0xE1U); data(gn,sizeof(gn));
  command(0x11U); HAL_Delay(120U); command(0x29U); HAL_Delay(20U); setRotation(1U);
}

uint8_t HmiDisplay::readRegister8(uint8_t commandValue, uint8_t index) {
  // Match TFT_eSPI::readcommand8() for ILI9341 SPI Interface II. Register
  // parameters are selected through 0xD9 before each indexed read.
  setSpiPrescaler(SPI_BAUDRATEPRESCALER_16); // 6 MHz, same as legacy read clock
  TouchCs(true); TftCs(false);
  TftDc(false);
  uint8_t tx = 0xD9U;
  if (HAL_SPI_Transmit(&hspi1, &tx, 1U, 20U) != HAL_OK) { TftCs(true); return 0U; }
  TftDc(true);
  tx = static_cast<uint8_t>(0x10U + (index & 0x0FU));
  if (HAL_SPI_Transmit(&hspi1, &tx, 1U, 20U) != HAL_OK) { TftCs(true); return 0U; }
  TftCs(true); Board_DelayUs(2U); TftCs(false);
  TftDc(false); tx = commandValue;
  if (HAL_SPI_Transmit(&hspi1, &tx, 1U, 20U) != HAL_OK) { TftCs(true); return 0U; }
  TftDc(true);
  uint8_t rx = 0U; tx = 0U;
  (void)HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1U, 20U);
  TftCs(true);
  return rx;
}

uint32_t HmiDisplay::readId() {
  return (static_cast<uint32_t>(readRegister8(0xD3U, 0U)) << 24U) |
         (static_cast<uint32_t>(readRegister8(0xD3U, 1U)) << 16U) |
         (static_cast<uint32_t>(readRegister8(0xD3U, 2U)) << 8U) |
         static_cast<uint32_t>(readRegister8(0xD3U, 3U));
}

void HmiDisplay::setRotation(uint8_t rotation) {
  rotation_=static_cast<uint8_t>(rotation&3U); uint8_t madctl=0x48U;
  switch(rotation_){case 0U:width_=240;height_=320;madctl=0x48U;break;case 1U:width_=320;height_=240;madctl=0x28U;break;case 2U:width_=240;height_=320;madctl=0x88U;break;default:width_=320;height_=240;madctl=0xE8U;break;}
  command(0x36U); data8(madctl);
}

void HmiDisplay::setWindow(int32_t x,int32_t y,int32_t w,int32_t h){
  const uint16_t x0=static_cast<uint16_t>(x),x1=static_cast<uint16_t>(x+w-1),y0=static_cast<uint16_t>(y),y1=static_cast<uint16_t>(y+h-1); uint8_t b[4];
  command(0x2AU);b[0]=x0>>8U;b[1]=x0;b[2]=x1>>8U;b[3]=x1;data(b,4U);
  command(0x2BU);b[0]=y0>>8U;b[1]=y0;b[2]=y1>>8U;b[3]=y1;data(b,4U);command(0x2CU);
}

void HmiDisplay::writeColor(uint16_t color,uint32_t count){
  if (count == 0U) return;
  uint8_t block[256];
  for(std::size_t i=0;i<sizeof(block);i+=2U){block[i]=static_cast<uint8_t>(color>>8U);block[i+1U]=static_cast<uint8_t>(color);}
  setSpiPrescaler(SPI_BAUDRATEPRESCALER_16);TouchCs(true);TftCs(false);TftDc(true);
  while(count>0U){const uint32_t n=std::min<uint32_t>(count,sizeof(block)/2U);if(HAL_SPI_Transmit(&hspi1,block,static_cast<uint16_t>(n*2U),100U)!=HAL_OK)break;count-=n;}TftCs(true);
}
void HmiDisplay::drawPixel(int32_t x,int32_t y,uint16_t color){if(x<0||y<0||x>=width_||y>=height_)return;setWindow(x,y,1,1);writeColor(color,1U);}
void HmiDisplay::fillScreen(uint16_t color){fillRect(0,0,width_,height_,color);}
void HmiDisplay::fillRect(int32_t x,int32_t y,int32_t w,int32_t h,uint16_t color){
  if(w<=0||h<=0||x>=width_||y>=height_||x+w<=0||y+h<=0)return;
  const int32_t x0=std::max<int32_t>(0,x),y0=std::max<int32_t>(0,y),x1=std::min<int32_t>(width_,x+w),y1=std::min<int32_t>(height_,y+h);const int32_t cw=x1-x0,ch=y1-y0;if(cw<=0||ch<=0)return;setWindow(x0,y0,cw,ch);writeColor(color,static_cast<uint32_t>(cw*ch));
}
void HmiDisplay::drawFastHLine(int32_t x,int32_t y,int32_t w,uint16_t color){fillRect(x,y,w,1,color);}
void HmiDisplay::drawFastVLine(int32_t x,int32_t y,int32_t h,uint16_t color){fillRect(x,y,1,h,color);}
void HmiDisplay::drawLine(int32_t x0,int32_t y0,int32_t x1,int32_t y1,uint16_t color){
  const int32_t dx=std::abs(x1-x0),sx=x0<x1?1:-1,dy=-std::abs(y1-y0),sy=y0<y1?1:-1;int32_t e=dx+dy;
  for(;;){drawPixel(x0,y0,color);if(x0==x1&&y0==y1)break;const int32_t e2=2*e;if(e2>=dy){e+=dy;x0+=sx;}if(e2<=dx){e+=dx;y0+=sy;}}
}
void HmiDisplay::drawCircle(int32_t x0,int32_t y0,int32_t r,uint16_t color){
  if (r < 0) return;
  int32_t f=1-r,ddx=1,ddy=-2*r,x=0,y=r;drawPixel(x0,y0+r,color);drawPixel(x0,y0-r,color);drawPixel(x0+r,y0,color);drawPixel(x0-r,y0,color);
  while(x<y){if(f>=0){--y;ddy+=2;f+=ddy;}++x;ddx+=2;f+=ddx;drawPixel(x0+x,y0+y,color);drawPixel(x0-x,y0+y,color);drawPixel(x0+x,y0-y,color);drawPixel(x0-x,y0-y,color);drawPixel(x0+y,y0+x,color);drawPixel(x0-y,y0+x,color);drawPixel(x0+y,y0-x,color);drawPixel(x0-y,y0-x,color);}
}
void HmiDisplay::fillCircle(int32_t x0,int32_t y0,int32_t r,uint16_t color){if(r<0)return;for(int32_t y=-r;y<=r;++y){const int32_t q=r*r-y*y;const int32_t span=static_cast<int32_t>(std::sqrt(static_cast<double>(q)));drawFastHLine(x0-span,y0+y,2*span+1,color);}}

void HmiDisplay::drawRoundRect(int32_t x,int32_t y,int32_t w,int32_t h,int32_t r,uint16_t color){
  if (w <= 0 || h <= 0) return;
  r=std::max<int32_t>(0,std::min<int32_t>(r,std::min(w,h)/2));drawFastHLine(x+r,y,w-2*r,color);drawFastHLine(x+r,y+h-1,w-2*r,color);drawFastVLine(x,y+r,h-2*r,color);drawFastVLine(x+w-1,y+r,h-2*r,color);
  for(int32_t yy=0;yy<=r;++yy){const int32_t q=r*r-yy*yy;const int32_t xx=static_cast<int32_t>(std::sqrt(static_cast<double>(q)));drawPixel(x+r-xx,y+r-yy,color);drawPixel(x+w-1-r+xx,y+r-yy,color);drawPixel(x+r-xx,y+h-1-r+yy,color);drawPixel(x+w-1-r+xx,y+h-1-r+yy,color);}
}
void HmiDisplay::fillRoundRect(int32_t x,int32_t y,int32_t w,int32_t h,int32_t r,uint16_t color){
  if (w <= 0 || h <= 0) return;
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
  if (pixels == nullptr || w <= 0 || h <= 0 || x < 0 || y < 0 || x + w > width_ || y + h > height_) return;
  setWindow(x,y,w,h);uint8_t block[256];const uint32_t total=static_cast<uint32_t>(w*h);uint32_t pos=0U;setSpiPrescaler(SPI_BAUDRATEPRESCALER_16);TouchCs(true);TftCs(false);TftDc(true);
  while(pos<total){const uint32_t n=std::min<uint32_t>(total-pos,sizeof(block)/2U);for(uint32_t i=0U;i<n;++i){const uint16_t c=pixels[pos+i];if(swap_bytes_){block[2U*i]=static_cast<uint8_t>(c>>8U);block[2U*i+1U]=static_cast<uint8_t>(c);}else{block[2U*i]=static_cast<uint8_t>(c);block[2U*i+1U]=static_cast<uint8_t>(c>>8U);}}if(HAL_SPI_Transmit(&hspi1,block,static_cast<uint16_t>(n*2U),100U)!=HAL_OK)break;pos+=n;}TftCs(true);
}

void HmiDisplay::textBounds(const char *text,int32_t &min_x,int32_t &min_y,int32_t &max_x,int32_t &max_y,int32_t &advance) const{
  min_x=min_y=max_x=max_y=advance=0;if(text==nullptr||*text=='\0')return;
  if(font_==nullptr){
    const int32_t scale=static_cast<int32_t>(text_size_);
    if (builtin_font_ == 2U) {
      for (const char *q=text; *q!='\0'; ++q) {
        const uint8_t c=static_cast<uint8_t>(*q);
        advance += (c >= 32U && c <= 127U ? widtbl_f16[c-32U] : widtbl_f16[0]) * scale;
      }
      max_x=advance>0?advance-1:0; max_y=16*scale-1; return;
    }
    advance=static_cast<int32_t>(std::strlen(text))*6*scale;
    max_x=advance-1; max_y=8*scale-1; return;
  }
  bool first=true;int32_t cursor=0;for(const char *q=text;*q!='\0';++q){const uint8_t c=static_cast<uint8_t>(*q);if(c<font_->first||c>font_->last)continue;const GFXglyph &g=font_->glyph[c-font_->first];const int32_t gx0=cursor+g.xOffset,gy0=g.yOffset,gx1=gx0+g.width-1,gy1=gy0+g.height-1;if(first){min_x=gx0;min_y=gy0;max_x=gx1;max_y=gy1;first=false;}else{min_x=std::min(min_x,gx0);min_y=std::min(min_y,gy0);max_x=std::max(max_x,gx1);max_y=std::max(max_y,gy1);}cursor+=g.xAdvance;}advance=cursor;if(first)min_x=min_y=max_x=max_y=0;
}
int16_t HmiDisplay::textWidth(const char *text) const{int32_t a,b,c,d,e;textBounds(text,a,b,c,d,e);(void)b;(void)d;const int32_t w=font_==nullptr?e:std::max(e,c-std::min<int32_t>(0,a)+1);return static_cast<int16_t>(std::min<int32_t>(32767,std::max<int32_t>(0,w)));}
void HmiDisplay::drawBuiltinChar(char ch,int32_t x,int32_t y,uint8_t scale){const uint8_t c=static_cast<uint8_t>(ch);for(uint8_t col=0U;col<5U;++col){uint8_t bits=kClassicFont[static_cast<uint16_t>(c)*5U+col];for(uint8_t row=0U;row<8U;++row){if((bits&1U)!=0U)fillRect(x+col*scale,y+row*scale,scale,scale,text_fg_);bits>>=1U;}}}
void HmiDisplay::drawFont2Char(char ch,int32_t x,int32_t y,uint8_t scale){
  const uint8_t c=static_cast<uint8_t>(ch);
  if(c<32U||c>127U)return;
  const uint8_t width=widtbl_f16[c-32U];
  const uint8_t bytes_per_row=static_cast<uint8_t>((width+6U)/8U);
  const unsigned char *glyph=chrtbl_f16[c-32U];
  for(uint8_t row=0U;row<16U;++row){
    for(uint8_t byte_index=0U;byte_index<bytes_per_row;++byte_index){
      const uint8_t bits=glyph[static_cast<uint16_t>(row)*bytes_per_row+byte_index];
      for(uint8_t bit=0U;bit<8U;++bit){
        const uint8_t px=static_cast<uint8_t>(byte_index*8U+bit);
        if(px>=width)break;
        if((bits & static_cast<uint8_t>(0x80U>>bit))!=0U)
          fillRect(x+static_cast<int32_t>(px)*scale,y+static_cast<int32_t>(row)*scale,scale,scale,text_fg_);
      }
    }
  }
}
void HmiDisplay::drawGfxGlyph(uint8_t c,int32_t bx,int32_t by){if(font_==nullptr||c<font_->first||c>font_->last)return;const GFXglyph &g=font_->glyph[c-font_->first];uint32_t off=g.bitmapOffset;uint8_t bits=0U,nbit=0U;for(uint8_t yy=0U;yy<g.height;++yy){for(uint8_t xx=0U;xx<g.width;++xx){if(nbit==0U){bits=font_->bitmap[off++];nbit=8U;}if((bits&0x80U)!=0U)drawPixel(bx+g.xOffset+xx,by+g.yOffset+yy,text_fg_);bits<<=1U;--nbit;}}}
int16_t HmiDisplay::drawString(const char *text,int32_t x,int32_t y){
  if (text == nullptr) return 0;
  int32_t minx,miny,maxx,maxy,advance;textBounds(text,minx,miny,maxx,maxy,advance);const int32_t cw=std::max<int32_t>(0,maxx-minx+1),ch=std::max<int32_t>(1,maxy-miny+1),lw=std::max<int32_t>(advance,cw);int32_t left=x,top=y;
  if(datum_==TC_DATUM||datum_==MC_DATUM||datum_==BC_DATUM)left-=lw/2;else if(datum_==TR_DATUM||datum_==MR_DATUM||datum_==BR_DATUM)left-=lw;
  if(datum_==ML_DATUM||datum_==MC_DATUM||datum_==MR_DATUM)top-=ch/2;else if(datum_==BL_DATUM||datum_==BC_DATUM||datum_==BR_DATUM)top-=ch;
  const int32_t clearw=std::max<int32_t>(lw,padding_);int32_t clearleft=left;if(padding_>lw){if(datum_==TC_DATUM||datum_==MC_DATUM||datum_==BC_DATUM)clearleft=x-clearw/2;else if(datum_==TR_DATUM||datum_==MR_DATUM||datum_==BR_DATUM)clearleft=x-clearw;}if(clearw>0)fillRect(clearleft,top,clearw,ch,text_bg_);
  if(font_==nullptr){
    const uint8_t scale=text_size_; int32_t cur=left;
    for(const char *q=text;*q!='\0';++q){
      if(builtin_font_==2U){
        const uint8_t c=static_cast<uint8_t>(*q);
        drawFont2Char(*q,cur,top,scale);
        cur+=static_cast<int32_t>(c>=32U&&c<=127U?widtbl_f16[c-32U]:widtbl_f16[0])*scale;
      }else{drawBuiltinChar(*q,cur,top,scale);cur+=6*scale;}
    }
  }
  else{int32_t cur=left-minx;const int32_t baseline=top-miny;for(const char *q=text;*q!='\0';++q){const uint8_t c=static_cast<uint8_t>(*q);if(c<font_->first||c>font_->last)continue;drawGfxGlyph(c,cur,baseline);cur+=font_->glyph[c-font_->first].xAdvance;}}
  return static_cast<int16_t>(lw);
}

void HmiDisplay::setTouch(const uint16_t *p) {
  if (p == nullptr) return;
  touch_x0_ = p[0] ? p[0] : 1U; touch_x1_ = p[1] ? p[1] : 1U;
  touch_y0_ = p[2] ? p[2] : 1U; touch_y1_ = p[3] ? p[3] : 1U;
  touch_rotate_ = (p[4] & 1U) != 0U;
  touch_invert_x_ = (p[4] & 2U) != 0U;
  touch_invert_y_ = (p[4] & 4U) != 0U;
}

uint8_t HmiDisplay::transfer(uint8_t value) {
  uint8_t rx = 0U;
  (void)HAL_SPI_TransmitReceive(&hspi1, &value, &rx, 1U, 20U);
  return rx;
}

uint16_t HmiDisplay::transfer16(uint16_t value) {
  uint8_t tx[2] = {static_cast<uint8_t>(value >> 8U), static_cast<uint8_t>(value)};
  uint8_t rx[2]{};
  (void)HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2U, 20U);
  return static_cast<uint16_t>((static_cast<uint16_t>(rx[0]) << 8U) | rx[1]);
}
uint16_t HmiDisplay::readTouchZ() {
  setSpiPrescaler(SPI_BAUDRATEPRESCALER_64); TftCs(true); TouchCs(false);
  int16_t z = 0x0FFF;
  (void)transfer(0xB0U);
  z = static_cast<int16_t>(z + static_cast<int16_t>(transfer16(0x00C0U) >> 3U));
  z = static_cast<int16_t>(z - static_cast<int16_t>(transfer16(0x0000U) >> 3U));
  TouchCs(true);
  return z == 4095 ? 0U : static_cast<uint16_t>(std::max<int16_t>(0, z));
}

void HmiDisplay::readTouchRaw(uint16_t *x, uint16_t *y) {
  setSpiPrescaler(SPI_BAUDRATEPRESCALER_64); TftCs(true); TouchCs(false);
  uint16_t t;
  (void)transfer(0xD0U); (void)transfer(0U); (void)transfer(0xD0U); (void)transfer(0U);
  (void)transfer(0xD0U); (void)transfer(0U); (void)transfer(0xD0U);
  t = static_cast<uint16_t>(transfer(0U)) << 5U;
  t |= static_cast<uint16_t>((transfer(0x90U) >> 3U) & 0x1FU); *x = t;
  (void)transfer(0U); (void)transfer(0x90U); (void)transfer(0U); (void)transfer(0x90U);
  (void)transfer(0U); (void)transfer(0x90U); (void)transfer(0U);
  t = static_cast<uint16_t>(transfer(0U)) << 5U;
  t |= static_cast<uint16_t>((transfer(0U) >> 3U) & 0x1FU); *y = t;
  TouchCs(true);
}
bool HmiDisplay::validTouch(uint16_t *x, uint16_t *y, uint16_t threshold) {
  uint16_t z1 = 1U, z2 = 0U;
  while (z1 > z2) { z2 = z1; z1 = readTouchZ(); HAL_Delay(1U); }
  if (z1 <= threshold) return false;
  uint16_t x1, y1, x2, y2;
  readTouchRaw(&x1, &y1); HAL_Delay(1U);
  if (readTouchZ() <= threshold) return false;
  HAL_Delay(2U); readTouchRaw(&x2, &y2);
  if (std::abs(static_cast<int32_t>(x1) - x2) > 20 ||
      std::abs(static_cast<int32_t>(y1) - y2) > 20) return false;
  *x = x1; *y = y1; return true;
}

bool HmiDisplay::getTouch(uint16_t *x, uint16_t *y, uint16_t threshold) {
  if (x == nullptr || y == nullptr) return false;
  threshold = std::max<uint16_t>(20U, threshold);
  if (static_cast<int32_t>(press_time_ms_ - HAL_GetTick()) > 0) threshold = 20U;
  uint16_t rx = 0U, ry = 0U; uint8_t valid = 0U;
  for (uint8_t i = 0U; i < 5U; ++i) if (validTouch(&rx, &ry, threshold)) ++valid;
  if (valid == 0U) { press_time_ms_ = 0U; return false; }
  press_time_ms_ = HAL_GetTick() + 50U;
  int32_t sx = 0, sy = 0;
  if (touch_rotate_) { sx = (static_cast<int32_t>(ry) - touch_x0_) * width_ / touch_x1_; sy = (static_cast<int32_t>(rx) - touch_y0_) * height_ / touch_y1_; }
  else { sx = (static_cast<int32_t>(rx) - touch_x0_) * width_ / touch_x1_; sy = (static_cast<int32_t>(ry) - touch_y0_) * height_ / touch_y1_; }
  if (touch_invert_x_) sx = width_ - sx;
  if (touch_invert_y_) sy = height_ - sy;
  if (sx < 0 || sy < 0 || sx >= width_ || sy >= height_) return false;
  *x = static_cast<uint16_t>(ClampI32(sx, 0, width_ - 1));
  *y = static_cast<uint16_t>(ClampI32(sy, 0, height_ - 1));
  return true;
}
