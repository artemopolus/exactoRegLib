#ifndef EXACTO_BUFFER_H_
#define EXACTO_BUFFER_H_

#include <stdint.h> 


#include <stdlib.h>


typedef struct{
    uint8_t * data;
    uint8_t str;
    uint8_t lst;
    uint8_t isExist;
    uint8_t datalen;
    uint8_t isEmpty;
    uint8_t mask;
} ExactoBufferUint8Type;

typedef enum{
    EXACTO_BUFLEN_02 = 2,
    EXACTO_BUFLEN_04 = 4,
    EXACTO_BUFLEN_08 = 8,
    EXACTO_BUFLEN_16 = 16,
    EXACTO_BUFLEN_64 = 64,
    EXACTO_BUFLEN_128 = 128
}ExactoBufferUint8TypSizes;

#define __static_inline static inline
    
__static_inline void    setini_exbu8(ExactoBufferUint8Type * buffer, ExactoBufferUint8TypSizes sz)
{
    buffer->str = 0;
    buffer->lst = 0;
    buffer->data = (uint8_t *)calloc(sz, sizeof(uint8_t));
    buffer->isExist = 1;
    buffer->isEmpty = 1;
    buffer->datalen = sz;
    buffer->mask = sz - 1;
}
__static_inline uint8_t getlen_exbu8(ExactoBufferUint8Type * buffer)
{
    if( buffer->lst >= buffer->str)
        return (buffer->lst - buffer->str);
    else
        return (buffer->lst + buffer->datalen - buffer->str);
}
__static_inline uint8_t grbfst_exbu8(ExactoBufferUint8Type * buffer, uint8_t * fstval)
{
    if(!buffer->isExist || buffer->isEmpty)     return 0;
    *fstval = buffer->data[buffer->str];
    if(buffer->str == buffer->lst)  {buffer->isEmpty = 1;   return 1;}
    buffer->str = (buffer->str + 1) & buffer->mask;
    return 1;
}
__static_inline void pshfrc_exbu8(ExactoBufferUint8Type * buffer,const uint8_t value)
{
    if(!buffer->isExist)     return;
    buffer->data[buffer->lst] = value;
    buffer->lst = (buffer->lst + 1) & buffer->mask;
    if(buffer->lst == buffer->str) buffer->str = (buffer->str + 1) & buffer->mask;
}
__static_inline uint8_t grball_exbu8(ExactoBufferUint8Type * buffer, uint8_t * dst)
{
    if(!buffer->isExist || buffer->isEmpty)     return 0;
    uint8_t flag, value, i = 0;
    flag = grbfst_exbu8(buffer, &value);
    while(flag)
    {
        dst[i] = value;
        flag = grbfst_exbu8(buffer, &value); 
    }
    return 1;
}
__static_inline uint8_t clrval_exbu8(ExactoBufferUint8Type * buffer)
{
    if(!buffer->isExist || buffer->isEmpty)     return 0;
    if(buffer->str == buffer->lst)  {buffer->isEmpty = 1;   return 0;}
    buffer->str = (buffer->str + 1) & buffer->mask;
    return 1;
}
__static_inline uint8_t clrsvr_exbu8(ExactoBufferUint8Type * buffer, const uint8_t cnt)
{
    if(!buffer->isExist || buffer->isEmpty)     return 0;
    uint8_t i = 0;
    while(clrval_exbu8(buffer) &&(i < cnt))
    {
        i++;
    }
    return 1;
}
__static_inline uint8_t getval_exbu8(ExactoBufferUint8Type * buffer, uint8_t index, uint8_t * value)
{
    if(!buffer->isExist || buffer->isEmpty)
        return 0;
    if(!getlen_exbu8(buffer))
        return 0;
    if(     
            ((buffer->str <= index)&&(index <= buffer->lst)) || 
            ((buffer->str <= index)&&(index <= buffer->datalen)) ||
            (index <= buffer->lst)
//            ((0 <= index)&&(index <= buffer->lst)) because of unsigned
      )
    {
        *value = buffer->data[index];
        return 1;
    }
    else
        return 0;
    
}
__static_inline uint8_t    getmas_exbu8(ExactoBufferUint8Type * buffer, uint8_t * dst)
{
    if (!buffer->isExist || buffer->isEmpty)
        return 0;
    if( buffer->lst > buffer->str)
    {
        for(uint8_t i = buffer->str; i <= buffer->lst; i++)
            dst[i - buffer->str] = buffer->data[i];
    }
    else
    {
        for(uint8_t i = buffer->str; i < buffer->datalen; i++)
            dst[i - buffer->str] = buffer->data[i];
        for(uint8_t i = 0; i <= buffer->str; i++)
            dst[i + buffer->datalen - buffer->str] = buffer->data[i];
    }
    return 1;
}
__static_inline uint8_t setval_exbu8 (ExactoBufferUint8Type * buffer, uint8_t value)
{
    if (!buffer->isExist)
        return 0;
    if(buffer->str == buffer->lst)
    {
        if(buffer->isEmpty)
        {
            buffer->data[buffer->lst] = value;
            buffer->isEmpty = 0;
        }
        else
        {
            if(buffer->lst == (buffer->datalen - 1))
                buffer->lst = 0;
            else
                buffer->lst++;
            buffer->data[buffer->lst] = value;
        }
    }
    else
    {
        if(buffer->lst == (buffer->datalen - 1) )
        {
            if (buffer->str == 0)
            {
                buffer->str++;
            }
            buffer->lst = 0;
            buffer->data[buffer->lst] = value;
        }
        else
        {
            if(buffer->lst == (buffer->str - 1) )
            {
                buffer->str++;
            }
            buffer->lst++;
            buffer->data[buffer->lst] = value;
        }
    }
    return 1;
}
__static_inline uint8_t setemp_exbu8 (ExactoBufferUint8Type * buffer)
{
    if (!buffer->isExist)
        return 0;
    if (buffer->isEmpty)
        return 1;
    buffer->lst = 0;
    buffer->str = 0;
    buffer->isEmpty = 1;
    return 1;
}
__static_inline uint8_t setclr_exbu8 (ExactoBufferUint8Type * buffer, uint8_t len)
{
    if (!buffer->isExist || buffer->isEmpty)
        return 0;
    if  (len >= buffer->datalen)
        return 0;
    if  (buffer->lst >= buffer->str)
    {
        if ((buffer->lst - buffer->str) <= len)
        {
            buffer->lst = 0;
            buffer->str = 0;
            buffer->isEmpty = 1;
        }
        else
        {
            buffer->str += len;
        }
    }
    else
    {
        if ((buffer->lst +(buffer->datalen - buffer->str)) <= len)
        {
            buffer->lst = 0;
            buffer->str = 0;
            buffer->isEmpty = 1;
        }
        else
        {
            if ((buffer->str + len) < buffer->datalen)
                buffer->str = buffer->datalen - buffer->str;
            else
                buffer->str += len;
        }
    }
    return 1;
}

// size 2 4 8 16 32 64 128
#define FIFO( size )\
  struct {\
    unsigned char buf[size];\
    unsigned char tail;\
    unsigned char head;\
  } 

#define FIFO_COUNT(fifo)     (fifo.head-fifo.tail)
#define FIFO_SIZE(fifo)      ( sizeof(fifo.buf)/sizeof(fifo.buf[0]) )
#define FIFO_IS_FULL(fifo)   (FIFO_COUNT(fifo)==FIFO_SIZE(fifo))
#define FIFO_IS_EMPTY(fifo)  (fifo.tail==fifo.head)
#define FIFO_SPACE(fifo)     (FIFO_SIZE(fifo)-FIFO_COUNT(fifo))
#define FIFO_PUSH(fifo, byte) \
  {\
    fifo.buf[fifo.head &amp; (FIFO_SIZE(fifo)-1)]=byte;\
    fifo.head++;\
  }
#define FIFO_FRONT(fifo) (fifo.buf[fifo.tail &amp; (FIFO_SIZE(fifo)-1)])
#define FIFO_POP(fifo)   \
  {\
      fifo.tail++; \
  }
#define FIFO_FLUSH(fifo)   \
  {\
    fifo.tail=0;\
    fifo.head=0;\
  } 
 

#endif /* EXACTO_BUFFER_H_ */
