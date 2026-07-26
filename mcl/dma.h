#ifndef DMA_H
#define DMA_H

#include <stdint.h>

#include <mcl/mcl.h>

typedef enum {
    DMA_DIR_PER_TO_MEM = 0,
    DMA_DIR_MEM_TO_PER = 1,
    DMA_DIR_MEM_TO_MEM = 2
} DMA_Direction_t;

typedef void (*transfer_callback_t)(int);
transfer_callback_t transfer_callback;

struct DMA_Config_t {
    uint32_t channel;
    uint32_t memorySize;
    uint8_t circularMode;
    DMA_TypeDef *instance;
    uint16_t transferCount;
    uint8_t memoryIncrement;
    uint32_t peripheralSize;
    DMA_Direction_t direction;
    DMA_Stream_TypeDef *stream;
    uint8_t peripheralIncrement;
    volatile void *memoryAddress;
    volatile void *peripheralAddress;
    transfer_callback_t transfer_callback;
};

static inline void dma_clear_flags(DMA_Stream_TypeDef *stream) {
    if(stream == DMA2_Stream0) {
        DMA2->LIFCR = DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 |
            DMA_LIFCR_CTEIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0;
    }
}

static inline void dma_init(const DMA_Config_t *cfg) {
    transfer_callback = cfg->transfer_callback;
    /* Enable clock for DMA */
    mcl::enableClockForDma(cfg->instance);
    /* Disable stream */
    cfg->stream->CR &= ~DMA_SxCR_EN;
    while (cfg->stream->CR & DMA_SxCR_EN);
    /* Clear pending DMA flags */
    dma_clear_flags(cfg->stream);
    /* Reset configuration */
    cfg->stream->CR = 0;
    cfg->stream->FCR = 0;
    /* Channel */
    cfg->stream->CR |= (cfg->channel << DMA_SxCR_CHSEL_Pos);
    /* Addresses */
    cfg->stream->PAR  = (uint32_t)cfg->peripheralAddress;
    cfg->stream->M0AR = (uint32_t)cfg->memoryAddress;
    /* Number of transfers */
    cfg->stream->NDTR = cfg->transferCount;
    /* Direction */
    cfg->stream->CR |= ((uint32_t)cfg->direction << DMA_SxCR_DIR_Pos);
    /* Peripheral increment */
    if (cfg->peripheralIncrement) {
        cfg->stream->CR |= DMA_SxCR_PINC;
    }
    /* Memory increment */
    if (cfg->memoryIncrement) {
        cfg->stream->CR |= DMA_SxCR_MINC;
    }
    /* Circular */
    if (cfg->circularMode) {
        cfg->stream->CR |= DMA_SxCR_CIRC;
    }

    /* Half Transfer Interrupt */
    cfg->stream->CR |= DMA_SxCR_HTIE;
    /* Transfer Complete Interrupt */
    cfg->stream->CR |= DMA_SxCR_TCIE;
    /* Transfer Error Interrupt */
    cfg->stream->CR |= DMA_SxCR_TEIE;

    /* Peripheral size */
    cfg->stream->CR |= cfg->peripheralSize;
    /* Memory size */
    cfg->stream->CR |= cfg->memorySize;
}

static inline void dma_start(const DMA_Config_t *cfg) {
    cfg->stream->CR |= DMA_SxCR_EN;
}

static inline void dma_stop(const DMA_Config_t *cfg) {
    cfg->stream->CR &= ~DMA_SxCR_EN;
    while(cfg->stream->CR & DMA_SxCR_EN);
}

static inline void dma_enable_irq(const DMA_Config_t *cfg) {
    if (cfg->stream == DMA2_Stream0) {
        NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    }
}

extern "C" {
    void DMA2_Stream0_IRQHandler(void) {
        uint32_t flags = DMA2->LISR;
        if (flags & DMA_LISR_HTIF0) {
            transfer_callback(1);
        }
        if (flags & DMA_LISR_TCIF0) {
            transfer_callback(2);
        }
        if (flags & DMA_LISR_TEIF0) {
            transfer_callback(3);
        }
        DMA2->LIFCR = DMA_LIFCR_CFEIF0 |
            DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0 |
            DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0;
    }
}

#endif