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

// DMA1/DMA2 each having 8 streams
inline const DMA_Config_t * dma_stream_cfg[2][8] = {nullptr};

static inline void dma_clear_flags(const DMA_Config_t *cfg) {
    // add to these as per use
    if (cfg->stream == DMA2_Stream0) {
        DMA2->LIFCR = DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 |
            DMA_LIFCR_CTEIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0;
    } else if (cfg->stream == DMA2_Stream1) {
        DMA2->LIFCR = DMA_LIFCR_CFEIF1 | DMA_LIFCR_CDMEIF1 |
            DMA_LIFCR_CTEIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTCIF1;
    } else if (cfg->stream == DMA2_Stream2) {
        DMA2->LIFCR = DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 |
            DMA_LIFCR_CTEIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2;
    }
}

static inline bool dma_get_indices(const DMA_Config_t *cfg,
        uint32_t *dma_index, uint32_t *stream_index) {
    if (cfg->instance == DMA1) {
        *dma_index = 0;
        if (cfg->stream == DMA1_Stream0) *stream_index = 0;
        else if (cfg->stream == DMA1_Stream1) *stream_index = 1;
        else if (cfg->stream == DMA1_Stream2) *stream_index = 2;
        else if (cfg->stream == DMA1_Stream3) *stream_index = 3;
        else if (cfg->stream == DMA1_Stream4) *stream_index = 4;
        else if (cfg->stream == DMA1_Stream5) *stream_index = 5;
        else if (cfg->stream == DMA1_Stream6) *stream_index = 6;
        else if (cfg->stream == DMA1_Stream7) *stream_index = 7;
        else return false;
        return true;
    }
    if (cfg->instance == DMA2) {
        *dma_index = 1;
        if (cfg->stream == DMA2_Stream0) *stream_index = 0;
        else if (cfg->stream == DMA2_Stream1) *stream_index = 1;
        else if (cfg->stream == DMA2_Stream2) *stream_index = 2;
        else if (cfg->stream == DMA2_Stream3) *stream_index = 3;
        else if (cfg->stream == DMA2_Stream4) *stream_index = 4;
        else if (cfg->stream == DMA2_Stream5) *stream_index = 5;
        else if (cfg->stream == DMA2_Stream6) *stream_index = 6;
        else if (cfg->stream == DMA2_Stream7) *stream_index = 7;
        else return false;
        return true;
    }
    return false;
}

static inline bool dma_init(const DMA_Config_t *cfg) {
    uint32_t d, s;
    if (!dma_get_indices(cfg, &d, &s))
        return false;
    dma_stream_cfg[d][s] = cfg;
    /* Enable clock for DMA */
    mcl::enableClockForDma(cfg->instance);
    /* Disable stream */
    cfg->stream->CR &= ~DMA_SxCR_EN;
    while (cfg->stream->CR & DMA_SxCR_EN);
    /* Clear pending DMA flags */
    dma_clear_flags(cfg);
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
    return true;
}

static inline void dma_start(const DMA_Config_t *cfg) {
    cfg->stream->CR |= DMA_SxCR_EN;
}

static inline void dma_stop(const DMA_Config_t *cfg) {
    cfg->stream->CR &= ~DMA_SxCR_EN;
    while(cfg->stream->CR & DMA_SxCR_EN);
}

static inline void dma_enable_irq(const DMA_Config_t *cfg) {
    // add to this as per use..
    if (cfg->stream == DMA2_Stream0) {
        NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    } else if (cfg->stream == DMA2_Stream1) {
        NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    } else if (cfg->stream == DMA2_Stream2) {
        NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    }
}

extern "C" void DMA2_Stream0_IRQHandler(void) {
    const DMA_Config_t *cfg = dma_stream_cfg[1][0];
    if (!cfg) return;
    uint32_t flags = cfg->instance->LISR;
    if (cfg->transfer_callback) {
        if (flags & DMA_LISR_HTIF0) {
            cfg->transfer_callback(1);
        }
        if (flags & DMA_LISR_TCIF0) {
            cfg->transfer_callback(2);
        }
        if (flags & DMA_LISR_TEIF0) {
            cfg->transfer_callback(3);
        }
    }
    cfg->instance->LIFCR = DMA_LIFCR_CFEIF0 |
        DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0 |
        DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0;
}

extern "C" void DMA2_Stream1_IRQHandler(void) {
    const DMA_Config_t *cfg = dma_stream_cfg[1][1];
    if (!cfg) return;
    // todo: clear the flags
}
extern "C" void DMA2_Stream2_IRQHandler(void) {
    const DMA_Config_t *cfg = dma_stream_cfg[1][2];
    if (!cfg) return;
    // todo: clear the flags
}

#endif