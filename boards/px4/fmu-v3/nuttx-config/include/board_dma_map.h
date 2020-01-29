#pragma once

/*   DMA Channel/Stream Selections
 *
 *   DMAMAP_USART3_RX   = DMA1, Stream 1, Channel 4
 *   DMAMAP_UART4_RX    = DMA1, Stream 2, Channel 4
 *   DMAMAP_UART7_RX    = DMA1, Stream 3, Channel 5
 *   DMAMAP_USART2_RX   = DMA1, Stream 5, Channel 4
 *   DMAMAP_TIM4_UP     = DMA1, Stream 6, Channel 2
 *
 *   DMAMAP_SPI4_RX_1   = DMA2, Stream 0, Channel 4
 *   DMAMAP_USART6_RX_1 = DMA2, Stream 1, Channel 5
 *   DMAMAP_SPI1_RX_2   = DMA2, Stream 2, Channel 3
 *   DMAMAP_SPI1_TX_1   = DMA2, Stream 3, Channel 3
 *   DMAMAP_SPI4_TX_2   = DMA2, Stream 4, Channel 5
 *   DMAMAP_TIM1_UP     = DMA2, Stream 5, Channel 6
 *   DMAMAP_SDIO_2      = DMA2, Stream 6, Channel 4
 *   DMAMAP_USART6_TX_2 = DMA2, Stream 7, Channel 5
 *
 */

#define DMACHAN_SPI4_RX DMAMAP_SPI4_RX_1
#define DMAMAP_USART6_RX DMAMAP_USART6_RX_1
#define DMACHAN_SPI1_RX DMAMAP_SPI1_RX_2
#define DMACHAN_SPI1_TX DMAMAP_SPI1_TX_1
#define DMACHAN_SPI4_TX DMAMAP_SPI4_TX_2
#define DMAMAP_SDIO DMAMAP_SDIO_2
#define DMAMAP_USART6_TX DMAMAP_USART6_TX_2
