#include "dma.h"
#include "usart.h"
#include "driver/uart.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/*! The busy indication for the uart */
static volatile bool isTxBusy_ = false;
/*! The buffer for the uart print */
static uint8_t buffer_[1024];
/*! The callback for the uart */
static uart_tx_callback_t txCallback_ = 0;
/*! The callback state for the uart */
static void * txCallbackState_ = 0;

/*!***************************************************************************
* @brief Initialize the Universal Asynchronous Receiver/Transmitter
* (UART or LPSCI) bus and DMA module
* @param -
* @return Returns the \link #status_t status\endlink (#STATUS_OK on success).
*****************************************************************************/
status_t UART_Init(void)
{
MX_DMA_Init();
MX_USART2_UART_Init();
return STATUS_OK;
}

/*!***************************************************************************
* @brief Writes several bytes to the UART connection.
* @param txBuff Data array to write to the uart connection
* @param txSize The size of the data array
* @param f Callback function after tx is done, set 0 if not needed;
* @param state Optional user state that will be passed to callback
* function; set 0 if not needed.
* @return Returns the \link #status_t status\endlink:
* - #STATUS_OK (0) on success.
* - #STATUS_BUSY on Tx line busy
* - #ERROR_NOT_INITIALIZED
* - #ERROR_INVALID_ARGUMENT
*****************************************************************************/
status_t UART_SendBuffer(uint8_t const * txBuff, size_t txSize, uart_tx_callback_t f, void * state)
{
/* Verify arguments. */
if( !txBuff || txSize == 0 )
return ERROR_INVALID_ARGUMENT;
if (isTxBusy_)
return STATUS_BUSY;
/* Set Tx Busy Status. */
isTxBusy_ = true;
txCallback_ = f;
txCallbackState_ = state;
HAL_UART_Transmit_DMA(&huart2, (uint8_t *) txBuff, txSize);
return STATUS_OK;
}

/**
* @brief Tx Transfer completed callbacks.
* @param huart Pointer to a UART_HandleTypeDef structure that contains
* the configuration information for the specified UART module.
* @retval None
*/
void HAL_UART_TxCpltCallback( UART_HandleTypeDef *huart )
{
isTxBusy_ = false;
status_t status = huart->gState == HAL_UART_STATE_ERROR ? ERROR_FAIL : STATUS_OK;
if (txCallback_)
{
txCallback_(status, txCallbackState_);
}
}

/*!***************************************************************************
* @brief printf-like function to send print messages via UART.
*
* @details Defined in "driver/uart.c" source file.
*
* Open an UART connection with 115200 bps, 8N1, no handshake to
* receive the data on a computer.
*
* @param fmt_s The usual printf parameters.
*
* @return Returns the \link #status_t status\endlink (#STATUS_OK on success).
*****************************************************************************/
status_t print(const char *fmt_s, ...)
{
va_list ap;
va_start(ap, fmt_s);
int len = vsnprintf((char *) buffer_, sizeof(buffer_), fmt_s, ap);
va_end(ap);
if (len < 0)
return ERROR_FAIL;
UART_SendBuffer(buffer_, len, 0, 0);
return STATUS_OK;
}
#undef UART_Print

