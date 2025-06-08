#ifndef SPI_USRT_H
#define SPI_USRT_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Configuration
#ifndef SPI_USRT_BUFFER_SIZE
#define SPI_USRT_BUFFER_SIZE 128
#endif

#define SPI_USRT_MAX_FRAME_SIZE 32

// Frame format: START | LENGTH | DATA | END
#define SPI_USRT_START_BYTE 0xAA
#define SPI_USRT_END_BYTE   0xBB
#define SPI_USRT_IDLE_BYTE  0xFF

// Error codes
typedef enum {
    SPI_USRT_OK = 0,
    SPI_USRT_ERROR_BUFFER_FULL,
    SPI_USRT_ERROR_BUFFER_EMPTY,
    SPI_USRT_ERROR_INVALID_PARAM
} spi_usrt_error_t;

// RX states
typedef enum {
    RX_IDLE,
    RX_LENGTH,
    RX_DATA,
    RX_END
} rx_state_t;

// TX states
typedef enum {
    TX_IDLE,
    TX_START,
    TX_LENGTH,
    TX_DATA,
    TX_END
} tx_state_t;

// Frame queue entry
typedef struct {
    uint8_t data[SPI_USRT_MAX_FRAME_SIZE];
    uint8_t length;
} frame_t;

// Frame queue
typedef struct {
    frame_t frames[8]; // Queue up to 8 complete frames
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} frame_queue_t;

// RX buffer
typedef struct {
    uint8_t data[SPI_USRT_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
} buffer_t;

// Main structure
typedef struct {
    buffer_t rx_buf;
    frame_queue_t tx_queue;
    
    // RX state
    volatile rx_state_t rx_state;
    volatile uint8_t rx_frame[SPI_USRT_MAX_FRAME_SIZE];
    volatile uint8_t rx_length;
    volatile uint8_t rx_pos;
    
    // TX state
    volatile tx_state_t tx_state;
    volatile uint8_t tx_pos;
    
    // Platform function
    void (*spi_transfer)(uint8_t tx_byte, uint8_t *rx_byte);
} spi_usrt_t;

// Function declarations
static inline void spi_usrt_init(spi_usrt_t *usrt, void (*spi_transfer_func)(uint8_t, uint8_t*));
static inline spi_usrt_error_t spi_usrt_write(spi_usrt_t *usrt, const uint8_t *data, uint8_t length);
static inline spi_usrt_error_t spi_usrt_read(spi_usrt_t *usrt, uint8_t *data, uint8_t max_length, uint8_t *actual_length);
static inline uint16_t spi_usrt_available(spi_usrt_t *usrt);
static inline void spi_usrt_process(spi_usrt_t *usrt);

// Helper functions
static inline bool buffer_put(buffer_t *buf, uint8_t data);
static inline bool buffer_get(buffer_t *buf, uint8_t *data);
static inline bool frame_queue_put(frame_queue_t *queue, const uint8_t *data, uint8_t length);
static inline bool frame_queue_get(frame_queue_t *queue, frame_t *frame);
static inline void process_rx_byte(spi_usrt_t *usrt, uint8_t byte);
static inline uint8_t get_tx_byte(spi_usrt_t *usrt);

// Implementation

static inline void spi_usrt_init(spi_usrt_t *usrt, void (*spi_transfer_func)(uint8_t, uint8_t*)) {
    if (!usrt || !spi_transfer_func) return;
    
    memset((void*)usrt, 0, sizeof(spi_usrt_t));
    usrt->spi_transfer = spi_transfer_func;
    usrt->rx_state = RX_IDLE;
    usrt->tx_state = TX_IDLE;
}

static inline bool buffer_put(buffer_t *buf, uint8_t data) {
    if (buf->count >= SPI_USRT_BUFFER_SIZE) return false;
    
    buf->data[buf->head] = data;
    buf->head = (buf->head + 1) % SPI_USRT_BUFFER_SIZE;
    buf->count++;
    return true;
}

static inline bool buffer_get(buffer_t *buf, uint8_t *data) {
    if (buf->count == 0) return false;
    
    *data = buf->data[buf->tail];
    buf->tail = (buf->tail + 1) % SPI_USRT_BUFFER_SIZE;
    buf->count--;
    return true;
}

static inline bool frame_queue_put(frame_queue_t *queue, const uint8_t *data, uint8_t length) {
    if (queue->count >= 8 || length == 0 || length > SPI_USRT_MAX_FRAME_SIZE) return false;
    
    frame_t *frame = &queue->frames[queue->head];
    memcpy(frame->data, data, length);
    frame->length = length;
    
    queue->head = (queue->head + 1) % 8;
    queue->count++;
    return true;
}

static inline bool frame_queue_get(frame_queue_t *queue, frame_t *frame) {
    if (queue->count == 0) return false;
    
    *frame = queue->frames[queue->tail];
    queue->tail = (queue->tail + 1) % 8;
    queue->count--;
    return true;
}

static inline void process_rx_byte(spi_usrt_t *usrt, uint8_t byte) {
    switch (usrt->rx_state) {
        case RX_IDLE:
            if (byte == SPI_USRT_START_BYTE) {
                usrt->rx_state = RX_LENGTH;
            }
            break;
            
        case RX_LENGTH:
            usrt->rx_length = byte;
            usrt->rx_pos = 0;
            if (usrt->rx_length == 0 || usrt->rx_length > SPI_USRT_MAX_FRAME_SIZE) {
                usrt->rx_state = RX_IDLE;
            } else {
                usrt->rx_state = RX_DATA;
            }
            break;
            
        case RX_DATA:
            usrt->rx_frame[usrt->rx_pos++] = byte;
            if (usrt->rx_pos >= usrt->rx_length) {
                usrt->rx_state = RX_END;
            }
            break;
            
        case RX_END:
            if (byte == SPI_USRT_END_BYTE) {
                // Frame complete - add to buffer
                for (uint8_t i = 0; i < usrt->rx_length; i++) {
                    buffer_put((buffer_t*)&usrt->rx_buf, usrt->rx_frame[i]);
                }
            }
            usrt->rx_state = RX_IDLE;
            break;
    }
}

static inline uint8_t get_tx_byte(spi_usrt_t *usrt) {
    static frame_t current_frame;
    static bool frame_loaded = false;
    
    switch (usrt->tx_state) {
        case TX_IDLE:
            // Load next frame if available
            if (!frame_loaded && frame_queue_get((frame_queue_t*)&usrt->tx_queue, &current_frame)) {
                frame_loaded = true;
                usrt->tx_pos = 0;
                usrt->tx_state = TX_LENGTH;
                return SPI_USRT_START_BYTE;
            }
            return SPI_USRT_IDLE_BYTE;
            
        case TX_START:
            // This state is no longer used - removed duplicate start byte
            usrt->tx_state = TX_LENGTH;
            return current_frame.length;
            
        case TX_LENGTH:
            usrt->tx_state = TX_DATA;
            return current_frame.length;
            
        case TX_DATA:
            if (usrt->tx_pos < current_frame.length) {
                return current_frame.data[usrt->tx_pos++];
            } else {
                usrt->tx_state = TX_IDLE;
                frame_loaded = false; // Frame transmission complete
                return SPI_USRT_END_BYTE;
            }
            
        case TX_END:
            usrt->tx_state = TX_IDLE;
            frame_loaded = false;
            return SPI_USRT_END_BYTE;
            
        default:
            usrt->tx_state = TX_IDLE;
            frame_loaded = false;
            return SPI_USRT_IDLE_BYTE;
    }
}

static inline void spi_usrt_process(spi_usrt_t *usrt) {
    if (!usrt || !usrt->spi_transfer) return;
    
    uint8_t tx_byte = get_tx_byte(usrt);
    uint8_t rx_byte;
    
    usrt->spi_transfer(tx_byte, &rx_byte);
    
    if (rx_byte != SPI_USRT_IDLE_BYTE || usrt->rx_state == RX_DATA) {
        process_rx_byte(usrt, rx_byte);
    }
}

static inline spi_usrt_error_t spi_usrt_write(spi_usrt_t *usrt, const uint8_t *data, uint8_t length) {
    if (!usrt || !data || length == 0) return SPI_USRT_ERROR_INVALID_PARAM;
    
    // Each write call creates exactly one frame - no mixing of commands
    if (!frame_queue_put((frame_queue_t*)&usrt->tx_queue, data, length)) {
        return SPI_USRT_ERROR_BUFFER_FULL;
    }
    
    return SPI_USRT_OK;
}

static inline spi_usrt_error_t spi_usrt_read(spi_usrt_t *usrt, uint8_t *data, uint8_t max_length, uint8_t *actual_length) {
    if (!usrt || !data || !actual_length) return SPI_USRT_ERROR_INVALID_PARAM;
    
    *actual_length = 0;
    
    while (*actual_length < max_length && usrt->rx_buf.count > 0) {
        if (!buffer_get((buffer_t*)&usrt->rx_buf, &data[*actual_length])) {
            break;
        }
        (*actual_length)++;
    }
    
    return (*actual_length > 0) ? SPI_USRT_OK : SPI_USRT_ERROR_BUFFER_EMPTY;
}

static inline uint16_t spi_usrt_available(spi_usrt_t *usrt) {
    return usrt ? usrt->rx_buf.count : 0;
}

#endif // SPI_USRT_H
