// fork by MaxWolf a273a4a05b7a683002832bc981cf2244cf4a898bfb503c42b54636d5a5b7982e
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

extern "C" {
#include "BIOS.h"
#include "stm32f10x.h"
#include "ds203_io.h"
#include "mathutils.h"
#include "Interrupt.h"
#include "irq.h"
#include "buttons.h"
#include "lcd.h"
}

#include "dsosignalstream.hh"
#include "xposhandler.hh"
#include "drawable.hh"
#include "textdrawable.hh"
#include "signalgraph.hh"
#include "breaklines.hh"
#include "window.hh"
#include "cursor.hh"
#include "timemeasure.hh"
#include "grid.hh"
#include "menudrawable.hh"

#include "rev.h"
 
//define some colors
#define WHITE   0xFFFF
#define BLACK   0x0000
#define GREY    0x8410

// For some reason, the headers don't have these registers
#define FSMC_BCR1   (*((vu32 *)(0xA0000000+0x00)))
#define FSMC_BTR1   (*((vu32 *)(0xA0000000+0x04)))
#define FSMC_BCR2   (*((vu32 *)(0xA0000008+0x00)))
#define FSMC_BTR2   (*((vu32 *)(0xA0000008+0x04)))

// GPIOC->BSRR values to toggle GPIOC5
static const uint32_t g_hl_set[2] = {1 << (16 + 5), 1 << 5};

// FIFO for stuff that is coming from the DMA.
static uint32_t g_adc_fifo[256];
#define ADC_FIFO_HALFSIZE (sizeof(g_adc_fifo) / sizeof(uint32_t) / 2)

struct signal_buffer_t g_signal_buffer[2] = {{0, 0}, {0, 0}};

// currently active (for DMA updates) g_signal_buffer
volatile int g_active_sb = 0;

// accumulate total data written to estimate remaining flash drive capacity
uint32_t g_bytes_written = 0;
// rough estimation (we do not have access to the actual free disk size anyway
#define FLASH_DISK_SIZE 8000000

// prev state of signal inputs to compare and find an edge
uint32_t g_prev_val = 0;

// clock cound from previous state of the inputs
signaltime_t g_count = 0;

// is double-buffer disk capture active
volatile bool g_db_capture = false;

// is non-active g_signal_buffer ready to be written to disk
volatile bool g_wr_ready = false;

// no data was ever written
volatile bool g_first_chunk = true;

// time offset for the next data chunk to be written
signaltime_t g_chunk_time_offset = 0;

// a kind of watchdog...
volatile bool g_write_ongoing = false;

// to raise overtake event
volatile bool g_stop_writing = false;


void init_signal_buffer(struct signal_buffer_t *sb, signaltime_t duration) {
    // Reset the signal buffer
    sb->last_duration = duration;
    sb->bytes = 0;
}

// This function is the hotspot of the whole capture process.
// It compares the samples until it finds an edge.
const uint32_t * __attribute__((optimize("O3")))
find_edge(const uint32_t *data, const uint32_t *end, const uint32_t mask, const uint32_t old)
{
    // Get to a 4xsizeof(int) boundary
    while (((uint32_t)data & 0x0F) != ((uint32_t)g_adc_fifo & 0x0F))
    {
        if ((*data & mask) != old) return data;
        data++;
    }
    
    while (data < end)
    {
        if ((*data & mask) != old) return data;
        data++;
        if ((*data & mask) != old) return data;
        data++;
        if ((*data & mask) != old) return data;
        data++;
        if ((*data & mask) != old) return data;
        data++;
    }
    
    return end;
}

static void
process_samples(const uint32_t *data) 
{
    
    // Compare the highest bit of each channel and the digital inputs.
    const uint32_t mask = 0x00038080;
    
    const uint32_t *end = data + ADC_FIFO_HALFSIZE;
    for(;;)
    {
        const uint32_t *start = data;
        data = find_edge(data, end, mask, g_prev_val);
        
        // Update count
        g_count += data - start;
        
        if (data == end)
            break; // All done.
        
        // Just a sanity-check
        if (*data & 0xFF000000)
        {
            crash_with_message("Lost the H_L sync", __builtin_return_address(0));
            while(1);
        }
        
        // We may need up to 10 bytes of space in the buffer
        if (sizeof(g_signal_buffer[g_active_sb].storage) < g_signal_buffer[g_active_sb].bytes + 10)
        {
        	if (!g_db_capture) 
        	{
                // Buffer is full, stop data capture
                NVIC_DisableIRQ(DMA1_Channel4_IRQn);
                return;
            }
            if (g_write_ongoing)
            {
                // crash_with_message("Data overtake flushing", __builtin_return_address(0));
                // while(1);
                NVIC_DisableIRQ(DMA1_Channel4_IRQn);
                g_stop_writing = true;
                return;
            }
            // switch buffers
            int next_sb = 1 - g_active_sb;
            g_signal_buffer[g_active_sb].last_duration = g_count;
            g_signal_buffer[next_sb].last_value = g_signal_buffer[g_active_sb].last_value;
            g_active_sb = next_sb;
            init_signal_buffer(&g_signal_buffer[g_active_sb], g_count);
            g_wr_ready = true;
        }

        // Write the value as base-128 varint (google protobuf-style)
        uint64_t value_to_write = (g_count << 4) + g_signal_buffer[g_active_sb].last_value;
        uint8_t *p = g_signal_buffer[g_active_sb].storage + g_signal_buffer[g_active_sb].bytes;
        int i = 0;
        while (value_to_write)
        {
            p[i] = (value_to_write & 0x7F) | 0x80;
            value_to_write >>= 7;
            i++;
        }
        p[i - 1] &= 0x7F; // Unset top bit on last byte
        g_signal_buffer[g_active_sb].bytes += i;

        // Prepare for seeking the next edge
        g_prev_val = (*data & mask);
        g_count = 0;
        
        g_signal_buffer[g_active_sb].last_value = 0;
        if (*data & 0x00000080) g_signal_buffer[g_active_sb].last_value |= 1; // Channel A
        if (*data & 0x00008000) g_signal_buffer[g_active_sb].last_value |= 2; // Channel B
        if (*data & 0x00010000) g_signal_buffer[g_active_sb].last_value |= 4; // Channel C
        if (*data & 0x00020000) g_signal_buffer[g_active_sb].last_value |= 8; // Channel D
    }
    
    g_signal_buffer[g_active_sb].last_duration = g_count;
}

void __irq__ DMA1_Channel4_IRQHandler()
{
    if (DMA1->ISR & DMA_ISR_TEIF4)
    {
        crash_with_message("Oh noes: DMA channel 4 transfer error!",
            __builtin_return_address(0)
        );
        while(1);
    }
    else if (DMA1->ISR & DMA_ISR_HTIF4)
    {
        process_samples(&g_adc_fifo[0]);
        DMA1->IFCR = DMA_IFCR_CHTIF4;
        if (DMA1->ISR & DMA_ISR_TCIF4)
        {
            crash_with_message("Oh noes: ADC fifo overflow in HTIF", __builtin_return_address(0));
            while(1);
        }
    }
    else if (DMA1->ISR & DMA_ISR_TCIF4)
    {
        process_samples(&g_adc_fifo[ADC_FIFO_HALFSIZE]);
        DMA1->IFCR = DMA_IFCR_CTCIF4;
        if (DMA1->ISR & DMA_ISR_HTIF4)
        {
            crash_with_message("Oh noes: ADC fifo overflow in TCIF", __builtin_return_address(0));
            while(1);
        }
    }
}

void start_capture()
{
    // Samplerate is 500kHz, two TMR1 cycles per sample -> PSC = 12 -1, ARR = 6 - 1
    // Channel 2: Trigger DMA Ch3 to write H_L bit
    // Channel 4: Trigger DMA Ch4 to read data to memory
    //
    // TMR cycle:    0  1  2  3  4  5  0  1  2  3  4  5 0
    // MCO output:  _|^^^^^^^^^^^^^^^^^|________________|^
    // H_L:         _|^^^^^^^^^^^^^^^^^|________________|^
    // DMA sample:         ^ read ch A&B     ^ read ch C&D
    TIM1->CR1 = 0; // Turn off TIM1 until we are ready
    TIM1->CR2 = 0;
    TIM1->CNT = 0;
    TIM1->SR = 0;
    TIM1->PSC = 11;
    TIM1->ARR = 5;
    TIM1->CCMR1 = 0x0000; // CC2 time base
    TIM1->CCMR2 = 0x0000; // CC4 time base
    TIM1->DIER = TIM_DIER_CC2DE | TIM_DIER_CC4DE;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR4 = 2;
    
    init_signal_buffer(&g_signal_buffer[0], 0);
    init_signal_buffer(&g_signal_buffer[1], 0);
    // DMA1 channel 3: copy data from g_hl_set to GPIOC->BSRR
    // Priority: very high
    // MSIZE = PSIZE = 32 bits
    // MINC enabled, CIRC mode enabled
    // Direction: read from memory
    // No interrupts
    DMA1_Channel3->CCR = 0;
    DMA1_Channel3->CNDTR = 2;
    DMA1_Channel3->CPAR = (uint32_t)&GPIOC->BSRR;
    DMA1_Channel3->CMAR = (uint32_t)g_hl_set;
    DMA1_Channel3->CCR = 0x3AB1;
    GPIOC->BSRR = g_hl_set[1];
    
    // DMA1 channel 4: copy data from FPGA to g_adc_fifo.
    // Priority: very high
    // MSIZE = PSIZE = 16 bits
    // MINC enabled, CIRC mode enabled
    // Direction: read from peripheral
    // Half- and Full-transfer interrupts, plus error interrupt
    DMA1_Channel4->CCR = 0;
    DMA1_Channel4->CNDTR = sizeof(g_adc_fifo) / 2;
    DMA1_Channel4->CPAR = 0x64000000; // FPGA memory-mapped address
    DMA1_Channel4->CMAR = (uint32_t)g_adc_fifo;
    DMA1_Channel4->CCR = 0x35AF;
    
    // Reduce the wait states of the FPGA & LCD interface
    FSMC_BTR1 = 0x10100110;
    FSMC_BTR2 = 0x10100110;
    FSMC_BCR1 |= FSMC_BCR1_CBURSTRW;
    
    // Clear any pending interrupts for ch 4
    DMA1->IFCR = 0x0000F000;
    
    // Enable ch 4 interrupt
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    NVIC_SetPriority(DMA1_Channel4_IRQn, 0); // Highest priority
    
    // Now, lets go!
    TIM1->CR1 |= TIM_CR1_CEN;

    g_prev_val = 0;
    g_count = 0;
}

void config_wave_out(uint32_t frequency) 
{
    uint32_t prescaler = 0;
    uint32_t arr = 0;
    uint32_t ccr = 0;

    // Calculate prescaler and ARR values
    // Iterate to find a suitable combination
    for (prescaler = 0; prescaler <= 0xFFFF; prescaler++) {
        arr = (CPUFREQ / ((prescaler + 1) * frequency)) - 1;
        if (arr <= 0xFFFF) {
            break;
        }
    }

    // Calculate the duty cycle for a square wave (50% duty cycle)
    ccr = arr / 2;

    // Set the prescaler (PSC)
    __Set(DIGTAL_PSC, prescaler);

    // Set the auto-reload register (ARR)
    __Set(DIGTAL_ARR, arr);

    // Set the duty cycle register (CCR)
    __Set(DIGTAL_CCR, ccr);

    // Additional settings if needed
    // For example, starting the waveform generation if required
    // __Set(SOME_START_REGISTER, START_VALUE);
}

void draw_screen(const std::vector<Drawable*> &objs, int startx, int endx)
{
    const int screenheight = 240;
    uint16_t buffer1[screenheight];
    uint16_t buffer2[screenheight];
    
    for (Drawable *d: objs)
    {
        d->Prepare(startx, endx);
    }
    
    lcd_set_location(startx, 0);
    for (int x = startx; x < endx; x++)
    {
        uint16_t *buffer = (x % 2) ? buffer1 : buffer2;
        memset(buffer, 0, screenheight * 2);
        
        for (Drawable *d: objs)
        {
            d->Draw(buffer, screenheight, x);
        }
        
        lcd_write_dma(buffer, screenheight);
    }
}

#include "gpio.h"
DECLARE_GPIO(usart1_tx, GPIOA, 9);
DECLARE_GPIO(usart1_rx, GPIOA, 10);

void show_status(const std::vector<Drawable*> &screenobjs, TextDrawable &statustext, const char *fmt, ...)
{
    char buffer[50];
    va_list va;
    va_start(va, fmt);
    int rv = vsnprintf(buffer, sizeof(buffer), fmt, va);
    va_end(va);
    
    statustext.set_text(buffer);
    
    draw_screen(screenobjs, 0, 400);
}

enum menu1_entry {
                 ENTRIES_ALL = -1,
                 ENTRY_SCROLL = 0,
                 ENTRY_CH_A_RANGE = 1,
                 ENTRY_CH_B_RANGE = 2,
                 ENTRY_SAVE_MODE = 3,
                 ENTRY_MEMORY_DUMP = 4,
                 ENTRY_WAVE_FRQ = 5 };
                 
enum scroll_mode_enum {NORMAL_SCROLL = 0, TRANSIENT_SCROLL = 1};

scroll_mode_enum scroll_mode;

const char *menu_scroll_val[2] = { "Normal", "Trans." };
scroll_mode_enum menu_scroll_mode_shown = NORMAL_SCROLL;
scroll_mode_enum menu_scroll_mode = NORMAL_SCROLL;

// assume they are all (ADC_50mV to ADC_10V) indexed 0-7
const char *menu_range_val[8] = { " 50mV", "100mV", "200mV", "500mV", "   1V", "   2V", "   5V", "  10V" };
int8_t menu_chA_shown = ADC_500mV;
int8_t menu_chA = ADC_500mV;
int8_t menu_chB_shown = ADC_500mV;
int8_t menu_chB = ADC_500mV;

enum save_mode_enum { SAVE_VCD = 0, SAVE_RAW = 1 };
const char *menu_save_mode_val[2] = { "VCD", "Raw" };
save_mode_enum menu_save_mode_shown = SAVE_VCD;
save_mode_enum menu_save_mode = SAVE_VCD;

const char *menu_dump_val = "Memory Dump";

uint16_t menu_wavefrq_shown = 100; // default output waveform frequency in Hz
uint16_t menu_wavefrq = 100;

void menu_update(MenuDrawable *menu, menu1_entry entry)
{
	char tb[20];
	if (entry == ENTRY_SCROLL || entry == ENTRIES_ALL)
	{
		strcpy(tb, "Scroll: ");
		strcat(tb, menu_scroll_val[menu_scroll_mode_shown]);
		menu->setText(ENTRY_SCROLL, tb);
		if (menu_scroll_mode_shown == menu_scroll_mode)
		{
			menu->setColor(ENTRY_SCROLL, WHITE);
		}
		else
		{
			menu->setColor(ENTRY_SCROLL, GREY);
		}

	}
	if (entry == ENTRY_CH_A_RANGE || entry == ENTRIES_ALL)
	{
		strcpy(tb, "CH(A) range: ");
		strcat(tb, menu_range_val[menu_chA_shown]);
		menu->setText(ENTRY_CH_A_RANGE, tb);
		if (menu_chA_shown == menu_chA)
		{
			menu->setColor(ENTRY_CH_A_RANGE, WHITE);
		}
		else
		{
			menu->setColor(ENTRY_CH_A_RANGE, GREY);
		}
	}
	if (entry == ENTRY_CH_B_RANGE || entry == ENTRIES_ALL)
	{
		strcpy(tb, "CH(B) range: ");
		strcat(tb, menu_range_val[menu_chB_shown]);
		menu->setText(ENTRY_CH_B_RANGE, tb);
		if (menu_chB_shown == menu_chB)
		{
			menu->setColor(ENTRY_CH_B_RANGE, WHITE);
		}
		else
		{
			menu->setColor(ENTRY_CH_B_RANGE, GREY);
		}
	}
	if (entry == ENTRY_SAVE_MODE || entry == ENTRIES_ALL)
	{
		strcpy(tb, "Save: ");
		strcat(tb, menu_save_mode_val[menu_save_mode_shown]);
		menu->setText(ENTRY_SAVE_MODE, tb);
		if (menu_save_mode_shown == menu_save_mode)
		{
			menu->setColor(ENTRY_SAVE_MODE, WHITE);
		}
		else
		{
			menu->setColor(ENTRY_SAVE_MODE, GREY);
		}
	}
	if (entry == ENTRY_MEMORY_DUMP || entry == ENTRIES_ALL)
	{
		menu->setText(ENTRY_MEMORY_DUMP, menu_dump_val);
		menu->setColor(ENTRY_MEMORY_DUMP, WHITE);
	}
	if (entry == ENTRY_WAVE_FRQ || entry == ENTRIES_ALL)
	{
	    snprintf(tb, sizeof(tb) - 1, "Out frq: %u Hz ", menu_wavefrq_shown);
		menu->setText(ENTRY_WAVE_FRQ, tb);
		if (menu_wavefrq_shown == menu_wavefrq)
		{
			menu->setColor(ENTRY_WAVE_FRQ, WHITE);
		}
		else
		{
			menu->setColor(ENTRY_WAVE_FRQ, GREY);
		}
	}
}


void menu_click(MenuDrawable *menu, menu1_entry index)
{
	switch(index)
	{
		case ENTRY_MEMORY_DUMP: //do a memory dump
            crash_with_message("User-initiated memory dump",
                                __builtin_return_address(0));
            break;
		case ENTRY_SCROLL:
    	    menu_scroll_mode = menu_scroll_mode_shown;
    	    break;
        case ENTRY_CH_A_RANGE:
    	    menu_chA = menu_chA_shown;
            __Set(CH_A_COUPLE, DC);
            __Set(CH_A_RANGE, menu_chA);
    	    break;
        case ENTRY_CH_B_RANGE:
    	    menu_chB = menu_chB_shown;
            __Set(CH_B_COUPLE, DC);
            __Set(CH_B_RANGE, menu_chB);
    	    break;
        case ENTRY_SAVE_MODE:
    	    menu_save_mode = menu_save_mode_shown;
    	    break;
    	case ENTRY_WAVE_FRQ:
    	    menu_wavefrq = menu_wavefrq_shown;
    	    config_wave_out(menu_wavefrq);
    	default:
    	    break;
    }
    menu_update(menu, index);
}

void save_data(DSOSignalStream *stream, bool last_chunk) 
{
    g_write_ongoing = true;
    if (g_first_chunk)
    {
        g_bytes_written += _fprintf("$version DSO Quad Logic Analyzer DB rev" SVN_REVISION " $end\n");
        g_bytes_written += _fprintf("$timescale 2us $end\n");
        g_bytes_written += _fprintf("$scope module logic $end\n");
        g_bytes_written += _fprintf("$var wire 1 A ChannelA $end\n");
        g_bytes_written += _fprintf("$var wire 1 B ChannelB $end\n");
        g_bytes_written += _fprintf("$var wire 1 C ChannelC $end\n");
        g_bytes_written += _fprintf("$var wire 1 D ChannelD $end\n");
        g_bytes_written += _fprintf("$upscope $end\n");
        g_bytes_written += _fprintf("$enddefinitions $end\n");
        g_bytes_written += _fprintf("$dumpvars 0A 0B 0C 0D $end\n");
        g_first_chunk = false;
    }
            
    SignalEvent event;
	while (stream->read_forwards(event) && !stream->was_last_event())
    {
        g_bytes_written += _fprintf("#%lu %dA %dB %dC %dD\n",
                 (uint32_t)(event.start + g_chunk_time_offset),
                 !!(event.levels & 1), !!(event.levels & 2),
                 !!(event.levels & 4), !!(event.levels & 8)
        );
    }
    _fprintf("\n\n\n"); // mark of end of chunk
    g_chunk_time_offset += event.start;
    if (last_chunk)
    {
        g_bytes_written += _fprintf("#%lu\n", (uint32_t)(event.end + g_chunk_time_offset));
    }
    g_wr_ready = false;
    g_write_ongoing = false;
}

void save_data_raw(struct signal_buffer_t *buf) 
{
    g_write_ongoing = true;
    if (g_first_chunk)
    {
        g_first_chunk = false;
    }
    size_t i;
    for (i = 0; i < buf->bytes; i++)
    {
        _fputc(buf->storage[i]);
    }
    g_bytes_written += buf->bytes;
    g_wr_ready = false;
    g_write_ongoing = false;
}


int main(void)
{   
    __Set(BEEP_VOLUME, 0);
    lcd_init();
    lcd_printf(120, 50, RGB565RGB(0,255,0), 0, "LCD TYPE %08lx", LCD_TYPE);
    __Display_Str(86, 33, RGB565RGB(0,255,0), 0, (u8*)"Logic Analyzer (c) 2012 jpa");
    __Display_Str(62, 16, RGB565RGB(0,255,0), 0, (u8*)"ExtWrite bld " SVN_REVISION " (c) 2024 MaxWolf" );

    // USART1 8N1 115200bps debug port
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    USART1->BRR = ((72000000 / (16 * 115200)) << 4) | 1;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    gpio_usart1_tx_mode(GPIO_AFOUT_10);
    gpio_usart1_rx_mode(GPIO_HIGHZ_INPUT);
    
    __Set(ADC_CTRL, EN);       
    __Set(ADC_MODE, SEPARATE);               

    __Set(CH_A_COUPLE, DC);
    __Set(CH_A_RANGE, menu_chA);
    
    __Set(CH_B_COUPLE, DC);
    __Set(CH_B_RANGE, menu_chB);

    config_wave_out(menu_wavefrq);
    
    __Set(TRIGG_MODE, UNCONDITION);
    __Set(T_BASE_PSC, 0);
    __Set(T_BASE_ARR, 1); // MCO as sysclock/2
    __Set(CH_A_OFFSET, 0);
    __Set(CH_B_OFFSET, 0);
    __Set_Param(FPGA_SP_PERCNT_L, 0);
    __Set_Param(FPGA_SP_PERCNT_H, 0);
    
    __Read_FIFO();
    __Read_FIFO();
    
    while (~__Get(KEY_STATUS) & ALL_KEYS);
    get_keys(ANY_KEY);
    delay_ms(500); // Wait for ADC to settle
    
    start_capture();
    
    DSOSignalStream stream[2] = { &g_signal_buffer[0], &g_signal_buffer[1] }; // view is attached only to stream[0]
    XPosHandler xpos(400, stream[0]);
    
    //init gui
    std::vector<Drawable*> screenobjs;
    Window graphwindow(64, 0, 400, 240);
    screenobjs.push_back(&graphwindow);

    Grid grid(stream[0], &xpos);
    grid.color = RGB565RGB(63, 63, 63);
    grid.y0 = 60;
    grid.y1 = 170;
    graphwindow.items.push_back(&grid);
    
    uint16_t colors[4] = {0xFFE0, 0x07FF, 0xF81F, 0x07E0};
    char names[4][6] = {"CH(A)", "CH(B)", "CH(C)", "CH(D)"};
    for (int i = 0; i < 4; i++)
    {
        SignalGraph* graph = new SignalGraph(stream[0], &xpos, i);
        graph->y0 = 150 - i * 30;
        graph->color = colors[i];
        
        graphwindow.items.push_back(graph);
        
        int middle_y = graph->y0 + graph->height / 2;
        TextDrawable* text = new TextDrawable(50, middle_y, names[i]);
        text->valign = TextDrawable::MIDDLE;
        text->halign = TextDrawable::RIGHT;
        text->color = colors[i];
        screenobjs.push_back(text);
    }
    
    BreakLines breaklines(&xpos, 500000);
    breaklines.linecolor = RGB565RGB(127, 127, 127);
    breaklines.textcolor = RGB565RGB(127, 127, 127);
    breaklines.y0 = 50;
    breaklines.y1 = 180;
    graphwindow.items.push_back(&breaklines);
    
    TimeMeasure timemeasure(&xpos);
    timemeasure.linecolor = 0xFF00;
    graphwindow.items.push_back(&timemeasure);
    
    Cursor cursor(&xpos);
    cursor.linecolor = 0x00FF;
    graphwindow.items.push_back(&cursor);
    
    TextDrawable button1txt(0, 240, " CLEAR ");
    button1txt.invert = true;
    screenobjs.push_back(&button1txt);
    
    TextDrawable button2txt(65, 240, " SAVE ");
    button2txt.invert = true;
    screenobjs.push_back(&button2txt);
    
    TextDrawable button3txt(130, 240, " BMP ");
    button3txt.invert = true;
    screenobjs.push_back(&button3txt);
    
    TextDrawable button4txt(180, 240, " SETTINGS ");
    button4txt.invert = true;
    screenobjs.push_back(&button4txt);
    
    MenuDrawable menu1(180,116,6);

    // just a placeholder (see menu_update() for actual values
    menu1.setText(ENTRY_SCROLL,"");
    menu1.setColor(ENTRY_SCROLL, WHITE);
    menu1.setSeparator(ENTRY_SCROLL,true);
    menu1.setText(ENTRY_CH_A_RANGE,"");
    menu1.setColor(ENTRY_CH_A_RANGE, WHITE);
    menu1.setText(ENTRY_CH_B_RANGE,"");
    menu1.setColor(ENTRY_CH_B_RANGE, WHITE);
    menu1.setSeparator(ENTRY_CH_B_RANGE,true);
    menu1.setText(ENTRY_SAVE_MODE,"");
    menu1.setColor(ENTRY_SAVE_MODE, WHITE);
    menu1.setSeparator(ENTRY_SAVE_MODE,true);
    menu1.setText(ENTRY_MEMORY_DUMP,"Memory Dump");
    menu1.setColor(ENTRY_MEMORY_DUMP, WHITE);
    menu1.setSeparator(ENTRY_MEMORY_DUMP,true);
    menu1.setText(ENTRY_WAVE_FRQ,"");
    menu1.setColor(ENTRY_WAVE_FRQ, WHITE);
    menu1.index = 0;
    menu1.visible = false;
    menu_update(&menu1, ENTRIES_ALL);
    screenobjs.push_back(&menu1);

    TextDrawable statustext(390, 0, "");
    statustext.halign = TextDrawable::RIGHT;
    statustext.valign = TextDrawable::BOTTOM;
    screenobjs.push_back(&statustext);
    
    scroll_mode = NORMAL_SCROLL;
    
    char *active_filename = NULL;

    while(1) {
        xpos.set_zoom(xpos.get_zoom());
        
        size_t free_bytes, largest_block;
        get_malloc_memory_status(&free_bytes, &largest_block);

        bool need_write = g_wr_ready;
        int sb2write = 1 - g_active_sb;
        
        // Show_status also redraws the screen.
        // Yeah yeah, I know it's ugly.
        show_status(screenobjs, statustext,
                    "Pos: %u us  Buf: %2ld%%  RAM: %4d  Dsk: %c%c %2ld%%", 
                    (unsigned)(xpos.get_xpos() * 1000000 / DSOSignalStream::frequency),
                    div_round(g_signal_buffer[g_active_sb].bytes * 100, sizeof(g_signal_buffer[g_active_sb].storage)),
                    free_bytes,
                    '0'+g_active_sb, g_db_capture?(need_write?'W':'@'):'-',
                    div_round(g_bytes_written * 100, FLASH_DISK_SIZE)
                   );

        if (g_db_capture && need_write)
        {
            if (menu_save_mode == SAVE_VCD)
            {
                stream[sb2write].seek(0);
                save_data(&stream[sb2write], false);
            }
            else
            {
                save_data_raw(&g_signal_buffer[sb2write]);
            }
			need_write = false;
        }
        
        uint32_t start = get_time();
        uint32_t keys;
        while (!(keys = get_keys(ANY_KEY)) && (get_time() - start) < 100);
        
        if ((keys & BUTTON1) && (!g_db_capture))
        {
            g_active_sb = 0;
            start_capture();
            xpos.set_xpos(0);
        }
        
        if ((keys & BUTTON2) || g_stop_writing)
        {
            if (!g_db_capture && !g_stop_writing)
            {
                active_filename = select_filename(menu_save_mode == SAVE_VCD ? "WAVES%03d.VCD" : "WAVES%03d.RAW");
                show_status(screenobjs, statustext, "Writing to %s initiated", active_filename);
                _fopen_wr(active_filename);
                g_first_chunk = true;
                g_db_capture = true;
                g_chunk_time_offset = 0;
                delay_ms(500);
            }
            else
            {
            	if (g_stop_writing) 
            	{
            	    show_status(screenobjs, statustext, "Data overtake! Emergency writing!");
            	}
                g_db_capture = false;
                if (menu_save_mode == SAVE_VCD)
                {
                    stream[g_active_sb].seek(0);
                    save_data(&stream[g_active_sb], true);
                }
                else
                {
                    save_data_raw(&g_signal_buffer[g_active_sb]);
                }
                if (_fclose())
                {
                    show_status(screenobjs, statustext, "%s written ok (%u bytes)", active_filename, g_bytes_written);
                }
                else
                {
                    show_status(screenobjs, statustext, "%s write failed", active_filename);
                }
                active_filename = NULL;
                delay_ms(2000);
            }
            g_stop_writing = false;
        }
        
        if (keys & BUTTON3)
        {
            if (active_filename != NULL)
            {
                g_db_capture = false;
                _fclose();
            }
            active_filename = select_filename("LOGIC%03d.BMP");
            if (write_bitmap(active_filename))
            {
                show_status(screenobjs, statustext, "Screenshot %s written ok!", active_filename);
            }
            else
            {
                show_status(screenobjs, statustext, "Screenshot %s write failed.", active_filename);
            }
            active_filename = NULL;
            delay_ms(3000);
        }
        
        if (keys & BUTTON4)
        {
           // toggle the menu
            menu1.visible = !menu1.visible;
        }
        
        if (menu1.visible)
        {
            if (keys & SCROLL1_LEFT)
            {
            	switch(menu1.index)
            	{
            		case ENTRY_SCROLL:
                	    menu_scroll_mode_shown = (menu_scroll_mode_shown == NORMAL_SCROLL) ? TRANSIENT_SCROLL : NORMAL_SCROLL;
                	    break;
                    case ENTRY_CH_A_RANGE:
                	    menu_chA_shown = ( menu_chA_shown == 0 ? (sizeof(menu_range_val)/sizeof(menu_range_val[0]) - 1) : menu_chA_shown - 1 );
                	    break;
                    case ENTRY_CH_B_RANGE:
                	    menu_chB_shown = ( menu_chB_shown == 0 ? (sizeof(menu_range_val)/sizeof(menu_range_val[0]) - 1) : menu_chB_shown - 1 );
                	    break;
                    case ENTRY_SAVE_MODE:
                	    menu_save_mode_shown = (menu_save_mode_shown == SAVE_VCD) ? SAVE_RAW : SAVE_VCD;
                	    break;
                	case ENTRY_WAVE_FRQ:
                	    if (menu_wavefrq_shown <= 10)
                	    {
                	        if (menu_wavefrq_shown > 2) --menu_wavefrq_shown;
                	    }
                	    else if (menu_wavefrq_shown <= 100)
                	    {
                	        menu_wavefrq_shown -= 10;
                	    }
                	    else if (menu_wavefrq_shown <= 2000)
                	    {
                	        menu_wavefrq_shown -= 100;
                	    }
                	    else
                	    {
                	        menu_wavefrq_shown -= 1000;
                	    }
                	    break;
                }
                menu_update(&menu1, (menu1_entry)menu1.index);
            }
            if (keys & SCROLL1_RIGHT)
            {
            	switch(menu1.index)
            	{
            		case ENTRY_SCROLL:
                	    menu_scroll_mode_shown = (menu_scroll_mode_shown == NORMAL_SCROLL) ? TRANSIENT_SCROLL : NORMAL_SCROLL;
                	    break;
                    case ENTRY_CH_A_RANGE:
                	    menu_chA_shown = ( menu_chA_shown == (sizeof(menu_range_val)/sizeof(menu_range_val[0]) - 1) ? 0 : menu_chA_shown + 1 );
                	    break;
                    case ENTRY_CH_B_RANGE:
                	    menu_chB_shown = ( menu_chB_shown == (sizeof(menu_range_val)/sizeof(menu_range_val[0]) - 1) ? 0 : menu_chB_shown + 1 );
                	    break;
                    case ENTRY_SAVE_MODE:
                	    menu_save_mode_shown = (menu_save_mode_shown == SAVE_VCD) ? SAVE_RAW : SAVE_VCD;
                	    break;
                	case ENTRY_WAVE_FRQ:
                	    if (menu_wavefrq_shown >= 2000)
                	    {
                	        if (menu_wavefrq_shown < 20000) menu_wavefrq_shown += 1000;
                	    }
                	    else if (menu_wavefrq_shown >= 100)
                	    {
                	        menu_wavefrq_shown += 100;
                	    }
                	    else if (menu_wavefrq_shown >= 10)
                	    {
                	        menu_wavefrq_shown += 10;
                	    }
                	    else
                	    {
                	        ++menu_wavefrq_shown;
                	    }
                	    break;
                }
                menu_update(&menu1, (menu1_entry)menu1.index);
            }
            if (keys & SCROLL2_LEFT)
            {
                menu1.previous();
            }
            if (keys & SCROLL2_RIGHT)
            {
                menu1.next();
            }
            if (keys & SCROLL2_PRESS)
            {
                menu_click(&menu1, (menu1_entry)menu1.index);
            }
        }
        else
        {
            if (keys & SCROLL2_LEFT)
            {
                if (menu_scroll_mode == NORMAL_SCROLL)
                {
                    xpos.move_xpos(-scroller_speed());
                } 
                else if (menu_scroll_mode == TRANSIENT_SCROLL)
                {
                    SignalEvent event;
                    int offset;
                    signaltime_t center_time = xpos.get_xpos();
                    stream[0].seek(center_time);
                    stream[0].read_backwards(event);
                    xpos.set_xpos(event.start);
                }
            }
            if (keys & SCROLL2_RIGHT)
            {
                if (menu_scroll_mode == NORMAL_SCROLL)
                {
                    xpos.move_xpos(scroller_speed());
                }
                else if (menu_scroll_mode == TRANSIENT_SCROLL)
                {
                    SignalEvent event;
                    int offset;
                    signaltime_t center_time = xpos.get_xpos();
                    stream[0].seek(center_time);
                    stream[0].read_forwards(event);
                    xpos.set_xpos(event.end);
                }
            }
            
            if (keys & SCROLL2_PRESS)
            {
                timemeasure.Click();
            }
            
            int zoom = xpos.get_zoom();
            if ((keys & SCROLL1_LEFT) && zoom > -30)
                xpos.set_zoom(zoom - 1);
            
            if ((keys & SCROLL1_RIGHT) && zoom < 3)
                xpos.set_zoom(zoom + 1);
        }

    }
    
    return 0;
}

