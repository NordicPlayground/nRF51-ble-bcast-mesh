/***********************************************************************************
Copyright (c) Nordic Semiconductor ASA
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of other
  contributors to this software may be used to endorse or promote products
  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************/
#include <string.h>
#include "bootloader.h"
#include "bootloader_rtc.h"
#include "transport.h"
#include "bootloader_util.h"
#include "bootloader_info.h"
#include "dfu_mesh.h"
#include "dfu_bank.h"
#include "dfu_types_mesh.h"
#include "nrf_mbr.h"
#include "nrf_flash.h"
#include "mesh_aci.h"
#include "SEGGER_RTT.h"
#include "app_error.h"
#include "fifo.h"
#include "serial_handler.h"
#include "bootloader_app_bridge.h"
#include "rtt_log.h"
#include "dfu_util.h"
#include "boards.h"

/*****************************************************************************
* Local defines
*****************************************************************************/
#define IRQ_ENABLED                 (0x01)  /**< Field that identifies if an interrupt is enabled. */
#define MAX_NUMBER_INTERRUPTS       (32)    /**< Maximum number of interrupts available. */
#define FLASH_FIFO_SIZE             (8)     /**< Size of the async-flash queue. Must be greater than 4 to end a transfer. */
#define FLASH_HANDLER_IRQn          (SWI3_IRQn)
#define FLASH_HANDLER_IRQHandler    SWI3_IRQHandler

#define TIMER_YIELD_TIMEOUT         (SECS_TO_RTC_TICKS(10)) /**< Time to wait before bootloader yields to application. */
#define TIMER_REQ_TIMEOUT           (SECS_TO_RTC_TICKS(10)) /**< Time to wait before giving up on an ongoing request. */
#define TIMER_START_TIMEOUT         (SECS_TO_RTC_TICKS( 5)) /**< Time to wait for first data during a transfer. */
#define TIMER_DATA_TIMEOUT          (SECS_TO_RTC_TICKS( 1)) /**< Time to wait for next data during a transfer. */
/*****************************************************************************
* Local typedefs
*****************************************************************************/
/** Actions to execute on timeout. */
typedef enum
{
    TIMEOUT_ACTION_NONE,        /**< No action. Shouldn't occur. */
    TIMEOUT_ACTION_DFU_ABORT,   /**< Abort the current DFU transfer. */
    TIMEOUT_ACTION_DFU_TIMEOUT, /**< Send a timeout event to the DFU. It will handle it. */
    TIMEOUT_ACTION_GO_TO_APP,   /**< Jump to application. */
} timeout_action_t;

/** Single entry in the flash FIFO. */
typedef struct
{
    flash_op_type_t type;   /**< Type of operation. Write or erase. */
    flash_op_t      op;     /**< Operation parameters. */
} flash_queue_entry_t;

/*****************************************************************************
* Static globals
*****************************************************************************/
static fifo_t               m_flash_fifo;
static flash_queue_entry_t  m_flash_fifo_buf[FLASH_FIFO_SIZE];
static bool                 m_go_to_app;
static timeout_action_t     m_timeout_action;

#ifdef RTT_LOG
static char*                mp_timeout_action_strs[] =
{
    "NONE",
    "DFU ABORT",
    "DFU TIMEOUT",
    "GO TO APP",
};
#endif

/*****************************************************************************
* Static functions
*****************************************************************************/

/**
 * Set timeout for bootloader.
 *
 * @param[in] time_rtc Number of RTC ticks to wait before timeout.
 * @param[in] action The action to perform upon timeout.
 */
static void set_timeout(uint32_t time_rtc, timeout_action_t action)
{
#ifndef NO_TIMEOUTS
    NRF_RTC0->EVENTS_COMPARE[RTC_BL_STATE_CH] = 0;
    NRF_RTC0->CC[RTC_BL_STATE_CH] = (NRF_RTC0->COUNTER + time_rtc) & RTC_MASK;
    NRF_RTC0->INTENSET = (1 << (RTC_BL_STATE_CH + RTC_INTENSET_COMPARE0_Pos));
    m_timeout_action = action;
#endif
}

static void stop_timeout(void)
{
    __LOG("STOP TIMEOUT\n");
    NRF_RTC0->INTENCLR = (1 << (RTC_BL_STATE_CH + RTC_INTENSET_COMPARE0_Pos));
    m_timeout_action = TIMEOUT_ACTION_NONE;
}

static void interrupts_disable(void)
{
    uint32_t interrupt_setting_mask;
    uint32_t irq;

    /* Fetch the current interrupt settings. */
    interrupt_setting_mask = NVIC->ISER[0];

    /* Loop from interrupt 0 for disabling of all interrupts. */
    for (irq = 0; irq < MAX_NUMBER_INTERRUPTS; irq++)
    {
        if (interrupt_setting_mask & (IRQ_ENABLED << irq))
        {
            /* The interrupt was enabled, hence disable it. */
            NVIC_DisableIRQ((IRQn_Type)irq);
        }
    }
}

/** Interrupt indicating new serial command */
#ifdef RBC_MESH_SERIAL
void SWI2_IRQHandler(void)
{
    mesh_aci_command_check();
}
#endif

/** Interrupt handling Flash operations. */
void FLASH_HANDLER_IRQHandler(void)
{
    flash_queue_entry_t flash_entry;
    uint32_t op_count = 0;
    while (fifo_pop(&m_flash_fifo, &flash_entry) == NRF_SUCCESS)
    {
        op_count++;
        bl_cmd_t rsp_cmd;
        if (flash_entry.type == FLASH_OP_TYPE_WRITE)
        {
            APP_ERROR_CHECK_BOOL(IS_WORD_ALIGNED(flash_entry.op.write.start_addr));
            APP_ERROR_CHECK_BOOL(IS_WORD_ALIGNED(flash_entry.op.write.length));
            APP_ERROR_CHECK_BOOL(IS_WORD_ALIGNED(flash_entry.op.write.p_data));
            __LOG("WRITING to 0x%x.(len %d)\n", flash_entry.op.write.start_addr, flash_entry.op.write.length);
            if (flash_entry.op.write.start_addr >= 0x20000000)
            {
                uint8_t* p_dst = ((uint8_t*) flash_entry.op.write.start_addr);
                for (uint32_t i = 0; i < flash_entry.op.write.length; ++i, p_dst++)
                {
                    *p_dst = (*p_dst & flash_entry.op.write.p_data[i]);
                }
            }
            else
            {
                nrf_flash_store((uint32_t*) flash_entry.op.write.start_addr,
                                            flash_entry.op.write.p_data,
                                            flash_entry.op.write.length, 0);
            }

            rsp_cmd.type                      = BL_CMD_TYPE_FLASH_WRITE_COMPLETE;
            rsp_cmd.params.flash.write.p_data = flash_entry.op.write.p_data;
        }
        else
        {
            __LOG("ERASING 0x%x.\n", flash_entry.op.erase.start_addr);
            if (flash_entry.op.erase.start_addr >= 0x20000000)
            {
                memset((uint32_t*) flash_entry.op.erase.start_addr, 0xFF, flash_entry.op.erase.length);
            }
            else
            {
                nrf_flash_erase((uint32_t*) flash_entry.op.erase.start_addr,
                                            flash_entry.op.erase.length);
            }
            rsp_cmd.type                      = BL_CMD_TYPE_FLASH_ERASE_COMPLETE;
            rsp_cmd.params.flash.erase.p_dest = (uint32_t*) flash_entry.op.erase.start_addr;
        }
        bl_cmd_handler(&rsp_cmd);
    }

    if (op_count > 0)
    {
        bl_cmd_t idle_cmd;
        idle_cmd.type = BL_CMD_TYPE_FLASH_ALL_COMPLETE;
        bl_cmd_handler(&idle_cmd);
    }
    if (fifo_is_empty(&m_flash_fifo) && m_go_to_app)
    {
        bl_info_entry_t* p_segment_entry = bootloader_info_entry_get(BL_INFO_TYPE_SEGMENT_APP);
        bootloader_util_app_start(p_segment_entry->segment.start);
    }
}

static void rx_cb(mesh_packet_t* p_packet)
{
    mesh_adv_data_t* p_adv_data = mesh_packet_adv_data_get(p_packet);
    if (p_adv_data && p_adv_data->handle > RBC_MESH_APP_MAX_HANDLE)
    {
        bl_cmd_t rx_cmd;
        rx_cmd.type = BL_CMD_TYPE_RX;
        rx_cmd.params.rx.p_dfu_packet = (dfu_packet_t*) &p_adv_data->handle;
        rx_cmd.params.rx.length = p_adv_data->adv_data_length - 3;
        bl_cmd_handler(&rx_cmd);
    }
}

static uint32_t bl_evt_handler(bl_evt_t* p_evt)
{
    static bl_cmd_t rsp_cmd;
    bool respond = false;
    switch (p_evt->type)
    {
        case BL_EVT_TYPE_DFU_ABORT:
            bootloader_abort(p_evt->params.dfu.abort.reason);

            /* If bootloader abort returned, it means that the application
             * doesn't work, and we should return to the dfu operation. */
            dfu_mesh_start();
            break;
        case BL_EVT_TYPE_TX_RADIO:
        {
            mesh_packet_t* p_packet = NULL;
            if (!mesh_packet_acquire(&p_packet))
            {
                return NRF_ERROR_NO_MEM;
            }

            mesh_packet_set_local_addr(p_packet);
            p_packet->header.type = BLE_PACKET_TYPE_ADV_NONCONN_IND;
            p_packet->header.length = DFU_PACKET_OVERHEAD + p_evt->params.tx.radio.length;
            ((ble_ad_t*) p_packet->payload)->adv_data_type = MESH_ADV_DATA_TYPE;
            ((ble_ad_t*) p_packet->payload)->data[0] = (MESH_UUID & 0xFF);
            ((ble_ad_t*) p_packet->payload)->data[1] = (MESH_UUID >> 8) & 0xFF;
            ((ble_ad_t*) p_packet->payload)->adv_data_length = DFU_PACKET_ADV_OVERHEAD + p_evt->params.tx.radio.length;
            memcpy(&p_packet->payload[4], p_evt->params.tx.radio.p_dfu_packet, p_evt->params.tx.radio.length);

            bool success = transport_tx(p_packet,
                                        p_evt->params.tx.radio.tx_slot,
                                        p_evt->params.tx.radio.tx_count,
                                        (tx_interval_type_t) p_evt->params.tx.radio.interval_type);
            mesh_packet_ref_count_dec(p_packet);

            if (!success)
            {
                return NRF_ERROR_INTERNAL;
            }
            break;
        }
        case BL_EVT_TYPE_TX_ABORT:
            transport_tx_abort(p_evt->params.tx.abort.tx_slot);
            break;
        case BL_EVT_TYPE_TX_SERIAL:
        {
#ifdef RBC_MESH_SERIAL
            serial_evt_t serial_evt;
            serial_evt.opcode = SERIAL_EVT_OPCODE_DFU;
            memcpy(&serial_evt.params.dfu.packet,
                    p_evt->params.tx.serial.p_dfu_packet,
                    p_evt->params.tx.serial.length);
            serial_evt.length = SERIAL_PACKET_OVERHEAD + p_evt->params.tx.serial.length;
            if (!serial_handler_event_send(&serial_evt))
            {
                return NRF_ERROR_INTERNAL;
            }
            break;
#endif
        }
        case BL_EVT_TYPE_TIMER_SET:
            set_timeout(US_TO_RTC_TICKS(p_evt->params.timer.set.delay_us), TIMEOUT_ACTION_DFU_TIMEOUT);
            break;

        case BL_EVT_TYPE_DFU_NEW_FW:
            {
                stop_timeout();
                __LOG("New FW event\n");
                switch (p_evt->params.dfu.new_fw.fw_type)
                {
                    case DFU_TYPE_APP:
                        __LOG("\tAPP: %08x.%04x:%08x\n",
                                (uint32_t) p_evt->params.dfu.new_fw.fwid.app.company_id,
                                (uint32_t) p_evt->params.dfu.new_fw.fwid.app.app_id,
                                (uint32_t) p_evt->params.dfu.new_fw.fwid.app.app_version);
                        break;
                    case DFU_TYPE_SD:
                        __LOG("\tSD: %04x\n",
                                (uint32_t) p_evt->params.dfu.new_fw.fwid.sd);
                        break;
                    case DFU_TYPE_BOOTLOADER:
                        __LOG("\tBL: %02x:%02x\n",
                                (uint32_t) p_evt->params.dfu.new_fw.fwid.bootloader.id,
                                (uint32_t) p_evt->params.dfu.new_fw.fwid.bootloader.ver);
                        break;
                    default: break;
                }
                /* accept all new firmware, as the bootloader wouldn't run
                   unless there's an actual reason for it. */
                rsp_cmd.type = BL_CMD_TYPE_DFU_START_TARGET;
                rsp_cmd.params.dfu.start.target.p_bank_start = (uint32_t*) 0xFFFFFFFF; /* no banking */
                rsp_cmd.params.dfu.start.target.type = p_evt->params.dfu.new_fw.fw_type;
                rsp_cmd.params.dfu.start.target.fwid = p_evt->params.dfu.new_fw.fwid;
                respond = true;
            }
            break;

        case BL_EVT_TYPE_BANK_AVAILABLE:
            __LOG("Bank:\n");
            switch (p_evt->params.bank_available.bank_dfu_type)
            {
                case DFU_TYPE_APP:
                    __LOG("\tAPP: %08x.%04x:%08x\n",
                            (uint32_t) p_evt->params.bank_available.bank_fwid.app.company_id,
                            (uint32_t) p_evt->params.bank_available.bank_fwid.app.app_id,
                            (uint32_t) p_evt->params.bank_available.bank_fwid.app.app_version);
                    break;
                case DFU_TYPE_SD:
                    __LOG("\tSD: %04x\n",
                            (uint32_t) p_evt->params.bank_available.bank_fwid.sd);
                    break;
                case DFU_TYPE_BOOTLOADER:
                    __LOG("\tBL: %02x:%02x\n",
                            (uint32_t) p_evt->params.bank_available.bank_fwid.bootloader.id,
                            (uint32_t) p_evt->params.bank_available.bank_fwid.bootloader.ver);
                    break;
                default: break;
            }
            __LOG("\tLocation: 0x%x\n", p_evt->params.bank_available.p_bank_addr);
            __LOG("\tLength: 0x%x\n", p_evt->params.bank_available.bank_length);
            if (p_evt->params.bank_available.bank_dfu_type == DFU_TYPE_BOOTLOADER)
            {
                if (!dfu_mesh_app_is_valid())
                {
                    dfu_bank_flash(DFU_TYPE_BOOTLOADER);
                }
            }
            break;

        case BL_EVT_TYPE_DFU_REQ:
            {
                /* Always attempt to relay incoming transfers in BL mode. Will
                   not abort ongoing transfers. */
                if (p_evt->params.dfu.req.role == DFU_ROLE_RELAY)
                {
                    stop_timeout();
                    bl_cmd_t relay_cmd;
                    relay_cmd.type = BL_CMD_TYPE_DFU_START_RELAY;
                    relay_cmd.params.dfu.start.relay.fwid = p_evt->params.dfu.req.fwid;
                    relay_cmd.params.dfu.start.relay.type = p_evt->params.dfu.req.dfu_type;
                    relay_cmd.params.dfu.start.relay.transaction_id = p_evt->params.dfu.req.transaction_id;
                    bootloader_cmd_send(&relay_cmd);
                }
            }
            break;

        case BL_EVT_TYPE_DFU_START:
            set_timeout(TIMER_START_TIMEOUT, TIMEOUT_ACTION_DFU_ABORT);
            break;
        case BL_EVT_TYPE_DFU_DATA_SEGMENT_RX:
            __LOG("RX %u/%u\n",
                    p_evt->params.dfu.data_segment.received_segment,
                    p_evt->params.dfu.data_segment.total_segments);
            set_timeout(TIMER_DATA_TIMEOUT, TIMEOUT_ACTION_DFU_ABORT);
            break;

        case BL_EVT_TYPE_DFU_END:
            if (p_evt->params.dfu.end.dfu_type == DFU_TYPE_APP ||
                p_evt->params.dfu.end.dfu_type == DFU_TYPE_SD)
            {
                /* attempt to reboot to app */
                bootloader_abort(DFU_END_SUCCESS);
            }
            break;

        /* Defer the flash operations to an asynchronous handler. Doing it
           inline causes stack overflow, as the bootloader continues in the
           response callback. */
        case BL_EVT_TYPE_FLASH_WRITE:
            {
                if (!IS_WORD_ALIGNED(p_evt->params.flash.write.start_addr) ||
                    !IS_WORD_ALIGNED(p_evt->params.flash.write.p_data))
                {
                    return NRF_ERROR_INVALID_ADDR;
                }
                if (!IS_WORD_ALIGNED(p_evt->params.flash.write.length))
                {
                    return NRF_ERROR_INVALID_LENGTH;
                }
                flash_queue_entry_t queue_entry;
                queue_entry.type = FLASH_OP_TYPE_WRITE;
                memcpy(&queue_entry.op, &p_evt->params.flash, sizeof(flash_op_t));
                if (fifo_push(&m_flash_fifo, &queue_entry) != NRF_SUCCESS)
                {
                    __LOG(RTT_CTRL_TEXT_RED "FLASH FIFO FULL :( Increase the fifo size.\n");
                    return NRF_ERROR_NO_MEM;
                }
                NVIC_SetPendingIRQ(FLASH_HANDLER_IRQn);
            }
            break;
        case BL_EVT_TYPE_FLASH_ERASE:
            {
                flash_queue_entry_t queue_entry;
                queue_entry.type = FLASH_OP_TYPE_ERASE;
                memcpy(&queue_entry.op, &p_evt->params.flash, sizeof(flash_op_t));
                if (fifo_push(&m_flash_fifo, &queue_entry) != NRF_SUCCESS)
                {
                    __LOG(RTT_CTRL_TEXT_RED "FLASH FIFO FULL :( Increase the fifo size.\n");
                    return NRF_ERROR_NO_MEM;
                }
                NVIC_SetPendingIRQ(FLASH_HANDLER_IRQn);
            }
            break;
        default:
            return NRF_ERROR_NOT_SUPPORTED;
    }
    if (respond)
    {
        /* tail recursion */
        return bl_cmd_handler(&rsp_cmd);
    }
    else
    {
        return NRF_SUCCESS;
    }
}

/*****************************************************************************
* Interface functions
*****************************************************************************/
void bootloader_init(void)
{
    rtc_init();

    memset(&m_flash_fifo, 0, sizeof(fifo_t));
    m_flash_fifo.elem_array = m_flash_fifo_buf;
    m_flash_fifo.elem_size = sizeof(flash_queue_entry_t);
    m_flash_fifo.array_len = FLASH_FIFO_SIZE;
    fifo_init(&m_flash_fifo);
    NVIC_SetPriority(FLASH_HANDLER_IRQn, 3);
    NVIC_EnableIRQ(FLASH_HANDLER_IRQn);

    bootloader_app_bridge_init();

    bl_cmd_t init_cmd;
    init_cmd.type = BL_CMD_TYPE_INIT;
    init_cmd.params.init.bl_if_version = BL_IF_VERSION;
    init_cmd.params.init.event_callback = bl_evt_handler;
    init_cmd.params.init.timer_count = 1;
    init_cmd.params.init.tx_slots = TRANSPORT_TX_SLOTS;
    init_cmd.params.init.in_app = false;
    APP_ERROR_CHECK(bl_cmd_handler(&init_cmd));

#ifdef RBC_MESH_SERIAL
    mesh_aci_init();
#endif

    transport_init(rx_cb, RBC_MESH_ACCESS_ADDRESS_BLE_ADV);

    bool dfu_bank_flash_start;
    dfu_bank_scan(&dfu_bank_flash_start);
    if (dfu_bank_flash_start)
    {
        /* A bank is to be flashed. Attempt to go to app if possible. */
        NRF_POWER->GPREGRET = RBC_MESH_GPREGRET_CODE_GO_TO_APP;
    }
}

void bootloader_enable(void)
{
#ifdef RBC_MESH_SERIAL
    mesh_aci_start();
#endif
    bl_cmd_t enable_cmd;
    enable_cmd.type = BL_CMD_TYPE_ENABLE;
    bl_cmd_handler(&enable_cmd);
    transport_start();

    /* Recover from broken state */
    if (dfu_mesh_app_is_valid())
    {
        set_timeout(TIMER_YIELD_TIMEOUT, TIMEOUT_ACTION_GO_TO_APP);
    }
    else
    {
#ifdef RTT_LOG
        __LOG(RTT_CTRL_TEXT_RED "APP is invalid.\n");
        bl_info_flags_t* p_flags = &bootloader_info_entry_get(BL_INFO_TYPE_FLAGS)->flags;
        bl_info_segment_t* p_seg = &bootloader_info_entry_get(BL_INFO_TYPE_SEGMENT_APP)->segment;
        __LOG("\tINTACT: SD: %d APP: %d BL: %d\n",
                p_flags->sd_intact,
                p_flags->app_intact,
                p_flags->bl_intact);
        if (*((uint32_t*) p_seg->start) == 0xFFFFFFFF)
        {
            __LOG("\tNo application at 0x%x\n", p_seg->start);
        }
#endif
        /* update the bootloader if a bank is available */
        if (dfu_bank_flash(DFU_TYPE_BOOTLOADER) == NRF_SUCCESS)
        {
            return;
        }

        dfu_type_t missing = dfu_mesh_missing_type_get();
        if (missing != DFU_TYPE_NONE && dfu_bank_flash(missing) == NRF_ERROR_NOT_FOUND)
        {
            fwid_union_t req_fwid;
            bl_info_entry_t* p_fwid_entry = bootloader_info_entry_get(BL_INFO_TYPE_VERSION);
            APP_ERROR_CHECK_BOOL(p_fwid_entry != NULL);

            switch (missing)
            {
                case DFU_TYPE_SD:
                    req_fwid.sd = p_fwid_entry->version.sd;
                    break;
                case DFU_TYPE_APP:
                    req_fwid.app = p_fwid_entry->version.app;
                    break;
                default:
                    APP_ERROR_CHECK(NRF_ERROR_INVALID_DATA);
            }

            dfu_mesh_req(missing, &req_fwid, (uint32_t*) 0xFFFFFFFF);
        }
    }
}

uint32_t bootloader_cmd_send(bl_cmd_t* p_bl_cmd)
{
    return bl_cmd_handler(p_bl_cmd);
}

void bootloader_abort(dfu_end_t end_reason)
{
    __LOG("ABORT...\n");
    bl_info_entry_t* p_segment_entry = bootloader_info_entry_get(BL_INFO_TYPE_SEGMENT_APP);
    switch (end_reason)
    {
        case DFU_END_SUCCESS:
        case DFU_END_ERROR_TIMEOUT:
        case DFU_END_FWID_VALID:
        case DFU_END_ERROR_MBR_CALL_FAILED:
            if (p_segment_entry && dfu_mesh_app_is_valid())
            {
                if (fifo_is_empty(&m_flash_fifo))
                {
                    interrupts_disable();

                    sd_mbr_command_t com = {SD_MBR_COMMAND_INIT_SD, };

                    uint32_t err_code = sd_mbr_command(&com);
                    APP_ERROR_CHECK(err_code);

                    err_code = sd_softdevice_vector_table_base_set(p_segment_entry->segment.start);
                    APP_ERROR_CHECK(err_code);
#ifdef DEBUG_LEDS
                    NRF_GPIO->OUTSET = LEDS_MASK;
#endif
                    bootloader_util_app_start(p_segment_entry->segment.start);
                }
                else
                {
                    __LOG("->Will go to app once flash is finished.\n");
                    m_go_to_app = true;
                }
            }
            else if (p_segment_entry)
            {
                __LOG("->App not valid.\n");
            }
            else
            {
                __LOG("->No segment entry found\n");
            }
            break;
        case DFU_END_ERROR_INVALID_PERSISTENT_STORAGE:
            APP_ERROR_CHECK_BOOL(false);
        default:
            __LOG(RTT_CTRL_TEXT_RED "SYSTEM RESET (reason 0x%x)\n", end_reason);
            __disable_irq();
            while(1);
            //NVIC_SystemReset();
    }
}

void bootloader_timeout(void)
{
    __LOG("BL: TIMEOUT! ACTION: %s\n", mp_timeout_action_strs[(uint32_t) m_timeout_action]);
    bl_cmd_t cmd;
    switch (m_timeout_action)
    {
        case TIMEOUT_ACTION_DFU_TIMEOUT:
            cmd.type = BL_CMD_TYPE_TIMEOUT;
            cmd.params.timeout.timer_index = 0;
            break;
        case TIMEOUT_ACTION_DFU_ABORT:
            cmd.type = BL_CMD_TYPE_DFU_ABORT;
            break;
        case TIMEOUT_ACTION_GO_TO_APP:
            bootloader_abort(DFU_END_SUCCESS);
            return;
        default:
            APP_ERROR_CHECK(NRF_ERROR_INVALID_STATE);
    }
    m_timeout_action = TIMEOUT_ACTION_NONE;
    bootloader_cmd_send(&cmd);
}
